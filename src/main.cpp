#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Wire.h>
#include <BMA580.h>
#include "SensairDisplay.h"
#include "SleepCountdown.h"
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <new>
#include <Preferences.h>

namespace {
constexpr int SensorSda = 2, SensorScl = 3, SensorCs = 10, SensorInterrupt = 10; // BMA580 INT2 is physically shared with CSB/P2.1
constexpr int TftCs = 25, TftDc = 26, TftSclk = 24, TftMosi = 23, DisplayPower = 5;
constexpr uint16_t MotionThreshold = 10, MotionDuration = 1;
SensairDisplay tft(TftCs, TftDc, -1);
BMA580 sensor;
bool online = false;
uint32_t lastAttempt = 0, lastOutput = 0, lastIrqPoll = 0, lastDiagnostic = 0;
uint32_t motionEvents = 0, displayFrames = 0, lastSamplePoll = 0;
volatile uint32_t interruptEdges = 0;
int lastIrqLevel = HIGH;
constexpr uint32_t SleepAfterMs = 20000;
constexpr uint64_t RecoveryWakeUs = 40000000ULL;
SleepCountdown sleepCountdown(SleepAfterMs);
int32_t displayedCountdown = -1;
uint32_t wakeCount = 0;
esp_sleep_wakeup_cause_t lastWakeCause = ESP_SLEEP_WAKEUP_UNDEFINED;
int wakeIrqLevel = HIGH;
esp_reset_reason_t bootResetReason = ESP_RST_UNKNOWN;
Preferences wakeTrace;
struct WakeTrace { uint32_t magic, stage; int32_t cause, irq, result; };
WakeTrace trace = {0xB5802026,0,0,1,0};

void saveWakeTrace(uint32_t stage, int cause=0, int irq=HIGH, int result=0) {
    trace={0xB5802026,stage,cause,irq,result};
    wakeTrace.putBytes("last",&trace,sizeof(trace));
}


void stopDisplay() {
    // A software reset/upload can leave GPIO holds from an earlier sleep.
    // A held-high CS would make every new LCD command ineffective.
    for (int pin : {DisplayPower, TftCs, TftDc, TftSclk, TftMosi}) {
        gpio_hold_dis(static_cast<gpio_num_t>(pin));
    }
    SPI.end();
    for (int pin : {TftCs, TftDc, TftSclk, TftMosi}) pinMode(pin, INPUT);
    pinMode(DisplayPower, OUTPUT);
    digitalWrite(DisplayPower, HIGH);
}

void resetDisplay() {
    // Recreate driver state as well as the bus and the powered panel.
    stopDisplay();
    delay(500);
    tft.~SensairDisplay();
    new (&tft) SensairDisplay(TftCs, TftDc, -1);
    digitalWrite(DisplayPower, LOW);
    delay(150);
    SPI.begin(TftSclk, -1, TftMosi, TftCs);
    initializeSensairPanel(tft);
}

void drawCountdown() {
    const uint32_t seconds = sleepCountdown.remainingSeconds(millis());
    if (displayedCountdown == static_cast<int32_t>(seconds)) return;
    displayedCountdown = seconds;
    tft.fillRect(10,232,230,20,ST77XX_BLACK);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(2);
    tft.setCursor(10,232); tft.printf("Sleep: %lu s", static_cast<unsigned long>(seconds));
}

void ARDUINO_ISR_ATTR sensorIsr() { interruptEdges = interruptEdges + 1; }

void status(const char* text) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 12); tft.print("BMA580");
    tft.setTextSize(1);
    tft.setCursor(10, 43); tft.print(text);
}

void failed(const char* message) {
    detachInterrupt(digitalPinToInterrupt(SensorInterrupt));
    online = false;
    lastAttempt = millis();
    Serial.printf("%s: error=%d\n", message, sensor.lastError());
    status(message);
}

void dumpConfiguration() {
    auto* dev = sensor.device();
    bma580_generic_interrupt_types cfg = {};
    cfg.generic_interrupt = BMA580_GEN_INT_1;
    bma580_feat_conf_err errors = {};
    bma580_feat_eng_gpr_0 enabled = {};
    bma580_feat_eng_gpr_1 source = {};
    bma580_int_map mapping = {};
    const int8_t r0 = bma580_get_generic_int_config(&cfg, 1, dev);
    const int8_t r1 = bma580_get_feat_conf_err(&errors, dev);
    const int8_t r2 = bma580_get_feat_eng_gpr_0(&enabled, dev);
    const int8_t r3 = bma580_get_feat_eng_gpr_1(&source, dev);
    const int8_t r4 = bma580_get_int_map(&mapping, dev);
    Serial.printf("Motion readback: rslt=[%d,%d,%d,%d,%d] enabled=%u src=%u map=%u error=%u\n",
                  r0,r1,r2,r3,r4,enabled.gen_int1_en,source.gen_int1_data_src,
                  mapping.gen_int1_int_map, errors.gen_int1_conf_err);
    Serial.printf("Config: threshold=%u hyst=%u axes=%u criterion=%u ref=%u duration=%u quiet=%u\n",
                  cfg.gen_int.slope_thres,cfg.gen_int.hysteresis,cfg.gen_int.axis_sel,
                  cfg.gen_int.criterion_sel,cfg.gen_int.acc_ref_up,cfg.gen_int.duration,cfg.gen_int.quiet_time);
}

void drawLayout() {
    status("100 Hz / INT2 GPIO10 / wake LOW");
    tft.setTextSize(2);
    tft.setCursor(10, 70); tft.print("Accel (g)");
    const char* labels[] = {"X", "Y", "Z", "C"};
    for (int i=0; i<4; ++i) { tft.setCursor(10, 100+i*30); tft.print(labels[i]); }
    displayedCountdown = -1;
    drawCountdown();
}

void startSensor() {
    detachInterrupt(digitalPinToInterrupt(SensorInterrupt));
    Wire.end();
    if (!Wire.begin(SensorSda, SensorScl, 100000)) { failed("I2C init failed"); return; }
    Wire.setTimeOut(50);
    if (!sensor.begin(Wire) || !sensor.enableMotionWake(MotionThreshold, MotionDuration, 2)) {
        failed("Sensor/interrupt init failed"); return;
    }
    dumpConfiguration();
    pinMode(SensorInterrupt, INPUT_PULLUP);
    interruptEdges = 0;
    attachInterrupt(digitalPinToInterrupt(SensorInterrupt), sensorIsr, FALLING);
    online = true;
    sleepCountdown.reset(millis());
    drawLayout();
    lastOutput = millis()-200;
    lastIrqPoll = lastDiagnostic = millis();
    Serial.println("Motion wake GPIO10 armed: idle 20 s, recovery timer 40 s.");
}

void output(const BMA580::Sample& sample) {
    ++displayFrames;
    const float values[] = {sample.x,sample.y,sample.z,sample.temperatureC};
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    for (int i=0; i<4; ++i) {
        tft.fillRect(36,100+i*30,204,18,ST77XX_BLACK);
        tft.setCursor(36,100+i*30); tft.print(values[i],i==3?1:4);
    }
    tft.setTextSize(1);
    tft.fillRect(10,216,230,10,ST77XX_BLACK);
    tft.setCursor(10,216); tft.printf("Events:%lu edges:%lu",static_cast<unsigned long>(motionEvents),static_cast<unsigned long>(interruptEdges));
    tft.fillRect(10,258,230,24,ST77XX_BLACK);
    tft.setCursor(10,258); tft.printf("IRQ:%s UP:%lus frame:%lu",lastIrqLevel?"HIGH":"LOW",static_cast<unsigned long>(millis()/1000),static_cast<unsigned long>(displayFrames));
    tft.setCursor(10,270); tft.printf("Wake:%lu %s",static_cast<unsigned long>(wakeCount),
        lastWakeCause==ESP_SLEEP_WAKEUP_GPIO?"SENSOR GPIO10":lastWakeCause==ESP_SLEEP_WAKEUP_TIMER?"RECOVERY TIMER":"BOOT");
    if (lastWakeCause==ESP_SLEEP_WAKEUP_UNDEFINED) tft.printf(" reset:%d",int(bootResetReason));
    Serial.printf("acc(g)=[%.4f,%.4f,%.4f] C=%.1f events=%lu edges=%lu IRQ=%d CS=%d SDO=%d INT1=%d up=%lu frame=%lu wake=%lu cause=%d wakeIRQ=%d\n",
                  sample.x,sample.y,sample.z,sample.temperatureC,
                  static_cast<unsigned long>(motionEvents),static_cast<unsigned long>(interruptEdges),
                  lastIrqLevel,digitalRead(10),digitalRead(9),digitalRead(28),
                  static_cast<unsigned long>(millis()/1000),static_cast<unsigned long>(displayFrames),
                  static_cast<unsigned long>(wakeCount),int(lastWakeCause),wakeIrqLevel);
}

bool serviceMotion() {
    if (millis()-lastIrqPoll>=10) {
        lastIrqPoll=millis();
        lastIrqLevel=digitalRead(SensorInterrupt); // sample BEFORE acknowledging
        const int pin0Before=digitalRead(0), pin9Before=digitalRead(9), pin28Before=digitalRead(28);
        bool motion=false;
        // Poll Bosch status independently of the GPIO: distinguish missing event
        // from an event generated by the sensor but absent on the physical pin.
        if (!sensor.readMotionInterrupt(motion)) { failed("Interrupt read failed"); return false; }
        if (motion) {
            ++motionEvents;
            sleepCountdown.reset(millis());
            Serial.printf("MOTION #%lu before ACK: GPIO0=%d GPIO10=%d GPIO9=%d GPIO28=%d edges=%lu\n",
                          static_cast<unsigned long>(motionEvents),pin0Before,lastIrqLevel,pin9Before,pin28Before,
                          static_cast<unsigned long>(interruptEdges));
        }
    }
    return true;
}

void sleepUntilMotion() {
    // A latched LOW arriving after this check will still cause immediate wake.
    if (digitalRead(SensorInterrupt)==LOW) return;
    detachInterrupt(digitalPinToInterrupt(SensorInterrupt));
    pinMode(SensorInterrupt,INPUT_PULLUP);
    gpio_sleep_sel_dis(static_cast<gpio_num_t>(SensorInterrupt));
    const esp_err_t gpioResult = gpio_wakeup_enable(static_cast<gpio_num_t>(SensorInterrupt),GPIO_INTR_LOW_LEVEL);
    const esp_err_t wakeResult = esp_sleep_enable_gpio_wakeup();
    const esp_err_t timerResult = esp_sleep_enable_timer_wakeup(RecoveryWakeUs);
    const esp_err_t cpuResult = esp_sleep_pd_config(ESP_PD_DOMAIN_CPU,ESP_PD_OPTION_ON);
    const esp_err_t topResult = esp_sleep_pd_config(ESP_PD_DOMAIN_TOP,ESP_PD_OPTION_ON);
    if (gpioResult!=ESP_OK || wakeResult!=ESP_OK || timerResult!=ESP_OK || cpuResult!=ESP_OK || topResult!=ESP_OK) {
        gpio_wakeup_disable(static_cast<gpio_num_t>(SensorInterrupt));
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        attachInterrupt(digitalPinToInterrupt(SensorInterrupt),sensorIsr,FALLING);
        sleepCountdown.reset(millis());
        Serial.printf("Sleep setup failed: gpio=%d wake=%d timer=%d cpu=%d top=%d\n",gpioResult,wakeResult,timerResult,cpuResult,topResult);
        return;
    }
    Serial.println("SLEEP: GPIO10 LOW wakes CPU; recovery timer 40 s");
    Serial.flush();
    stopDisplay();
    gpio_hold_en(static_cast<gpio_num_t>(DisplayPower)); // keep LCD supply off
    // Apply the wake GPIO after every display/bus/pin operation.
    gpio_wakeup_enable(static_cast<gpio_num_t>(SensorInterrupt),GPIO_INTR_LOW_LEVEL);
    saveWakeTrace(1);
    disableLoopWDT();
    const esp_err_t sleepResult = esp_light_sleep_start();
    wakeIrqLevel=digitalRead(SensorInterrupt);
    saveWakeTrace(2,int(esp_sleep_get_wakeup_cause()),wakeIrqLevel,int(sleepResult));
    enableLoopWDT();
    lastWakeCause = esp_sleep_get_wakeup_cause();
    ++wakeCount;
    gpio_wakeup_disable(static_cast<gpio_num_t>(SensorInterrupt));
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    pinMode(SensorInterrupt,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SensorInterrupt),sensorIsr,FALLING);
    gpio_hold_dis(static_cast<gpio_num_t>(DisplayPower));
    resetDisplay();
    saveWakeTrace(3,int(lastWakeCause),wakeIrqLevel,int(sleepResult));
    Serial.printf("WAKE #%lu: result=%s cause=%d IRQ=%d\n",static_cast<unsigned long>(wakeCount),esp_err_to_name(sleepResult),int(lastWakeCause),digitalRead(SensorInterrupt));
    // Poll and acknowledge the sensor only after the CPU and display are awake.
    lastIrqPoll=millis()-10;
    if (!serviceMotion()) return;
    sleepCountdown.reset(millis());
    lastOutput=millis()-200;
    drawLayout();
    saveWakeTrace(4,int(lastWakeCause),wakeIrqLevel,int(sleepResult));
}

}

void setup() {
    bootResetReason=esp_reset_reason();
    pinMode(SensorCs,INPUT_PULLUP);
    pinMode(9,INPUT);
    pinMode(0,INPUT_PULLUP); // unused shuttle INT2 connector, not BMA580 CSB
    pinMode(28,INPUT_PULLUP);
    pinMode(SensorInterrupt,INPUT_PULLUP);
    Serial.begin(115200);
    Serial.setTxTimeoutMs(10);
    delay(2000);
    wakeTrace.begin("wake-trace",false);
    WakeTrace previous={};
    wakeTrace.getBytes("last",&previous,sizeof(previous));
    Serial.printf("PREVIOUS TRACE: magic=%lX stage=%lu cause=%ld IRQ=%ld result=%ld reset=%d\n",
        static_cast<unsigned long>(previous.magic),static_cast<unsigned long>(previous.stage),
        static_cast<long>(previous.cause),static_cast<long>(previous.irq),static_cast<long>(previous.result),int(esp_reset_reason()));
    resetDisplay();
    enableLoopWDT();
    startSensor();
}

void loop() {
    if (!online) {
        if (millis()-lastAttempt>=3000) startSensor();
        delay(1); return;
    }
    if (!serviceMotion()) return;
    drawCountdown();
    if (sleepCountdown.expired(millis())) { sleepUntilMotion(); return; }
    if (millis()-lastSamplePoll<10) { delay(1); return; }
    lastSamplePoll=millis();
    if (sensor.dataReady()) {
        BMA580::Sample sample;
        if (!sensor.read(sample)) { failed("Sample read failed"); return; }
        if (millis()-lastOutput>=200) { lastOutput=millis(); output(sample); }
    } else if (sensor.lastError()!=BMA5_OK) { failed("Status read failed"); return; }
    if (millis()-lastDiagnostic>=5000) { lastDiagnostic=millis(); dumpConfiguration(); }
    delay(1);
}
