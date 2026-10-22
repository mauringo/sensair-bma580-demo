#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Wire.h>
#include <BMA580.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include "SleepCountdown.h"

namespace {
constexpr int SensorInterrupt = 0; // INT2 on an RTC-capable GPIO, active-low open-drain
constexpr int SensorSda = 2, SensorScl = 3, SensorCs = 10;
constexpr int TftCs = 25, TftDc = 26, TftSclk = 24, TftMosi = 23, DisplayPower = 5;
constexpr uint32_t RetryMs = 3000, OutputMs = 200, SampleTimeoutMs = 2000;
Adafruit_ST7789 tft(TftCs, TftDc, -1);
BMA580 sensor;
bool online = false;
uint32_t lastAttempt = 0, lastSample = 0, lastOutput = 0;
constexpr uint32_t SleepAfterMs = 20000;
SleepCountdown sleepCountdown(SleepAfterMs);
int32_t displayedSeconds = -1;
uint32_t wakeCount = 0;
esp_sleep_wakeup_cause_t lastWakeCause = ESP_SLEEP_WAKEUP_UNDEFINED;
// Awake: read the latched level directly. RTC EXT1 handles the sleep IRQ.
// No CPU GPIO ISR is needed and no ISR can contend with display/I2C work.
RTC_NOINIT_ATTR uint32_t previousStage;
RTC_NOINIT_ATTR uint32_t stageMagic;
void checkpoint(uint32_t stage) { stageMagic = 0xB580CAFE; previousStage = stage; }

void updateCountdown() {
    const uint32_t seconds = sleepCountdown.remainingSeconds(millis());
    if (displayedSeconds == static_cast<int32_t>(seconds)) return;
    displayedSeconds = seconds;
    tft.fillRect(10, 272, 230, 22, ST77XX_BLACK);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(2);
    tft.setCursor(10, 272);
    tft.print("Sleep in: "); tft.print(seconds); tft.print(" s");
    tft.setTextColor(ST77XX_WHITE);
}

void initializeDisplay() {
    pinMode(DisplayPower, OUTPUT);
    digitalWrite(DisplayPower, LOW);
    delay(20);
    SPI.end();
    SPI.begin(TftSclk, -1, TftMosi, TftCs);
    tft.init(240, 320, SPI_MODE3);
    tft.setSPISpeed(20000000);
    tft.setRotation(0);
    tft.invertDisplay(true);
    tft.setTextWrap(false);
}

void status(const char* message) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 16);
    tft.print("BMA580");
    tft.setTextSize(1);
    tft.setCursor(10, 55);
    tft.print(message);
}

void failed(const char* message) {
    online = false;
    lastAttempt = millis();
    Serial.printf("%s (Bosch error %d, chip 0x%02X)\n", message, sensor.lastError(), sensor.chipId());
    status(message);
}

void drawLayout() {
    status("16-bit / 100 Hz / I2C 0x18");
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(10, 86); tft.print("Accel (g)");
    tft.setTextColor(ST77XX_WHITE);
    const char* labels[] = {"X", "Y", "Z", "C"};
    for (int i = 0; i < 4; ++i) {
        tft.setCursor(10, 120 + i * 36); tft.print(labels[i]);
    }
    displayedSeconds = -1;
    tft.setTextSize(1);
    tft.setCursor(10, 304);
    tft.printf("Wake #%lu cause %d", static_cast<unsigned long>(wakeCount), int(lastWakeCause));
    tft.setTextSize(2);
}

void startSensor() {
    // Recreate the ESP32 I2C driver after a failed/NACK transaction.
    Wire.end();
    if (!Wire.begin(SensorSda, SensorScl, 100000)) {
        failed("I2C restart failed");
        return;
    }
    Wire.setTimeOut(50);
    Serial.println("BMA580: I2C SDA=2 SCL=3, address 0x18");
    online = sensor.begin(Wire);
    lastAttempt = millis();
    if (!online) {
        failed("Init failed; retrying");
        Serial.println("Check sensor power, shuttle orientation and SDA/SCL. Expected chip ID 0xC4.");
        return;
    }
    Serial.printf("BMA580 ready: chip 0x%02X, +/-8 g, 100 Hz, 16 bit\n", sensor.chipId());
    if (!sensor.enableMotionWake(160, 2, 2)) { failed("Motion init failed"); return; }
    Serial.println("Motion wake armed: INT2=GPIO0 (RTC EXT1), sleep after 20 seconds idle");
    drawLayout();
    sleepCountdown.reset(millis());
    updateCountdown();
    lastSample = millis();
    lastOutput = millis() - OutputMs;
}

void output(const BMA580::Sample& sample) {
    const float values[] = {sample.x, sample.y, sample.z, sample.temperatureC};
    for (int i = 0; i < 4; ++i) {
        tft.fillRect(38, 120 + i * 36, 202, 20, ST77XX_BLACK);
        tft.setCursor(38, 120 + i * 36);
        tft.print(values[i], i == 3 ? 1 : 4);
    }
    Serial.printf("raw=[%d,%d,%d] acc(g)=[%.4f,%.4f,%.4f] temp(C)=%.1f wake=%lu cause=%d\n",
                  sample.rawX, sample.rawY, sample.rawZ,
                  sample.x, sample.y, sample.z, sample.temperatureC,
                  static_cast<unsigned long>(wakeCount), int(lastWakeCause));
}

bool handleMotion() {
    if (digitalRead(SensorInterrupt) == HIGH) return true;
    checkpoint(2);
    bool motion = false;
    if (!sensor.readMotionInterrupt(motion)) {
        failed("Motion read failed");
        return false;
    }
    if (motion || digitalRead(SensorInterrupt) == LOW) {
        sleepCountdown.reset(millis());
        if (motion) Serial.println("Motion interrupt: countdown reset to 20 s");
    }
    return true;
}

void sleepUntilMotion() {
    // No pending event is acknowledged here: latched LOW also covers events
    // arriving between this check and the CPU actually entering light sleep.
    if (digitalRead(SensorInterrupt) == LOW) return;
    Serial.println("Entering light sleep; move BMA580 to wake; 10-second recovery timer armed");
    Serial.flush();
    tft.enableDisplay(false);
    tft.enableSleep(true);
    // Keep LCD power/CS stable while the CPU sleeps; both pins are on HP GPIO.
    gpio_hold_en(static_cast<gpio_num_t>(DisplayPower));
    gpio_hold_en(static_cast<gpio_num_t>(TftCs));
    rtc_gpio_init(static_cast<gpio_num_t>(SensorInterrupt));
    rtc_gpio_set_direction(static_cast<gpio_num_t>(SensorInterrupt), RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_en(static_cast<gpio_num_t>(SensorInterrupt));
    rtc_gpio_pulldown_dis(static_cast<gpio_num_t>(SensorInterrupt));
    esp_sleep_enable_ext1_wakeup_io(1ULL << SensorInterrupt, ESP_EXT1_WAKEUP_ANY_LOW);
    // Keep CPU powered while diagnosing resume, avoiding CPU retention restore.
    esp_sleep_pd_config(ESP_PD_DOMAIN_CPU, ESP_PD_OPTION_ON);
    // Always retain a recovery path until the physical motion IRQ is proven.
    esp_sleep_enable_timer_wakeup(10000000ULL);
    checkpoint(7);
    disableLoopWDT(); // recovery timer is 10 s, longer than the task watchdog
    const esp_err_t result = esp_light_sleep_start();
    enableLoopWDT();
    checkpoint(8);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    // EXT1 temporarily owns this pad through the RTC IO mux.
    rtc_gpio_deinit(static_cast<gpio_num_t>(SensorInterrupt));
    pinMode(SensorInterrupt, INPUT_PULLUP);
    gpio_hold_dis(static_cast<gpio_num_t>(DisplayPower));
    gpio_hold_dis(static_cast<gpio_num_t>(TftCs));
    checkpoint(9);
    initializeDisplay();
    const auto cause = esp_sleep_get_wakeup_cause();
    lastWakeCause = cause;
    ++wakeCount;
    Serial.printf("Light sleep returned: %s, wake cause=%d, INT2=%d\n",
                  esp_err_to_name(result), static_cast<int>(cause), digitalRead(SensorInterrupt));
    // All I2C and interrupt acknowledgement happen after waking, outside ISR.
    if (!handleMotion()) return;
    sleepCountdown.reset(millis());
    lastSample = millis();
    lastOutput = millis() - OutputMs;
    drawLayout();
    updateCountdown();
    if (cause == ESP_SLEEP_WAKEUP_TIMER) {
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_ORANGE);
        tft.setCursor(10, 304);
        tft.print("Timer recovery - IRQ not received");
        tft.setTextSize(2);
        tft.setTextColor(ST77XX_WHITE);
    }
}

}

void setup() {
    // CS high selects I2C. BMA580 address is programmable, default 0x18;
    // SDO is not used as the BMI423 address-selection pin.
    pinMode(SensorCs, INPUT_PULLUP);
    pinMode(9, INPUT);
    pinMode(28, INPUT_PULLUP); // BOOT; sensor INT1 output remains disabled
    pinMode(SensorInterrupt, INPUT_PULLUP);
    Serial.begin(115200);
    Serial.setTxTimeoutMs(10);
    delay(2500);
    initializeDisplay();
    const uint32_t lastStage = stageMagic == 0xB580CAFE ? previousStage : 0;
    Serial.printf("Boot reset reason=%d, previous stage=%lu\n", int(esp_reset_reason()), static_cast<unsigned long>(lastStage));
    enableLoopWDT();
    status("Starting sensor...");
    if (!Wire.begin(SensorSda, SensorScl, 100000)) {
        status("I2C initialization failed");
        Serial.println("Cannot initialize I2C on GPIO2/GPIO3");
        while (true) delay(1000);
    }
    Wire.setTimeOut(50);
    const esp_err_t wakeResult = esp_sleep_enable_ext1_wakeup_io(
        1ULL << SensorInterrupt, ESP_EXT1_WAKEUP_ANY_LOW);
    if (wakeResult != ESP_OK) {
        Serial.printf("RTC wake config failed: %s\n", esp_err_to_name(wakeResult));
        status("Wake config failed");
        while (true) delay(1000);
    }
    startSensor();
}

void loop() {
    checkpoint(1);
    if (!online) {
        if (millis() - lastAttempt >= RetryMs) startSensor();
        delay(1);
        return;
    }
    if (!handleMotion()) return;
    checkpoint(6);
    updateCountdown();
    if (sleepCountdown.expired(millis())) {
        sleepUntilMotion();
        return;
    }
    checkpoint(3);
    if (sensor.dataReady()) {
        BMA580::Sample sample;
        checkpoint(4);
        if (!sensor.read(sample)) { failed("Read failed; retrying"); return; }
        lastSample = millis();
        if (lastSample - lastOutput >= OutputMs) {
            lastOutput = lastSample;
            checkpoint(5);
            output(sample);
        }
    } else if (sensor.lastError() != BMA5_OK) {
        failed("Status failed; retrying");
    } else if (millis() - lastSample >= SampleTimeoutMs) {
        failed("No samples; retrying");
    }
    delay(1);
}
