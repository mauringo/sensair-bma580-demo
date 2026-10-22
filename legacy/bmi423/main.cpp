#ifdef PLATFORMIO

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Wire.h>
#include "Bmi423Data.h"

namespace {

constexpr int SensorSda = 2;
constexpr int SensorScl = 3;
constexpr int SensorCs = 10;
constexpr int SensorSdo = 9;
constexpr uint8_t SensorAddress = 0x69;
constexpr uint8_t ChipIdRegister = 0x00;
constexpr uint8_t DataRegister = 0x09;
constexpr uint8_t AccelConfigRegister = 0x26;
constexpr uint8_t GyroConfigRegister = 0x27;
constexpr uint8_t InterruptIoRegister = 0x33;
constexpr uint8_t InterruptMapRegister = 0x35;
constexpr uint8_t ResolutionRegister = 0x7B;
constexpr uint8_t CommandRegister = 0x7E;
constexpr uint32_t OutputIntervalMs = 200;
constexpr uint32_t PollIntervalMs = 10;
constexpr uint32_t RetryIntervalMs = 3000;

constexpr int TftCs = 25;
constexpr int TftDc = 26;
constexpr int TftSclk = 24;
constexpr int TftMosi = 23;
constexpr int DisplayPower = 5;

Adafruit_ST7789 tft(TftCs, TftDc, -1);
bool sensorOnline = false;
uint32_t lastSampleMs = 0;
uint32_t lastOutputMs = 0;
uint32_t lastAttemptMs = 0;

bool probeSensor() {
    Serial.printf("Shuttle I2C: SDA=GPIO%d (%d), SCL=GPIO%d (%d), CS=GPIO%d (%d), SDO=GPIO%d (%d)\n",
                  SensorSda, digitalRead(SensorSda), SensorScl, digitalRead(SensorScl),
                  SensorCs, digitalRead(SensorCs), SensorSdo, digitalRead(SensorSdo));
    Wire.beginTransmission(SensorAddress);
    const uint8_t selectedResult = Wire.endTransmission(true);
    Serial.printf("Address probe 0x%02X: %s (Wire status %u)\n", SensorAddress,
                  selectedResult == 0 ? "ACK" : "failed", selectedResult);
    if (selectedResult == 0) return true;

    Wire.beginTransmission(0x68);
    const uint8_t alternateResult = Wire.endTransmission(true);
    Serial.printf("Address probe 0x68: %s (Wire status %u)\n",
                  alternateResult == 0 ? "ACK" : "failed", alternateResult);
    if (alternateResult == 0) {
        Serial.println("Device responds at 0x68, not selected 0x69; check P2.3 SDO/GPIO9.");
    } else {
        Serial.println("No ACK at either IMU address. Check sensor power jumper (3.3 V), socket orientation and board revision.");
    }
    return false;
}

void showStatus(const char* message) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 16);
    tft.println("BMI423");
    tft.setTextSize(1);
    tft.setCursor(10, 60);
    tft.println(message);
}

bool writeRegister(uint8_t address, uint16_t value) {
    Wire.beginTransmission(SensorAddress);
    Wire.write(address);
    Wire.write(static_cast<uint8_t>(value));
    Wire.write(static_cast<uint8_t>(value >> 8));
    const uint8_t result = Wire.endTransmission();
    if (result != 0) {
        Serial.printf("BMI423 write 0x%02X failed: I2C error %u\n", address, result);
        return false;
    }
    return true;
}

bool readBytes(uint8_t address, uint8_t* bytes, size_t length) {
    Wire.beginTransmission(SensorAddress);
    Wire.write(address);
    const uint8_t result = Wire.endTransmission(false);
    if (result != 0) {
        Serial.printf("BMI423 read 0x%02X failed: I2C error %u\n", address, result);
        return false;
    }
    const size_t received = Wire.requestFrom(SensorAddress, length, true);
    if (received != length) {
        while (Wire.available()) Wire.read();
        Serial.printf("BMI423 short read 0x%02X: %u/%u bytes\n", address,
                      static_cast<unsigned>(received), static_cast<unsigned>(length));
        return false;
    }
    for (size_t index = 0; index < length; ++index) {
        bytes[index] = static_cast<uint8_t>(Wire.read());
    }
    return true;
}

bool checkRegister(uint8_t address, uint16_t expected) {
    uint8_t bytes[4];
    if (!readBytes(address, bytes, sizeof(bytes))) return false;
    const uint16_t value = Bmi423::decodeWord(bytes + 2);
    if (value != expected) {
        Serial.printf("BMI423 register 0x%02X: expected 0x%04X, got 0x%04X\n",
                      address, expected, value);
        return false;
    }
    return true;
}

bool initializeSensor() {
    if (!probeSensor()) return false;
    if (!checkRegister(ChipIdRegister, 0x00AB)) return false;
    if (!writeRegister(CommandRegister, 0xDEAF)) return false;
    delay(100);
    if (!checkRegister(ChipIdRegister, 0x00AB) ||
        !checkRegister(ResolutionRegister, 0x0000)) return false;
    if (!writeRegister(AccelConfigRegister, 0xE028)) return false;
    delay(10);
    if (!writeRegister(GyroConfigRegister, 0xE058)) return false;
    delay(100);
    if (!checkRegister(AccelConfigRegister, 0xE028) ||
        !checkRegister(GyroConfigRegister, 0xE058)) return false;
    if (!writeRegister(InterruptMapRegister, 0x0000) ||
        !writeRegister(InterruptIoRegister, 0x0000) ||
        !checkRegister(InterruptMapRegister, 0x0000) ||
        !checkRegister(InterruptIoRegister, 0x0000)) return false;
    lastSampleMs = millis();
    lastOutputMs = millis() - OutputIntervalMs;
    Serial.println("BMI423 ready: 24-bit, +/-8 g, +/-4000 dps, 100 Hz ODR, 10 ms polling, interrupts disabled");
    showStatus("24-bit / POLL / I2C 0x69");
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(10, 86);
    tft.print("Accel (g)");
    tft.setCursor(10, 170);
    tft.print("Gyro (dps)");
    tft.setTextColor(ST77XX_WHITE);
    const char* labels[] = {"X", "Y", "Z", "X", "Y", "Z", "C"};
    const int rows[] = {110, 130, 150, 194, 214, 234, 262};
    for (size_t index = 0; index < 7; ++index) {
        tft.setCursor(10, rows[index]);
        tft.print(labels[index]);
    }
    return true;
}

void sensorFailed(const char* message) {
    sensorOnline = false;
    lastAttemptMs = millis();
    Serial.println(message);
    showStatus(message);
}

void displayValue(int row, float value, int decimals) {
    tft.fillRect(38, row, 202, 16, ST77XX_BLACK);
    tft.setCursor(38, row);
    tft.print(value, decimals);
}

void outputSample(const Bmi423::Sample& sample) {
    for (size_t axis = 0; axis < 3; ++axis) {
        displayValue(110 + axis * 20, sample.accel[axis] / Bmi423::AccelLsbPerG, 4);
        displayValue(194 + axis * 20, sample.gyro[axis] / Bmi423::GyroLsbPerDps, 3);
    }
    displayValue(262, sample.temperatureC(), 2);
    Serial.printf("raw acc=[%ld,%ld,%ld] gyro=[%ld,%ld,%ld] temp=%d\n",
                  static_cast<long>(sample.accel[0]), static_cast<long>(sample.accel[1]),
                  static_cast<long>(sample.accel[2]), static_cast<long>(sample.gyro[0]),
                  static_cast<long>(sample.gyro[1]), static_cast<long>(sample.gyro[2]),
                  sample.temperature);
    Serial.printf("acc(g)=[%.4f,%.4f,%.4f] gyro(dps)=[%.3f,%.3f,%.3f] temp(C)=%.2f\n",
                  sample.accel[0] / Bmi423::AccelLsbPerG,
                  sample.accel[1] / Bmi423::AccelLsbPerG,
                  sample.accel[2] / Bmi423::AccelLsbPerG,
                  sample.gyro[0] / Bmi423::GyroLsbPerDps,
                  sample.gyro[1] / Bmi423::GyroLsbPerDps,
                  sample.gyro[2] / Bmi423::GyroLsbPerDps, sample.temperatureC());
}

}

void setup() {
    pinMode(SensorCs, INPUT_PULLUP);
    pinMode(SensorSdo, INPUT_PULLUP);
    digitalWrite(SensorSdo, HIGH);
    pinMode(SensorSdo, OUTPUT);

    Serial.begin(115200);
    delay(2500);
    pinMode(DisplayPower, OUTPUT);
    digitalWrite(DisplayPower, LOW);
    delay(20);
    SPI.begin(TftSclk, -1, TftMosi, TftCs);
    tft.init(240, 320, SPI_MODE3);
    tft.setSPISpeed(20000000);
    tft.setRotation(0);
    tft.invertDisplay(true);
    tft.setTextWrap(false);
    showStatus("Starting sensor...");

    delay(100);
    if (!Wire.begin(SensorSda, SensorScl, 100000)) {
        Serial.println("Cannot initialize I2C on GPIO2/GPIO3.");
        showStatus("I2C initialization failed");
        while (true) delay(1000);
    }
    Wire.setTimeOut(50);
    sensorOnline = initializeSensor();
    lastAttemptMs = millis();
    if (!sensorOnline) sensorFailed("BMI423 init failed; retrying");
}

void loop() {
    if (!sensorOnline) {
        if (millis() - lastAttemptMs >= RetryIntervalMs) {
            sensorOnline = initializeSensor();
            lastAttemptMs = millis();
            if (!sensorOnline) sensorFailed("BMI423 init failed; retrying");
        }
        delay(1);
        return;
    }
    const uint32_t now = millis();
    if (now - lastSampleMs >= PollIntervalMs) {
        uint8_t frame[Bmi423::FrameBytes];
        Bmi423::Sample sample;
        if (!readBytes(DataRegister, frame, sizeof(frame)) ||
            !Bmi423::decodeFrame(frame, sizeof(frame), sample)) {
            sensorFailed("BMI423 read failed; retrying");
            return;
        }
        lastSampleMs = now;
        if (lastSampleMs - lastOutputMs >= OutputIntervalMs) {
            lastOutputMs = lastSampleMs;
            outputSample(sample);
        }
    }
    delay(1);
}

#endif