#include <Arduino.h>
#include <Wire.h>

namespace Bmi423 {

constexpr size_t FrameBytes = 28;
constexpr float AccelLsbPerG = 1048576.0f;
constexpr float GyroLsbPerDps = 2097.152f;

constexpr uint16_t decodeWord(const uint8_t* bytes) {
    return static_cast<uint16_t>(bytes[0]) |
           (static_cast<uint16_t>(bytes[1]) << 8);
}

constexpr int32_t decodeAxis(const uint8_t* bytes) {
    return (static_cast<int32_t>(bytes[0]) |
            (static_cast<int32_t>(bytes[1]) << 8) |
            (static_cast<int32_t>(bytes[2] & 0x7F) << 16)) -
           ((bytes[2] & 0x80) ? 0x800000 : 0);
}

constexpr int16_t decodeTemperature(const uint8_t* bytes) {
    return static_cast<int16_t>(static_cast<int32_t>(decodeWord(bytes)) -
                               ((bytes[1] & 0x80) ? 0x10000 : 0));
}

struct Sample {
    int32_t accel[3];
    int32_t gyro[3];
    int16_t temperature;

    float temperatureC() const {
        return 25.0f + temperature / 512.0f;
    }
};

inline bool decodeFrame(const uint8_t* frame, size_t length, Sample& sample) {
    if (frame == nullptr || length != FrameBytes) return false;
    for (size_t axis = 0; axis < 3; ++axis) {
        sample.accel[axis] = decodeAxis(frame + 2 + axis * 4);
        sample.gyro[axis] = decodeAxis(frame + 14 + axis * 4);
    }
    sample.temperature = decodeTemperature(frame + 26);
    return true;
}

}

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
    return true;
}

void sensorFailed(const char* message) {
    sensorOnline = false;
    lastAttemptMs = millis();
    Serial.println(message);
}

void outputSample(const Bmi423::Sample& sample) {
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
    Serial.println("Starting sensor...");

    delay(100);
    if (!Wire.begin(SensorSda, SensorScl, 100000)) {
        Serial.println("Cannot initialize I2C on GPIO2/GPIO3.");
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