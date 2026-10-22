#pragma once
#include <Arduino.h>
#include <Wire.h>
extern "C" {
#include "bosch/bma580_features.h"
}

class BMA580 {
public:
    struct Sample {
        int16_t rawX, rawY, rawZ;
        float x, y, z; // g; multiply by 9.80665 for m/s^2
        float temperatureC;
    };
    // Caller initializes Wire (pins, frequency and timeout) before begin().
    bool begin(TwoWire& wire = Wire, uint8_t address = BMA5_I2C_ADDRESS,
               uint8_t range = BMA5_ACC_RANGE_MAX_8G,
               uint8_t odr = BMA5_ACC_ODR_HZ_100);
    bool dataReady(); // false with lastError()==0 means no new sample
    bool read(Sample& sample); // latest registers; call after dataReady()
    // Latched active-low, open-drain motion interrupt on INT1 or INT2.
    // Threshold and duration use Bosch feature units, independent of readout range.
    bool enableMotionWake(uint16_t threshold = 160, uint16_t duration = 2,
                          uint8_t interruptPin = 1);
    bool readMotionInterrupt(bool& motion); // acknowledges and rearms; never call from ISR
    int8_t lastError() const { return error_; }
    uint8_t chipId() const { return dev_.chip_id; }
    uint8_t address() const { return address_; }
    // Advanced Bosch calls can change configuration/scaling; call begin() to restore.
    bma5_dev* device() { return &dev_; }
    BMA580() = default;
    BMA580(const BMA580&) = delete;
    BMA580& operator=(const BMA580&) = delete;
private:
    static BMA5_INTF_RET_TYPE readBus(uint8_t, uint8_t*, uint32_t, void*);
    static BMA5_INTF_RET_TYPE writeBus(uint8_t, const uint8_t*, uint32_t, void*);
    static void delayUs(uint32_t, void*);
    bool result(int8_t value) { error_ = value; return value == BMA5_OK; }
    bool expired() const;
    bool setMotionEnabled(bool enabled);
    bool motionEnabled_ = false;
    uint8_t motionPin_ = 1;
    TwoWire* wire_ = nullptr;
    bma5_dev dev_ = {};
    uint8_t address_ = BMA5_I2C_ADDRESS;
    int8_t error_ = BMA5_E_DEV_NOT_FOUND;
    float scale_ = 8.0f / 32768.0f;
    bool initialized_ = false;
    bool initializing_ = false;
    uint32_t initStart_ = 0;
};
