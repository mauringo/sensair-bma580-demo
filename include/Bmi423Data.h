#pragma once

#include <stddef.h>
#include <stdint.h>

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