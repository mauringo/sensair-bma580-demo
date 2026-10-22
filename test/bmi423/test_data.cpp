#include "Bmi423Data.h"
#include <assert.h>
#include <math.h>

constexpr uint8_t BoschFrame[] = {
    0xAA, 0x55,
    0x36, 0xA1, 0x00, 0x00, 0x47, 0xC8, 0xFF, 0xFF,
    0x4D, 0xD0, 0x0F, 0x00, 0xE4, 0xFF, 0xFF, 0xFF,
    0xCE, 0xFF, 0xFF, 0xFF, 0x55, 0xFF, 0xFF, 0xFF,
    0x22, 0x02
};
constexpr uint8_t MinimumAxis[] = {0x00, 0x00, 0x80, 0x00};
constexpr uint8_t MaximumAxis[] = {0xFF, 0xFF, 0x7F, 0xFF};
constexpr uint8_t NegativeTemperature[] = {0x00, 0xFE};

static_assert(sizeof(BoschFrame) == Bmi423::FrameBytes, "28-byte frame");
static_assert(Bmi423::decodeAxis(BoschFrame + 2) == 41270, "accel X");
static_assert(Bmi423::decodeAxis(BoschFrame + 6) == -14265, "accel Y");
static_assert(Bmi423::decodeAxis(BoschFrame + 10) == 1036365, "accel Z");
static_assert(Bmi423::decodeAxis(BoschFrame + 14) == -28, "gyro X");
static_assert(Bmi423::decodeAxis(BoschFrame + 18) == -50, "gyro Y");
static_assert(Bmi423::decodeAxis(BoschFrame + 22) == -171, "gyro Z");
static_assert(Bmi423::decodeAxis(MinimumAxis) == -8388608, "minimum 24-bit");
static_assert(Bmi423::decodeAxis(MaximumAxis) == 8388607, "maximum 24-bit");
static_assert(Bmi423::decodeTemperature(BoschFrame + 26) == 546, "temperature");
static_assert(Bmi423::decodeTemperature(NegativeTemperature) == -512, "negative temperature");

int main() {
    Bmi423::Sample sample = {};
    assert(!Bmi423::decodeFrame(nullptr, sizeof(BoschFrame), sample));
    assert(!Bmi423::decodeFrame(BoschFrame, sizeof(BoschFrame) - 1, sample));
    assert(Bmi423::decodeFrame(BoschFrame, sizeof(BoschFrame), sample));
    assert(sample.accel[0] == 41270 && sample.accel[1] == -14265);
    assert(sample.accel[2] == 1036365);
    assert(sample.gyro[0] == -28 && sample.gyro[1] == -50 && sample.gyro[2] == -171);
    assert(fabsf(sample.accel[2] / Bmi423::AccelLsbPerG - 0.9884f) < 0.0001f);
    assert(fabsf(sample.gyro[0] / Bmi423::GyroLsbPerDps + 0.0134f) < 0.0001f);
    assert(fabsf(sample.temperatureC() - 26.06640625f) < 0.00001f);
}