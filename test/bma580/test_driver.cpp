#include <BMA580.h>
#include "SleepCountdown.h"
#include <assert.h>
#include <cmath>
#include <iostream>
uint32_t fakeMs = 0;
TwoWire Wire;
static void ready(TwoWire& bus) {
    bus.regs[0] = BMA580_CHIP_ID;
    bus.regs[BMA580_REG_FEAT_ENG_GP_FLAGS] = BMA580_FEAT_INIT_STAT_INIT_OK;
    bus.regs[BMA5_REG_HEALTH_STATUS] = BMA5_SENSOR_HEALTH_STATUS_MSK;
}
int main() {
    TwoWire bus;
    ready(bus);
    BMA580 sensor;
    assert(sensor.begin(bus));
    assert(bus.address == 0x18 && sensor.chipId() == 0xC4);
    assert(bus.regs[BMA5_REG_INT2_CONF] == 1);
    assert(!sensor.dataReady() && sensor.lastError() == 0);
    bus.regs[0x11] = 1;
    bus.regs[0x18] = 0; bus.regs[0x19] = 0x10; // +1 g at 8 g range
    bus.regs[0x1A] = 0; bus.regs[0x1B] = 0x80; // -8 g
    bus.regs[0x1C] = 0xff; bus.regs[0x1D] = 0x7f;
    bus.regs[0x1E] = 0xfe; // -2 + 23 = 21 C
    assert(sensor.dataReady());
    BMA580::Sample sample = {};
    assert(sensor.read(sample));
    assert(sample.rawX == 4096 && sample.rawY == -32768 && sample.rawZ == 32767);
    assert(sample.x == 1.0f && sample.y == -8.0f);
    assert(std::fabs(sample.z - (32767.0f / 4096.0f)) < 0.00001f);
    assert(sample.temperatureC == 21.0f);
    assert(!sensor.dataReady() && sensor.lastError() == 0);
    bus.shortRead = true;
    sample.x = 42;
    assert(!sensor.read(sample) && sample.x == 42 && sensor.lastError() == BMA5_E_COM_FAIL);
    bus.shortRead = false;
    bus.nack = true;
    assert(!sensor.dataReady() && sensor.lastError() == BMA5_E_COM_FAIL);
    bus.nack = false;
    bus.regs[0] = 0xAB;
    assert(!sensor.begin(bus) && sensor.lastError() == BMA5_E_DEV_NOT_FOUND);
    ready(bus);
    bus.regs[2] = 0; // Bosch health-status loop must terminate
    const uint32_t start = fakeMs;
    assert(!sensor.begin(bus));
    assert(fakeMs - start >= 1500 && fakeMs - start < 1600);
    ready(bus);
    for (uint8_t range = 0; range < 4; ++range) {
        assert(sensor.begin(bus, 0x18, range));
        bus.regs[0x18] = 0; bus.regs[0x19] = 0x40;
        assert(sensor.read(sample));
        assert(sample.x == float(1 << range));
    }
    assert(sensor.enableMotionWake());
    assert(bus.regs[BMA5_REG_INT1_CONF] == 5); // latched, open-drain, active-low
    assert(bus.regs[BMA580_REG_FEAT_ENG_GPR_0] & BMA580_GEN_INT1_EN_MSK);
    bool motion = true;
    assert(sensor.readMotionInterrupt(motion) && !motion);
    bus.regs[BMA580_REG_INT_STATUS_INT1_0] = 8; // generic interrupt 1
    assert(sensor.readMotionInterrupt(motion) && motion);
    assert(bus.regs[BMA580_REG_INT_STATUS_INT1_0] == 0);
    assert(sensor.readMotionInterrupt(motion) && !motion);
    bus.nack = true;
    assert(!sensor.readMotionInterrupt(motion) && !motion);
    bus.nack = false;
    assert(sensor.enableMotionWake(160, 2, 2));
    assert(bus.regs[BMA5_REG_INT2_CONF] == 5);
    bus.regs[BMA580_REG_INT_STATUS_INT2_0] = 8;
    assert(sensor.readMotionInterrupt(motion) && motion);
    assert(bus.regs[BMA580_REG_INT_STATUS_INT2_0] == 0);
    bus.regs[BMA580_REG_FEAT_ENG_GP_FLAGS] = 0;
    assert(!sensor.enableMotionWake() && sensor.lastError() == -12);
    ready(bus);
    assert(!sensor.enableMotionWake(160, 2, 3) && sensor.lastError() == -10);
    assert(!sensor.enableMotionWake(0) && sensor.lastError() == -10);
    assert(!sensor.readMotionInterrupt(motion));
    SleepCountdown countdown(15000);
    countdown.reset(100);
    assert(countdown.remainingSeconds(100) == 15);
    assert(countdown.remainingSeconds(1099) == 15);
    assert(countdown.remainingSeconds(1100) == 14);
    assert(countdown.remainingSeconds(15099) == 1);
    assert(countdown.remainingSeconds(15100) == 0 && countdown.expired(15100));
    countdown.reset(0xfffffff0U);
    assert(countdown.remainingSeconds(0x000003d8U) == 14); // rollover +1000 ms
    countdown.reset(20000);
    assert(countdown.remainingSeconds(20000) == 15 && !countdown.expired(20000));
    assert(!sensor.begin(bus, 0x18, 4) && sensor.lastError() == -10);
    assert(!sensor.read(sample));
    std::cout << "BMA580 driver tests passed\n";
}
