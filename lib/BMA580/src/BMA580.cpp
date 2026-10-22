#include "BMA580.h"

bool BMA580::expired() const {
    return initializing_ && uint32_t(millis() - initStart_) >= 1500;
}

BMA5_INTF_RET_TYPE BMA580::readBus(uint8_t reg, uint8_t* data, uint32_t len, void* ptr) {
    auto& self = *static_cast<BMA580*>(ptr);
    // Bounds avoid overflowing Wire and Bosch's fixed register-read buffer.
    if (self.expired() || !data || !len || len > 32) return -1;
    auto& bus = *self.wire_;
    bus.beginTransmission(self.address_);
    if (bus.write(reg) != 1 || bus.endTransmission(false) != 0) return -1;
    if (bus.requestFrom(self.address_, static_cast<size_t>(len), true) != len) {
        while (bus.available()) bus.read();
        return -1;
    }
    for (uint32_t i = 0; i < len; ++i) data[i] = static_cast<uint8_t>(bus.read());
    return BMA5_INTF_RET_SUCCESS;
}

BMA5_INTF_RET_TYPE BMA580::writeBus(uint8_t reg, const uint8_t* data, uint32_t len, void* ptr) {
    auto& self = *static_cast<BMA580*>(ptr);
    if (self.expired() || !data || !len || len > 31) return -1;
    auto& bus = *self.wire_;
    bus.beginTransmission(self.address_);
    if (bus.write(reg) != 1 || bus.write(data, len) != len) {
        bus.endTransmission(true);
        return -1;
    }
    const auto status = bus.endTransmission(true);
    // Required idle time for writes while the sensor is suspended.
    delayMicroseconds(500);
    return status == 0 ? BMA5_INTF_RET_SUCCESS : -1;
}

void BMA580::delayUs(uint32_t us, void*) {
    if (us >= 1000) delay(us / 1000);
    delayMicroseconds(us % 1000);
}

bool BMA580::begin(TwoWire& wire, uint8_t address, uint8_t range, uint8_t odr) {
    initialized_ = false;
    motionEnabled_ = false;
    if (address < 0x08 || address > 0x77 || range > BMA5_ACC_RANGE_MAX_16G ||
        odr < BMA5_ACC_ODR_HZ_12P5 || odr > BMA5_ACC_ODR_HZ_6K4) return result(-10);
    wire_ = &wire;
    address_ = address;
    dev_ = {};
    dev_.intf = BMA5_I2C_INTF;
    dev_.bus_read = readBus;
    dev_.bus_write = writeBus;
    dev_.delay_us = delayUs;
    dev_.intf_ptr = this;
    dev_.context = BMA5_SMARTPHONE;
    initStart_ = millis();
    initializing_ = true;
    // Bosch init contains an unbounded health-status loop. Callback deadline bounds it.
    const int8_t initResult = bma580_init(&dev_);
    initializing_ = false;
    if (!result(initResult)) return false;
    if (!result(bma5_set_acc_conf_0(BMA5_SENSOR_CTRL_DISABLE, &dev_))) return false;
    bma580_int_map mapping = {}; // polling; no interrupt output mapping
    if (!result(bma580_set_int_map(&mapping, &dev_))) return false;
    // Datasheet startup sequence: configure INT2/CSB as an output in I2C
    // mode. Active-low with no mapped events keeps the shared CSB pin high.
    bma5_int_conf_types int2 = {};
    int2.int_src = BMA5_INT_2;
    int2.int_conf.int_mode = BMA5_INT2_MODE_LATCHED;
    int2.int_conf.int_lvl = BMA5_INT2_LVL_ACTIVE_LOW;
    int2.int_conf.int_od = BMA5_INT2_OD_PUSH_PULL;
    if (!result(bma5_set_int_conf(&int2, 1, &dev_))) return false;
    bma5_acc_conf cfg = {};
    cfg.acc_odr = odr;
    cfg.acc_range = range;
    cfg.acc_bwp = BMA5_ACC_BWP_NORM_AVG4;
    cfg.power_mode = BMA5_POWER_MODE_HPM;
    cfg.acc_iir_ro = BMA5_ACC_IIR_RO_DB_40;
    cfg.noise_mode = BMA5_NOISE_MODE_LOWER_NOISE;
    cfg.acc_drdy_int_auto_clear = BMA5_ACC_DRDY_INT_AUTO_CLEAR_DISABLED;
    if (!result(bma5_set_acc_conf(&cfg, &dev_))) return false;
    bma5_temp_conf temp = {};
    temp.temp_rate = BMA5_TEMP_RATE_HZ_25;
    temp.temp_meas_src = BMA5_TEMP_MEAS_SRC_TMP_INT;
    if (!result(bma5_set_temp_conf(&temp, &dev_))) return false;
    if (!result(bma5_set_acc_conf_0(BMA5_SENSOR_CTRL_ENABLE, &dev_))) return false;
    delay(50);
    bma5_config_status status = {};
    if (!result(bma5_get_config_status(&status, &dev_))) return false;
    if (status.acc_conf_err) return result(-11);
    scale_ = static_cast<float>(2U << range) / 32768.0f;
    initialized_ = true;
    return true;
}

bool BMA580::dataReady() {
    if (!initialized_) return result(BMA5_E_DEV_NOT_FOUND);
    bma5_sensor_status status = {};
    return result(bma5_get_sensor_status(&status, &dev_)) && status.acc_data_rdy;
}

bool BMA580::read(Sample& sample) {
    if (!initialized_) return result(BMA5_E_DEV_NOT_FOUND);
    bma5_accel raw = {};
    uint8_t temp = 0;
    // Clear only ACC ready, before reading, following Bosch polling sequence.
    bma5_sensor_status clear = {};
    clear.acc_data_rdy = 1;
    if (!result(bma5_set_sensor_status(&clear, &dev_)) ||
        !result(bma5_get_acc(&raw, &dev_)) ||
        !result(bma5_get_temp_data(&temp, &dev_))) return false;
    Sample next = {};
    next.rawX = raw.x; next.rawY = raw.y; next.rawZ = raw.z;
    next.x = raw.x * scale_; next.y = raw.y * scale_; next.z = raw.z * scale_;
    const int16_t signedTemp = temp < 128 ? temp : static_cast<int16_t>(temp) - 256;
    next.temperatureC = 23.0f + signedTemp;
    sample = next;
    return true;
}


bool BMA580::setMotionEnabled(bool enabled) {
    bma580_feat_eng_gpr_0 features = {};
    if (!result(bma580_get_feat_eng_gpr_0(&features, &dev_))) return false;
    features.gen_int1_en = enabled ? BMA5_ENABLE : BMA5_DISABLE;
    if (!result(bma580_set_feat_eng_gpr_0(&features, &dev_))) return false;
    const uint8_t commit = BMA5_ENABLE;
    return result(bma5_set_regs(BMA5_REG_FEAT_ENG_GPR_CTRL, &commit, 1, &dev_));
}

bool BMA580::enableMotionWake(uint16_t threshold, uint16_t duration, uint8_t interruptPin) {
    motionEnabled_ = false;
    if (!initialized_) return result(BMA5_E_DEV_NOT_FOUND);
    if (!threshold || threshold > 4095 || !duration || duration > 8191 ||
        (interruptPin != 1 && interruptPin != 2)) return result(-10);
    motionPin_ = interruptPin;
    if (!result(bma5_set_feat_eng_conf(BMA5_FEAT_ENG_CTRL_ENABLE, &dev_))) return false;
    delay(50);
    bma580_feat_eng_gp_flags flags = {};
    const uint32_t start = millis();
    do {
        if (!result(bma580_get_feat_eng_gp_flags(&flags, &dev_))) return false;
        if (flags.feat_init_stat == BMA580_FEAT_INIT_STAT_INIT_OK) break;
        delay(5);
    } while (uint32_t(millis() - start) < 500);
    if (flags.feat_init_stat != BMA580_FEAT_INIT_STAT_INIT_OK) return result(-12);
    if (!setMotionEnabled(false)) return false;
    delay(40); // Bosch generic-interrupt reset time
    bma580_generic_interrupt_types cfg = {};
    cfg.generic_interrupt = BMA580_GEN_INT_1;
    cfg.gen_int.slope_thres = threshold;
    cfg.gen_int.hysteresis = threshold < 16 ? threshold / 4 : 8;
    cfg.gen_int.comb_sel = BMA580_GEN_INT_COMB_SEL_LOGICAL_OR;
    cfg.gen_int.axis_sel = 7;
    cfg.gen_int.criterion_sel = BMA580_GEN_INT_CRI_SEL_ACT;
    cfg.gen_int.acc_ref_up = BMA580_GEN_INT_ACC_REF_UP_ON_ALWAYS;
    cfg.gen_int.duration = duration;
    cfg.gen_int.wait_time = 3;
    cfg.gen_int.quiet_time = 5;
    if (!result(bma580_set_generic_int_config(&cfg, 1, &dev_))) return false;
    bma580_feat_eng_gpr_1 source = {};
    if (!result(bma580_get_feat_eng_gpr_1(&source, &dev_))) return false;
    source.gen_int1_data_src = BMA580_GEN_INT1_DATA_SRC_DATA_SRC_1; // 50 Hz filter
    if (!result(bma580_set_feat_eng_gpr_1(&source, &dev_))) return false;
    const uint8_t commit = BMA5_ENABLE;
    if (!result(bma5_set_regs(BMA5_REG_FEAT_ENG_GPR_CTRL, &commit, 1, &dev_))) return false;
    bma5_int_conf_types pin = {};
    pin.int_src = motionPin_ == 1 ? BMA5_INT_1 : BMA5_INT_2;
    pin.int_conf.int_mode = BMA5_INT1_MODE_LATCHED;
    pin.int_conf.int_od = BMA5_INT1_OD_OPEN_DRAIN;
    pin.int_conf.int_lvl = BMA5_INT1_LVL_ACTIVE_LOW;
    if (!result(bma5_set_int_conf(&pin, 1, &dev_))) return false;
    bma580_int_map mapping = {};
    if (!result(bma580_get_int_map(&mapping, &dev_))) return false;
    mapping.gen_int1_int_map = motionPin_ == 1 ? BMA580_GEN_INT1_INT_MAP_INT1 : BMA580_GEN_INT1_INT_MAP_INT2;
    if (!result(bma580_set_int_map(&mapping, &dev_))) return false;
    bma580_int_status_types clear = {};
    clear.int_src = motionPin_ == 1 ? BMA580_INT_STATUS_INT1 : BMA580_INT_STATUS_INT2;
    clear.int_status.gen_int1_int_status = 1;
    if (!result(bma580_set_int_status(&clear, 1, &dev_))) return false;
    if (!setMotionEnabled(true)) return false;
    motionEnabled_ = true;
    return true;
}

bool BMA580::readMotionInterrupt(bool& motion) {
    motion = false;
    if (!initialized_ || !motionEnabled_) return result(BMA5_E_DEV_NOT_FOUND);
    bma580_int_status_types status = {};
    status.int_src = motionPin_ == 1 ? BMA580_INT_STATUS_INT1 : BMA580_INT_STATUS_INT2;
    if (!result(bma580_get_int_status(&status, 1, &dev_))) return false;
    if (!status.int_status.gen_int1_int_status) return true;
    bma580_int_status_types clear = {};
    clear.int_src = motionPin_ == 1 ? BMA580_INT_STATUS_INT1 : BMA580_INT_STATUS_INT2;
    clear.int_status.gen_int1_int_status = 1; // W1C only our event
    if (!result(bma580_set_int_status(&clear, 1, &dev_))) return false;
    if (!setMotionEnabled(false)) return false;
    delay(40);
    if (!setMotionEnabled(true)) return false;
    motion = true;
    return true;
}
