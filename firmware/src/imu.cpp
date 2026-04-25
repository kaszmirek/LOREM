#include "imu.h"

// LIS3DH register addresses
static constexpr uint8_t REG_WHO_AM_I  = 0x0F;
static constexpr uint8_t REG_CTRL1     = 0x20;
static constexpr uint8_t REG_CTRL3     = 0x22;
static constexpr uint8_t REG_CTRL4     = 0x23;
static constexpr uint8_t REG_OUT_X_L   = 0x28;
static constexpr uint8_t REG_CLICK_CFG = 0x38;
static constexpr uint8_t REG_CLICK_SRC = 0x39;
static constexpr uint8_t REG_CLICK_THS = 0x3A;
static constexpr uint8_t REG_TIME_LIM  = 0x3B;
static constexpr uint8_t REG_TIME_LAT  = 0x3C;
static constexpr uint8_t REG_TIME_WIN  = 0x3D;

static constexpr uint8_t WHO_AM_I_VAL = 0x33;

bool IMU::_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_write_blocking(_i2c, _addr, buf, 2, false) == 2;
}

uint8_t IMU::_read_reg(uint8_t reg) {
    i2c_write_blocking(_i2c, _addr, &reg, 1, true);
    uint8_t val = 0;
    i2c_read_blocking(_i2c, _addr, &val, 1, false);
    return val;
}

bool IMU::init() {
    if (_read_reg(REG_WHO_AM_I) != WHO_AM_I_VAL) return false;

    // 400 Hz ODR, all axes on
    _write_reg(REG_CTRL1, 0x77);
    // BDU=1, ±2 g, high-res
    _write_reg(REG_CTRL4, 0x88);
    // INT1 = click interrupt
    _write_reg(REG_CTRL3, 0x80);

    // Single-tap on X, Y, Z
    _write_reg(REG_CLICK_CFG, 0x15);
    // Threshold: ~0.75 g (48 * 2g/128)
    _write_reg(REG_CLICK_THS, 0x30);
    // Time limit: 8 * 2.5 ms = 20 ms window
    _write_reg(REG_TIME_LIM, 0x08);
    // No latency/window (single tap only)
    _write_reg(REG_TIME_LAT, 0x00);
    _write_reg(REG_TIME_WIN, 0x00);

    return true;
}

Vec3 IMU::accel_g() {
    // Auto-increment read: set bit 7 of register address
    uint8_t reg = REG_OUT_X_L | 0x80;
    uint8_t raw[6] = {};
    i2c_write_blocking(_i2c, _addr, &reg, 1, true);
    i2c_read_blocking(_i2c, _addr, raw, 6, false);

    // 16-bit left-justified, HR = 12-bit → >>4; 1 LSB = 1 mg
    auto to_g = [](uint8_t lo, uint8_t hi) -> float {
        int16_t raw = (int16_t)((uint16_t)hi << 8 | lo);
        return (raw >> 4) * 0.001f;
    };
    return {to_g(raw[0], raw[1]), to_g(raw[2], raw[3]), to_g(raw[4], raw[5])};
}

bool IMU::tap_detected() {
    // Reading CLICK_SRC clears the latch; bit 6 = IA (interrupt active)
    return (_read_reg(REG_CLICK_SRC) & 0x40) != 0;
}
