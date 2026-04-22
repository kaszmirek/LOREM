#pragma once
#include "hardware/i2c.h"

struct Vec3 { float x, y, z; };

// LIS3DH accelerometer over I2C.
// SDO/SA0 pin sets address: low → 0x18, high → 0x19.
class IMU {
public:
    static constexpr uint8_t ADDR_LO = 0x18;
    static constexpr uint8_t ADDR_HI = 0x19;

    IMU(i2c_inst_t* i2c, uint8_t address = ADDR_LO)
        : _i2c(i2c), _addr(address) {}

    // Configure ±2 g, 400 Hz ODR, enable tap detection on INT1.
    // Returns true if WHO_AM_I matches.
    bool init();

    // Acceleration in g (±2 g range).
    Vec3 accel_g();

    // True if a single-tap was detected (clears flag on read).
    bool tap_detected();

private:
    i2c_inst_t* _i2c;
    uint8_t     _addr;

    bool    _write_reg(uint8_t reg, uint8_t val);
    uint8_t _read_reg(uint8_t reg);
};
