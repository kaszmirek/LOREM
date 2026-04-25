#pragma once
#include "VL53L0X.h"
#include "hardware/gpio.h"

// VL53L0X time-of-flight distance sensor over I2C.
// Call init() once after power-up; use a different address per sensor instance
// after sequentially enabling each via its XSHUT pin.
class ToFSensor {
public:
    static constexpr uint8_t DEFAULT_ADDR = VL53L0X_DEFAULT_ADDRESS;

    ToFSensor(i2c_inst_t* i2c, uint pin_xshut, uint8_t address = DEFAULT_ADDR)
        : _sensor(i2c), _xshut(pin_xshut), _addr(address)
    {
        gpio_init(_xshut);
        gpio_set_dir(_xshut, GPIO_OUT);
        gpio_put(_xshut, 0);  // hold in reset
    }

    void enable()  { gpio_put(_xshut, 1); }
    void disable() { gpio_put(_xshut, 0); }

    // Configure device and optionally reassign I2C address.
    // Returns true on success.
    bool init();

    // True when a new measurement result is waiting to be read.
    bool data_ready();

    // Returns distance in mm, or -1 on timeout / error.
    int16_t read_mm();

private:
    VL53L0X _sensor;
    uint    _xshut;
    uint8_t _addr;
    int16_t _last_good = -1;
    bool    _prev_rejected = false;
};
