#pragma once
#include "hardware/gpio.h"

// Reflective IR line sensor — active low (low = white line detected).
class LineSensor {
public:
    explicit LineSensor(uint pin) : _pin(pin) {
        gpio_init(_pin);
        gpio_set_dir(_pin, GPIO_IN);
        gpio_pull_up(_pin);
    }

    bool on_line() const { return !gpio_get(_pin); }

private:
    uint _pin;
};
