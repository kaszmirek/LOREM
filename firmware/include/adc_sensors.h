#pragma once
#include "hardware/adc.h"

// RP2040 ADC: 12-bit, Vref = 3.3 V, GPIO26-28 = ADC0-2.
namespace detail {
    inline float adc_read_voltage(uint channel) {
        adc_select_input(channel);
        return adc_read() * (3.3f / 4096.0f);
    }
}

// Battery voltage via resistor divider. Multiplier from pins.h: 0.1803 V/V.
class Battery {
public:
    explicit Battery(uint adc_gpio) : _chan(adc_gpio - 26u) {
        adc_init();
        adc_gpio_init(adc_gpio);
    }

    float voltage() const {
        return detail::adc_read_voltage(_chan) / 0.1803f;
    }

private:
    uint _chan;
};

// Motor current sense. Multiplier from pins.h: 5 V/A.
class CurrentSensor {
public:
    explicit CurrentSensor(uint adc_gpio) : _chan(adc_gpio - 26u) {
        adc_init();
        adc_gpio_init(adc_gpio);
    }

    float current_a() const {
        return detail::adc_read_voltage(_chan) / 5.0f;
    }

private:
    uint _chan;
};
