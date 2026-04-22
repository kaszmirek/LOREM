#pragma once
#include "hardware/pwm.h"
#include <cmath>

// Single motor driven by two PWM pins (DRV8848 half-bridge).
// set()  -1.0 = full reverse, 0.0 = coast, +1.0 = full forward
// brake() short-circuits winding (fast stop)
class Motor {
public:
    static constexpr uint16_t WRAP = 4999;  // 125 MHz / 5000 = 25 kHz PWM

    Motor(uint pin_in1, uint pin_in2) : _in1(pin_in1), _in2(pin_in2) {
        gpio_set_function(_in1, GPIO_FUNC_PWM);
        gpio_set_function(_in2, GPIO_FUNC_PWM);

        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_wrap(&cfg, WRAP);

        uint s1 = pwm_gpio_to_slice_num(_in1);
        uint s2 = pwm_gpio_to_slice_num(_in2);
        pwm_init(s1, &cfg, true);
        if (s1 != s2) pwm_init(s2, &cfg, true);

        coast();
    }

    void set(float speed) {
        if (speed > 1.0f)  speed =  1.0f;
        if (speed < -1.0f) speed = -1.0f;
        uint16_t duty = (uint16_t)(fabsf(speed) * WRAP);
        if (speed > 0.0f) {
            pwm_set_gpio_level(_in1, duty);
            pwm_set_gpio_level(_in2, 0);
        } else if (speed < 0.0f) {
            pwm_set_gpio_level(_in1, 0);
            pwm_set_gpio_level(_in2, duty);
        } else {
            coast();
        }
    }

    void brake() {
        pwm_set_gpio_level(_in1, WRAP);
        pwm_set_gpio_level(_in2, WRAP);
    }

    void coast() {
        pwm_set_gpio_level(_in1, 0);
        pwm_set_gpio_level(_in2, 0);
    }

private:
    uint _in1, _in2;
};
