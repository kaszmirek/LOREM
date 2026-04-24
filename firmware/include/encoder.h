#pragma once
#include "hardware/gpio.h"
#include "pico/time.h"

// Quadrature encoder, X4 decoding via a 5 kHz repeating timer.
// Both pins are sampled each tick; a 4-state transition table determines direction.
// Up to 2 encoders share one repeating timer. Read _count directly — no IRQ races.
class Encoder {
public:
    static constexpr uint    MAX       = 2;
    static constexpr int32_t SAMPLE_US = 200;   // 5 kHz

    Encoder(uint pin_a, uint pin_b) : _pin_a(pin_a), _pin_b(pin_b), _count(0) {
        gpio_init(_pin_a); gpio_set_dir(_pin_a, GPIO_IN); gpio_pull_up(_pin_a);
        gpio_init(_pin_b); gpio_set_dir(_pin_b, GPIO_IN); gpio_pull_up(_pin_b);

        _prev = (gpio_get(_pin_a) << 1) | gpio_get(_pin_b);
        _reg(this);

        if (_n == 1)
            add_repeating_timer_us(-SAMPLE_US, _cb, nullptr, &_timer);
    }

    int32_t read()  const { return _count; }
    void    reset()       { _count = 0; }

private:
    uint              _pin_a, _pin_b;
    volatile int32_t  _count;
    uint8_t           _prev;

    inline static Encoder*          _inst[MAX] = {};
    inline static uint              _n         = 0;
    inline static repeating_timer_t _timer     = {};

    // Full X4 Gray-code transition table indexed by (prev<<2)|cur
    static constexpr int8_t _tbl[16] = {
         0, -1, +1,  0,
        +1,  0,  0, -1,
        -1,  0,  0, +1,
         0, +1, -1,  0,
    };

    static void _reg(Encoder* e) { if (_n < MAX) _inst[_n++] = e; }

    static bool _cb(repeating_timer_t*) {
        for (uint i = 0; i < _n; i++) {
            Encoder* e   = _inst[i];
            uint8_t  cur = (gpio_get(e->_pin_a) << 1) | gpio_get(e->_pin_b);
            e->_count   += _tbl[(e->_prev << 2) | cur];
            e->_prev     = cur;
        }
        return true;
    }
};
