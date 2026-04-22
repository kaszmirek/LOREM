#pragma once
#include "hardware/gpio.h"

// Quadrature encoder, X2 decoding (counts on both edges of channel A).
// Up to 2 encoders can be registered; they share the single GPIO IRQ callback.
class Encoder {
public:
    static constexpr uint MAX = 2;

    Encoder(uint pin_a, uint pin_b) : _pin_a(pin_a), _pin_b(pin_b), _count(0) {
        gpio_init(_pin_a); gpio_set_dir(_pin_a, GPIO_IN); gpio_pull_up(_pin_a);
        gpio_init(_pin_b); gpio_set_dir(_pin_b, GPIO_IN); gpio_pull_up(_pin_b);

        _reg(this);

        // First encoder sets the shared callback; subsequent ones just enable IRQ.
        uint32_t edges = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
        if (_n == 1) {
            gpio_set_irq_enabled_with_callback(_pin_a, edges, true, &_isr);
        } else {
            gpio_set_irq_enabled(_pin_a, edges, true);
        }
        gpio_set_irq_enabled(_pin_b, edges, true);
    }

    int32_t read()  const { return _count; }
    void    reset()       { _count = 0; }

private:
    uint              _pin_a, _pin_b;
    volatile int32_t  _count;

    inline static Encoder* _inst[MAX] = {};
    inline static uint      _n        = 0;

    static void _reg(Encoder* e) { if (_n < MAX) _inst[_n++] = e; }

    static void _isr(uint gpio, uint32_t /*events*/) {
        for (uint i = 0; i < _n; i++) {
            Encoder* e = _inst[i];
            if (gpio != e->_pin_a) continue;
            bool a = gpio_get(e->_pin_a);
            bool b = gpio_get(e->_pin_b);
            // X2: rising A + B low → +1; rising A + B high → -1; falling inverts
            e->_count += (a == b) ? 1 : -1;
            return;
        }
    }
};
