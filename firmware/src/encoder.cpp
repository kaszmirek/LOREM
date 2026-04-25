#include "encoder.h"

// Full X4 Gray-code transition table indexed by (prev << 2) | cur
static constexpr int8_t tbl[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0,
};

Encoder*          Encoder::_inst[Encoder::MAX] = {};
uint              Encoder::_n                  = 0;
repeating_timer_t Encoder::_timer              = {};

Encoder::Encoder(uint pin_a, uint pin_b) : _pin_a(pin_a), _pin_b(pin_b), _count(0) {
    gpio_init(_pin_a); gpio_set_dir(_pin_a, GPIO_IN); gpio_pull_up(_pin_a);
    gpio_init(_pin_b); gpio_set_dir(_pin_b, GPIO_IN); gpio_pull_up(_pin_b);

    _prev = (gpio_get(_pin_a) << 1) | gpio_get(_pin_b);
    _reg(this);

    if (_n == 1)
        add_repeating_timer_us(-SAMPLE_US, _cb, nullptr, &_timer);
}

void Encoder::_reg(Encoder* e) {
    if (_n < MAX) _inst[_n++] = e;
}

bool Encoder::_cb(repeating_timer_t*) {
    for (uint i = 0; i < _n; i++) {
        Encoder* e   = _inst[i];
        uint8_t  cur = (gpio_get(e->_pin_a) << 1) | gpio_get(e->_pin_b);
        e->_count   += tbl[(e->_prev << 2) | cur];
        e->_prev     = cur;
    }
    return true;
}
