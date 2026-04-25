#pragma once
#include "hardware/gpio.h"
#include "pico/time.h"

// Quadrature encoder, X4 decoding via a 5 kHz repeating timer.
// Both pins are sampled each tick; a 4-state transition table determines direction.
// Up to 2 encoders share one repeating timer.
class Encoder {
public:
    static constexpr uint    MAX       = 2;
    static constexpr int32_t SAMPLE_US = 200;   // 5 kHz

    Encoder(uint pin_a, uint pin_b);

    int32_t read()  const { return _count; }
    void    reset()       { _count = 0; }

private:
    uint              _pin_a, _pin_b;
    volatile int32_t  _count;
    uint8_t           _prev;

    static Encoder*          _inst[MAX];
    static uint              _n;
    static repeating_timer_t _timer;

    static void _reg(Encoder* e);
    static bool _cb(repeating_timer_t*);
};
