#pragma once
#include "pico/stdlib.h"

// Debounced button input. Active-low (external pull-up).
// Call update() each loop. Query pressed()/released()/is_down().
class Button {
public:
    static constexpr uint32_t DEBOUNCE_MS = 20;

    explicit Button(uint pin) : _pin(pin) {
        gpio_init(_pin);
        gpio_set_dir(_pin, GPIO_IN);
    }

    // Must be called every loop iteration.
    void update() {
        bool raw = !gpio_get(_pin);  // active-low → true when pressed
        uint64_t now = time_us_64();

        if (raw != _raw_prev) {
            _last_change = now;
            _raw_prev    = raw;
        }

        if ((now - _last_change) >= DEBOUNCE_MS * 1000ULL) {
            bool prev   = _state;
            _state      = raw;
            _pressed    = !prev &&  _state;
            _released   =  prev && !_state;
        } else {
            _pressed  = false;
            _released = false;
        }
    }

    bool pressed()  const { return _pressed;  }  // true for one update() after press
    bool released() const { return _released; }  // true for one update() after release
    bool is_down()  const { return _state;    }  // current debounced state

private:
    uint     _pin;
    bool     _raw_prev   = false;
    bool     _state      = false;
    bool     _pressed    = false;
    bool     _released   = false;
    uint64_t _last_change = 0;
};
