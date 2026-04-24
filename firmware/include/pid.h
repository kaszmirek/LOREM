#pragma once
#include <algorithm>

// Simple PID controller.
// update() returns the control output given the current error and time step.
// Integral is clamped to [-i_limit, i_limit] to prevent windup.
// Derivative uses a first-order backward difference; skipped on the first call.
struct PID {
    float kp, ki, kd;
    float i_limit = 1.0f;

    PID(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

    float update(float err, float dt) {
        _integral += err * dt;
        _integral = std::clamp(_integral, -i_limit, i_limit);
        float deriv = _first ? 0.0f : (err - _prev_err) / dt;
        _prev_err = err;
        _first    = false;
        return kp * err + ki * _integral + kd * deriv;
    }

    void reset() { _integral = 0.0f; _prev_err = 0.0f; _first = true; }

private:
    float _integral = 0.0f;
    float _prev_err = 0.0f;
    bool  _first    = true;
};
