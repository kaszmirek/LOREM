#pragma once
#include "adc_sensors.h"
#include "encoder.h"
#include "hardware/pwm.h"
#include "pid.h"
#include "pico/time.h"
#include <algorithm>
#include <cmath>

// Motor with integrated encoder, current sense, and PID.
// Three control modes — calling any setter switches to that mode:
//   set_pwm(speed)           raw drive [-1, 1], no feedback
//   set_vel_target(mm_s)     velocity PID via encoder
//   set_curr_target(amps)    current PID via sense resistor
// brake() / coast() stop the motor and clear PID state.
// Call update() every loop iteration for PID modes.
class Motor {
public:
    static constexpr uint16_t WRAP            = 4999;           // 125 MHz / 5000 = 25 kHz PWM
    static constexpr float    WHEEL_RADIUS_MM = 18.0f;          // 36 mm diameter
    static constexpr float    CPR             = 19.0f * 1024.0f; // 19:1 gearbox * 1024 enc

    Motor(uint pin_in1, uint pin_in2,
          uint pin_enc_a, uint pin_enc_b,
          uint pin_isense,
          float vel_kp, float vel_ki, float vel_kd,
          float curr_kp = 0.5f, float curr_ki = 0.1f, float curr_kd = 0.0f)
        : _in1(pin_in1), _in2(pin_in2),
          _enc(pin_enc_a, pin_enc_b),
          _isense(pin_isense),
          _vel_pid(vel_kp, vel_ki, vel_kd),
          _curr_pid(curr_kp, curr_ki, curr_kd),
          _prev_count(_enc.read()),
          _last_t(time_us_64())
    {
        gpio_set_function(_in1, GPIO_FUNC_PWM);
        gpio_set_function(_in2, GPIO_FUNC_PWM);

        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_wrap(&cfg, WRAP);

        uint s1 = pwm_gpio_to_slice_num(_in1);
        uint s2 = pwm_gpio_to_slice_num(_in2);
        pwm_init(s1, &cfg, true);
        if (s1 != s2) pwm_init(s2, &cfg, true);

        _pwm(0, 0);
    }

    // Raw PWM drive [-1, 1]. No feedback; switches out of PID modes.
    void set_pwm(float speed) {
        _mode = Mode::PWM;
        _vel_pid.reset();
        _curr_pid.reset();
        _drive(speed);
    }

    // Velocity setpoint in mm/s. Engages encoder PID.
    void set_vel_target(float mm_s) {
        _vel_target  = mm_s * CPR / (2.0f * 3.14159265f * WHEEL_RADIUS_MM);
        _mode        = Mode::VELOCITY;
    }

    // Current setpoint in amps. Engages current sense PID.
    void set_curr_target(float amps) {
        _curr_target = amps;
        _mode        = Mode::CURRENT;
    }

    // Run one measurement + PID iteration. Rate-limited to 100 Hz; call every loop.
    void update() {
        uint64_t now = time_us_64();
        float    dt  = (now - _last_t) * 1e-6f;
        if (dt < 0.010f) return;
        _last_t = now;

        int32_t count = _enc.read();
        _velocity   = (count - _prev_count) / dt;
        _prev_count = count;
        _current    = _isense.current_a();

        if (_mode == Mode::COAST || _mode == Mode::PWM) return;

        float out = 0.0f;
        if (_mode == Mode::VELOCITY)
            out = std::clamp(_vel_pid.update(_vel_target - _velocity, dt), -1.0f, 1.0f);
        else if (_mode == Mode::CURRENT)
            out = std::clamp(_curr_pid.update(_curr_target - _current, dt),  0.0f, 1.0f);

        _drive(out);
    }

    void brake() {
        _mode = Mode::COAST;
        _vel_pid.reset();
        _curr_pid.reset();
        _pwm(WRAP, WRAP);
    }

    void coast() {
        _mode = Mode::COAST;
        _vel_pid.reset();
        _curr_pid.reset();
        _pwm(0, 0);
    }

    float   velocity()      const { return _velocity; }
    float   velocity_mm_s() const { return _velocity / CPR * (2.0f * 3.14159265f * WHEEL_RADIUS_MM); }
    float   current_a()     const { return _current; }
    int32_t encoder()       const { return _enc.read(); }
    void    reset_encoder()       { _enc.reset(); }

private:
    enum class Mode { COAST, PWM, VELOCITY, CURRENT };

    uint          _in1, _in2;
    Encoder       _enc;
    CurrentSensor _isense;
    PID           _vel_pid;
    PID           _curr_pid;
    Mode          _mode        = Mode::COAST;
    float         _vel_target  = 0.0f;  // counts/s
    float         _curr_target = 0.0f;  // amps
    uint64_t      _last_t;
    int32_t       _prev_count;
    float         _velocity    = 0.0f;
    float         _current     = 0.0f;

    void _pwm(uint16_t a, uint16_t b) {
        pwm_set_gpio_level(_in1, a);
        pwm_set_gpio_level(_in2, b);
    }

    void _drive(float speed) {
        uint16_t duty = (uint16_t)(fabsf(speed) * WRAP);
        if      (speed > 0.0f) _pwm(duty, 0);
        else if (speed < 0.0f) _pwm(0, duty);
        else                   _pwm(0, 0);
    }
};
