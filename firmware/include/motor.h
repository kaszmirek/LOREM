#pragma once
#include "adc_sensors.h"
#include "encoder.h"
#include "hardware/pwm.h"
#include "pid.h"
#include "pico/time.h"
#include <algorithm>

// Motor with integrated encoder, current sense, and PID.
// Three control modes — calling any setter switches to that mode:
//   set_pwm(speed)           raw drive [-1, 1], no feedback
//   set_vel_target(mm_s)     velocity PID via encoder
//   set_curr_target(amps)    current PID via sense resistor
// brake() / coast() stop the motor and clear PID state.
// Call update() every loop iteration for PID modes.
class Motor {
public:
    static constexpr uint16_t WRAP            = 4999;            // 125 MHz / 5000 = 25 kHz PWM
    static constexpr float    WHEEL_RADIUS_MM = 18.0f;           // 36 mm diameter
    static constexpr float    CPR             = 19.0f * 1024.0f; // 19:1 gearbox * 1024 enc

    Motor(uint pin_in1, uint pin_in2,
          uint pin_enc_a, uint pin_enc_b,
          uint pin_isense,
          float vel_kp, float vel_ki, float vel_kd,
          float curr_kp = 0.5f, float curr_ki = 0.1f, float curr_kd = 0.0f);

    void set_pwm(float speed);
    void set_vel_target(float mm_s);
    void set_curr_target(float amps);
    void update();
    void brake();
    void coast();

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

    void _pwm(uint16_t a, uint16_t b);
    void _drive(float speed);
};
