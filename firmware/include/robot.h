#pragma once
#include "button.h"
#include "imu.h"
#include "motor.h"
#include "oled.h"
#include "tof.h"
#include <cstdint>

enum class Strategy { HARD_LEFT, SLIGHT_LEFT, ROT_LEFT, ROT_RIGHT, SLIGHT_RIGHT, HARD_RIGHT };
enum class State    { WAIT_START, COUNTDOWN, MANEUVER, WAIT, SEARCH, HOME, ATTACK, PUSH };

// Combat state machine.
// Construct once after all peripherals are ready, then call update() every loop iteration.
class Robot {
public:
    Robot(Motor& left, Motor& right, OLED& oled,
          ToFSensor tofs[6], const bool tof_ok[6], IMU& imu);

    void update();

    State get_state() const { return _state; }

protected:
    virtual void _check_start() {}  // called in WAIT_START; subclass transitions to COUNTDOWN or MANEUVER
    virtual void _check_stop()  {}  // called in active states; subclass brakes + returns to WAIT_START
    void _brake_all();

    Motor&     _left;
    Motor&     _right;
    State      _state;
    uint64_t   _state_t;

private:
    OLED&      _oled;
    ToFSensor* _tofs;
    IMU&       _imu;

    bool     _tof_ok[6]    = {};
    bool     _tof_ready[6] = {};
    int16_t  _d[6]         = {};
    Strategy _strategy;
    int32_t  _wait_enc_start = 0;
    int8_t   _wait_dir       = 1;
    int8_t   _search_dir     = 1;
    float    _lpwm = 0.0f, _rpwm = 0.0f;

    bool _poll_tofs();
    void _draw_tofs(const char* centre = "");
    void _set_motors(float l, float r);
    void _draw_fight();
};

// Start: PIN_START high → MANEUVER. Stop: PIN_START low → WAIT_START.
class PinStartRobot : public Robot {
public:
    using Robot::Robot;
protected:
    void _check_start() override;
    void _check_stop()  override;
};

// Start: BTN_0 press → COUNTDOWN → MANEUVER. Stop: BTN_0 press → WAIT_START.
// PIN_START is ignored entirely.
class ButtonRobot : public Robot {
public:
    ButtonRobot(Motor& left, Motor& right, OLED& oled,
                ToFSensor tofs[6], const bool tof_ok[6], IMU& imu);
protected:
    void _check_start() override;
    void _check_stop()  override;
private:
    Button _btn;
};
