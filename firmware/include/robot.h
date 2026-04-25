#pragma once
#include "imu.h"
#include "motor.h"
#include "oled.h"
#include "tof.h"
#include <cstdint>

enum class Strategy { HARD_LEFT, SLIGHT_LEFT, ROT_LEFT, ROT_RIGHT, SLIGHT_RIGHT, HARD_RIGHT };
enum class State    { WAIT_START, COUNTDOWN, MANEUVER, SEARCH, HOME, ATTACK, PUSH };

// Combat state machine.
// Construct once after all peripherals are ready, then call update() every loop iteration.
class Robot {
public:
    Robot(Motor& left, Motor& right, OLED& oled,
          ToFSensor tofs[6], const bool tof_ok[6], IMU& imu);

    void update();

private:
    Motor&     _left;
    Motor&     _right;
    OLED&      _oled;
    ToFSensor* _tofs;
    IMU&       _imu;

    bool     _tof_ok[6]    = {};
    bool     _tof_ready[6] = {};
    int16_t  _d[6]         = {};
    State    _state;
    Strategy _strategy;
    uint64_t _state_t;

    bool _poll_tofs();
    void _draw_tofs(const char* centre = "");
};
