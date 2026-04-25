#include "robot.h"
#include "config.h"
#include "pins.h"
#include "pico/time.h"
#include <algorithm>
#include <cstdio>
#include <cstring>

// ── Internal helpers (file-local) ─────────────────────────────────────────

static bool valid(int16_t d) { return d > 0 && d < TOF_MAX_MM; }

static const char* state_name(State s) {
    switch (s) {
        case State::WAIT_START: return "WAIT";
        case State::COUNTDOWN:  return "CNTDWN";
        case State::MANEUVER:   return "MANUV";
        case State::WAIT:       return "WAIT";
        case State::SEARCH:     return "SEARCH";
        case State::HOME:       return "HOME";
        case State::ATTACK:     return "ATTACK";
        case State::PUSH:       return "PUSH";
    }
    return "?";
}

static const char* strategy_name(Strategy s) {
    switch (s) {
        case Strategy::HARD_LEFT:    return "HARD LEFT";
        case Strategy::SLIGHT_LEFT:  return "SLIGHT LEFT";
        case Strategy::ROT_LEFT:     return "ROT LEFT";
        case Strategy::ROT_RIGHT:    return "ROT RIGHT";
        case Strategy::SLIGHT_RIGHT: return "SLIGHT RIGHT";
        case Strategy::HARD_RIGHT:   return "HARD RIGHT";
    }
    return "?";
}

// Closest TOF below SELECT_MM wins. Returns false (no update) when nothing is in range.
static bool pick_strategy(const int16_t d[6], Strategy& out) {
    int     best_i = -1;
    int16_t best_d = SELECT_MM;
    for (int i = 0; i < 6; i++) {
        if (valid(d[i]) && d[i] < best_d) {
            best_d = d[i];
            best_i = i;
        }
    }
    switch (best_i) {
        case 0: out = Strategy::HARD_LEFT;    return true;
        case 1: out = Strategy::SLIGHT_LEFT;  return true;
        case 2: out = Strategy::ROT_LEFT;     return true;
        case 3: out = Strategy::ROT_RIGHT;    return true;
        case 4: out = Strategy::SLIGHT_RIGHT; return true;
        case 5: out = Strategy::HARD_RIGHT;   return true;
        default: return false;
    }
}

struct Detection { bool attack; bool visible; };

static Detection detect(const int16_t d[6]) {
    bool attack = (valid(d[2]) && d[2] < ENEMY_FRONT_MM) && (valid(d[3]) && d[3] < ENEMY_FRONT_MM);
    bool visible = false;
    for (int i = 0; i < 6; i++) visible |= valid(d[i]) && d[i] < ENEMY_ANY_MM;
    return { attack, visible };
}

// Steering bias: negative = turn left, positive = turn right, range roughly [-1, 1].
// Uses the four side+diagonal sensors weighted by closeness.
static float steer(const int16_t d[6]) {
    auto c = [](int16_t v) -> float {
        if (!valid(v)) return 0.0f;
        return 1.0f - std::min((float)v / (float)ENEMY_ANY_MM, 1.0f);
    };
    float l   = c(d[0]) * STEER_CORNER_WEIGHT + c(d[1]) + c(d[2]) * STEER_FRONT_WEIGHT;
    float r   = c(d[5]) * STEER_CORNER_WEIGHT + c(d[4]) + c(d[3]) * STEER_FRONT_WEIGHT;
    if (l + r < STEER_DEADBAND) return 0.0f;
    // divide by max possible per-side so weights actually affect magnitude
    return std::clamp((r - l) / (STEER_CORNER_WEIGHT + 1.0f + STEER_FRONT_WEIGHT), -1.0f, 1.0f);
}

// ── Robot ─────────────────────────────────────────────────────────────────

void Robot::_brake_all() {
    _left.brake();
    _right.brake();
    _lpwm = _rpwm = 0.0f;
}

void Robot::_set_motors(float l, float r) {
    _left.set_pwm(l);
    _right.set_pwm(r);
    _lpwm = l;
    _rpwm = r;
}

// Row 0: state name
// Row 1: LC  LF  RF  RC   (front arc)
// Row 2: L  [Lmot Rmot]  R
void Robot::_draw_fight() {
    char motor_buf[8];
    int lm = std::clamp((int)(_lpwm * 100), -99, 99);
    int rm = std::clamp((int)(_rpwm * 100), -99, 99);
    snprintf(motor_buf, 8, "%+3d%+3d", lm, rm);
    _oled.clear();
    _oled.printf(0, 0, "%-21s", state_name(_state));
    _draw_tofs(motor_buf);
    _oled.display();
}

Robot::Robot(Motor& left, Motor& right, OLED& oled,
             ToFSensor tofs[6], const bool tof_ok[6], IMU& imu)
    : _left(left), _right(right), _oled(oled), _tofs(tofs), _imu(imu),
      _state(State::WAIT_START), _strategy(Strategy::ROT_RIGHT),
      _state_t(time_us_64())
{
    for (int i = 0; i < 6; i++)
        _tof_ok[i] = tof_ok[i];
}

bool Robot::_poll_tofs() {
    bool all = true;
    for (int i = 0; i < 6; i++) {
        if (!_tof_ok[i]) continue;
        if (!_tof_ready[i]) _tof_ready[i] = _tofs[i].data_ready();
        if (!_tof_ready[i]) all = false;
    }
    if (!all) return false;

    for (int i = 0; i < 6; i++)
        _d[i] = _tof_ok[i] ? _tofs[i].read_mm() : -1;
    for (int i = 0; i < 6; i++)
        _tof_ready[i] = false;
    return true;
}

// Draw spatial TOF percentage layout on rows 1–2:
//   Row 1:  LC   LF   RF   RC    (front arc)
//   Row 2:  L    <centre>    R   (sides)
// s[0]=L  s[1]=LC  s[2]=LF  s[3]=RF  s[4]=RC  s[5]=R
void Robot::_draw_tofs(const char* centre) {
    char s[6][4];
    for (int i = 0; i < 6; i++) {
        if (!valid(_d[i])) { s[i][0]='-'; s[i][1]='-'; s[i][2]='-'; s[i][3]='\0'; }
        else {
            int p = (int)_d[i] * 100 / TOF_MAX_MM;
            if (p > 999) p = 999;
            snprintf(s[i], 4, "%3d", p);
        }
    }
    _oled.printf(0, 1, "%s   %s   %s   %s", s[1], s[2], s[3], s[4]);
    _oled.printf(0, 2, "%s    %-7s    %s",   s[0], centre, s[5]);
}

void Robot::update() {
    bool     new_tof     = _poll_tofs();
    uint64_t now         = time_us_64();
    uint32_t in_state_ms = (uint32_t)((now - _state_t) / 1000);

    // ── Stop signal: delegated to subclass ────────────────────────────────
    if (_state != State::WAIT_START && _state != State::COUNTDOWN)
        _check_stop();

    switch (_state) {

    // ──────────────────────────────────────────────────────────────────────
    case State::WAIT_START:
        if (new_tof) {
            pick_strategy(_d, _strategy);
            _oled.clear();
            _oled.printf(0, 0, "%-21s", strategy_name(_strategy));
            _draw_tofs();
            _oled.display();
            printf("%-12s | LC:%3d%% LF:%3d%% RF:%3d%% RC:%3d%% | L:%3d%% R:%3d%%\n",
                strategy_name(_strategy),
                valid(_d[1]) ? _d[1]*100/TOF_MAX_MM : -1,
                valid(_d[2]) ? _d[2]*100/TOF_MAX_MM : -1,
                valid(_d[3]) ? _d[3]*100/TOF_MAX_MM : -1,
                valid(_d[4]) ? _d[4]*100/TOF_MAX_MM : -1,
                valid(_d[0]) ? _d[0]*100/TOF_MAX_MM : -1,
                valid(_d[5]) ? _d[5]*100/TOF_MAX_MM : -1);
        }

        _check_start();
        break;

    // ──────────────────────────────────────────────────────────────────────
    case State::COUNTDOWN: {
        uint32_t remaining = COUNTDOWN_MS / 1000 - (in_state_ms / 1000);
        _oled.clear();
        _oled.printf(0, 0, "%-12s", strategy_name(_strategy));
        _oled.printf(0, 1, "START in %u", remaining);
        _oled.display();
        if (in_state_ms >= COUNTDOWN_MS) {
            _state   = State::MANEUVER;
            _state_t = now;
        }
        break;
    }

    // ──────────────────────────────────────────────────────────────────────
    case State::MANEUVER:
        switch (_strategy) {
        case Strategy::HARD_LEFT:
            _set_motors(-START_HARD_TURN, +START_HARD_TURN);
            break;
        case Strategy::SLIGHT_LEFT:
            _set_motors(START_FWD_SPEED - START_SLIGHT_BIAS, START_FWD_SPEED);
            break;
        case Strategy::ROT_LEFT:
            _set_motors(-START_HARD_TURN, +START_HARD_TURN);
            break;
        case Strategy::ROT_RIGHT:
            _set_motors(+START_HARD_TURN, -START_HARD_TURN);
            break;
        case Strategy::SLIGHT_RIGHT:
            _set_motors(START_FWD_SPEED, START_FWD_SPEED - START_SLIGHT_BIAS);
            break;
        case Strategy::HARD_RIGHT:
            _set_motors(+START_HARD_TURN, -START_HARD_TURN);
            break;
        }

        if (new_tof) _draw_fight();

        if (in_state_ms >= START_MANEUVER_MS) {
            bool rot = (_strategy == Strategy::ROT_LEFT || _strategy == Strategy::ROT_RIGHT);
            if (new_tof) {
                Detection det = detect(_d);
                float     s   = steer(_d);
                if (det.attack)        _state = State::ATTACK;
                else if (det.visible) { if (s != 0.0f) _search_dir = s > 0.0f ? 1 : -1; _state = State::HOME; }
                else if (rot)         { _state = State::WAIT; _wait_enc_start = _left.encoder(); _wait_dir = 1; }
                else                  { _state = State::SEARCH; }
            } else {
                if (rot) { _state = State::WAIT; _wait_enc_start = _left.encoder(); _wait_dir = 1; }
                else     { _state = State::SEARCH; }
            }
            _state_t = now;
        }
        break;

    // ──────────────────────────────────────────────────────────────────────
    case State::WAIT: {
        int32_t delta = std::abs(_left.encoder() - _wait_enc_start);
        if (delta >= static_cast<int32_t>(WAIT_SCAN_REVS * Motor::CPR)) {
            _wait_dir       = -_wait_dir;
            _wait_enc_start = _left.encoder();
        }
        _set_motors(WAIT_SCAN_SPEED * _wait_dir, -WAIT_SCAN_SPEED * _wait_dir);

        if (new_tof) {
            Detection det = detect(_d);
            float     s   = steer(_d);
            if (det.attack)        { _state = State::ATTACK; _state_t = now; }
            else if (det.visible)  { if (s != 0.0f) _search_dir = s > 0.0f ? 1 : -1; _state = State::HOME; _state_t = now; }
            _draw_fight();
        }
        break;
    }

    // ──────────────────────────────────────────────────────────────────────
    case State::SEARCH:
        _set_motors(SEARCH_TURN_SPEED * _search_dir, -SEARCH_TURN_SPEED * _search_dir);

        if (new_tof) {
            Detection det = detect(_d);
            float     s   = steer(_d);
            if (det.attack)        { _state = State::ATTACK; _state_t = now; }
            else if (det.visible)  { if (s != 0.0f) _search_dir = s > 0.0f ? 1 : -1; _state = State::HOME; _state_t = now; }
            _draw_fight();
        }
        break;

    // ──────────────────────────────────────────────────────────────────────
    case State::HOME:
        if (new_tof) {
            Detection det = detect(_d);
            if (det.attack)   { _state = State::ATTACK; _state_t = now; break; }
            if (!det.visible) { _state = State::SEARCH; _state_t = now; break; }
            float s   = steer(_d);
            float fwd = HOME_FWD * (1.0f - std::abs(s));
            if (s != 0.0f) _search_dir = s > 0.0f ? 1 : -1;
            _set_motors(std::clamp(fwd + HOME_TURN * s, -1.0f, 1.0f),
                        std::clamp(fwd - HOME_TURN * s, -1.0f, 1.0f));
            _draw_fight();
        }
        break;

    // ──────────────────────────────────────────────────────────────────────
    case State::ATTACK:
        _set_motors(ATTACK_SPEED, ATTACK_SPEED);

        if (new_tof) {
            bool close = (valid(_d[2]) && _d[2] < PUSH_DIST_MM) ||
                         (valid(_d[3]) && _d[3] < PUSH_DIST_MM);
            if (close) {
                _state   = State::PUSH;
                _state_t = now;
            } else {
                Detection det = detect(_d);
                float     s   = steer(_d);
                if (!det.attack && !det.visible)  { _state = State::SEARCH; _state_t = now; }
                else if (!det.attack)             { if (s != 0.0f) _search_dir = s > 0.0f ? 1 : -1; _state = State::HOME; _state_t = now; }
            }
            _draw_fight();
        }
        break;

    // ──────────────────────────────────────────────────────────────────────
    case State::PUSH:
        _set_motors(PUSH_SPEED, PUSH_SPEED);

        if (new_tof) _draw_fight();
        break;
    }
}

// ── PinStartRobot ─────────────────────────────────────────────────────────

void PinStartRobot::_check_start() {
    if (gpio_get(PIN_START)) {
        _state = State::MANEUVER; _state_t = time_us_64();
    }
}

void PinStartRobot::_check_stop() {
    if (!gpio_get(PIN_START)) {
        _brake_all();
        _state = State::WAIT_START; _state_t = time_us_64();
    }
}

// ── ButtonRobot ───────────────────────────────────────────────────────────

ButtonRobot::ButtonRobot(Motor& left, Motor& right, OLED& oled,
                         ToFSensor tofs[6], const bool tof_ok[6], IMU& imu)
    : Robot(left, right, oled, tofs, tof_ok, imu), _btn(PIN_BTN_0)
{}

void ButtonRobot::_check_start() {
    _btn.update();
    if (_btn.pressed()) {
        _state = State::COUNTDOWN; _state_t = time_us_64();
    }
}

void ButtonRobot::_check_stop() {
    _btn.update();
    if (_btn.pressed()) {
        _brake_all();
        _state = State::WAIT_START; _state_t = time_us_64();
    }
}
