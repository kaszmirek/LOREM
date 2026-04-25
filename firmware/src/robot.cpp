#include "robot.h"
#include "config.h"
#include "pins.h"
#include "pico/time.h"
#include <algorithm>
#include <cstdio>
#include <cstring>

// ── Internal helpers (file-local) ─────────────────────────────────────────

static bool valid(int16_t d) { return d > 0; }

static const char* strategy_name(Strategy s) {
    switch (s) {
        case Strategy::HARD_LEFT:    return "HARD LEFT";
        case Strategy::SLIGHT_LEFT:  return "SLIGHT LEFT";
        case Strategy::COAST:        return "COAST";
        case Strategy::SLIGHT_RIGHT: return "SLIGHT RIGHT";
        case Strategy::HARD_RIGHT:   return "HARD RIGHT";
    }
    return "?";
}

// Closest TOF below SELECT_MM wins; default COAST.
static Strategy pick_strategy(const int16_t d[6]) {
    int     best_i = -1;
    int16_t best_d = SELECT_MM;
    for (int i = 0; i < 6; i++) {
        if (valid(d[i]) && d[i] < best_d) {
            best_d = d[i];
            best_i = i;
        }
    }
    switch (best_i) {
        case 0: return Strategy::HARD_LEFT;
        case 1: return Strategy::SLIGHT_LEFT;
        case 2: return Strategy::COAST;
        case 3: return Strategy::COAST;
        case 4: return Strategy::SLIGHT_RIGHT;
        case 5: return Strategy::HARD_RIGHT;
        default: return Strategy::COAST;
    }
}

struct Detection { bool left, front, right; };

static Detection detect(const int16_t d[6]) {
    return {
        .left  = (valid(d[0]) && d[0] < ENEMY_ANY_MM) || (valid(d[1]) && d[1] < ENEMY_ANY_MM),
        .front = (valid(d[2]) && d[2] < ENEMY_FRONT_MM) && (valid(d[3]) && d[3] < ENEMY_FRONT_MM),
        .right = (valid(d[4]) && d[4] < ENEMY_ANY_MM) || (valid(d[5]) && d[5] < ENEMY_ANY_MM),
    };
}

// Steering bias: negative = turn left, positive = turn right, range roughly [-1, 1].
// Uses the four side+diagonal sensors weighted by closeness.
static float steer(const int16_t d[6]) {
    auto c = [](int16_t v) -> float {
        if (!valid(v)) return 0.0f;
        return 1.0f - std::min((float)v / (float)ENEMY_ANY_MM, 1.0f);
    };
    float l   = c(d[0]) * 2.0f + c(d[1]);   // left weight
    float r   = c(d[5]) * 2.0f + c(d[4]);   // right weight
    float sum = l + r;
    if (sum < 0.01f) return 0.0f;
    return (r - l) / sum;   // positive → turn right
}

// ── Robot ─────────────────────────────────────────────────────────────────

Robot::Robot(Motor& left, Motor& right, OLED& oled,
             ToFSensor tofs[6], const bool tof_ok[6])
    : _left(left), _right(right), _oled(oled), _tofs(tofs),
      _state(State::WAIT_START), _strategy(Strategy::COAST),
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
        if (_d[i] <= 0) { s[i][0]='-'; s[i][1]='-'; s[i][2]='-'; s[i][3]='\0'; }
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

    // ── Stop signal: PIN_START went low mid-fight ──────────────────────────
    if (_state != State::WAIT_START && _state != State::COUNTDOWN) {
        if (!gpio_get(PIN_START)) {
            _left.brake();
            _right.brake();
            _state   = State::WAIT_START;
            _state_t = now;
        }
    }

    switch (_state) {

    // ──────────────────────────────────────────────────────────────────────
    case State::WAIT_START:
        if (new_tof) {
            _strategy = pick_strategy(_d);
            _oled.clear();
            _oled.printf(0, 0, "%-21s", strategy_name(_strategy));
            _draw_tofs();
            _oled.display();
            printf("%-12s | LC:%3d%% LF:%3d%% RF:%3d%% RC:%3d%% | L:%3d%% R:%3d%%\n",
                strategy_name(_strategy),
                _d[1]>0 ? _d[1]*100/TOF_MAX_MM : -1,
                _d[2]>0 ? _d[2]*100/TOF_MAX_MM : -1,
                _d[3]>0 ? _d[3]*100/TOF_MAX_MM : -1,
                _d[4]>0 ? _d[4]*100/TOF_MAX_MM : -1,
                _d[0]>0 ? _d[0]*100/TOF_MAX_MM : -1,
                _d[5]>0 ? _d[5]*100/TOF_MAX_MM : -1);
        }

        if (gpio_get(PIN_START)) {
            _state   = State::MANEUVER;
            _state_t = now;
            break;
        }
        if (!gpio_get(PIN_BTN_0)) {
            _state   = State::COUNTDOWN;
            _state_t = now;
        }
        break;

    // ──────────────────────────────────────────────────────────────────────
    case State::COUNTDOWN: {
        uint32_t remaining = 5 - (in_state_ms / 1000);
        _oled.clear();
        _oled.printf(0, 0, "%-12s", strategy_name(_strategy));
        _oled.printf(0, 1, "START in %u", remaining);
        _oled.display();
        if (in_state_ms >= 5000) {
            _state   = State::MANEUVER;
            _state_t = now;
        }
        break;
    }

    // ──────────────────────────────────────────────────────────────────────
    case State::MANEUVER:
        switch (_strategy) {
        case Strategy::HARD_LEFT:
            _left.set_pwm(-START_HARD_TURN);
            _right.set_pwm(+START_HARD_TURN);
            break;
        case Strategy::SLIGHT_LEFT:
            _left.set_pwm(START_FWD_SPEED - START_SLIGHT_BIAS);
            _right.set_pwm(START_FWD_SPEED);
            break;
        case Strategy::COAST:
            _left.coast(); _right.coast();
            break;
        case Strategy::SLIGHT_RIGHT:
            _left.set_pwm(START_FWD_SPEED);
            _right.set_pwm(START_FWD_SPEED - START_SLIGHT_BIAS);
            break;
        case Strategy::HARD_RIGHT:
            _left.set_pwm(+START_HARD_TURN);
            _right.set_pwm(-START_HARD_TURN);
            break;
        }

        if (in_state_ms >= START_MANEUVER_MS) {
            if (new_tof) {
                Detection det = detect(_d);
                if (det.front)                _state = State::ATTACK;
                else if (det.left || det.right) _state = State::HOME;
                else                           _state = State::SEARCH;
            } else {
                _state = State::SEARCH;
            }
            _state_t = now;
        }
        break;

    // ──────────────────────────────────────────────────────────────────────
    case State::SEARCH:
        if ((in_state_ms / SEARCH_FLIP_MS) % 2 == 0) {
            _left.set_pwm(+SEARCH_TURN_SPEED);
            _right.set_pwm(-SEARCH_TURN_SPEED);
        } else {
            _left.set_pwm(-SEARCH_TURN_SPEED);
            _right.set_pwm(+SEARCH_TURN_SPEED);
        }

        if (new_tof) {
            Detection det = detect(_d);
            if (det.front)                  { _state = State::ATTACK; _state_t = now; }
            else if (det.left || det.right)  { _state = State::HOME;   _state_t = now; }
        }
        break;

    // ──────────────────────────────────────────────────────────────────────
    case State::HOME:
        if (new_tof) {
            Detection det = detect(_d);
            if (det.front) {
                _state = State::ATTACK; _state_t = now; break;
            }
            if (!det.left && !det.right) {
                _state = State::SEARCH; _state_t = now; break;
            }
            float s = steer(_d);  // positive → turn right
            _left.set_pwm( std::clamp(HOME_FWD + HOME_TURN * s, -1.0f, 1.0f));
            _right.set_pwm(std::clamp(HOME_FWD - HOME_TURN * s, -1.0f, 1.0f));
        }
        break;

    // ──────────────────────────────────────────────────────────────────────
    case State::ATTACK:
        _left.set_pwm(ATTACK_SPEED);
        _right.set_pwm(ATTACK_SPEED);

        if (_left.current_a()  > IMPACT_CURR_A ||
            _right.current_a() > IMPACT_CURR_A) {
            _left.set_curr_target(PUSH_CURR_A);
            _right.set_curr_target(PUSH_CURR_A);
            _state   = State::PUSH;
            _state_t = now;
            break;
        }

        if (new_tof) {
            Detection det = detect(_d);
            if (!det.front && !det.left && !det.right) { _state = State::SEARCH; _state_t = now; }
            else if (!det.front)                        { _state = State::HOME;   _state_t = now; }
        }
        break;

    // ──────────────────────────────────────────────────────────────────────
    case State::PUSH:
        // Current PID drives both motors at max push current.
        // If load drops off (enemy fell/escaped), go back to searching.
        if (_left.current_a()  < IMPACT_CURR_A * 0.4f &&
            _right.current_a() < IMPACT_CURR_A * 0.4f) {
            _state = State::SEARCH; _state_t = now;
        }
        break;
    }
}
