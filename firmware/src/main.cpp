#include "config.h"
#include "hardware/i2c.h"
#include "motor.h"
#include "oled.h"
#include "pico/stdlib.h"
#include "pins.h"
#include "tof.h"
#include "adc_sensors.h"
#include <algorithm>
#include <cstring>

// ── Enums ──────────────────────────────────────────────────────────────────

enum class Strategy { HARD_LEFT, SLIGHT_LEFT, COAST, SLIGHT_RIGHT, HARD_RIGHT };
enum class State    { WAIT_START, COUNTDOWN, MANEUVER, SEARCH, HOME, ATTACK, PUSH };

static const char* strategy_name(Strategy s) {
    switch (s) {
        case Strategy::HARD_LEFT:   return "HARD LEFT";
        case Strategy::SLIGHT_LEFT: return "SLIGHT LEFT";
        case Strategy::COAST:       return "COAST";
        case Strategy::SLIGHT_RIGHT:return "SLIGHT RIGHT";
        case Strategy::HARD_RIGHT:  return "HARD RIGHT";
    }
    return "?";
}

// ── TOF helpers ────────────────────────────────────────────────────────────

// Returns true if d is a valid (non-error) reading.
static bool valid(int16_t d) { return d > 0; }

// Pick strategy from sensor readings taken during WAIT_START.
// Closest TOF below SELECT_MM wins; default COAST.
static Strategy pick_strategy(const int16_t d[6]) {
    int   best_i = -1;
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
    // Convert to "closeness" (inverse distance, 0 if invalid)
    auto c = [](int16_t v) -> float {
        if (!valid(v)) return 0.0f;
        return 1.0f - std::min((float)v / (float)ENEMY_ANY_MM, 1.0f);
    };
    float l = c(d[0]) * 2.0f + c(d[1]);   // left weight
    float r = c(d[5]) * 2.0f + c(d[4]);   // right weight
    float sum = l + r;
    if (sum < 0.01f) return 0.0f;
    return (r - l) / sum;   // positive → turn right
}

// ── Motor helpers ──────────────────────────────────────────────────────────

static void drive(Motor& left, Motor& right, float l, float r) {
    left.set_pwm( std::clamp(l, -1.0f, 1.0f));
    right.set_pwm(std::clamp(r, -1.0f, 1.0f));
}

static void stop(Motor& left, Motor& right) {
    left.brake();
    right.brake();
}

// ── Non-blocking ToF batch read ────────────────────────────────────────────
// Returns true when all (working) sensors have fresh data; fills d[6].
static bool poll_tofs(ToFSensor tofs[], const bool tof_ok[6],
                      bool tof_ready[], int16_t d[6]) {
    bool all = true;
    for (int i = 0; i < 6; i++) {
        if (!tof_ok[i]) continue;
        if (!tof_ready[i]) tof_ready[i] = tofs[i].data_ready();
        if (!tof_ready[i]) all = false;
    }
    if (!all) return false;

    for (int i = 0; i < 6; i++)
        d[i] = tof_ok[i] ? tofs[i].read_mm() : -1;
    for (int i = 0; i < 6; i++)
        tof_ready[i] = false;
    return true;
}

// ── Main ───────────────────────────────────────────────────────────────────

int main() {
    stdio_init_all();

    // ── GPIO init ──────────────────────────────────────────────────────────
    gpio_init(PIN_START);
    gpio_set_dir(PIN_START, GPIO_IN);

    gpio_init(PIN_BTN_0);
    gpio_set_dir(PIN_BTN_0, GPIO_IN);

    gpio_init(PIN_DBG_1);
    gpio_set_dir(PIN_DBG_1, GPIO_OUT);
    gpio_put(PIN_DBG_1, 1);

    sleep_ms(200);

    // ── I2C + OLED ─────────────────────────────────────────────────────────
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
    i2c_init(i2c1, 400'000);

    OLED oled(i2c1);
    oled.init();

    Battery battery(PIN_V_BATT);
    oled.printf(0, 0, "bat: %.2fV", battery.voltage());
    oled.display();

    // ── ToF sensors ────────────────────────────────────────────────────────
    ToFSensor tofs[] = {
        {i2c1, PIN_XSHUT_0, 0x30}, {i2c1, PIN_XSHUT_1, 0x31},
        {i2c1, PIN_XSHUT_2, 0x32}, {i2c1, PIN_XSHUT_3, 0x33},
        {i2c1, PIN_XSHUT_4, 0x34}, {i2c1, PIN_XSHUT_5, 0x35},
    };

    bool tof_ok[6] = {};
    for (int i = 0; i < 6; i++) {
        tof_ok[i] = tofs[i].init();
        sleep_ms(5);
    }

    oled.printf(0, 1, "tof:%c%c%c%c%c%c",
        tof_ok[0]?'O':'x', tof_ok[1]?'O':'x', tof_ok[2]?'O':'x',
        tof_ok[3]?'O':'x', tof_ok[4]?'O':'x', tof_ok[5]?'O':'x');
    oled.display();

    // ── Motors ─────────────────────────────────────────────────────────────
    Motor left (PIN_AIN1, PIN_AIN2, PIN_ENC_L_B, PIN_ENC_L_A, PIN_AISEN,
                0.002f, 0.005f, 0.0001f);
    Motor right(PIN_BIN1, PIN_BIN2, PIN_ENC_R_A, PIN_ENC_R_B, PIN_BISEN,
                0.002f, 0.005f, 0.0001f);

    // ── State ──────────────────────────────────────────────────────────────
    State    state    = State::WAIT_START;
    Strategy strategy = Strategy::COAST;

    bool     tof_ready[6] = {};
    int16_t  d[6]         = {};
    uint64_t state_t      = time_us_64();   // timestamp of last state change
    bool     search_right = true;           // current search spin direction

    gpio_put(PIN_DBG_1, 0);

    // ── Main loop ──────────────────────────────────────────────────────────
    while (true) {
        left.update();
        right.update();

        bool new_tof = poll_tofs(tofs, tof_ok, tof_ready, d);
        uint64_t now = time_us_64();
        uint32_t in_state_ms = (uint32_t)((now - state_t) / 1000);

        // ── Stop signal: PIN_START went low mid-fight ──────────────────────
        if (state != State::WAIT_START && state != State::COUNTDOWN) {
            if (!gpio_get(PIN_START)) {
                stop(left, right);
                state  = State::WAIT_START;
                state_t = now;
            }
        }

        switch (state) {

        // ──────────────────────────────────────────────────────────────────
        case State::WAIT_START:
            if (new_tof) {
                strategy = pick_strategy(d);
                oled.clear();
                oled.printf(0, 0, "%-12s", strategy_name(strategy));
                oled.printf(0, 1, "%4d %4d %4d", d[0], d[1], d[2]);
                oled.printf(0, 2, "%4d %4d %4d", d[3], d[4], d[5]);
                oled.display();
            }

            // Remote start: immediate
            if (gpio_get(PIN_START)) {
                state   = State::MANEUVER;
                state_t = now;
                break;
            }
            // Button: 5 s countdown
            if (!gpio_get(PIN_BTN_0)) {
                state   = State::COUNTDOWN;
                state_t = now;
            }
            break;

        // ──────────────────────────────────────────────────────────────────
        case State::COUNTDOWN: {
            uint32_t remaining = 5 - (in_state_ms / 1000);
            oled.clear();
            oled.printf(0, 0, "%-12s", strategy_name(strategy));
            oled.printf(0, 1, "START in %u", remaining);
            oled.display();
            if (in_state_ms >= 5000) {
                state   = State::MANEUVER;
                state_t = now;
            }
            break;
        }

        // ──────────────────────────────────────────────────────────────────
        case State::MANEUVER:
            switch (strategy) {
            case Strategy::HARD_LEFT:
                drive(left, right, -START_HARD_TURN, +START_HARD_TURN);
                break;
            case Strategy::SLIGHT_LEFT:
                drive(left, right,
                    START_FWD_SPEED - START_SLIGHT_BIAS, START_FWD_SPEED);
                break;
            case Strategy::COAST:
                left.coast(); right.coast();
                break;
            case Strategy::SLIGHT_RIGHT:
                drive(left, right,
                    START_FWD_SPEED, START_FWD_SPEED - START_SLIGHT_BIAS);
                break;
            case Strategy::HARD_RIGHT:
                drive(left, right, +START_HARD_TURN, -START_HARD_TURN);
                break;
            }

            if (in_state_ms >= START_MANEUVER_MS) {
                // If enemy already spotted, skip straight to the right state
                if (new_tof) {
                    Detection det = detect(d);
                    if (det.front) {
                        state = State::ATTACK;
                    } else if (det.left || det.right) {
                        state = State::HOME;
                    } else {
                        state = State::SEARCH;
                    }
                } else {
                    state = State::SEARCH;
                }
                state_t = now;
            }
            break;

        // ──────────────────────────────────────────────────────────────────
        case State::SEARCH:
            // Spin slowly, flip direction every SEARCH_FLIP_MS
            if ((in_state_ms / SEARCH_FLIP_MS) % 2 == 0) {
                drive(left, right, +SEARCH_TURN_SPEED, -SEARCH_TURN_SPEED);
            } else {
                drive(left, right, -SEARCH_TURN_SPEED, +SEARCH_TURN_SPEED);
            }

            if (new_tof) {
                Detection det = detect(d);
                if (det.front) {
                    state = State::ATTACK; state_t = now;
                } else if (det.left || det.right) {
                    state = State::HOME;   state_t = now;
                }
            }
            break;

        // ──────────────────────────────────────────────────────────────────
        case State::HOME:
            if (new_tof) {
                Detection det = detect(d);
                if (det.front) {
                    state = State::ATTACK; state_t = now; break;
                }
                if (!det.left && !det.right) {
                    state = State::SEARCH; state_t = now; break;
                }
                float s = steer(d);  // positive → turn right
                drive(left, right,
                    HOME_FWD + HOME_TURN * s,
                    HOME_FWD - HOME_TURN * s);
            }
            break;

        // ──────────────────────────────────────────────────────────────────
        case State::ATTACK:
            drive(left, right, ATTACK_SPEED, ATTACK_SPEED);

            // Impact detection
            if (left.current_a()  > IMPACT_CURR_A ||
                right.current_a() > IMPACT_CURR_A) {
                left.set_curr_target(PUSH_CURR_A);
                right.set_curr_target(PUSH_CURR_A);
                state   = State::PUSH;
                state_t = now;
                break;
            }

            if (new_tof) {
                Detection det = detect(d);
                if (!det.front && !det.left && !det.right) {
                    state = State::SEARCH; state_t = now;
                } else if (!det.front) {
                    state = State::HOME;   state_t = now;
                }
            }
            break;

        // ──────────────────────────────────────────────────────────────────
        case State::PUSH:
            // Current PID drives both motors at max push current.
            // If load drops off (enemy fell/escaped), go back to searching.
            if (left.current_a()  < IMPACT_CURR_A * 0.4f &&
                right.current_a() < IMPACT_CURR_A * 0.4f) {
                state = State::SEARCH; state_t = now;
            }
            break;
        }
    }
}
