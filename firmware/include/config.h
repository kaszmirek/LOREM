#pragma once
#include <cstdint>

// ── Strategy selection ─────────────────────────────────────────────────────
// Hold a finger within SELECT_MM of a TOF before the match to choose strategy:
//   tof[0] = HARD_LEFT   tof[1] = SLIGHT_LEFT   tof[2]/[3] = COAST
//   tof[4] = SLIGHT_RIGHT               tof[5] = HARD_RIGHT
constexpr int16_t SELECT_MM = 80;

// ── Enemy detection ────────────────────────────────────────────────────────
constexpr int16_t ENEMY_ANY_MM   = 900;   // any TOF < this → enemy visible
constexpr int16_t ENEMY_FRONT_MM = 900;   // both front TOFs < this → attack straight

// ── Display scaling ────────────────────────────────────────────────────────
constexpr int16_t TOF_MIN_MM   = 30;      // VL53L0X reliable min (crosstalk below this)
constexpr int16_t TOF_MAX_MM   = 1000;    // VL53L0X reliable max @ 20 ms budget
constexpr int16_t TOF_SPIKE_MM = 150;     // single-reading jump larger than this → discard

// ── Countdown ─────────────────────────────────────────────────────────────
constexpr uint32_t COUNTDOWN_MS = 5000;  // pre-match countdown duration

// ── Start maneuver ─────────────────────────────────────────────────────────
constexpr float    START_FWD_SPEED   = 0.75f;  // forward speed during slight-turn start
constexpr float    START_HARD_TURN   = 1.0f;   // outer wheel speed for hard pivot
constexpr float    START_SLIGHT_BIAS = 0.35f;  // inner wheel reduction for slight curve
constexpr uint32_t START_MANEUVER_MS = 400;    // duration of start maneuver

// ── Steering ──────────────────────────────────────────────────────────────
constexpr float STEER_CORNER_WEIGHT = 2.0f;   // outer sensor weight vs inner diagonal
constexpr float STEER_FRONT_WEIGHT  = 0.3f;   // front sensor steering contribution (gives forward bias)
constexpr float STEER_DEADBAND      = 0.01f;  // steer sum below this → treat as zero

// ── Wait (post-rot scan) ───────────────────────────────────────────────────
constexpr float WAIT_SCAN_SPEED = 0.3f;  // motor speed during oscillating scan
constexpr float WAIT_SCAN_REVS  = 0.25f;  // wheel revolutions per swing

// ── Search ─────────────────────────────────────────────────────────────────
constexpr float SEARCH_TURN_SPEED = 0.50f;

// ── Homing (enemy visible but not centred) ────────────────────────────────
constexpr float HOME_FWD  = 0.60f;   // base forward speed
constexpr float HOME_TURN = 0.55f;   // max differential added/subtracted per side

// ── Attack ─────────────────────────────────────────────────────────────────
constexpr float ATTACK_SPEED = 1.0f;

// ── Push ──────────────────────────────────────────────────────────────────
constexpr float   PUSH_SPEED   = 1.0f;
constexpr int16_t PUSH_DIST_MM = TOF_MAX_MM * 15 / 100;  // 15% of max range → push
