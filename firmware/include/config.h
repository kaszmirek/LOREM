#pragma once
#include <cstdint>

// ── Strategy selection ─────────────────────────────────────────────────────
// Hold a finger within SELECT_MM of a TOF before the match to choose strategy:
//   tof[0] = HARD_LEFT   tof[1] = SLIGHT_LEFT   tof[2]/[3] = COAST
//   tof[4] = SLIGHT_RIGHT               tof[5] = HARD_RIGHT
constexpr int16_t SELECT_MM = 80;

// ── Enemy detection ────────────────────────────────────────────────────────
constexpr int16_t ENEMY_ANY_MM   = 800;   // any TOF < this → enemy visible
constexpr int16_t ENEMY_FRONT_MM = 650;   // both front TOFs < this → attack straight

// ── Display scaling ────────────────────────────────────────────────────────
constexpr int16_t TOF_MAX_MM = 1200;      // VL53L0X reliable max @ 20 ms budget

// ── Start maneuver ─────────────────────────────────────────────────────────
constexpr float    START_FWD_SPEED   = 0.75f;  // forward speed during slight-turn start
constexpr float    START_HARD_TURN   = 1.0f;   // outer wheel speed for hard pivot
constexpr float    START_SLIGHT_BIAS = 0.35f;  // inner wheel reduction for slight curve
constexpr uint32_t START_MANEUVER_MS = 400;    // duration of start maneuver

// ── Search ─────────────────────────────────────────────────────────────────
constexpr float    SEARCH_TURN_SPEED = 0.50f;  // one wheel speed while spinning
constexpr uint32_t SEARCH_FLIP_MS    = 1000;   // reverse spin direction every N ms

// ── Homing (enemy visible but not centred) ────────────────────────────────
constexpr float HOME_FWD  = 0.60f;   // base forward speed
constexpr float HOME_TURN = 0.55f;   // max differential added/subtracted per side

// ── Attack ─────────────────────────────────────────────────────────────────
constexpr float ATTACK_SPEED = 1.0f;

// ── Impact → push (current PID) ───────────────────────────────────────────
constexpr float IMPACT_CURR_A = 0.8f;   // spike on either motor → contact
constexpr float PUSH_CURR_A   = 1.5f;   // current PID target while pushing
