#pragma once
#include <stdint.h>

// Stats written by core 0, read by core 1 (WiFi thread).
// All fields are naturally aligned — individual reads/writes are atomic on M0+.
struct RobotStats {
    volatile float   battery;
    volatile int16_t tof[6];
    volatile int32_t enc_l;
    volatile int32_t enc_r;
    volatile float   vel_l;    // encoder velocity, counts/s
    volatile float   vel_r;
    volatile float   curr_l;   // motor current, A
    volatile float   curr_r;
};

// Initialise WiFi and launch the HTTP + UDP server on core 1.
// `stats` must remain valid for the lifetime of the program.
void wifi_server_launch(RobotStats* stats);
