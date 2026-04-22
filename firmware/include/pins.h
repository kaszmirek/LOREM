#pragma once

// --- Line sensor threshold (PWM out) ---
constexpr uint PIN_ITR_THR_UC   = 0;   // GPIO0 - OUT_PWM

// --- VL53L0X XSHUT pins (active low, ext. pull-up) ---
constexpr uint PIN_XSHUT_0      = 1;   // GPIO1
constexpr uint PIN_XSHUT_1      = 2;   // GPIO2
constexpr uint PIN_XSHUT_2      = 20;  // GPIO20
constexpr uint PIN_XSHUT_3      = 17;  // GPIO17
constexpr uint PIN_XSHUT_4      = 14;  // GPIO14
constexpr uint PIN_XSHUT_5      = 15;  // GPIO15

// --- User input ---
constexpr uint PIN_BTN_0        = 3;   // GPIO3  - IN, ext. pull-up, low when pressed
constexpr uint PIN_START        = 4;   // GPIO4  - IN/OUT, remote start active high / debug LED

// --- Debug LEDs ---
constexpr uint PIN_DBG_1        = 5;   // GPIO5  - OUT

// --- Motor control (DRV8833, int. pull-down) ---
constexpr uint PIN_AIN1         = 6;   // GPIO6  - OUT_PWM
constexpr uint PIN_AIN2         = 7;   // GPIO7  - OUT_PWM
constexpr uint PIN_BIN2         = 8;   // GPIO8  - OUT_PWM
constexpr uint PIN_BIN1         = 9;   // GPIO9  - OUT_PWM

// --- Encoders ---
constexpr uint PIN_ENC_L_A      = 10;  // GPIO10 - IN
constexpr uint PIN_ENC_L_B      = 11;  // GPIO11 - IN
constexpr uint PIN_ENC_R_A      = 12;  // GPIO12 - IN
constexpr uint PIN_ENC_R_B      = 13;  // GPIO13 - IN

// --- IMU ---
constexpr uint PIN_IMU_INT1     = 16;  // GPIO16 - IN, active high

// --- I2C (bus shared by VL53L0X, IMU) ---
constexpr uint PIN_I2C_SDA      = 18;  // GPIO18
constexpr uint PIN_I2C_SCL      = 19;  // GPIO19

// --- Line sensors (active low) ---
constexpr uint PIN_ITR0         = 21;  // GPIO21 - IN
constexpr uint PIN_ITR1         = 22;  // GPIO22 - IN

// --- ADC ---
// Multiplier: 0.1803 V/V  =>  V_batt = adc_voltage / 0.1803
constexpr uint PIN_V_BATT       = 26;  // ADC0
// Multiplier: 5 V/A  =>  I = adc_voltage / 5.0
constexpr uint PIN_AISEN        = 27;  // ADC1 - left motor current
constexpr uint PIN_BISEN        = 28;  // ADC2 - right motor current
