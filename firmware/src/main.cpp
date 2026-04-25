#include "adc_sensors.h"
#include "config.h"
#include "hardware/i2c.h"
#include "imu.h"
#include "motor.h"
#include "oled.h"
#include "pins.h"
#include "pico/stdlib.h"
#include "robot.h"
#include "tof.h"

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

    // ── IMU ────────────────────────────────────────────────────────────────
    IMU imu(i2c1);
    bool imu_ok = imu.init();
    oled.printf(0, 2, "imu:%s", imu_ok ? "OK" : "ERR");
    oled.display();

    // ── Motors ─────────────────────────────────────────────────────────────
    Motor left (PIN_AIN1, PIN_AIN2, PIN_ENC_L_B, PIN_ENC_L_A, PIN_AISEN,
                0.002f, 0.005f, 0.0001f);
    Motor right(PIN_BIN1, PIN_BIN2, PIN_ENC_R_A, PIN_ENC_R_B, PIN_BISEN,
                0.002f, 0.005f, 0.0001f);

    gpio_put(PIN_DBG_1, 0);

    // ── Run ────────────────────────────────────────────────────────────────
    Robot robot(left, right, oled, tofs, tof_ok, imu);

    while (true) {
        left.update();
        right.update();
        robot.update();
    }
}
