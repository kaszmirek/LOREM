#include "hardware/i2c.h"
#include "motor.h"
#include "oled.h"
#include "pins.h"
#include "pico/stdlib.h"
#include <stdio.h>

int main() {
    stdio_init_all();

    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
    i2c_init(i2c1, 400'000);

    OLED oled(i2c1);
    oled.init();

    Motor left (PIN_AIN1, PIN_AIN2, PIN_ENC_L_B, PIN_ENC_L_A, PIN_AISEN,
                0.002f, 0.005f, 0.0001f);
    Motor right(PIN_BIN1, PIN_BIN2, PIN_ENC_R_A, PIN_ENC_R_B, PIN_BISEN,
                0.002f, 0.005f, 0.0001f);

    // Ramp 30% -> 100% in 5% steps, 200 ms per step
    for (int pct = 30; pct <= 100; pct += 5) {
        float speed = pct / 100.0f;
        left.set_pwm(speed);
        right.set_pwm(speed);

        uint64_t step_end = time_us_64() + 200'000;
        while (time_us_64() < step_end) {
            left.update();
            right.update();

            float l_curr = left.current_a();
            float r_curr = right.current_a();

            // JSON line — compatible with VSCode serial monitor / plotters
            printf("{\"pwm\":%d,\"l_curr\":%.3f,\"r_curr\":%.3f}\n",
                   pct, l_curr, r_curr);

            oled.clear();
            oled.printf(0, 0, "RAMP  %3d%%", pct);
            oled.printf(0, 1, "L: %.2f A", l_curr);
            oled.printf(0, 2, "R: %.2f A", r_curr);
            oled.display();

            sleep_ms(50);
        }
    }

    left.brake();
    right.brake();

    oled.clear();
    oled.print(0, 0, "DONE");
    oled.display();

    printf("{\"done\":true}\n");

    while (true) tight_loop_contents();
}
