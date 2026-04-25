#include "button.h"
#include "hardware/i2c.h"
#include "motor.h"
#include "oled.h"
#include "pins.h"
#include "pico/stdlib.h"

int main() {
    stdio_init_all();

    gpio_init(PIN_BTN_0);
    gpio_set_dir(PIN_BTN_0, GPIO_IN);

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

    Button btn0(PIN_BTN_0);
    bool running = false;

    oled.clear();
    oled.print(0, 0, "MOTOR TEST");
    oled.print(0, 1, "BTN0: toggle");
    oled.display();

    while (true) {
        btn0.update();
        left.update();
        right.update();

        if (btn0.pressed()) {
            running = !running;
            if (running) {
                left.set_pwm(1.0f);
                right.set_pwm(1.0f);
            } else {
                left.brake();
                right.brake();
            }
        }

        oled.clear();
        oled.printf(0, 0, "%s", running ? "RUNNING" : "STOPPED");
        oled.printf(0, 1, "L: %.2f A", left.current_a());
        oled.printf(0, 2, "R: %.2f A", right.current_a());
        oled.display();

        sleep_ms(50);
    }
}
