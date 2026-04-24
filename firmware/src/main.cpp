#include "adc_sensors.h"
#include "hardware/i2c.h"
#include "motor.h"
#include "oled.h"
#include "pico/stdlib.h"
#include "pins.h"
#include "tof.h"

// Ramp test: steps PWM 0.0 → 1.0 in 0.05 increments every 2 s.
// Watch the OLED — note the PWM level where it stops updating.
// Motors coast after reaching 1.0.

static constexpr float STEP    = 0.05f;
static constexpr int   HOLD_MS = 2000;

int main() {
  stdio_init_all();

  gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(PIN_I2C_SDA);
  gpio_pull_up(PIN_I2C_SCL);
  i2c_init(i2c1, 400'000);

  OLED oled(i2c1);
  oled.init();

  Battery battery(PIN_V_BATT);
  oled.printf(0, 0, "RAMP TEST");
  oled.printf(0, 1, "bat: %.2fV", battery.voltage());
  oled.display();

  ToFSensor tofs[] = {
      {i2c1, PIN_XSHUT_0, 0x30}, {i2c1, PIN_XSHUT_1, 0x31},
      {i2c1, PIN_XSHUT_2, 0x32}, {i2c1, PIN_XSHUT_3, 0x33},
      {i2c1, PIN_XSHUT_4, 0x34}, {i2c1, PIN_XSHUT_5, 0x35},
  };
  bool    tof_ok[6]    = {};
  bool    tof_ready[6] = {};
  int16_t tof_d[6]     = {-1, -1, -1, -1, -1, -1};

  for (int i = 0; i < 6; i++)
    tof_ok[i] = tofs[i].init();

  sleep_ms(1500);

  Motor left(PIN_AIN1, PIN_AIN2, PIN_ENC_L_B, PIN_ENC_L_A, PIN_AISEN, 0.002f,
             0.005f, 0.0001f);
  Motor right(PIN_BIN1, PIN_BIN2, PIN_ENC_R_A, PIN_ENC_R_B, PIN_BISEN, 0.002f,
              0.005f, 0.0001f);

  for (float pwm = 0.0f; pwm <= 1.001f; pwm += STEP) {
    float clamped = pwm < 1.0f ? pwm : 1.0f;
    left.set_pwm(clamped);
    right.set_pwm(clamped);

    uint64_t step_end = time_us_64() + HOLD_MS * 1000ULL;
    while (time_us_64() < step_end) {
      left.update();
      right.update();

      // ToF non-blocking poll — update tof_d[] whenever all sensors ready
      bool all_ready = true;
      for (int i = 0; i < 6; i++) {
        if (!tof_ok[i]) continue;
        if (!tof_ready[i]) tof_ready[i] = tofs[i].data_ready();
        if (!tof_ready[i]) all_ready = false;
      }
      if (all_ready) {
        for (int i = 0; i < 6; i++)
          tof_d[i] = tof_ok[i] ? tofs[i].read_mm() : -1;
        for (int i = 0; i < 6; i++)
          tof_ready[i] = false;
      }

      oled.clear();
      oled.printf(0, 0, "PWM:%.2f L:%4.0f", clamped, left.velocity_mm_s());
      oled.printf(0, 1, "I:%.2fA R:%4.0f", left.current_a(), right.velocity_mm_s());
      oled.printf(0, 2, "%d:%4d %d:%4d %d:%4d", 0, tof_d[0], 1, tof_d[1], 2, tof_d[2]);
      oled.printf(0, 3, "%d:%4d %d:%4d %d:%4d", 3, tof_d[3], 4, tof_d[4], 5, tof_d[5]);
      oled.display();

      sleep_ms(100);
    }
  }

  left.coast();
  right.coast();

  oled.clear();
  oled.printf(0, 0, "DONE");
  oled.printf(0, 2, "%d:%4d %d:%4d %d:%4d", 0, tof_d[0], 1, tof_d[1], 2, tof_d[2]);
  oled.printf(0, 3, "%d:%4d %d:%4d %d:%4d", 3, tof_d[3], 4, tof_d[4], 5, tof_d[5]);
  oled.display();

  while (true) tight_loop_contents();
}
