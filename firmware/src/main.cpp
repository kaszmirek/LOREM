#include "adc_sensors.h"
#include "encoder.h"
#include "hardware/i2c.h"
#include "line_sensor.h"
#include "motor.h"
#include "oled.h"
#include "pico/stdlib.h"
#include "pins.h"
#include "tof.h"

int main() {
  stdio_init_all();
  gpio_put(PIN_START, 1);
  gpio_put(PIN_DBG_1, 1);
  sleep_ms(700);

  gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(PIN_I2C_SDA);
  gpio_pull_up(PIN_I2C_SCL);

  gpio_put(PIN_START, 1);
  gpio_put(PIN_DBG_1, 0);
  i2c_init(i2c1, 400'000);

  OLED oled(i2c1);
  oled.init();

  Battery battery(PIN_V_BATT);
  oled.printf(0, 0, "bat: %.2fV", battery.voltage());
  oled.display();

  // DBG1 (GPIO4/PIN_START) — boot indicator, flashes during init
  gpio_init(PIN_START);
  gpio_set_dir(PIN_START, GPIO_OUT);

  // DBG2 (GPIO5/PIN_DBG_1) — ready indicator, flashes after boot
  gpio_init(PIN_DBG_1);
  gpio_set_dir(PIN_DBG_1, GPIO_OUT);

  // ToF sensors — all XSHUT held low in constructors until init().
  // Enabled one at a time so each comes up at 0x29, gets reassigned,
  // then stays online without conflicting with the next sensor.
  ToFSensor tofs[] = {
      {i2c1, PIN_XSHUT_0, 0x30}, {i2c1, PIN_XSHUT_1, 0x31},
      {i2c1, PIN_XSHUT_2, 0x32}, {i2c1, PIN_XSHUT_3, 0x33},
      {i2c1, PIN_XSHUT_4, 0x34}, {i2c1, PIN_XSHUT_5, 0x35},
  };

  bool tof_ok[6] = {};
  for (int i = 0; i < 6; i++) {
    gpio_put(PIN_START, 1);
    tof_ok[i] = tofs[i].init();
    gpio_put(PIN_START, 0);
    sleep_ms(5);
  }
  gpio_put(PIN_START, 0);
  gpio_put(PIN_DBG_1, 1);

  oled.printf(0, 1, "tof:%c%c%c%c%c%c", tof_ok[0] ? 'O' : 'x',
              tof_ok[1] ? 'O' : 'x', tof_ok[2] ? 'O' : 'x',
              tof_ok[3] ? 'O' : 'x', tof_ok[4] ? 'O' : 'x',
              tof_ok[5] ? 'O' : 'x');
  oled.printf(0, 2, "boot DONE");
  oled.display();

  sleep_ms(700);
  gpio_put(PIN_START, 0);

  Motor left(PIN_AIN1, PIN_AIN2);
  Motor right(PIN_BIN1, PIN_BIN2);

  Encoder enc_l(PIN_ENC_L_A, PIN_ENC_L_B);
  Encoder enc_r(PIN_ENC_R_A, PIN_ENC_R_B);
  CurrentSensor curr_l(PIN_AISEN);
  CurrentSensor curr_r(PIN_BISEN);

  bool dbg = false;
  float dir = 1.0f;
  left.set(1.0f * dir);
  right.set(1.0f * dir);
  while (true) {

    bool ready[6] = {};
    while (true) {
      bool all = true;
      for (int i = 0; i < 6; i++) {
        if (!tof_ok[i])
          continue;
        if (!ready[i])
          ready[i] = tofs[i].data_ready();
        if (!ready[i])
          all = false;
      }
      if (all)
        break;
    }

    int16_t d[6];
    for (int i = 0; i < 6; i++)
      d[i] = tof_ok[i] ? tofs[i].read_mm() : -1;

    oled.clear();
    oled.printf(0, 0, "bat: %.2fV", battery.voltage());
    oled.printf(0, 1, "%d:%4d %d:%4d %d:%4d", 0, d[0], 1, d[1], 2, d[2]);
    oled.printf(0, 2, "%d:%4d %d:%4d %d:%4d", 3, d[3], 4, d[4], 5, d[5]);
    oled.printf(0, 3, "mot1: %4d, mot2: %4d", enc_l.read(), enc_r.read());
    oled.display();
  }
}
