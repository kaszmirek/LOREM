#include "adc_sensors.h"
#include "config.h"
#include "hardware/i2c.h"
#include "motor.h"
#include "oled.h"
#include "pico/stdlib.h"
#include "pins.h"
#include "tof.h"
#include <algorithm>

// Steering bias toward closest target: negative = left, positive = right.
// Uses all 6 sensors weighted by closeness.
static float steer(const int16_t d[6]) {
  auto c = [](int16_t v) -> float {
    if (v <= 0)
      return 0.0f;
    return 1.0f - std::min((float)v / (float)ENEMY_ANY_MM, 1.0f);
  };
  float l = c(d[0]) * 2.0f + c(d[1]) + c(d[2]);
  float r = c(d[5]) * 2.0f + c(d[4]) + c(d[3]);
  float sum = l + r;
  if (sum < 0.01f)
    return 0.0f;
  return (r - l) / sum;
}

// Non-blocking batch ToF poll. Returns true when all working sensors have fresh
// data.
static bool poll_tofs(ToFSensor tofs[], const bool tof_ok[6], bool tof_ready[],
                      int16_t d[6]) {
  bool all = true;
  for (int i = 0; i < 6; i++) {
    if (!tof_ok[i])
      continue;
    if (!tof_ready[i])
      tof_ready[i] = tofs[i].data_ready();
    if (!tof_ready[i])
      all = false;
  }
  if (!all)
    return false;
  for (int i = 0; i < 6; i++)
    d[i] = tof_ok[i] ? tofs[i].read_mm() : -1;
  for (int i = 0; i < 6; i++)
    tof_ready[i] = false;
  return true;
}

int main() {
  stdio_init_all();

  // gpio_init(PIN_START);
  // gpio_set_dir(PIN_START, GPIO_IN);
  gpio_init(PIN_BTN_0);
  gpio_set_dir(PIN_BTN_0, GPIO_IN);
  gpio_init(PIN_DBG_1);
  gpio_set_dir(PIN_DBG_1, GPIO_OUT);
  gpio_put(PIN_DBG_1, 1);

  gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(PIN_I2C_SDA);
  gpio_pull_up(PIN_I2C_SCL);
  i2c_init(i2c1, 400'000);

  OLED oled(i2c1);
  oled.init();

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

  oled.printf(0, 0, "tof:%c%c%c%c%c%c", tof_ok[0] ? 'O' : 'x',
              tof_ok[1] ? 'O' : 'x', tof_ok[2] ? 'O' : 'x',
              tof_ok[3] ? 'O' : 'x', tof_ok[4] ? 'O' : 'x',
              tof_ok[5] ? 'O' : 'x');
  oled.display();

  Motor left(PIN_AIN1, PIN_AIN2, PIN_ENC_L_B, PIN_ENC_L_A, PIN_AISEN, 0.002f,
             0.005f, 0.0001f);
  Motor right(PIN_BIN1, PIN_BIN2, PIN_ENC_R_A, PIN_ENC_R_B, PIN_BISEN, 0.002f,
              0.005f, 0.0001f);

  bool     tof_ready[6] = {};
  int16_t  d[6]         = {};
  bool     running      = false;
  bool     btn_prev     = false;
  uint64_t btn_edge_t   = UINT64_MAX;
  uint64_t oled_t       = 0;

  gpio_put(PIN_DBG_1, 0);

  while (true) {
    left.update();
    right.update();

    // Button: toggle after 50 ms stable press
    bool btn = !gpio_get(PIN_BTN_0);
    if (btn && !btn_prev)  btn_edge_t = time_us_64();
    if (!btn)              btn_edge_t = UINT64_MAX;
    if (btn && btn_edge_t != UINT64_MAX && (time_us_64() - btn_edge_t) >= 50'000) {
      running = !running;
      if (running) gpio_put(PIN_DBG_1, 1);
      else       { left.brake(); right.brake(); gpio_put(PIN_DBG_1, 0); }
      btn_edge_t = UINT64_MAX;
    }
    btn_prev = btn;

    // Motor logic: runs whenever fresh TOF data arrives
    if (poll_tofs(tofs, tof_ok, tof_ready, d) && running) {
      bool target = false;
      for (int i = 0; i < 6; i++)
        if (d[i] > 0 && d[i] < ENEMY_ANY_MM) { target = true; break; }

      if (target) {
        float spd = std::clamp(steer(d) * HOME_TURN, -1.0f, 1.0f);
        left.set_pwm( spd);
        right.set_pwm(-spd);
      } else {
        left.brake();
        right.brake();
      }
    }

    // OLED: update independently at ~10 Hz
    uint64_t now = time_us_64();
    if (now - oled_t < 100'000) continue;
    oled_t = now;

    bool target = false;
    for (int i = 0; i < 6; i++)
      if (d[i] > 0 && d[i] < ENEMY_ANY_MM) { target = true; break; }

    // Convert mm → 3-char percentage string ("---" if invalid, right-justified)
    auto pct = [](char* buf, int16_t mm) {
        if (mm <= 0) { buf[0]='-'; buf[1]='-'; buf[2]='-'; buf[3]='\0'; return; }
        int p = (int)mm * 100 / TOF_MAX_MM;
        if (p > 999) p = 999;
        snprintf(buf, 4, "%3d", p);
    };
    char s[6][4];
    for (int i = 0; i < 6; i++) pct(s[i], d[i]);

    // Spatial layout (21 cols × 4 rows):
    //  Row 0: LC    LF    RF    RC   ← front arc, by position
    //  Row 1: L    RUN aim    R      ← sides + center status
    //  s[0]=L  s[1]=LC  s[2]=LF  s[3]=RF  s[4]=RC  s[5]=R
    //  "xxx   xxx   xxx   xxx"  = 3+3+3+3+3+3+3 = 21
    //  "xxx    RUN aim    xxx"  = 3+4+3+1+3+4+3 = 21
    oled.clear();
    oled.printf(0, 0, "%s   %s   %s   %s", s[1], s[2], s[3], s[4]);
    oled.printf(0, 1, "%s    %s %s    %s",
                s[0], running ? "RUN" : "STP", target ? "aim" : "---", s[5]);
    oled.display();

    printf("LC:%s LF:%s RF:%s RC:%s | L:%s R:%s | %s %s\n",
           s[1], s[2], s[3], s[4], s[0], s[5],
           running ? "RUN" : "STP", target ? "aim" : "---");
  }
}
