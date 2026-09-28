/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  FULL 3x3, nine servos, LEDC + MCPWM, ESP32-S3 SuperMini
 *
 * v5.0. The grid is finally complete. Same breeze as v4.1, but all nine
 * positions are driven.
 *
 * HOW WE GOT PAST EIGHT. The S3's LEDC has 8 channels and that is a hard
 * limit (SOC_LEDC_CHANNEL_NUM), which is why servo9 has been dark all along
 * and why one servo kept appearing "dead" on a wandering pin. But LEDC is
 * not the chip's only PWM peripheral, it is just the only one servo
 * libraries use. The S3 also has MCPWM: 2 groups, 6 outputs each, 12 more
 * channels, and it is the peripheral actually intended for motor control.
 *
 * So: servos 1-8 on LEDC as before, servo9 on MCPWM. Nothing else changes.
 *
 *   LEDC  : 14-bit over a 20ms frame  -> 1.22us per step
 *   MCPWM : timer at 1MHz, 20000 tick period -> 1.00us per step, and the
 *           compare value IS the pulse width in microseconds, which makes
 *           it the more legible of the two.
 *
 * The two peripherals are close enough in resolution that servo9 should not
 * look different from its neighbours. If it does, that is worth knowing.
 *
 * HONEST CAVEAT: this is two code paths for one rig, which is why the
 * PCA9685 remains the better destination. One I2C board gives 16 uniform
 * channels, up to 62 boards on the bus, and hands the pulse timing to
 * dedicated hardware. Treat this as the bridge that gets the grid whole
 * before that arrives. It does also mean the rig can reach 12 servos on
 * the 12 pins this board breaks out, if you want to grow it meanwhile.
 *
 * GPIO9 IS FINE. It was cleared: the original fault was ESP32Servo's
 * allocator, never the pin. It's in the pool again if you want it.
 *
 * WIRING:
 *   GPIO 1,3,4,5,6,7,8,10  -> servo 1..8   (LEDC)
 *   GPIO 11                -> servo 9      (MCPWM)   <-- plug this in
 *   GPIO 2 unused (strapping). GPIO 9, 12, 13 free.
 */

#include "driver/mcpwm_prelude.h"

#define VERSION "5.0"

// ---------------- CONFIG ----------------

// LEDC-driven servos, 1..8
const int LEDC_COUNT = 8;
const int LEDC_PINS[LEDC_COUNT] = {1, 3, 4, 5, 6, 7, 8, 10};

// MCPWM-driven servos, 9 onward. Up to 3 here without more plumbing.
const int MC_COUNT = 1;
const int MC_PINS[MC_COUNT] = {11};

const int SERVO_COUNT = LEDC_COUNT + MC_COUNT;   // 9, the full grid

// reporting only, update for the longer horns
const float SHAFT_TO_TIP_MM = 450.0;

const int SERVO_HZ = 50;
const int RES_BITS = 14;
const int MIN_US   = 500;
const int MAX_US   = 2400;

const float REST_DEG = 100.0;

// --- the breeze, unchanged from v4.1 ---
const float SWAY_DEG = 5.0;
const float FLUTTER_A_S = 2.5;
const float FLUTTER_B_S = 3.7;
const float GUST_PERIOD_A_S = 11.0;
const float GUST_PERIOD_B_S = 17.0;
const float GUST_FLOOR = 0.18;
const float LAG_PER_COL = 1.10;
const float LAG_PER_ROW = 0.35;

float angleTrim[SERVO_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0};  // all zero

// --- startup ---
const bool  DO_STARTUP_RAISE = true;   // false = ease straight to rest
const float LOWER_DEG = 0.0;
const unsigned long LOWER_HOLD_MS = 1500;
const float RISE_STEP_DEG = 1.0;
const int   RISE_STEP_MS  = 15;
const int   PAUSE_BETWEEN_SERVOS_MS = 200;

// -----------------------------------------

bool servoOk[SERVO_COUNT];
float spatialPhase[SERVO_COUNT];
float phaseJitter[SERVO_COUNT];

const uint32_t TICKS = 1UL << RES_BITS;

// MCPWM handles. One timer shared, one operator+comparator+generator each.
static mcpwm_timer_handle_t mcTimer = NULL;
static mcpwm_cmpr_handle_t  mcCmp[MC_COUNT];

const uint32_t MC_RES_HZ = 1000000;   // 1MHz -> 1 tick = 1us
const uint32_t MC_PERIOD = 20000;     // 20000us = 20ms = 50Hz

int pinOf(int i) {
  return (i < LEDC_COUNT) ? LEDC_PINS[i] : MC_PINS[i - LEDC_COUNT];
}
const char *busOf(int i) { return (i < LEDC_COUNT) ? "ledc" : "mcpwm"; }

float degToUs(float deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  return MIN_US + (deg / 180.0f) * (MAX_US - MIN_US);
}

void writeDeg(int i, float deg) {
  if (!servoOk[i]) return;
  float us = degToUs(deg + angleTrim[i]);
  if (i < LEDC_COUNT) {
    uint32_t duty = (uint32_t)lroundf(us * (float)TICKS / (1000000.0f / SERVO_HZ));
    ledcWrite(LEDC_PINS[i], duty);
  } else {
    // at 1MHz the compare value is simply the pulse width in microseconds
    mcpwm_comparator_set_compare_value(mcCmp[i - LEDC_COUNT], (uint32_t)lroundf(us));
  }
}

bool mcpwmServosInit() {
  mcpwm_timer_config_t tcfg = {};
  tcfg.group_id = 0;
  tcfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  tcfg.resolution_hz = MC_RES_HZ;
  tcfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  tcfg.period_ticks = MC_PERIOD;
  if (mcpwm_new_timer(&tcfg, &mcTimer) != ESP_OK) return false;

  for (int n = 0; n < MC_COUNT; n++) {
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t ocfg = {};
    ocfg.group_id = 0;                       // operator must share the timer's group
    if (mcpwm_new_operator(&ocfg, &oper) != ESP_OK) return false;
    if (mcpwm_operator_connect_timer(oper, mcTimer) != ESP_OK) return false;

    mcpwm_comparator_config_t ccfg = {};
    ccfg.flags.update_cmp_on_tez = true;     // latch new value at period start
    if (mcpwm_new_comparator(oper, &ccfg, &mcCmp[n]) != ESP_OK) return false;

    mcpwm_gen_handle_t gen = NULL;
    mcpwm_generator_config_t gcfg = {};
    gcfg.gen_gpio_num = MC_PINS[n];
    if (mcpwm_new_generator(oper, &gcfg, &gen) != ESP_OK) return false;

    // high at the start of each period, low when the counter passes compare:
    // that is exactly a servo pulse of `compare` microseconds.
    mcpwm_generator_set_action_on_timer_event(gen,
      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                   MCPWM_TIMER_EVENT_EMPTY,
                                   MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     mcCmp[n],
                                     MCPWM_GEN_ACTION_LOW));

    mcpwm_comparator_set_compare_value(mcCmp[n], (uint32_t)lroundf(degToUs(REST_DEG)));
  }

  if (mcpwm_timer_enable(mcTimer) != ESP_OK) return false;
  if (mcpwm_timer_start_stop(mcTimer, MCPWM_TIMER_START_NO_STOP) != ESP_OK) return false;
  return true;
}

float gust(float t) {
  float a = 0.5f + 0.5f * sinf(2.0f * PI * t / GUST_PERIOD_A_S);
  float b = 0.5f + 0.5f * sinf(2.0f * PI * t / GUST_PERIOD_B_S + 1.7f);
  float g = 0.45f * a + 0.55f * b;
  return GUST_FLOOR + (1.0f - GUST_FLOOR) * g;
}

float breezeOffset(int i, float t) {
  float p = spatialPhase[i] + phaseJitter[i];
  float s = 0.62f * sinf(2.0f * PI * t / FLUTTER_A_S + p)
          + 0.38f * sinf(2.0f * PI * t / FLUTTER_B_S + p * 1.3f);
  return SWAY_DEG * gust(t) * s;
}

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.printf("\nTuft FULL 3x3 breeze, firmware v%s\n", VERSION);
  randomSeed(esp_random());

  for (int i = 0; i < LEDC_COUNT; i++) {
    servoOk[i] = ledcAttach(LEDC_PINS[i], SERVO_HZ, RES_BITS);
  }

  bool mcOk = mcpwmServosInit();
  for (int n = 0; n < MC_COUNT; n++) servoOk[LEDC_COUNT + n] = mcOk;

  for (int i = 0; i < SERVO_COUNT; i++) {
    // grid position, matching   7 8 9  <- row 2
    //                           4 5 6
    //                           1 2 3  <- row 0
    int row = i / 3, col = i % 3;
    spatialPhase[i] = -(col * LAG_PER_COL + row * LAG_PER_ROW);
    phaseJitter[i] = (float)random(-300, 301) / 1000.0f;
  }

  Serial.print("servos:");
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.printf(" s%d/GPIO%d/%s=%s", i + 1, pinOf(i), busOf(i),
                  servoOk[i] ? "OK" : "FAIL");
  }
  Serial.println();
  Serial.printf("LEDC %d-bit = %.2fus/step | MCPWM %luHz = %.2fus/step\n",
                RES_BITS, (1000000.0f / SERVO_HZ) / (float)TICKS,
                (unsigned long)MC_RES_HZ, 1000000.0f / (float)MC_RES_HZ);
  Serial.printf("rest %.0f deg, breeze +/-%.1f deg peak (%.0fmm at a %.0fmm tip)\n",
                REST_DEG, SWAY_DEG,
                SHAFT_TO_TIP_MM * sinf(radians(SWAY_DEG)), SHAFT_TO_TIP_MM);

  if (DO_STARTUP_RAISE) {
    for (int i = 0; i < SERVO_COUNT; i++) writeDeg(i, LOWER_DEG);
    Serial.println("LOWER");
    delay(LOWER_HOLD_MS);
    Serial.println("RAISE: one at a time.");
    for (int i = 0; i < SERVO_COUNT; i++) {
      Serial.printf("  >> servo%d GPIO%d (%s)\n", i + 1, pinOf(i), busOf(i));
      Serial.flush();
      for (float d = LOWER_DEG; d <= REST_DEG; d += RISE_STEP_DEG) {
        writeDeg(i, d);
        delay(RISE_STEP_MS);
      }
      writeDeg(i, REST_DEG);
      delay(PAUSE_BETWEEN_SERVOS_MS);
    }
  } else {
    for (int i = 0; i < SERVO_COUNT; i++) writeDeg(i, REST_DEG);
    delay(1500);
  }

  Serial.println("BREEZE, all nine.");
}

void loop() {
  float t = millis() / 1000.0;

  for (int i = 0; i < SERVO_COUNT; i++) {
    writeDeg(i, REST_DEG + breezeOffset(i, t));
  }

  delay(1000 / SERVO_HZ);
}
