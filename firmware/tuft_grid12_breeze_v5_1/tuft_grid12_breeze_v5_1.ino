/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  12-servo grid (3 rows x 4 cols), LEDC + MCPWM, ESP32-S3 SuperMini
 *
 * v5.1. Twelve servos, which fills every pin this board breaks out, with no
 * soldering and no PCA9685.
 *
 * CHANNEL BUDGET
 *   LEDC   8 channels (SOC_LEDC_CHANNEL_NUM, hard limit)      -> servos 1-8
 *   MCPWM  12 channels (2 groups x 3 operators x 2 generators) -> servos 9-12
 *   Total available on this chip: 20. Twelve is what the twelve broken-out
 *   pins allow; the remaining 8 channels need the underside pads soldered.
 *
 * MCPWM ALLOCATION. Each group has one timer we share, 3 operators, and each
 * operator drives 2 generators with their own comparators. So 6 servos fit
 * in group 0 before group 1 is needed. v5.0 used one operator per servo and
 * so capped at 3; this pairs them properly.
 *
 *   LEDC  14-bit over a 20ms frame  -> 1.22us per step
 *   MCPWM 1MHz, 20000 tick period   -> 1.00us per step, and the compare
 *                                      value IS the pulse width in us
 *
 * GRID ORIENTATION. Assumes 4 columns x 3 rows, numbered bottom-left first:
 *
 *      9 10 11 12     <- row 2 (top)
 *      5  6  7  8     <- row 1
 *      1  2  3  4     <- row 0 (bottom)
 *
 * If the rig is physically 3 wide and 4 tall instead, set GRID_COLS to 3 and
 * the breeze's travel direction follows automatically.
 *
 * The breeze itself is unchanged from v4.1: continuous, spatially correlated
 * so it crosses the grid rather than each stalk twitching alone, and gusting
 * on incommensurate periods so it never visibly loops.
 *
 * WIRING:
 *   GPIO 1,3,4,5,6,7,8,10   -> servo 1..8    (LEDC)
 *   GPIO 11,9,12,13         -> servo 9..12   (MCPWM)
 *   GPIO 2 unused (strapping pin, do not use).
 *   That is every broken-out pin on the board.
 */

#include "driver/mcpwm_prelude.h"

#define VERSION "5.1"

// ---------------- CONFIG ----------------

const int LEDC_COUNT = 8;
const int LEDC_PINS[LEDC_COUNT] = {1, 3, 4, 5, 6, 7, 8, 10};

// up to 6 supported here (group 0: 3 operators x 2 generators)
const int MC_COUNT = 4;
const int MC_PINS[MC_COUNT] = {11, 9, 12, 13};

const int SERVO_COUNT = LEDC_COUNT + MC_COUNT;   // 12
const int GRID_COLS = 4;                          // set 3 if the rig is 3 wide

// reporting only
const float SHAFT_TO_TIP_MM = 450.0;

const int SERVO_HZ = 50;
const int RES_BITS = 14;
const int MIN_US   = 500;
const int MAX_US   = 2400;

const float REST_DEG = 100.0;

// --- breeze, unchanged from v4.1 ---
const float SWAY_DEG = 5.0;
const float FLUTTER_A_S = 2.5;
const float FLUTTER_B_S = 3.7;
const float GUST_PERIOD_A_S = 11.0;
const float GUST_PERIOD_B_S = 17.0;
const float GUST_FLOOR = 0.18;
const float LAG_PER_COL = 1.10;
const float LAG_PER_ROW = 0.35;

float angleTrim[SERVO_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// --- startup ---
const bool  DO_STARTUP_RAISE = true;
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

static mcpwm_timer_handle_t mcTimer = NULL;
static mcpwm_cmpr_handle_t  mcCmp[MC_COUNT];

const uint32_t MC_RES_HZ = 1000000;   // 1MHz -> 1 tick = 1us
const uint32_t MC_PERIOD = 20000;     // 20ms = 50Hz
const int MC_MAX_PER_GROUP = 6;       // 3 operators x 2 generators

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
    mcpwm_comparator_set_compare_value(mcCmp[i - LEDC_COUNT], (uint32_t)lroundf(us));
  }
}

bool mcpwmServosInit() {
  if (MC_COUNT > MC_MAX_PER_GROUP) return false;  // would need group 1 as well

  mcpwm_timer_config_t tcfg = {};
  tcfg.group_id = 0;
  tcfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  tcfg.resolution_hz = MC_RES_HZ;
  tcfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  tcfg.period_ticks = MC_PERIOD;
  if (mcpwm_new_timer(&tcfg, &mcTimer) != ESP_OK) return false;

  mcpwm_oper_handle_t opers[3] = {NULL, NULL, NULL};

  for (int n = 0; n < MC_COUNT; n++) {
    int operIdx = n / 2;              // two servos share one operator

    if (opers[operIdx] == NULL) {
      mcpwm_operator_config_t ocfg = {};
      ocfg.group_id = 0;              // must match the timer's group
      if (mcpwm_new_operator(&ocfg, &opers[operIdx]) != ESP_OK) return false;
      if (mcpwm_operator_connect_timer(opers[operIdx], mcTimer) != ESP_OK) return false;
    }

    mcpwm_comparator_config_t ccfg = {};
    ccfg.flags.update_cmp_on_tez = true;   // latch at period start
    if (mcpwm_new_comparator(opers[operIdx], &ccfg, &mcCmp[n]) != ESP_OK) return false;

    mcpwm_gen_handle_t gen = NULL;
    mcpwm_generator_config_t gcfg = {};
    gcfg.gen_gpio_num = MC_PINS[n];
    if (mcpwm_new_generator(opers[operIdx], &gcfg, &gen) != ESP_OK) return false;

    // high at period start, low when the counter passes compare:
    // a servo pulse of exactly `compare` microseconds
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
  Serial.printf("\nTuft 12-servo grid breeze, firmware v%s\n", VERSION);
  randomSeed(esp_random());

  for (int i = 0; i < LEDC_COUNT; i++) {
    servoOk[i] = ledcAttach(LEDC_PINS[i], SERVO_HZ, RES_BITS);
  }
  bool mcOk = mcpwmServosInit();
  for (int n = 0; n < MC_COUNT; n++) servoOk[LEDC_COUNT + n] = mcOk;

  for (int i = 0; i < SERVO_COUNT; i++) {
    int row = i / GRID_COLS, col = i % GRID_COLS;
    spatialPhase[i] = -(col * LAG_PER_COL + row * LAG_PER_ROW);
    phaseJitter[i] = (float)random(-300, 301) / 1000.0f;
  }

  Serial.print("servos:");
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.printf(" s%d/GPIO%d/%s=%s", i + 1, pinOf(i), busOf(i),
                  servoOk[i] ? "OK" : "FAIL");
  }
  Serial.println();
  Serial.printf("grid %d cols x %d rows, %d servos, all broken-out pins used\n",
                GRID_COLS, SERVO_COUNT / GRID_COLS, SERVO_COUNT);
  Serial.printf("LEDC %d-bit = %.2fus/step | MCPWM %luHz = %.2fus/step\n",
                RES_BITS, (1000000.0f / SERVO_HZ) / (float)TICKS,
                (unsigned long)MC_RES_HZ, 1000000.0f / (float)MC_RES_HZ);

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

  Serial.println("BREEZE, all twelve.");
}

void loop() {
  float t = millis() / 1000.0;
  for (int i = 0; i < SERVO_COUNT; i++) {
    writeDeg(i, REST_DEG + breezeOffset(i, t));
  }
  delay(1000 / SERVO_HZ);
}
