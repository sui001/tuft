/*
 * PROJECT: Polyp (notwled servo console)
 * DEVICE:  up to 19 servos, LEDC + MCPWM both groups, ESP32-S3 SuperMini
 *
 * v6.0. Wired for 19 channels, running 16 (4x4) for the first test.
 *
 * CHANNEL BUDGET, and why 19 rather than 20:
 *   LEDC   8  channels, hard limit (SOC_LEDC_CHANNEL_NUM)
 *   MCPWM  12 channels, 2 groups x 3 operators x 2 generators
 *   -----  20 available on the chip
 * but GPIO48 is deliberately left free (it drives the onboard WS2812 RGB LED
 * on this board, confirmed against the board's own pinout card 10 Sep 2026),
 * so 19 is the practical maximum here. Pins and channels run out together.
 *
 * v5.1 only allocated MCPWM in group 0, which caps at 6. This walks both
 * groups: servo index -> group n/6, operator (n%6)/2, two generators per
 * operator each with their own comparator.
 *
 *   LEDC  14-bit over a 20ms frame -> 1.22us per step
 *   MCPWM 1MHz, 20000 tick period  -> 1.00us per step, compare value IS the
 *                                     pulse width in microseconds
 *
 * ACTIVE_COUNT decides how many of the pin list are actually brought up, so
 * the same firmware covers the 4x4 test now and the full 19 later without
 * re-pinning. Set it to 19 and GRID_COLS to suit when the rest is wired.
 *
 * PINS. Header pins 1,3-13 were always there; 14,15,16,17,18,21,47 are the
 * underside pads Sui is soldering. GPIO2 avoided (strapping), GPIO48 left
 * for the LED, 19/20 are USB, 26-32 are SPI flash, 33-38 are PSRAM territory
 * on this module and not worth the risk since they aren't needed.
 *
 * ACTIVE 4x4 LAYOUT (first 16 of the list), numbered bottom-left first:
 *      13 14 15 16    <- row 3 (top)
 *       9 10 11 12
 *       5  6  7  8
 *       1  2  3  4    <- row 0 (bottom)
 */

#include "driver/mcpwm_prelude.h"

#define VERSION "6.0"

// ---------------- CONFIG ----------------

// how many of the pins below to actually bring up. 16 = 4x4 test, 19 = all.
const int ACTIVE_COUNT = 16;
const int GRID_COLS    = 4;

// Sequential wiring, per the rewire on 2026-09-10 (servo8 -> GPIO9,
// servo9 -> GPIO10). The whole map collapses to one rule:
//     servo1 -> GPIO1, then servo N -> GPIO N+1 for N >= 2
// because GPIO2 is skipped as a strapping pin and everything shifts by one.
// So servos 1-16 occupy GPIO 1 and 3-17 with no gaps.
const int LEDC_COUNT = 8;
const int LEDC_PINS[LEDC_COUNT] = {1, 3, 4, 5, 6, 7, 8, 9};

// 11 MCPWM pins available; the first (ACTIVE_COUNT - 8) get used.
// 14,15,16,17,18,21,47 are the soldered pads.
const int MC_AVAILABLE = 11;
const int MC_PINS[MC_AVAILABLE] = {10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 47};

const int MC_ACTIVE = ACTIVE_COUNT - LEDC_COUNT;   // 8 for the 4x4 test

const float SHAFT_TO_TIP_MM = 450.0;   // reporting only

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

float angleTrim[19] = {0};

// --- startup ---
const bool  DO_STARTUP_RAISE = true;
const float LOWER_DEG = 0.0;
const unsigned long LOWER_HOLD_MS = 1500;
const float RISE_STEP_DEG = 1.0;
const int   RISE_STEP_MS  = 15;
const int   PAUSE_BETWEEN_SERVOS_MS = 200;

// -----------------------------------------

bool  servoOk[19];
float spatialPhase[19];
float phaseJitter[19];

const uint32_t TICKS = 1UL << RES_BITS;

// one timer per MCPWM group, 3 operators each, 2 generators per operator
static mcpwm_timer_handle_t mcTimer[2] = {NULL, NULL};
static mcpwm_oper_handle_t  mcOper[2][3] = {{NULL, NULL, NULL}, {NULL, NULL, NULL}};
static mcpwm_cmpr_handle_t  mcCmp[12];

const uint32_t MC_RES_HZ = 1000000;   // 1MHz -> 1 tick = 1us
const uint32_t MC_PERIOD = 20000;     // 20ms = 50Hz

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
  if (i >= ACTIVE_COUNT || !servoOk[i]) return;
  float us = degToUs(deg + angleTrim[i]);
  if (i < LEDC_COUNT) {
    uint32_t duty = (uint32_t)lroundf(us * (float)TICKS / (1000000.0f / SERVO_HZ));
    ledcWrite(LEDC_PINS[i], duty);
  } else {
    mcpwm_comparator_set_compare_value(mcCmp[i - LEDC_COUNT], (uint32_t)lroundf(us));
  }
}

// brings up `count` MCPWM servos, walking both groups: 6 per group,
// 2 generators per operator, one shared timer per group.
bool mcpwmServosInit(int count) {
  if (count > 12) return false;

  for (int n = 0; n < count; n++) {
    int grp     = n / 6;          // group 0 then group 1
    int inGrp   = n % 6;
    int operIdx = inGrp / 2;      // two servos share an operator

    if (mcTimer[grp] == NULL) {
      mcpwm_timer_config_t tcfg = {};
      tcfg.group_id = grp;
      tcfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
      tcfg.resolution_hz = MC_RES_HZ;
      tcfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
      tcfg.period_ticks = MC_PERIOD;
      if (mcpwm_new_timer(&tcfg, &mcTimer[grp]) != ESP_OK) return false;
    }

    if (mcOper[grp][operIdx] == NULL) {
      mcpwm_operator_config_t ocfg = {};
      ocfg.group_id = grp;        // operator must share its timer's group
      if (mcpwm_new_operator(&ocfg, &mcOper[grp][operIdx]) != ESP_OK) return false;
      if (mcpwm_operator_connect_timer(mcOper[grp][operIdx], mcTimer[grp]) != ESP_OK)
        return false;
    }

    mcpwm_comparator_config_t ccfg = {};
    ccfg.flags.update_cmp_on_tez = true;
    if (mcpwm_new_comparator(mcOper[grp][operIdx], &ccfg, &mcCmp[n]) != ESP_OK)
      return false;

    mcpwm_gen_handle_t gen = NULL;
    mcpwm_generator_config_t gcfg = {};
    gcfg.gen_gpio_num = MC_PINS[n];
    if (mcpwm_new_generator(mcOper[grp][operIdx], &gcfg, &gen) != ESP_OK) return false;

    // high at period start, low when the counter passes compare
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

  for (int g = 0; g < 2; g++) {
    if (mcTimer[g] != NULL) {
      if (mcpwm_timer_enable(mcTimer[g]) != ESP_OK) return false;
      if (mcpwm_timer_start_stop(mcTimer[g], MCPWM_TIMER_START_NO_STOP) != ESP_OK)
        return false;
    }
  }
  return true;
}

float gust(float t) {
  float a = 0.5f + 0.5f * sinf(2.0f * PI * t / GUST_PERIOD_A_S);
  float b = 0.5f + 0.5f * sinf(2.0f * PI * t / GUST_PERIOD_B_S + 1.7f);
  return GUST_FLOOR + (1.0f - GUST_FLOOR) * (0.45f * a + 0.55f * b);
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
  Serial.println();
  Serial.println("=== Polyp grid breeze v" VERSION " ===");
  Serial.println("4x4 SG92R servo grid (Tuft), direct GPIO via LEDC + MCPWM, breeze effect");
  Serial.println("https://github.com/sui001/polyp/tree/master/firmware/polyp_grid19_breeze_v6_0");
  randomSeed(esp_random());

  int ledcActive = (ACTIVE_COUNT < LEDC_COUNT) ? ACTIVE_COUNT : LEDC_COUNT;
  for (int i = 0; i < ledcActive; i++) {
    servoOk[i] = ledcAttach(LEDC_PINS[i], SERVO_HZ, RES_BITS);
  }

  bool mcOk = (MC_ACTIVE > 0) ? mcpwmServosInit(MC_ACTIVE) : true;
  for (int n = 0; n < MC_ACTIVE; n++) servoOk[LEDC_COUNT + n] = mcOk;

  for (int i = 0; i < ACTIVE_COUNT; i++) {
    int row = i / GRID_COLS, col = i % GRID_COLS;
    spatialPhase[i] = -(col * LAG_PER_COL + row * LAG_PER_ROW);
    phaseJitter[i] = (float)random(-300, 301) / 1000.0f;
  }

  // report in short chunks; long bursts get dropped by the S3's USB CDC
  for (int i = 0; i < ACTIVE_COUNT; i++) {
    Serial.printf("s%-2d GPIO%-2d %-5s %s\n", i + 1, pinOf(i), busOf(i),
                  servoOk[i] ? "OK" : "FAIL");
    Serial.flush();
    delay(25);
  }
  Serial.printf("%d servos active (%d ledc + %d mcpwm), grid %d cols\n",
                ACTIVE_COUNT, ledcActive, MC_ACTIVE, GRID_COLS);
  Serial.printf("chip ceiling 20 (ledc 8 + mcpwm 12); GPIO48 left free for the LED\n");

  if (DO_STARTUP_RAISE) {
    for (int i = 0; i < ACTIVE_COUNT; i++) writeDeg(i, LOWER_DEG);
    Serial.println("LOWER");
    delay(LOWER_HOLD_MS);
    Serial.println("RAISE: one at a time.");
    for (int i = 0; i < ACTIVE_COUNT; i++) {
      Serial.printf("  >> s%d GPIO%d\n", i + 1, pinOf(i));
      Serial.flush();
      for (float d = LOWER_DEG; d <= REST_DEG; d += RISE_STEP_DEG) {
        writeDeg(i, d);
        delay(RISE_STEP_MS);
      }
      writeDeg(i, REST_DEG);
      delay(PAUSE_BETWEEN_SERVOS_MS);
    }
  } else {
    for (int i = 0; i < ACTIVE_COUNT; i++) writeDeg(i, REST_DEG);
    delay(1500);
  }

  Serial.printf("BREEZE, all %d.\n", ACTIVE_COUNT);
}

void loop() {
  float t = millis() / 1000.0;
  for (int i = 0; i < ACTIVE_COUNT; i++) {
    writeDeg(i, REST_DEG + breezeOffset(i, t));
  }

  // Repeat the roll call forever as ONE line. The boot table is easy to miss
  // and the S3's USB CDC drops multi-line bursts when nothing is listening.
  static unsigned long lastReport = 0;
  if (millis() - lastReport > 6000) {
    lastReport = millis();
    int up = 0;
    for (int i = 0; i < ACTIVE_COUNT; i++) if (servoOk[i]) up++;
    Serial.printf("%d/%d up", up, ACTIVE_COUNT);
    for (int i = 0; i < ACTIVE_COUNT; i++) {
      if (!servoOk[i]) Serial.printf("  s%d/GPIO%d=FAIL", i + 1, pinOf(i));
    }
    Serial.println();
  }

  delay(1000 / SERVO_HZ);
}
