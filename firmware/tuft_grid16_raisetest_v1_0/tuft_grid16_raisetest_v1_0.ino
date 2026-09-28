/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  4x4 panel, sequential raise test, ESP32-S3 SuperMini
 *
 * A wiring/liveness test for the newly assembled 4x4, not an effect. Loops:
 *
 *   1. LOWER   all 16 to 0 degrees, together, brief hold
 *   2. RAISE   servo 1, then 2, then 3 ... through 16, each alone
 *   3. HOLD    all up at 100 degrees for 5 seconds
 *   4. repeat
 *
 * Because each servo rises by itself with a pause after it, anything that
 * doesn't move on its turn is unambiguous: you get the servo number and its
 * GPIO on serial at the moment it should be moving, so a miswire or a dead
 * unit identifies itself rather than having to be hunted for.
 *
 * PIN MAP (2026-09-10 rewire). One rule: servo1 -> GPIO1, then servo N ->
 * GPIO N+1, because GPIO2 is skipped as a strapping pin and everything
 * shifts by one. Servos 1-16 sit on GPIO 1 and 3-17, no gaps.
 *
 *   LEDC  (8) : GPIO 1,3,4,5,6,7,8,9   -> servos 1-8
 *   MCPWM (8) : GPIO 10,11,12,13,14,15,16,17 -> servos 9-16
 *               14-17 are the soldered pads.
 *
 * GRID, numbered bottom-left first:
 *      13 14 15 16   <- row 3 (top)
 *       9 10 11 12
 *       5  6  7  8
 *       1  2  3  4   <- row 0 (bottom)
 *
 * MCPWM spans both groups (6 per group, 2 generators per operator), so
 * servos 15 and 16 live in group 1. Confirmed 16/16 up on 2026-09-10.
 */

#include "driver/mcpwm_prelude.h"

#define VERSION "1.0"

// ---------------- CONFIG ----------------

const int LEDC_COUNT = 8;
const int LEDC_PINS[LEDC_COUNT] = {1, 3, 4, 5, 6, 7, 8, 9};

const int MC_COUNT = 8;
const int MC_PINS[MC_COUNT] = {10, 11, 12, 13, 14, 15, 16, 17};

const int SERVO_COUNT = LEDC_COUNT + MC_COUNT;   // 16

const float LOWER_DEG = 0.0;
const float RAISE_DEG = 100.0;

const unsigned long LOWER_HOLD_MS = 1500;   // pause after everything drops
const unsigned long UP_HOLD_MS    = 5000;   // the 5s all-up hold
const float RISE_STEP_DEG = 1.0;
const int   RISE_STEP_MS  = 15;             // ~1.5s per servo
const int   PAUSE_BETWEEN_SERVOS_MS = 300;

const int SERVO_HZ = 50;
const int RES_BITS = 14;
const int MIN_US   = 500;
const int MAX_US   = 2400;

float angleTrim[SERVO_COUNT] = {0};

// -----------------------------------------

bool servoOk[SERVO_COUNT];
const uint32_t TICKS = 1UL << RES_BITS;

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
  if (!servoOk[i]) return;
  float us = degToUs(deg + angleTrim[i]);
  if (i < LEDC_COUNT) {
    ledcWrite(LEDC_PINS[i],
              (uint32_t)lroundf(us * (float)TICKS / (1000000.0f / SERVO_HZ)));
  } else {
    mcpwm_comparator_set_compare_value(mcCmp[i - LEDC_COUNT], (uint32_t)lroundf(us));
  }
}

bool mcpwmServosInit(int count) {
  if (count > 12) return false;
  for (int n = 0; n < count; n++) {
    int grp = n / 6, inGrp = n % 6, operIdx = inGrp / 2;

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
      ocfg.group_id = grp;
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

    mcpwm_generator_set_action_on_timer_event(gen,
      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                   MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     mcCmp[n], MCPWM_GEN_ACTION_LOW));

    mcpwm_comparator_set_compare_value(mcCmp[n], (uint32_t)lroundf(degToUs(LOWER_DEG)));
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

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.printf("\nTuft 4x4 raise test, firmware v%s\n", VERSION);

  for (int i = 0; i < LEDC_COUNT; i++) {
    servoOk[i] = ledcAttach(LEDC_PINS[i], SERVO_HZ, RES_BITS);
  }
  bool mcOk = mcpwmServosInit(MC_COUNT);
  for (int n = 0; n < MC_COUNT; n++) servoOk[LEDC_COUNT + n] = mcOk;

  int up = 0;
  for (int i = 0; i < SERVO_COUNT; i++) if (servoOk[i]) up++;
  Serial.printf("%d/%d channels up\n", up, SERVO_COUNT);
  for (int i = 0; i < SERVO_COUNT; i++) {
    if (!servoOk[i]) Serial.printf("  s%d GPIO%d %s FAILED\n", i + 1, pinOf(i), busOf(i));
  }
  Serial.println("cycle: lower all -> raise 1..16 in turn -> hold 5s -> repeat");
}

void loop() {
  // 1. everything down together
  Serial.println("\nLOWER all");
  for (int i = 0; i < SERVO_COUNT; i++) writeDeg(i, LOWER_DEG);
  delay(LOWER_HOLD_MS);

  // 2. up one at a time, so a non-mover names itself
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.printf("  raise s%-2d GPIO%-2d (%s)\n", i + 1, pinOf(i), busOf(i));
    Serial.flush();
    for (float d = LOWER_DEG; d <= RAISE_DEG; d += RISE_STEP_DEG) {
      writeDeg(i, d);
      delay(RISE_STEP_MS);
    }
    writeDeg(i, RAISE_DEG);
    delay(PAUSE_BETWEEN_SERVOS_MS);
  }

  // 3. all up, hold
  Serial.printf("ALL UP, holding %lums\n", UP_HOLD_MS);
  delay(UP_HOLD_MS);
}
