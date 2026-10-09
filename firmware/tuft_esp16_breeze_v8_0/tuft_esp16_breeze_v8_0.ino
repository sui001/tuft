/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  4x4 grid, 16 SG92R servos straight off an ESP32-S3 SuperMini,
 *          perfboard, no PCA9685
 *
 * v8.0. Back to direct PWM, on the perfboard rebuild (docs/wiring_direct_esp.md).
 * The PCA9685 rig went jittery past ~12 servos moving whatever the pattern.
 * v6's LEDC + MCPWM drivers with a new pin map that needs only two
 * underside pads, plus three things learned on the PCA rig:
 *
 *   - Staggered pulses. LEDC and MCPWM, like the PCA, start every servo's
 *     pulse at the same instant, so 16 current bursts stack 50 times a
 *     second. LEDC channels get spread hpoints; the MCPWM servos run in
 *     pairs off separate timers started at offsets.
 *   - Gentle wake. On power-up a servo's first pulse is a full-speed move
 *     nothing can soften, so servos wake one at a time, straight to REST.
 *     Every output stays low (servo limp) until its turn.
 *   - Sway in the range that looked good on the tuner: about +-6 to +-12
 *     deg over a ~4.5 s swing. Tiny sways look dead.
 *
 * PIN MAP, servo N numbered bottom-left first:
 *      13 14 15 16    <- top row
 *       9 10 11 12
 *       5  6  7  8
 *       1  2  3  4    <- bottom row
 *   LEDC  : s1-s8  -> GPIO 1,3,4,5,6,7,8,9
 *   MCPWM : s9-s16 -> GPIO 10,11,12,13, 43 (TX), 44 (RX), 14, 15 (pads)
 * TX/RX are free because USB CDC On Boot puts Serial on native USB. The
 * boot ROM prints on GPIO43 at power-on, so s13 may twitch once.
 *
 * Servo power from the buck into the perfboard V+ row only, never the
 * SuperMini's 5V pin. Grounds joined.
 */

#include "driver/mcpwm_prelude.h"
#include "driver/ledc.h"

#define VERSION "8.0"

// ---------------- CONFIG ----------------

const int ACTIVE_COUNT = 16;
const int GRID_COLS    = 4;

const int LEDC_COUNT = 8;
const int LEDC_PINS[LEDC_COUNT] = {1, 3, 4, 5, 6, 7, 8, 9};
const int MC_COUNT = 8;
const int MC_PINS[MC_COUNT] = {10, 11, 12, 13, 43, 44, 14, 15};

const int SERVO_HZ = 50;
const int RES_BITS = 14;
const int MIN_US   = 500;
const int MAX_US   = 2400;

const float REST_DEG = 90.0;     // tuner centre; nudge if the grid rests off vertical

// --- breeze ---
const float SPEED = 0.55;        // 2.5 s flutter / 0.55 = ~4.5 s swing, the tuned range
const float SWAY_DEG = 12.0;
const float FLUTTER_A_S = 2.5;
const float FLUTTER_B_S = 3.7;
const float GUST_PERIOD_A_S = 11.0;
const float GUST_PERIOD_B_S = 17.0;
const float GUST_FLOOR = 0.5;    // quietest stretch still +-6 deg
const float LAG_PER_COL = 1.10;
const float LAG_PER_ROW = 0.35;

float angleTrim[16] = {0};

// --- startup ---
const int   WAKE_GAP_MS = 300;   // one servo at a time
const float FADE_IN_S   = 8.0;   // sway amplitude ramps 0 -> full

// -----------------------------------------

const uint32_t TICKS = 1UL << RES_BITS;
const uint32_t FRAME_US = 1000000UL / SERVO_HZ;
const uint32_t MC_RES_HZ = 1000000;   // 1 tick = 1us
const uint32_t MC_PERIOD = 20000;     // 20ms

int ledcCh[LEDC_COUNT];
bool servoOk[16], awake[16];
float spatialPhase[16], phaseJitter[16];
unsigned long breezeStartMs = 0;

// MCPWM: one operator + one timer per pair of servos, 3 per group
static mcpwm_timer_handle_t mcTimer[4];
static mcpwm_oper_handle_t  mcOper[4];
static mcpwm_cmpr_handle_t  mcCmp[MC_COUNT];
static mcpwm_gen_handle_t   mcGen[MC_COUNT];

int pinOf(int i) { return i < LEDC_COUNT ? LEDC_PINS[i] : MC_PINS[i - LEDC_COUNT]; }
const char *busOf(int i) { return i < LEDC_COUNT ? "ledc" : "mcpwm"; }

float degToUs(float deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  return MIN_US + (deg / 180.0f) * (MAX_US - MIN_US);
}

// LEDC servo i pulses from hpoint i/16 of the frame; with 8 channels that
// spreads them over the first half, MCPWM pairs take the second half.
uint32_t ledcHpoint(int i) { return i * (TICKS / 16); }

void writeDeg(int i, float deg) {
  if (!servoOk[i] || !awake[i]) return;
  float us = degToUs(deg + angleTrim[i]);
  if (i < LEDC_COUNT) {
    uint32_t duty = (uint32_t)lroundf(us * (float)TICKS / FRAME_US);
    ledc_set_duty_with_hpoint(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ledcCh[i], duty, ledcHpoint(i));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ledcCh[i]);
  } else {
    mcpwm_comparator_set_compare_value(mcCmp[i - LEDC_COUNT], (uint32_t)lroundf(us));
  }
}

// first pulse for servo i: release it from forced-low and send REST
void wake(int i) {
  awake[i] = true;
  writeDeg(i, REST_DEG);
  if (i >= LEDC_COUNT) mcpwm_generator_set_force_level(mcGen[i - LEDC_COUNT], -1, true);
}

bool ledcServosInit() {
  bool ok = true;
  for (int i = 0; i < LEDC_COUNT; i++) {
    ledcCh[i] = i;
    servoOk[i] = ledcAttachChannel(LEDC_PINS[i], SERVO_HZ, RES_BITS, i);
    if (servoOk[i]) ledcWrite(LEDC_PINS[i], 0);   // no pulse until woken
    ok &= servoOk[i];
  }
  return ok;
}

bool mcpwmServosInit() {
  for (int p = 0; p < MC_COUNT / 2; p++) {
    int grp = p / 3;
    mcpwm_timer_config_t tcfg = {};
    tcfg.group_id = grp;
    tcfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    tcfg.resolution_hz = MC_RES_HZ;
    tcfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    tcfg.period_ticks = MC_PERIOD;
    if (mcpwm_new_timer(&tcfg, &mcTimer[p]) != ESP_OK) return false;

    mcpwm_operator_config_t ocfg = {};
    ocfg.group_id = grp;
    if (mcpwm_new_operator(&ocfg, &mcOper[p]) != ESP_OK) return false;
    if (mcpwm_operator_connect_timer(mcOper[p], mcTimer[p]) != ESP_OK) return false;

    for (int k = 0; k < 2; k++) {
      int n = p * 2 + k;
      mcpwm_comparator_config_t ccfg = {};
      ccfg.flags.update_cmp_on_tez = true;
      if (mcpwm_new_comparator(mcOper[p], &ccfg, &mcCmp[n]) != ESP_OK) return false;

      mcpwm_generator_config_t gcfg = {};
      gcfg.gen_gpio_num = MC_PINS[n];
      if (mcpwm_new_generator(mcOper[p], &gcfg, &mcGen[n]) != ESP_OK) return false;
      mcpwm_generator_set_force_level(mcGen[n], 0, true);   // limp until woken
      mcpwm_generator_set_action_on_timer_event(mcGen[n],
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
      mcpwm_generator_set_action_on_compare_event(mcGen[n],
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       mcCmp[n], MCPWM_GEN_ACTION_LOW));
      mcpwm_comparator_set_compare_value(mcCmp[n], (uint32_t)lroundf(degToUs(REST_DEG)));
    }
    if (mcpwm_timer_enable(mcTimer[p]) != ESP_OK) return false;
  }
  // Start the four timers 2.5ms apart, filling the second half of the frame.
  // They share one clock, so the offsets hold for as long as it runs.
  for (int p = 0; p < MC_COUNT / 2; p++) {
    if (p) delayMicroseconds(2500);
    if (mcpwm_timer_start_stop(mcTimer[p], MCPWM_TIMER_START_NO_STOP) != ESP_OK) return false;
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
  Serial.println("=== Tuft direct-ESP breeze v" VERSION " ===");
  Serial.println("4x4 SG92R servo grid on LEDC + MCPWM, staggered pulses, breeze effect");
  Serial.println("https://github.com/sui001/tuft/tree/master/firmware/tuft_esp16_breeze_v8_0");
  randomSeed(esp_random());

  bool lOk = ledcServosInit();
  bool mOk = mcpwmServosInit();
  for (int n = 0; n < MC_COUNT; n++) servoOk[LEDC_COUNT + n] = mOk;
  Serial.printf("LEDC %s, MCPWM %s\n", lOk ? "OK" : "FAILED", mOk ? "OK" : "FAILED");

  for (int i = 0; i < ACTIVE_COUNT; i++) {
    int row = i / GRID_COLS, col = i % GRID_COLS;
    spatialPhase[i] = -(col * LAG_PER_COL + row * LAG_PER_ROW);
    phaseJitter[i] = (float)random(-300, 301) / 1000.0f;
  }

  Serial.printf("WAKE one at a time to %.0f deg\n", REST_DEG);
  for (int i = 0; i < ACTIVE_COUNT; i++) {
    Serial.printf("  s%-2d GPIO%-2d %-5s %s\n", i + 1, pinOf(i), busOf(i),
                  servoOk[i] ? "" : "FAILED");
    if (servoOk[i]) wake(i);
    delay(WAKE_GAP_MS);
  }

  Serial.printf("BREEZE, fading in over %.0fs, sway up to +-%.0f deg\n", FADE_IN_S, SWAY_DEG);
  breezeStartMs = millis();
}

void loop() {
  float t = SPEED * millis() / 1000.0f;
  float fade = (millis() - breezeStartMs) / 1000.0f / FADE_IN_S;
  if (fade > 1.0f) fade = 1.0f;
  for (int i = 0; i < ACTIVE_COUNT; i++)
    writeDeg(i, REST_DEG + fade * breezeOffset(i, t));

  static unsigned long lastReport = 0;
  if (millis() - lastReport > 6000) {
    lastReport = millis();
    int up = 0;
    for (int i = 0; i < ACTIVE_COUNT; i++) if (servoOk[i]) up++;
    Serial.printf("v%s breeze, %d/%d up, gust %.2f\n", VERSION, up, ACTIVE_COUNT, gust(t));
  }

  delay(1000 / SERVO_HZ);
}
