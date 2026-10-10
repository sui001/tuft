/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  4x4 grid, 16 SG92R servos straight off an ESP32-S3 SuperMini,
 *          perfboard, no PCA9685
 *
 * v8.1. Calm, gusts and settles, from tuning on the perfboard rig:
 *   - smooth on every tuner setting at centre 90, jittery the moment the
 *     centre moves off it (the servos holding a leaning 1.1m rod against
 *     gravity shiver), so REST stays 90
 *   - faster than ~4 s for a +-10 deg swing gets rough; +-10 over 6 s is nice
 * So three moods, chosen at random:
 *   CALM    each servo sways +-6 deg over 6 s, scattered timing. 40-90 s.
 *   GUST    crossfades over 6 s into a wave crossing the grid, +-10 deg
 *           over 5 s, holds 2-4 waves, crossfades back. ~70% of the time.
 *   SETTLE  eases to still at 90 over 5 s, rests 8-20 s, eases back in
 *           over 8 s. ~30% of the time.
 * Both layers (scattered calm, travelling wave) run all the time with their
 * own phase; only their mix and the overall level change, and those change
 * by smoothstep, so nothing ever jumps.
 *
 * v8.0. Back to direct PWM, on the perfboard rebuild (docs/wiring_direct_esp.md).
 * v6's LEDC + MCPWM drivers with a pin map needing only two underside pads,
 * staggered pulse starts (so 16 current bursts don't stack), and servos
 * woken one at a time with outputs held low until their turn.
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

#define VERSION "8.1"

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

const float REST_DEG = 90.0;     // keep at 90: off-vertical, the held rods make the servos jitter

// --- calm: scattered sway ---
const float CALM_DEG = 6.0;
const float CALM_PERIOD_S = 6.0;
const float CALM_MIN_S = 40, CALM_MAX_S = 90;

// --- gust: travelling wave ---
const float GUST_DEG = 10.0;
const float GUST_PERIOD_S = 5.0;     // faster than ~4 s at this depth got rough
const float GUST_LAG_COL = 0.9;      // rad of wave phase per column
const float GUST_LAG_ROW = 0.3;      // and per row, so it crosses on a slant
const float GUST_FADE_S = 6.0;
const int   GUST_MIN_WAVES = 2, GUST_MAX_WAVES = 4;
const int   GUST_CHANCE_PCT = 70;    // otherwise a settle

// --- settle ---
const float SETTLE_OUT_S = 5.0, SETTLE_IN_S = 8.0;
const float SETTLE_MIN_S = 8, SETTLE_MAX_S = 20;

float angleTrim[16] = {0};

// --- startup ---
const int   WAKE_GAP_MS = 300;   // one servo at a time
const float FADE_IN_S   = 8.0;   // first level ramp 0 -> calm

// -----------------------------------------

const uint32_t TICKS = 1UL << RES_BITS;
const uint32_t FRAME_US = 1000000UL / SERVO_HZ;
const uint32_t MC_RES_HZ = 1000000;   // 1 tick = 1us
const uint32_t MC_PERIOD = 20000;     // 20ms

int ledcCh[LEDC_COUNT];
bool servoOk[16], awake[16];

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

// ---------------- MOODS ----------------

enum Mood { CALM, GUST_IN, GUST_HOLD, GUST_OUT, SETTLE_OUT, SETTLE_REST, SETTLE_IN };
const char *MOOD_NAMES[] = {"calm", "gust in", "gust", "gust out", "settling", "still", "waking"};
Mood mood = SETTLE_IN;           // the first fade-in is a waking
unsigned long moodStartMs = 0;
float moodLenS = FADE_IN_S;
float scatter[16];
float calmPhase = 0, gustPhase = 0;
float mixGust = 0;               // 0 = all calm layer, 1 = all gust layer
float level = 0;                 // overall amplitude multiplier, 0 = still

float smooth(float k) {
  if (k < 0) k = 0;
  if (k > 1) k = 1;
  return k * k * (3 - 2 * k);
}

float randRange(float a, float b) { return a + (b - a) * random(0, 10001) / 10000.0f; }

void enterMood(int mi) {
  Mood m = (Mood)mi;
  mood = m;
  moodStartMs = millis();
  switch (m) {
    case CALM:        moodLenS = randRange(CALM_MIN_S, CALM_MAX_S); break;
    case GUST_IN:     moodLenS = GUST_FADE_S; break;
    case GUST_HOLD:   moodLenS = GUST_PERIOD_S * random(GUST_MIN_WAVES, GUST_MAX_WAVES + 1); break;
    case GUST_OUT:    moodLenS = GUST_FADE_S; break;
    case SETTLE_OUT:  moodLenS = SETTLE_OUT_S; break;
    case SETTLE_REST: moodLenS = randRange(SETTLE_MIN_S, SETTLE_MAX_S); break;
    case SETTLE_IN:   moodLenS = SETTLE_IN_S; break;
  }
  Serial.printf("mood: %s for %.0fs\n", MOOD_NAMES[m], moodLenS);
}

void updateMood() {
  float k = (millis() - moodStartMs) / 1000.0f / moodLenS;
  switch (mood) {
    case CALM:        mixGust = 0; level = 1; break;
    case GUST_IN:     mixGust = smooth(k); level = 1; break;
    case GUST_HOLD:   mixGust = 1; level = 1; break;
    case GUST_OUT:    mixGust = 1 - smooth(k); level = 1; break;
    case SETTLE_OUT:  mixGust = 0; level = 1 - smooth(k); break;
    case SETTLE_REST: mixGust = 0; level = 0; break;
    case SETTLE_IN:   mixGust = 0; level = smooth(k); break;
  }
  if (k < 1) return;
  switch (mood) {
    case CALM:        enterMood(random(100) < GUST_CHANCE_PCT ? GUST_IN : SETTLE_OUT); break;
    case GUST_IN:     enterMood(GUST_HOLD); break;
    case GUST_HOLD:   enterMood(GUST_OUT); break;
    case GUST_OUT:    enterMood(CALM); break;
    case SETTLE_OUT:  enterMood(SETTLE_REST); break;
    case SETTLE_REST: enterMood(SETTLE_IN); break;
    case SETTLE_IN:   enterMood(CALM); break;
  }
}

float swayOffset(int i) {
  int row = i / GRID_COLS, col = i % GRID_COLS;
  float calm = CALM_DEG * sinf(calmPhase + scatter[i]);
  float wave = GUST_DEG * sinf(gustPhase - col * GUST_LAG_COL - row * GUST_LAG_ROW);
  return level * ((1 - mixGust) * calm + mixGust * wave);
}

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println();
  Serial.println("=== Tuft direct-ESP breeze v" VERSION " ===");
  Serial.println("4x4 SG92R servo grid on LEDC + MCPWM: calm sway, occasional gust waves and settles");
  Serial.println("https://github.com/sui001/tuft/tree/master/firmware/tuft_esp16_breeze_v8_1");
  randomSeed(esp_random());
  for (int i = 0; i < ACTIVE_COUNT; i++) scatter[i] = randRange(0, 2 * PI);

  bool lOk = ledcServosInit();
  bool mOk = mcpwmServosInit();
  for (int n = 0; n < MC_COUNT; n++) servoOk[LEDC_COUNT + n] = mOk;
  Serial.printf("LEDC %s, MCPWM %s\n", lOk ? "OK" : "FAILED", mOk ? "OK" : "FAILED");

  Serial.printf("WAKE one at a time to %.0f deg\n", REST_DEG);
  for (int i = 0; i < ACTIVE_COUNT; i++) {
    Serial.printf("  s%-2d GPIO%-2d %-5s %s\n", i + 1, pinOf(i), busOf(i),
                  servoOk[i] ? "" : "FAILED");
    if (servoOk[i]) wake(i);
    delay(WAKE_GAP_MS);
  }
  enterMood(SETTLE_IN);
}

void loop() {
  static unsigned long lastMs = millis();
  float dt = (millis() - lastMs) / 1000.0f;
  lastMs = millis();
  if (dt > 0.1f) dt = 0.1f;

  calmPhase += 2 * PI * dt / CALM_PERIOD_S;
  gustPhase += 2 * PI * dt / GUST_PERIOD_S;
  if (calmPhase > 2 * PI) calmPhase -= 2 * PI;
  if (gustPhase > 2 * PI) gustPhase -= 2 * PI;
  updateMood();

  for (int i = 0; i < ACTIVE_COUNT; i++) writeDeg(i, REST_DEG + swayOffset(i));

  static unsigned long lastReport = 0;
  if (millis() - lastReport > 6000) {
    lastReport = millis();
    int up = 0;
    for (int i = 0; i < ACTIVE_COUNT; i++) if (servoOk[i]) up++;
    Serial.printf("v%s %s, %d/%d up, level %.2f, gust mix %.2f\n",
                  VERSION, MOOD_NAMES[mood], up, ACTIVE_COUNT, level, mixGust);
  }

  delay(1000 / SERVO_HZ);
}
