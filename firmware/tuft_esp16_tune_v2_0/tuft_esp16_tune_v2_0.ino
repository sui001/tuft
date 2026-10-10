/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  4x4 grid, 16 SG92R servos straight off an ESP32-S3 SuperMini,
 *          perfboard, no PCA9685
 *
 * tune v2.0. The sway tuner (tune.html) ported to the direct-ESP perfboard
 * rig, same serial protocol as the PCA tuner v1.2 so the same page drives
 * it. v8.0's breeze twitched a bit aggressively on first run; this is for
 * finding out why. Servos 1-8 are on LEDC and 9-16 on MCPWM, so twitching
 * on only one half points at that driver, not at power.
 *
 * Commands (one per line, 115200):
 *   M n     which servos sway: 16-bit mask, bit 0 = servo 1
 *   A deg   sway amplitude, +- degrees around centre (0-30)
 *   P s     sway period in seconds (0.5-60)
 *   C deg   centre for all servos (60-120). Glides there.
 *   W n     pattern: 0 in step, 1 wave, 2 scattered
 *   L deg   wave lag between neighbouring servos, degrees of the cycle
 *   O n     pulse starts: 1 staggered across the frame (default), 0 together
 *   G / X   go / stop. Stop eases everything out to centre.
 *   V       print "ver <VERSION>"
 *   ?       print state
 *
 * Status line, same shape as v1.2 (i2c is always 0/0 here, there is no bus):
 *   st n 4 amp 9.00/9.00 period 4.5 centre 90.0 run 1 pattern 1 lag 90 stagger 1 frame 20 i2c 0/0
 *
 * PIN MAP as v8.0 (docs/wiring_direct_esp.md):
 *   LEDC  : s1-s8  -> GPIO 1,3,4,5,6,7,8,9
 *   MCPWM : s9-s16 -> GPIO 10,11,12,13, 43 (TX), 44 (RX), 14, 15 (pads)
 */

#include "driver/mcpwm_prelude.h"
#include "driver/ledc.h"

#define VERSION "2.0"

const int ACTIVE_COUNT = 16;
const int GRID_COLS = 4;

const int LEDC_COUNT = 8;
const int LEDC_PINS[LEDC_COUNT] = {1, 3, 4, 5, 6, 7, 8, 9};
const int MC_COUNT = 8;
const int MC_PINS[MC_COUNT] = {10, 11, 12, 13, 43, 44, 14, 15};

const int SERVO_HZ = 50;
const int FRAME_MS = 1000 / SERVO_HZ;
const int RES_BITS = 14;
const int MIN_US   = 500;
const int MAX_US   = 2400;

const float AMP_RATE   = 2.0;    // deg/s that each servo's amplitude eases at
const float GLIDE_RATE = 10.0;   // deg/s for centre changes
const int   WAKE_GAP_MS = 300;   // cold start: one servo at a time

const uint32_t TICKS = 1UL << RES_BITS;
const uint32_t FRAME_US = 1000000UL / SERVO_HZ;
const uint32_t MC_RES_HZ = 1000000;
const uint32_t MC_PERIOD = 20000;

uint16_t mask = 1;
float ampTarget = 3.0, amp[16];
float period = 6.0;
float centre = 90.0, centreTarget = 90.0;
bool  running = false;
bool  stagger = true;
float phase = 0;
int   pattern = 0, patternNext = 0;
float lagDeg = 90, lagNext = 90;
float offset[16], scatter[16];
unsigned long frameMs = 0;

bool servoOk[16], awake[16];
uint32_t lastVal[16];            // last duty / compare written, 0 = never

static mcpwm_timer_handle_t mcTimer[4];
static mcpwm_oper_handle_t  mcOper[4];
static mcpwm_cmpr_handle_t  mcCmp[MC_COUNT];
static mcpwm_gen_handle_t   mcGen[MC_COUNT];

float degToUs(float deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  return MIN_US + (deg / 180.0f) * (MAX_US - MIN_US);
}

uint32_t ledcHpoint(int i) { return stagger ? i * (TICKS / 16) : 0; }

void writeDeg(int i, float deg) {
  if (!servoOk[i] || !awake[i]) return;
  float us = degToUs(deg);
  if (i < LEDC_COUNT) {
    uint32_t duty = (uint32_t)lroundf(us * (float)TICKS / FRAME_US);
    if (duty == lastVal[i]) return;
    ledc_set_duty_with_hpoint(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, duty, ledcHpoint(i));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
    lastVal[i] = duty;
  } else {
    uint32_t cmp = (uint32_t)lroundf(us);
    if (cmp == lastVal[i]) return;
    mcpwm_comparator_set_compare_value(mcCmp[i - LEDC_COUNT], cmp);
    lastVal[i] = cmp;
  }
}

void wake(int i, float deg) {
  awake[i] = true;
  writeDeg(i, deg);
  if (i >= LEDC_COUNT) mcpwm_generator_set_force_level(mcGen[i - LEDC_COUNT], -1, true);
}

bool ledcServosInit() {
  bool ok = true;
  for (int i = 0; i < LEDC_COUNT; i++) {
    servoOk[i] = ledcAttachChannel(LEDC_PINS[i], SERVO_HZ, RES_BITS, i);
    if (servoOk[i]) ledcWrite(LEDC_PINS[i], 0);   // no pulse until woken
    ok &= servoOk[i];
  }
  return ok;
}

// MCPWM pairs run off their own timers; staggered = started 2.5ms apart,
// together = started back to back
void startMcTimers() {
  for (int p = 0; p < MC_COUNT / 2; p++) {
    if (p && stagger) delayMicroseconds(2500);
    mcpwm_timer_start_stop(mcTimer[p], MCPWM_TIMER_START_NO_STOP);
  }
}

void restartMcTimers() {
  for (int p = 0; p < MC_COUNT / 2; p++)
    mcpwm_timer_start_stop(mcTimer[p], MCPWM_TIMER_STOP_EMPTY);   // finish the frame first
  delay(FRAME_MS + 5);
  startMcTimers();
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
      mcpwm_comparator_set_compare_value(mcCmp[n], (uint32_t)lroundf(degToUs(centre)));
    }
    if (mcpwm_timer_enable(mcTimer[p]) != ESP_OK) return false;
  }
  startMcTimers();
  return true;
}

void applyPattern() {
  pattern = patternNext;
  lagDeg = lagNext;
  for (int i = 0; i < ACTIVE_COUNT; i++) {
    int row = i / GRID_COLS, col = i % GRID_COLS;
    if (pattern == 1)      offset[i] = -(col + row) * lagDeg * PI / 180.0f;
    else if (pattern == 2) offset[i] = scatter[i];
    else                   offset[i] = 0;
  }
}

bool patternPending() { return patternNext != pattern || lagNext != lagDeg; }

void printState() {
  int n = 0;
  float a = 0;
  for (int i = 0; i < ACTIVE_COUNT; i++) {
    if (mask >> i & 1) n++;
    if (amp[i] > a) a = amp[i];
  }
  Serial.printf("st n %d amp %.2f/%.2f period %.1f centre %.1f run %d pattern %d lag %.0f stagger %d frame %lu i2c 0/0\n",
                n, a, ampTarget, period, centreTarget, running ? 1 : 0,
                patternNext, lagNext, stagger ? 1 : 0, frameMs);
}

void handleLine(char *line) {
  char c = toupper(line[0]);
  float v = atof(line + 1);
  switch (c) {
    case 'M': mask = (uint16_t)strtoul(line + 1, NULL, 10); break;
    case 'S': if (v >= 1 && v <= ACTIVE_COUNT) mask = 1 << ((int)v - 1); break;
    case 'A': ampTarget = constrain(v, 0.0f, 30.0f); break;
    case 'P': period = constrain(v, 0.5f, 60.0f); break;
    case 'C': centreTarget = constrain(v, 60.0f, 120.0f); break;
    case 'O':
      if ((v != 0) != stagger) {
        stagger = v != 0;
        memset(lastVal, 0, sizeof(lastVal));          // LEDC rewrites with new hpoints
        restartMcTimers();
      }
      break;
    case 'W': if (v >= 0 && v <= 2) patternNext = (int)v; break;
    case 'L': lagNext = constrain(v, 0.0f, 180.0f); break;
    case 'G': running = true; break;
    case 'X': running = false; break;
    case '?': break;
    case 'V': Serial.println("ver " VERSION); return;
    default: Serial.printf("err unknown '%s'\n", line); return;
  }
  printState();
}

void readSerial() {
  static char buf[32];
  static int n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      buf[n] = 0;
      if (n) handleLine(buf);
      n = 0;
    } else if (n < (int)sizeof(buf) - 1) {
      buf[n++] = ch;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println();
  Serial.println("=== Tuft direct-ESP tune v" VERSION " ===");
  Serial.println("Sway tuner for the 4x4 perfboard grid on LEDC + MCPWM (see tune.html)");
  Serial.println("https://github.com/sui001/tuft/tree/master/firmware/tuft_esp16_tune_v2_0");
  randomSeed(esp_random());
  for (int i = 0; i < ACTIVE_COUNT; i++) {
    scatter[i] = random(0, 6283) / 1000.0f;
    amp[i] = 0;
  }
  applyPattern();

  bool lOk = ledcServosInit();
  bool mOk = mcpwmServosInit();
  for (int n = 0; n < MC_COUNT; n++) servoOk[LEDC_COUNT + n] = mOk;
  Serial.printf("LEDC %s, MCPWM %s\n", lOk ? "OK" : "FAILED", mOk ? "OK" : "FAILED");

  // a reset leaves every servo limp, so each wakes cold, one at a time
  for (int i = 0; i < ACTIVE_COUNT; i++) {
    if (servoOk[i]) wake(i, centre);
    delay(WAKE_GAP_MS);
  }
  Serial.println("ok ready, all at centre, send G to sway");
  printState();
}

void loop() {
  unsigned long frameStart = millis();
  readSerial();

  static unsigned long lastMs = millis();
  float dt = (millis() - lastMs) / 1000.0f;
  lastMs = millis();
  if (dt > 0.1f) dt = 0.1f;

  float dc = centreTarget - centre, cs = GLIDE_RATE * dt;
  centre = fabsf(dc) > cs ? centre + (dc > 0 ? cs : -cs) : centreTarget;

  bool pending = patternPending();
  bool allZero = true;
  for (int i = 0; i < ACTIVE_COUNT; i++) {
    float want = (running && !pending && (mask >> i & 1)) ? ampTarget : 0;
    float da = want - amp[i], as = AMP_RATE * dt;
    amp[i] = fabsf(da) > as ? amp[i] + (da > 0 ? as : -as) : want;
    if (amp[i] != 0) allZero = false;
  }
  if (pending && allZero) { applyPattern(); printState(); }

  phase += 2.0f * PI * dt / period;
  if (phase > 2.0f * PI) phase -= 2.0f * PI;

  for (int i = 0; i < ACTIVE_COUNT; i++)
    writeDeg(i, centre + amp[i] * sinf(phase + offset[i]));

  static unsigned long lastReport = 0;
  if (millis() - lastReport > 1000) { lastReport = millis(); printState(); }

  while (millis() - frameStart < FRAME_MS) readSerial();
  frameMs = millis() - frameStart;
}
