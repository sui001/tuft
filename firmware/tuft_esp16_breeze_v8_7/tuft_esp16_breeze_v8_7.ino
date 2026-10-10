/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  4x4 grid, 16 SG92R servos straight off an ESP32-S3 SuperMini,
 *          perfboard, no PCA9685
 *
 * v8.7. Swell removed. At +-18 deg it jittered badly: the rods spend long
 * stretches well off vertical, where holding their weight makes the servos
 * shiver (same cause as the centre having to stay 90). Keep sways around
 * +-12 deg or under. High's event after its waves is now turn or bend,
 * half each.
 *
 * v8.6. High's swell half again deeper, +-12 -> +-18 deg, still over 7 s
 * (peak 16 deg/s but gentler turnarounds than the +-10 / 5 s base wave).
 * Serial 's' pins high and starts a swell now, for showing it off.
 * Low and medium unchanged from v8.5, which Sui locked in.
 *
 * v8.5. Printing never stalls the motion. Over native USB a Serial write
 * waits up to 100 ms (the core's default) when nothing on the host is
 * reading, and this prints at every step change, i.e. right as a gust
 * starts. Every servo held still for it, then caught up at once: the
 * "pause across all, then jitter" left after v8.4. setTxTimeoutMs(0) drops
 * the line instead of waiting. A simulation of the v8.4 morph showed no
 * moment where the commanded motion itself pauses.
 *
 * v8.4. One sway per servo, morphed, instead of crossfading layers. v8.3
 * faded each servo from one sway into another, and where a servo's two
 * sways were in opposite phase they cancelled mid-fade: the servo paused for
 * 2-3 s, then leapt as the new sway's timing swung round. Now every servo
 * runs a single oscillator, and a transition bends its timing offset (the
 * short way round), size and speed from one pattern to the next, so nothing
 * can cancel. Fades are 8 s, so no servo has to hurry its timing much.
 *
 * v8.3. Variety. A second wave layer, TURN, crosses bottom to top on the
 * other slant; every gust in low and medium picks wave or turn at random.
 * High gets a third event, BEND: the whole field leans together, +-8 deg
 * over 8 s, in step. High's event after its base is now swell, turn or
 * bend.
 *
 * v8.2. Three settings and a display mode that walks through them.
 *   LOW     v8.1 unchanged, locked in: calm +-6 deg / 6 s scattered for
 *           40-90 s, then a gust (70%) or a settle (30%). ~1.5 min a cycle.
 *   MEDIUM  no settles. Calm 40-90 s, then a run of 2-3 gusts with 10-20 s
 *           of calm between them. ~2.5 min a cycle.
 *   HIGH    the gust wave (+-10 / 5 s) is the base, 30-60 s of it, then a
 *           deeper, slower swell (+-12 / 7 s) for 2-3 swells. ~1.25 min.
 *   DISPLAY 2-3 cycles of each, low -> medium -> high, about 11-16 min a
 *           loop. The default at power-up.
 * Serial: 1 low, 2 medium, 3 high, 0 display. A change starts at once and
 * crossfades from wherever the grid is.
 *
 * How it moves: three layers run all the time, each with its own phase
 * (scatter: +-6/6 s, random per servo; wave: +-10/5 s crossing the grid on
 * a slant; swell: +-12/7 s, same slant). Each setting is a script of steps
 * that crossfade between layers or ease the overall level, always by
 * smoothstep, so nothing ever jumps. Gusts fade in and out over 6 s.
 *
 * Limits found with the tuner (v2.0), 10 Oct: centre must stay 90 (off
 * vertical the held 1.1m rods jitter); +-10 deg faster than ~4 s is rough.
 *
 * v8.1. Low only: calm, gusts, settles. v8.0. Back to direct PWM on the
 * perfboard rebuild (docs/wiring_direct_esp.md), staggered pulse starts,
 * servos woken one at a time.
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

#define VERSION "8.7"

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

// --- layers ---
const float SCATTER_DEG = 6.0,  SCATTER_PERIOD_S = 6.0;
const float WAVE_DEG    = 10.0, WAVE_PERIOD_S    = 5.0;   // faster than ~4 s at this depth got rough
const float TURN_DEG    = 10.0, TURN_PERIOD_S    = 5.0;   // the wave on the other slant
const float BEND_DEG    = 8.0,  BEND_PERIOD_S    = 8.0;   // whole field in step, high only
const float LAG_COL = 0.9;       // rad of wave phase per column
const float LAG_ROW = 0.3;       // and per row, so waves cross on a slant

// --- scripts ---
const float FADE_S = 8.0;                         // any pattern morph; shorter = servos hurry more
const float CALM_MIN_S = 40, CALM_MAX_S = 90;     // low + medium
const int   GUST_MIN_WAVES = 2, GUST_MAX_WAVES = 4;
const int   LOW_GUST_PCT = 70;                    // low: else a settle
const float SETTLE_OUT_S = 5.0, SETTLE_IN_S = 8.0;
const float SETTLE_MIN_S = 8, SETTLE_MAX_S = 20;
const int   MED_GUSTS_MIN = 2, MED_GUSTS_MAX = 3;
const float MED_GAP_MIN_S = 10, MED_GAP_MAX_S = 20;
const float HIGH_BASE_MIN_S = 30, HIGH_BASE_MAX_S = 60;
const int   TURN_PCT = 50;                        // share of gusts that cross the other way
const int   BEND_MIN = 2, BEND_MAX = 3;
const int   HIGH_TURN_PCT = 50;                   // else a bend
const int   DISPLAY_CYCLES_MIN = 2, DISPLAY_CYCLES_MAX = 3;

float angleTrim[16] = {0};

// --- startup ---
const int   WAKE_GAP_MS = 300;   // one servo at a time
const float FADE_IN_S   = 8.0;   // first level ramp 0 -> full

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

// ---------------- MOTION ----------------

const int SCATTER = 0, WAVE = 1, TURN = 2, BEND = 3, LAYERS = 4;
const float LAYER_DEG[LAYERS]    = {SCATTER_DEG, WAVE_DEG, TURN_DEG, BEND_DEG};
const char *LAYER_NAMES[LAYERS]  = {"scatter", "wave", "turn", "bend"};
const float LAYER_PERIOD[LAYERS] = {SCATTER_PERIOD_S, WAVE_PERIOD_S,
                                    TURN_PERIOD_S, BEND_PERIOD_S};

const int S_LOW = 1, S_MEDIUM = 2, S_HIGH = 3, DISPLAY_MODE = 0;   // plain LOW/HIGH are Arduino macros
const char *SETTING_NAMES[] = {"display", "low", "medium", "high"};

// one shared oscillator; each servo sits at its own timing offset on it
float phase = 0;
float freq = 2 * PI / SCATTER_PERIOD_S, freqFrom;   // rad/s
float amp = SCATTER_DEG, ampFrom;
int   layer = SCATTER;                              // pattern the grid is on, or heading to
float off[16], offFrom[16], offDelta[16];
float level = 0, fromLevel = 0;
float scatter[16];

// a script is a queue of steps; each morphs the pattern, eases the level, or holds
const int HOLD = 0, MIX = 1, LEVEL = 2;
struct Step { int kind; int target; float seconds; const char *name; };
Step script[16];
int scriptLen = 0, stepIdx = 0;
unsigned long stepStartMs = 0;

int pinned = DISPLAY_MODE;       // what the user asked for
int setting = S_LOW;               // what is playing now
int cyclesLeft = 0;              // display mode: cycles of this setting still to play

float smooth(float k) {
  if (k < 0) k = 0;
  if (k > 1) k = 1;
  return k * k * (3 - 2 * k);
}

float randRange(float a, float b) { return a + (b - a) * random(0, 10001) / 10000.0f; }

void add(int kind, int target, float seconds, const char *name) {
  if (scriptLen < 16) script[scriptLen++] = {kind, target, seconds, name};
}

// a gust crosses either way, for variety
void addGustIn() {
  if (random(100) < TURN_PCT) add(MIX, TURN, FADE_S, "gust in (turned)");
  else                        add(MIX, WAVE, FADE_S, "gust in");
}

// where servo i sits on the shared oscillator for each pattern
float layerOffset(int l, int i) {
  int row = i / GRID_COLS, col = i % GRID_COLS;
  switch (l) {
    case SCATTER: return scatter[i];
    case WAVE:    return -(col * LAG_COL + row * LAG_ROW);
    case TURN:    return -(row * LAG_COL + (GRID_COLS - 1 - col) * LAG_ROW);   // bottom to top, other slant
    default:      return 0;                                                   // bend: all in step
  }
}

float wrapPi(float a) {
  a = fmodf(a + PI, 2 * PI);
  if (a < 0) a += 2 * PI;
  return a - PI;
}

int baseLayer(int s) { return s == S_HIGH ? WAVE : SCATTER; }

void buildCycle(int s) {
  scriptLen = 0;
  // arrive from wherever the grid is: awake, and on this setting's base layer
  if (level < 1) add(LEVEL, 1, level == 0 ? FADE_IN_S : SETTLE_IN_S, "waking");
  if (layer != baseLayer(s)) add(MIX, baseLayer(s), FADE_S, "to base");

  if (s == S_LOW) {
    add(HOLD, 0, randRange(CALM_MIN_S, CALM_MAX_S), "calm");
    if (random(100) < LOW_GUST_PCT) {
      addGustIn();
      add(HOLD, 0, WAVE_PERIOD_S * random(GUST_MIN_WAVES, GUST_MAX_WAVES + 1), "gust");
      add(MIX, SCATTER, FADE_S, "gust out");
    } else {
      add(LEVEL, 0, SETTLE_OUT_S, "settling");
      add(HOLD, 0, randRange(SETTLE_MIN_S, SETTLE_MAX_S), "still");
      add(LEVEL, 1, SETTLE_IN_S, "waking");
    }
  } else if (s == S_MEDIUM) {
    add(HOLD, 0, randRange(CALM_MIN_S, CALM_MAX_S), "calm");
    int n = random(MED_GUSTS_MIN, MED_GUSTS_MAX + 1);
    for (int g = 0; g < n; g++) {
      if (g) add(HOLD, 0, randRange(MED_GAP_MIN_S, MED_GAP_MAX_S), "lull");
      addGustIn();
      add(HOLD, 0, WAVE_PERIOD_S * random(GUST_MIN_WAVES, GUST_MAX_WAVES + 1), "gust");
      add(MIX, SCATTER, FADE_S, "gust out");
    }
  } else {
    add(HOLD, 0, randRange(HIGH_BASE_MIN_S, HIGH_BASE_MAX_S), "waves");
    int r = random(100);
    if (r < HIGH_TURN_PCT) {
      add(MIX, TURN, FADE_S, "turn in");
      add(HOLD, 0, TURN_PERIOD_S * random(GUST_MIN_WAVES, GUST_MAX_WAVES + 1), "turned");
      add(MIX, WAVE, FADE_S, "turn out");
    } else {
      add(MIX, BEND, FADE_S, "bend in");
      add(HOLD, 0, BEND_PERIOD_S * random(BEND_MIN, BEND_MAX + 1), "bend");
      add(MIX, WAVE, FADE_S, "bend out");
    }
  }
  stepIdx = -1;
}

void startStep(int i) {
  stepIdx = i;
  stepStartMs = millis();
  fromLevel = level;
  if (script[i].kind == MIX) {
    layer = script[i].target;
    ampFrom = amp;
    freqFrom = freq;
    for (int n = 0; n < ACTIVE_COUNT; n++) {
      offFrom[n] = off[n];
      offDelta[n] = wrapPi(layerOffset(layer, n) - off[n]);   // the short way round
    }
  }
  Serial.printf("[%s%s] %s for %.0fs\n", SETTING_NAMES[setting],
                pinned == DISPLAY_MODE ? ", display" : "", script[i].name, script[i].seconds);
}

void nextCycle() {
  if (pinned != DISPLAY_MODE) {
    setting = pinned;
  } else if (cyclesLeft <= 0) {
    setting = setting == S_HIGH ? S_LOW : setting + 1;
    cyclesLeft = random(DISPLAY_CYCLES_MIN, DISPLAY_CYCLES_MAX + 1);
    Serial.printf("display: %s for %d cycles\n", SETTING_NAMES[setting], cyclesLeft);
  }
  if (pinned == DISPLAY_MODE) cyclesLeft--;
  buildCycle(setting);
  startStep(0);
}

void updateScript() {
  if (stepIdx < 0 || stepIdx >= scriptLen) { nextCycle(); return; }
  const Step &st = script[stepIdx];
  float k = smooth((millis() - stepStartMs) / 1000.0f / st.seconds);
  if (st.kind == MIX)
  {
    amp = ampFrom + (LAYER_DEG[layer] - ampFrom) * k;
    freq = freqFrom + (2 * PI / LAYER_PERIOD[layer] - freqFrom) * k;
    for (int n = 0; n < ACTIVE_COUNT; n++) off[n] = offFrom[n] + offDelta[n] * k;
  }
  else if (st.kind == LEVEL)
    level = fromLevel + (st.target - fromLevel) * k;
  if (k >= 1) {
    if (stepIdx + 1 < scriptLen) startStep(stepIdx + 1);
    else nextCycle();
  }
}

float swayOffset(int i) { return level * amp * sinf(phase + off[i]); }

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c < '0' || c > '3') continue;
    pinned = c - '0';
    cyclesLeft = 0;
    if (pinned == DISPLAY_MODE) setting = S_HIGH;   // display restarts at low
    Serial.printf("setting: %s\n", SETTING_NAMES[pinned]);
    nextCycle();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);      // never let printing stall the servos
  delay(2500);
  Serial.println();
  Serial.println("=== Tuft direct-ESP breeze v" VERSION " ===");
  Serial.println("4x4 SG92R servo grid on LEDC + MCPWM: low / medium / high breeze, display mode cycles them");
  Serial.println("https://github.com/sui001/tuft/tree/master/firmware/tuft_esp16_breeze_v8_7");
  Serial.println("serial: 1 low, 2 medium, 3 high, 0 display");
  randomSeed(esp_random());
  for (int i = 0; i < ACTIVE_COUNT; i++) off[i] = scatter[i] = randRange(0, 2 * PI);

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
  setting = S_HIGH;                // display steps round to low first
  nextCycle();
}

void loop() {
  readSerial();
  static unsigned long lastMs = millis();
  float dt = (millis() - lastMs) / 1000.0f;
  lastMs = millis();
  if (dt > 0.1f) dt = 0.1f;

  phase += freq * dt;
  if (phase > 2 * PI) phase -= 2 * PI;
  updateScript();

  for (int i = 0; i < ACTIVE_COUNT; i++) writeDeg(i, REST_DEG + swayOffset(i));

  static unsigned long lastReport = 0;
  if (millis() - lastReport > 6000) {
    lastReport = millis();
    int up = 0;
    for (int i = 0; i < ACTIVE_COUNT; i++) if (servoOk[i]) up++;
    Serial.printf("v%s %s: %s, %d/%d up, level %.2f, %s +-%.1f deg, %.1f s\n",
                  VERSION, SETTING_NAMES[setting], script[stepIdx].name, up, ACTIVE_COUNT,
                  level, LAYER_NAMES[layer], amp, 2 * PI / freq);
  }

  delay(1000 / SERVO_HZ);
}
