/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  5x4 grid, 20 servos, LEDC + MCPWM, ESP32-S3 SuperMini
 *
 * v6.1. Twenty channels, which is this chip's absolute ceiling:
 *   LEDC   8  (SOC_LEDC_CHANNEL_NUM, hard limit)
 *   MCPWM  12 (2 groups x 3 operators x 2 generators)
 *   ----   20
 * There is no 21st without adding an external driver.
 *
 * THE 20th PIN, WITHOUT SOLDERING ANYTHING MORE. Header pins 1,3-13 plus the
 * soldered pads 14,15,16,17,18,21,47 come to 19. The twentieth is GPIO43,
 * which was on the header all along, marked TX. TX/RX (43/44) are only spoken
 * for if Serial runs over UART0, and with USB CDC On Boot enabled Serial goes
 * over native USB (19/20) instead, so 43 and 44 are ordinary GPIOs here.
 * GPIO44 is left spare and GPIO48 stays free for the onboard RGB LED.
 *
 * One consequence worth knowing: the ROM bootloader prints a short log to
 * UART0/GPIO43 at power-on, before any of this runs. Servo 20 may twitch once
 * on boot. Harmless, and unavoidable short of not using the pin.
 *
 * TORQUE, WITH 1100mm RODS. A 1.1m 2mm stainless rod is roughly 28g, so held
 * HORIZONTAL it asks about 1.5 kg-cm at the shaft, which is around 60% of an
 * SG92R's stall torque. Near-vertical it is trivial: at 10 degrees off
 * vertical the same rod needs only ~0.26 kg-cm. So the breeze itself is easy
 * and THE STARTUP RAISE IS THE HARD PART, because it begins at 0 degrees with
 * every rod horizontal. If servos buzz, stall or get warm during the raise,
 * raise RAISE_FROM_DEG (start part-way up) or set DO_STARTUP_RAISE false.
 * That is why RAISE_FROM_DEG exists as its own constant here.
 *
 * PIN MAP. servo1 -> GPIO1, then servo N -> GPIO N+1 up to servo 16 (GPIO2
 * skipped as a strapping pin), then the remaining pads and TX:
 *   LEDC  (8)  GPIO 1,3,4,5,6,7,8,9          -> servos 1-8
 *   MCPWM (12) GPIO 10,11,12,13,14,15,16,17  -> servos 9-16
 *              GPIO 18,21,47,43              -> servos 17-20
 *
 * GRID, 5 wide x 4 tall, numbered bottom-left first:
 *      16 17 18 19 20   <- row 3 (top)
 *      11 12 13 14 15
 *       6  7  8  9 10
 *       1  2  3  4  5   <- row 0 (bottom)
 * If the panel is actually 4 wide x 5 tall, set GRID_COLS to 4 and the
 * breeze's travel direction follows.
 */

#include "driver/mcpwm_prelude.h"

#define VERSION "6.1"

// ---------------- CONFIG ----------------

const int ACTIVE_COUNT = 20;
const int GRID_COLS    = 5;     // 5 wide x 4 tall; set 4 if the panel is 4 wide

const int LEDC_COUNT = 8;
const int LEDC_PINS[LEDC_COUNT] = {1, 3, 4, 5, 6, 7, 8, 9};

// 12 MCPWM channels, the peripheral's full complement across both groups.
// 14-18,21,47 are soldered pads; 43 is the header pin marked TX.
const int MC_AVAILABLE = 12;
const int MC_PINS[MC_AVAILABLE] = {10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 47, 43};

const int MC_ACTIVE = ACTIVE_COUNT - LEDC_COUNT;   // 12

const float SHAFT_TO_TIP_MM = 1100.0;   // reporting only. 2mm stainless rod.

const int SERVO_HZ = 50;
const int RES_BITS = 14;
const int MIN_US   = 500;
const int MAX_US   = 2400;

const float REST_DEG = 100.0;

// --- breeze ---
const float SWAY_DEG = 5.0;
const float FLUTTER_A_S = 2.5;
const float FLUTTER_B_S = 3.7;
const float GUST_PERIOD_A_S = 11.0;
const float GUST_PERIOD_B_S = 17.0;
const float GUST_FLOOR = 0.18;
const float LAG_PER_COL = 1.10;
const float LAG_PER_ROW = 0.35;

float angleTrim[20] = {0};

// --- startup ---
const bool  DO_STARTUP_RAISE = true;
const float RAISE_FROM_DEG = 0.0;   // raise the rods less far if torque bites
const unsigned long LOWER_HOLD_MS = 1500;
const float RISE_STEP_DEG = 1.0;
const int   RISE_STEP_MS  = 15;
const int   PAUSE_BETWEEN_SERVOS_MS = 200;

// -----------------------------------------

bool  servoOk[20];
float spatialPhase[20];
float phaseJitter[20];

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
  if (i >= ACTIVE_COUNT || !servoOk[i]) return;
  float us = degToUs(deg + angleTrim[i]);
  if (i < LEDC_COUNT) {
    ledcWrite(LEDC_PINS[i],
              (uint32_t)lroundf(us * (float)TICKS / (1000000.0f / SERVO_HZ)));
  } else {
    mcpwm_comparator_set_compare_value(mcCmp[i - LEDC_COUNT], (uint32_t)lroundf(us));
  }
}

// walks both MCPWM groups: 6 per group, 2 generators per operator,
// one shared timer per group. 12 is the full peripheral.
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

    mcpwm_comparator_set_compare_value(mcCmp[n], (uint32_t)lroundf(degToUs(RAISE_FROM_DEG)));
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
  Serial.printf("\nTuft 5x4 breeze, firmware v%s\n", VERSION);
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

  int up = 0;
  for (int i = 0; i < ACTIVE_COUNT; i++) if (servoOk[i]) up++;
  Serial.printf("%d/%d up (%d ledc + %d mcpwm), grid %d cols\n",
                up, ACTIVE_COUNT, ledcActive, MC_ACTIVE, GRID_COLS);
  for (int i = 0; i < ACTIVE_COUNT; i++) {
    if (!servoOk[i]) Serial.printf("  s%d GPIO%d %s FAILED\n", i + 1, pinOf(i), busOf(i));
  }
  Serial.printf("sway +/-%.1f deg = %.0fmm at a %.0fmm rod\n", SWAY_DEG,
                SHAFT_TO_TIP_MM * sinf(radians(SWAY_DEG)), SHAFT_TO_TIP_MM);

  if (DO_STARTUP_RAISE) {
    for (int i = 0; i < ACTIVE_COUNT; i++) writeDeg(i, RAISE_FROM_DEG);
    Serial.printf("LOWER to %.0f deg\n", RAISE_FROM_DEG);
    delay(LOWER_HOLD_MS);
    Serial.println("RAISE: one at a time.");
    for (int i = 0; i < ACTIVE_COUNT; i++) {
      Serial.printf("  >> s%d GPIO%d (%s)\n", i + 1, pinOf(i), busOf(i));
      Serial.flush();
      for (float d = RAISE_FROM_DEG; d <= REST_DEG; d += RISE_STEP_DEG) {
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
