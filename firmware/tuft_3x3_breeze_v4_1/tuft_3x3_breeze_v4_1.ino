/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  3x3 rig, gentle breeze, ESP32-S3 SuperMini
 *
 * v4.1. Rests at 100 degrees and never stops moving, gently. Where v4.0
 * fired discrete random shivers (which reads as "something startled it"),
 * this aims at a breeze, which is a different thing in three ways:
 *
 *   1. CONTINUOUS, NOT EVENT-DRIVEN. Nothing switches on or off. The stalks
 *      are always in motion, sometimes almost imperceptibly.
 *
 *   2. SPATIALLY CORRELATED. Real wind crosses a field, so neighbours move
 *      together with a lag rather than independently. Each servo's phase is
 *      offset by its position in the grid (mostly by column, slightly by
 *      row), so the motion visibly travels across the rig on a diagonal.
 *      This is the part that most separates "breeze" from "random twitching".
 *
 *   3. GUSTY. A slow envelope swells and fades the whole field. It is built
 *      from two slow sines at incommensurate periods (11s and 17s) summed,
 *      so the pattern never actually repeats, and it never reaches zero, so
 *      the field is never quite still.
 *
 * The flutter itself is likewise two sines at incommensurate periods (2.5s
 * and 3.7s) rather than one, plus a fixed random phase per servo generated
 * at boot. Sums of incommensurate sines are the standard cheap trick for
 * organic-looking motion: no shared period means no visible loop.
 *
 * Suits this hardware, too. Amplitude is small and speed is low, but unlike
 * the old slow sway the servos are never asked to hold a near-static target,
 * which is the regime where cheap analog units stall and lurch.
 *
 * TRIM: all zero, as asked, following the horn change. If servos sit
 * visibly out of line with each other, put their offsets in angleTrim.
 *
 * SHAFT_TO_TIP_MM only affects printed millimetre figures, not motion.
 * Update it for the longer horns.
 *
 * STARTUP: the 0 -> rest raise sequence is kept because it was liked, but
 * note it now sweeps a much longer arm through a much bigger arc. If the
 * new horns can hit anything on the way up, set DO_STARTUP_RAISE false and
 * it will simply ease to rest instead.
 *
 * WIRING: GPIO 1,3,4,5,6,7,8,10 -> servo 1..8. GPIO 2,9,11 unused.
 */

#define VERSION "4.1"

// ---------------- CONFIG ----------------

const int SERVO_COUNT = 8;
const int SERVO_PINS[SERVO_COUNT] = {1, 3, 4, 5, 6, 7, 8, 10};

// reporting only, update for the longer horns
const float SHAFT_TO_TIP_MM = 450.0;

const int SERVO_HZ = 50;
const int RES_BITS = 14;
const int MIN_US   = 500;
const int MAX_US   = 2400;

const float REST_DEG = 100.0;

// --- the breeze ---
const float SWAY_DEG = 5.0;        // peak degrees either side, at full gust

const float FLUTTER_A_S = 2.5;     // the two flutter periods. Deliberately
const float FLUTTER_B_S = 3.7;     // incommensurate, so they never re-align.

const float GUST_PERIOD_A_S = 11.0; // slow swell, likewise incommensurate
const float GUST_PERIOD_B_S = 17.0;
const float GUST_FLOOR = 0.18;     // never fully still; 0 would mean dead calm

// how far the wave lags from one column/row to the next, radians.
// larger = the breeze visibly travels; 0 = whole field moves as one.
const float LAG_PER_COL = 1.10;
const float LAG_PER_ROW = 0.35;

float angleTrim[SERVO_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};  // all zero, as asked

// --- startup ---
const bool  DO_STARTUP_RAISE = true;   // false = just ease to rest
const float LOWER_DEG = 0.0;
const unsigned long LOWER_HOLD_MS = 1500;
const float RISE_STEP_DEG = 1.0;
const int   RISE_STEP_MS  = 15;
const int   PAUSE_BETWEEN_SERVOS_MS = 200;

// -----------------------------------------

bool attachedOk[SERVO_COUNT];
const uint32_t TICKS = 1UL << RES_BITS;

float spatialPhase[SERVO_COUNT];  // fixed lag from grid position
float phaseJitter[SERVO_COUNT];   // fixed per-servo randomness

uint32_t usToDuty(float us) {
  return (uint32_t)lroundf(us * (float)TICKS / (1000000.0f / SERVO_HZ));
}

float degToUs(float deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  return MIN_US + (deg / 180.0f) * (MAX_US - MIN_US);
}

void writeDeg(int i, float deg) {
  if (!attachedOk[i]) return;
  ledcWrite(SERVO_PINS[i], usToDuty(degToUs(deg + angleTrim[i])));
}

// slow swell of the whole field, 0..1, never quite reaching zero
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
  Serial.printf("\nTuft 3x3 breeze, firmware v%s\n", VERSION);
  randomSeed(esp_random());

  for (int i = 0; i < SERVO_COUNT; i++) {
    attachedOk[i] = ledcAttach(SERVO_PINS[i], SERVO_HZ, RES_BITS);
    // grid position: servo n sits at row (n-1)/3, col (n-1)%3, matching
    //   7 8 9   <- row 2
    //   4 5 6
    //   1 2 3   <- row 0
    int row = i / 3, col = i % 3;
    spatialPhase[i] = -(col * LAG_PER_COL + row * LAG_PER_ROW);
    phaseJitter[i] = (float)random(-300, 301) / 1000.0f; // +/-0.3 rad
  }

  Serial.print("attached:");
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.printf(" s%d/GPIO%d=%s", i + 1, SERVO_PINS[i],
                  attachedOk[i] ? "OK" : "FAIL");
  }
  Serial.println();
  Serial.printf("rest %.0f deg, breeze +/-%.1f deg peak (%.0fmm at a %.0fmm tip)\n",
                REST_DEG, SWAY_DEG,
                SHAFT_TO_TIP_MM * sinf(radians(SWAY_DEG)), SHAFT_TO_TIP_MM);
  Serial.printf("flutter %.1fs/%.1fs, gusts %.0fs/%.0fs, trim all zero\n",
                FLUTTER_A_S, FLUTTER_B_S, GUST_PERIOD_A_S, GUST_PERIOD_B_S);

  if (DO_STARTUP_RAISE) {
    for (int i = 0; i < SERVO_COUNT; i++) writeDeg(i, LOWER_DEG);
    Serial.println("LOWER");
    delay(LOWER_HOLD_MS);
    Serial.println("RAISE: one at a time.");
    for (int i = 0; i < SERVO_COUNT; i++) {
      Serial.printf("  >> servo%d GPIO%d\n", i + 1, SERVO_PINS[i]);
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

  Serial.println("BREEZE.");
}

void loop() {
  float t = millis() / 1000.0;

  for (int i = 0; i < SERVO_COUNT; i++) {
    writeDeg(i, REST_DEG + breezeOffset(i, t));
  }

  static unsigned long lastReport = 0;
  if (millis() - lastReport > 10000) {
    lastReport = millis();
    Serial.printf("gust %.2f\n", gust(t));
  }

  delay(1000 / SERVO_HZ);
}
