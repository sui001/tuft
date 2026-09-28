/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  3x3 rig, resting pose with sparse random wiggles
 *
 * v4.0, a new effect rather than a variant of the sway. Everything rests
 * still at 100 degrees. Every 2-3 seconds a small random handful of tips
 * gets a brief 5 degree shiver, then settles again. Nothing else moves.
 *
 * The stillness is the point: a sparse twitch in an otherwise motionless
 * rig reads as something stirring, where continuous motion reads as a
 * machine running. Because the base pose is static, the servos also spend
 * most of their time not being asked to creep, which is the regime where
 * these cheap analog units look worst anyway.
 *
 * Each wiggle is an enveloped oscillation, not a jerk: amplitude fades in
 * and out over WIGGLE_MS via a sine window, with WIGGLE_CYCLES oscillations
 * inside it, so it starts and ends at exactly the rest position with no
 * step discontinuity at either end.
 *
 * Servos wiggle independently. Each keeps its own start time and can be
 * mid-shiver while others are still, so events overlap naturally instead
 * of the rig pulsing in lockstep.
 *
 * TRIM WARNING: angleTrim corrects servos 2,5,8 for horns seated one spline
 * tooth (14.4 deg) high. If the horns are pulled and refitted, that offset
 * will almost certainly change. Re-check and zero these if they now line up.
 *
 * SHAFT_TO_TIP_MM only affects the printed millimetre figures, not the
 * motion, since everything here is commanded in degrees. Update it after
 * the horn change so the reporting stays honest.
 *
 * WIRING: GPIO 1,3,4,5,6,7,8,10 -> servo 1..8. GPIO 2,9,11 unused.
 */

#define VERSION "4.0"

// ---------------- CONFIG ----------------

const int SERVO_COUNT = 8;
const int SERVO_PINS[SERVO_COUNT] = {1, 3, 4, 5, 6, 7, 8, 10};

// reporting only, update after the longer horns go on
const float SHAFT_TO_TIP_MM = 450.0;

const int SERVO_HZ = 50;
const int RES_BITS = 14;
const int MIN_US   = 500;
const int MAX_US   = 2400;

// the resting pose everything returns to
const float REST_DEG = 100.0;

// the wiggle itself
const float WIGGLE_DEG    = 5.0;   // amplitude, degrees either side of rest
const int   WIGGLE_MS     = 900;   // how long one shiver lasts
const float WIGGLE_CYCLES = 2.0;   // oscillations inside that window

// how often a wiggle event fires, and how many tips it grabs
const unsigned long GAP_MIN_MS = 2000;
const unsigned long GAP_MAX_MS = 3000;
const int WIGGLE_MIN_SERVOS = 1;
const int WIGGLE_MAX_SERVOS = 3;

// servos 2,5,8 sit one spline tooth high, see v3.2. RE-CHECK AFTER RE-HORNING.
float angleTrim[SERVO_COUNT] = {0.0, -15.0, 0.0, 0.0, -15.0, 0.0, 0.0, -15.0};

// startup
const float LOWER_DEG = 0.0;
const unsigned long LOWER_HOLD_MS = 1500;
const float RISE_STEP_DEG = 1.0;
const int   RISE_STEP_MS  = 15;
const int   PAUSE_BETWEEN_SERVOS_MS = 200;

// -----------------------------------------

bool attachedOk[SERVO_COUNT];
const uint32_t TICKS = 1UL << RES_BITS;

unsigned long wiggleStart[SERVO_COUNT];  // millis when this servo's shiver began
bool          wiggling[SERVO_COUNT];
unsigned long nextEventAt = 0;

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

// enveloped oscillation: starts and ends at exactly zero offset, so there
// is no step into or out of the shiver
float wiggleOffset(unsigned long elapsed) {
  float k = (float)elapsed / (float)WIGGLE_MS;      // 0..1
  if (k < 0.0f || k > 1.0f) return 0.0f;
  float envelope = sinf(PI * k);                    // 0 -> 1 -> 0
  return WIGGLE_DEG * envelope * sinf(2.0f * PI * WIGGLE_CYCLES * k);
}

void scheduleNextEvent() {
  nextEventAt = millis() + random(GAP_MIN_MS, GAP_MAX_MS + 1);
}

// grab a random handful of currently-idle servos and start them shivering
void triggerWiggles() {
  int idle[SERVO_COUNT];
  int idleCount = 0;
  for (int i = 0; i < SERVO_COUNT; i++) {
    if (attachedOk[i] && !wiggling[i]) idle[idleCount++] = i;
  }
  if (idleCount == 0) return;

  int want = random(WIGGLE_MIN_SERVOS, WIGGLE_MAX_SERVOS + 1);
  if (want > idleCount) want = idleCount;

  Serial.print("wiggle:");
  for (int n = 0; n < want; n++) {
    int pick = random(0, idleCount);
    int i = idle[pick];
    idle[pick] = idle[--idleCount];   // remove so it isn't picked twice
    wiggling[i] = true;
    wiggleStart[i] = millis();
    Serial.printf(" s%d", i + 1);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.printf("\nTuft 3x3 wiggle, firmware v%s\n", VERSION);
  randomSeed(esp_random());

  for (int i = 0; i < SERVO_COUNT; i++) {
    attachedOk[i] = ledcAttach(SERVO_PINS[i], SERVO_HZ, RES_BITS);
    wiggling[i] = false;
  }
  Serial.print("attached:");
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.printf(" s%d/GPIO%d=%s", i + 1, SERVO_PINS[i],
                  attachedOk[i] ? "OK" : "FAIL");
  }
  Serial.println();

  Serial.printf("rest %.0f deg, wiggle +/-%.1f deg over %dms, "
                "%d-%d tips every %.1f-%.1fs\n",
                REST_DEG, WIGGLE_DEG, WIGGLE_MS,
                WIGGLE_MIN_SERVOS, WIGGLE_MAX_SERVOS,
                GAP_MIN_MS / 1000.0, GAP_MAX_MS / 1000.0);
  Serial.printf("(%.1f deg = %.0fmm at a %.0fmm tip)\n", WIGGLE_DEG,
                SHAFT_TO_TIP_MM * sinf(radians(WIGGLE_DEG)), SHAFT_TO_TIP_MM);

  // startup: down, then up one at a time, then settle at rest
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

  Serial.println("RESTING. Wiggles from here.");
  scheduleNextEvent();
}

void loop() {
  unsigned long now = millis();

  if ((long)(now - nextEventAt) >= 0) {
    triggerWiggles();
    scheduleNextEvent();
  }

  for (int i = 0; i < SERVO_COUNT; i++) {
    float offset = 0.0f;
    if (wiggling[i]) {
      unsigned long elapsed = now - wiggleStart[i];
      if (elapsed >= (unsigned long)WIGGLE_MS) {
        wiggling[i] = false;          // land exactly back on rest
      } else {
        offset = wiggleOffset(elapsed);
      }
    }
    writeDeg(i, REST_DEG + offset);
  }

  delay(1000 / SERVO_HZ);
}
