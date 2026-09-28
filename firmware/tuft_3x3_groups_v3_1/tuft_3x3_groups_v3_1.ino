/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  3x3 rig, three speed groups, ESP32-S3 SuperMini
 *
 * v3.1, built on v3.0's native LEDC driver (ESP32Servo dropped, see that
 * file's header for the four faults that led to it).
 *
 * Three groups sway at the same amplitude but different speeds:
 *   group 0: servos 1,4,7  super slow
 *   group 1: servos 2,5,8  mid
 *   group 2: servos 3,6    fast   (servo 9 doesn't exist, the S3's LEDC
 *                                  has only 8 channels)
 *
 * Those groups are the COLUMNS of the grid, which falls out as index % 3:
 *
 *     7  8  9        col0 = 1,4,7   slow
 *     4  5  6        col1 = 2,5,8   mid
 *     1  2  3        col2 = 3,6     fast
 *
 * so it reads left-to-right as a speed gradient.
 *
 * The groups run free against a shared clock rather than being phase-locked,
 * so they start aligned and drift apart, periodically falling back into and
 * out of sync. That drift is the point, it stops the rig reading as one
 * machine doing one thing.
 *
 * WORTH WATCHING: if the fast column looks smoother than the slow one at
 * identical amplitude, that is the servo deadband showing itself. Faster
 * motion means a bigger commanded change between updates, so more of them
 * clear the servo's threshold instead of accumulating into a visible catch-up.
 * Same servos, same resolution, only the speed differs, which makes this a
 * clean test of what's left after the firmware faults were fixed.
 *
 * WIRING: GPIO 1,3,4,5,6,7,8,10 -> servo 1..8. GPIO 2,9,11 unused.
 */

#define VERSION "3.1"

// ---------------- CONFIG ----------------

const int SERVO_COUNT = 8;
const int SERVO_PINS[SERVO_COUNT] = {1, 3, 4, 5, 6, 7, 8, 10};

const float SHAFT_TO_TIP_MM = 250.0;

const int   SERVO_HZ = 50;
const int   RES_BITS = 14;    // ESP32-S3 hardware maximum
const int   MIN_US   = 500;
const int   MAX_US   = 2400;

// sway extent, shared by every group so only speed varies
const float OSC_LOW_DEG  = 75.0;
const float OSC_HIGH_DEG = 105.0;

// seconds per full back-and-forth, per group
const int   GROUP_COUNT = 3;
const float GROUP_PERIOD_S[GROUP_COUNT] = {
  12.0,  // group 0, servos 1,4,7 - super slow
  5.0,   // group 1, servos 2,5,8 - mid
  2.0    // group 2, servos 3,6   - fast
};
const char *GROUP_NAME[GROUP_COUNT] = {"slow", "mid", "fast"};

// startup sequence
const float LOWER_DEG = 0.0;
const unsigned long LOWER_HOLD_MS = 2000;
const float RAISE_TO_DEG  = 75.0;
const float RISE_STEP_DEG = 1.0;
const int   RISE_STEP_MS  = 15;
const int   PAUSE_BETWEEN_SERVOS_MS = 200;

float angleTrim[SERVO_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};

// -----------------------------------------

bool attachedOk[SERVO_COUNT];
const uint32_t TICKS = 1UL << RES_BITS;

// column of the 3x3, and therefore the speed group
int groupOf(int i) { return i % GROUP_COUNT; }

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

void printStatusLine() {
  // one line, never a burst: the S3's USB CDC drops multi-line bursts
  Serial.print("attached:");
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.printf(" s%d/GPIO%d/%s=%s", i + 1, SERVO_PINS[i],
                  GROUP_NAME[groupOf(i)], attachedOk[i] ? "OK" : "FAIL");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.printf("\nTuft 3x3 speed groups, firmware v%s\n", VERSION);

  for (int i = 0; i < SERVO_COUNT; i++) {
    attachedOk[i] = ledcAttach(SERVO_PINS[i], SERVO_HZ, RES_BITS);
  }
  for (int i = 0; i < SERVO_COUNT; i++) {
    writeDeg(i, LOWER_DEG);
  }
  printStatusLine();

  for (int g = 0; g < GROUP_COUNT; g++) {
    Serial.printf("group %d (%s): %.1fs per cycle\n", g, GROUP_NAME[g],
                  GROUP_PERIOD_S[g]);
  }

  Serial.printf("LOWER: all to %.0f deg, holding %lums\n", LOWER_DEG, LOWER_HOLD_MS);
  delay(LOWER_HOLD_MS);

  Serial.println("RAISE: one at a time, in order.");
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.printf("  >> servo%d GPIO%d (%s)\n", i + 1, SERVO_PINS[i],
                  GROUP_NAME[groupOf(i)]);
    Serial.flush();
    for (float d = LOWER_DEG; d <= RAISE_TO_DEG; d += RISE_STEP_DEG) {
      writeDeg(i, d);
      delay(RISE_STEP_MS);
    }
    writeDeg(i, RAISE_TO_DEG);
    delay(PAUSE_BETWEEN_SERVOS_MS);
  }

  Serial.println("SWAY: three speed groups, free-running.");
}

void loop() {
  const float mid = (OSC_LOW_DEG + OSC_HIGH_DEG) / 2.0;
  const float amp = (OSC_HIGH_DEG - OSC_LOW_DEG) / 2.0;

  float t = millis() / 1000.0;

  // one angle per group, then fan it out to that group's servos
  float groupAngle[GROUP_COUNT];
  for (int g = 0; g < GROUP_COUNT; g++) {
    groupAngle[g] = mid + amp * sinf((2.0f * PI / GROUP_PERIOD_S[g]) * t);
  }

  for (int i = 0; i < SERVO_COUNT; i++) {
    writeDeg(i, groupAngle[groupOf(i)]);
  }

  static unsigned long lastReport = 0;
  if (millis() - lastReport > 8000) {
    lastReport = millis();
    printStatusLine();
  }

  delay(1000 / SERVO_HZ);
}
