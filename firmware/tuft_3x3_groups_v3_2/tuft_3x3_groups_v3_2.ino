/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  3x3 rig, three speed groups, wide sway, ESP32-S3 SuperMini
 *
 * v3.2, on v3.0's native LEDC driver (ESP32Servo dropped, see that file's
 * header for the four faults that led to it).
 *
 * Changes from v3.1:
 *   - sway widened to 60-120 degrees for every group (was 75-105). At a
 *     250mm horn that is +/-125mm, a quarter-metre of travel.
 *   - servos 2, 5, 8 trimmed by -15 degrees. On the bench they swayed
 *     90-120 while everything else did 75-105, i.e. sitting 15 degrees
 *     high. Nothing in firmware offsets a group, so that is mechanical:
 *     a 25-tooth spline is 14.4 degrees per tooth, so those three horns
 *     are seated one tooth off. Corrected in angleTrim rather than by
 *     re-seating; zero those entries if the read was wrong.
 *   - startup reset + sequential raise kept as-is, it was working.
 *
 * Groups are the columns of the 3x3, which falls out as index % 3:
 *
 *     7  8  9        col0 = 1,4,7   slow  (12s)
 *     4  5  6        col1 = 2,5,8   mid   (5s)
 *     1  2  3        col2 = 3,6     fast  (2s)   servo9 doesn't exist,
 *                                                 the S3 has 8 LEDC channels
 *
 * CONFIRMED ON THE BENCH: the fast column reads visibly smoother than the
 * slow one at identical amplitude and resolution. That is the servo's
 * deadband, the last real limit here. Faster motion means a larger
 * commanded change between updates, so more of them clear the servo's
 * threshold rather than accumulating into a visible catch-up. It is bought
 * with speed, not with code; digital servos with a 1-2us deadband are the
 * only way to buy it outright.
 *
 * WATCH AT THIS WIDER RANGE: the fast group now peaks near 94 deg/s. If it
 * lags or overshoots at the turnarounds it is running out of torque against
 * the inertia of a 250mm arm, in which case slow that group or narrow it.
 *
 * WIRING: GPIO 1,3,4,5,6,7,8,10 -> servo 1..8. GPIO 2,9,11 unused.
 */

#define VERSION "3.2"

// ---------------- CONFIG ----------------

const int SERVO_COUNT = 8;
const int SERVO_PINS[SERVO_COUNT] = {1, 3, 4, 5, 6, 7, 8, 10};

const float SHAFT_TO_TIP_MM = 250.0;

const int   SERVO_HZ = 50;
const int   RES_BITS = 14;    // ESP32-S3 hardware maximum
const int   MIN_US   = 500;
const int   MAX_US   = 2400;

// sway extent, shared by every group so only speed varies
const float OSC_LOW_DEG  = 60.0;
const float OSC_HIGH_DEG = 120.0;

// seconds per full back-and-forth, per group
const int   GROUP_COUNT = 3;
const float GROUP_PERIOD_S[GROUP_COUNT] = {
  12.0,  // group 0, servos 1,4,7 - super slow
  5.0,   // group 1, servos 2,5,8 - mid
  2.0    // group 2, servos 3,6   - fast
};
const char *GROUP_NAME[GROUP_COUNT] = {"slow", "mid", "fast"};

// Per-servo mechanical trim, degrees. servos 2,5,8 (indices 1,4,7) sat 15
// degrees high on the bench, one spline tooth. Set to 0 to disable.
float angleTrim[SERVO_COUNT] = {
   0.0,   // servo1
 -15.0,   // servo2  one tooth high
   0.0,   // servo3
   0.0,   // servo4
 -15.0,   // servo5  one tooth high
   0.0,   // servo6
   0.0,   // servo7
 -15.0    // servo8  one tooth high
};

// startup sequence
const float LOWER_DEG = 0.0;
const unsigned long LOWER_HOLD_MS = 2000;
const float RAISE_TO_DEG  = 75.0;
const float RISE_STEP_DEG = 1.0;
const int   RISE_STEP_MS  = 15;
const int   PAUSE_BETWEEN_SERVOS_MS = 200;

// -----------------------------------------

bool attachedOk[SERVO_COUNT];
const uint32_t TICKS = 1UL << RES_BITS;

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

  Serial.printf("sway %.0f-%.0f deg = +/-%.0fmm at a %.0fmm tip\n",
                OSC_LOW_DEG, OSC_HIGH_DEG,
                SHAFT_TO_TIP_MM * sinf(radians((OSC_HIGH_DEG - OSC_LOW_DEG) / 2.0)),
                SHAFT_TO_TIP_MM);
  for (int g = 0; g < GROUP_COUNT; g++) {
    Serial.printf("group %d (%s): %.1fs per cycle\n", g, GROUP_NAME[g],
                  GROUP_PERIOD_S[g]);
  }
  Serial.printf("trim: servos 2,5,8 at %.1f deg (one spline tooth)\n", angleTrim[1]);

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
