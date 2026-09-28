/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  3x3 rig, 50Hz vs 100Hz A/B test, ESP32-S3 SuperMini
 *
 * v3.3, on v3.0's native LEDC driver. Same three speed groups and same
 * 60-120 sway as v3.2, but the servo frame rate alternates between 50Hz
 * and 100Hz every PHASE_MS so the two can be compared back to back rather
 * than from memory.
 *
 * WHY THIS IS WORTH TESTING TWICE OVER:
 *
 * 1. Resolution. The S3's LEDC timer is 14-bit and that is a hardware
 *    register width, so it cannot be raised. But the tick size is the
 *    FRAME divided by those 16384 steps, so shortening the frame makes
 *    each step finer:
 *       50Hz : 20000us / 16384 = 1.221us = 0.116 deg = 0.505mm at tip
 *      100Hz : 10000us / 16384 = 0.610us = 0.058 deg = 0.253mm at tip
 *    Raising the rate is the only remaining way to buy resolution here.
 *
 * 2. Whether these servos tolerate 100Hz at all. That answer carries
 *    straight over to the PCA9685 that's on order: it is 12-bit, which at
 *    50Hz is 4.88us (2.0mm at the tip, ~4x coarser than this rig is
 *    running now). Its resolution scales with frame rate the same way, so
 *    if the SG92Rs are happy at 100Hz the PCA9685 can be run there too and
 *    lands near 1mm instead of 2mm.
 *
 * WATCH FOR: buzzing, humming, or the servos getting warm during the 100Hz
 * phase. Analog servos are specified for 50Hz; many tolerate 100Hz fine but
 * some overheat or hunt. If that happens, this answers the question, stop
 * the test and stay at 50Hz. If 100Hz is clean, try 200 in TEST_HZ below,
 * though that is beyond what most analog servos will accept.
 *
 * WIRING: GPIO 1,3,4,5,6,7,8,10 -> servo 1..8. GPIO 2,9,11 unused.
 */

#define VERSION "3.3"

// ---------------- CONFIG ----------------

const int SERVO_COUNT = 8;
const int SERVO_PINS[SERVO_COUNT] = {1, 3, 4, 5, 6, 7, 8, 10};

const float SHAFT_TO_TIP_MM = 250.0;

const int RES_BITS = 14;   // ESP32-S3 hardware maximum, cannot be raised
const int MIN_US   = 500;
const int MAX_US   = 2400;

// the A/B: frame rates to cycle through, and how long to hold each
const int TEST_HZ[] = {50, 100};
const int TEST_HZ_COUNT = sizeof(TEST_HZ) / sizeof(TEST_HZ[0]);
const unsigned long PHASE_MS = 15000;

const float OSC_LOW_DEG  = 60.0;
const float OSC_HIGH_DEG = 120.0;

const int   GROUP_COUNT = 3;
const float GROUP_PERIOD_S[GROUP_COUNT] = {12.0, 5.0, 2.0};
const char *GROUP_NAME[GROUP_COUNT] = {"slow", "mid", "fast"};

// servos 2,5,8 sit one spline tooth (14.4 deg) high, see v3.2
float angleTrim[SERVO_COUNT] = {0.0, -15.0, 0.0, 0.0, -15.0, 0.0, 0.0, -15.0};

const float LOWER_DEG = 0.0;
const unsigned long LOWER_HOLD_MS = 2000;
const float RAISE_TO_DEG  = 75.0;
const float RISE_STEP_DEG = 1.0;
const int   RISE_STEP_MS  = 15;
const int   PAUSE_BETWEEN_SERVOS_MS = 200;

// -----------------------------------------

bool attachedOk[SERVO_COUNT];
const uint32_t TICKS = 1UL << RES_BITS;

int currentHz = TEST_HZ[0];
int phaseIdx = 0;

int groupOf(int i) { return i % GROUP_COUNT; }

// duty is the fraction of the CURRENT frame the pulse is high, so this has
// to follow currentHz, not a fixed 50.
uint32_t usToDuty(float us) {
  return (uint32_t)lroundf(us * (float)TICKS / (1000000.0f / currentHz));
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

void reportPhase() {
  float usPerTick = (1000000.0f / currentHz) / (float)TICKS;
  float degPerTick = usPerTick / ((float)(MAX_US - MIN_US) / 180.0f);
  Serial.printf("PHASE %dHz: %.3f us/step, %.4f deg/step, %.3f mm at tip\n",
                currentHz, usPerTick, degPerTick,
                SHAFT_TO_TIP_MM * sinf(radians(degPerTick)));
}

void applyFrequency(int hz) {
  currentHz = hz;
  for (int i = 0; i < SERVO_COUNT; i++) {
    if (attachedOk[i]) ledcChangeFrequency(SERVO_PINS[i], currentHz, RES_BITS);
  }
  reportPhase();
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.printf("\nTuft 3x3 frame-rate A/B, firmware v%s\n", VERSION);

  for (int i = 0; i < SERVO_COUNT; i++) {
    attachedOk[i] = ledcAttach(SERVO_PINS[i], currentHz, RES_BITS);
  }
  for (int i = 0; i < SERVO_COUNT; i++) writeDeg(i, LOWER_DEG);

  Serial.print("attached:");
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.printf(" s%d/GPIO%d=%s", i + 1, SERVO_PINS[i],
                  attachedOk[i] ? "OK" : "FAIL");
  }
  Serial.println();
  reportPhase();

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

  Serial.printf("SWAY: alternating %dHz / %dHz every %lus. "
                "Listen for buzzing and check for heat at the higher rate.\n",
                TEST_HZ[0], TEST_HZ[1], PHASE_MS / 1000);
}

void loop() {
  const float mid = (OSC_LOW_DEG + OSC_HIGH_DEG) / 2.0;
  const float amp = (OSC_HIGH_DEG - OSC_LOW_DEG) / 2.0;

  // swap frame rate on schedule
  static unsigned long lastPhase = 0;
  if (millis() - lastPhase > PHASE_MS) {
    lastPhase = millis();
    phaseIdx = (phaseIdx + 1) % TEST_HZ_COUNT;
    applyFrequency(TEST_HZ[phaseIdx]);
  }

  float t = millis() / 1000.0;

  float groupAngle[GROUP_COUNT];
  for (int g = 0; g < GROUP_COUNT; g++) {
    groupAngle[g] = mid + amp * sinf((2.0f * PI / GROUP_PERIOD_S[g]) * t);
  }
  for (int i = 0; i < SERVO_COUNT; i++) {
    writeDeg(i, groupAngle[groupOf(i)]);
  }

  delay(1000 / currentHz);
}
