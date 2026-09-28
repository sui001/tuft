/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  3x3 rig, native LEDC servo driver, ESP32-S3 SuperMini
 *
 * v3.0 drops the ESP32Servo library entirely and drives LEDC directly.
 *
 * WHY. Four separate faults were traced to ESP32Servo over this bring-up,
 * every one of which presented as "the servos are bad":
 *   1. DEFAULT_TIMER_WIDTH is 10, so the 20ms frame had only 1024 steps:
 *      19.5us = 1.85 deg = an 8.1mm jump at a 250mm tip. That was the jerk.
 *   2. attach() resets timer_width back to that 10-bit default, so setting
 *      the width before attach() is silently discarded.
 *   3. setTimerWidth()'s tick rescale divides by the bit difference rather
 *      than 2^difference, so the carried tick value is garbage.
 *   4. Its channel allocator handed servos 7 and 8 slots (ids 110, 111,
 *      outside the working contiguous 100-105 block) that the ESP32-S3
 *      doesn't physically have. Both reported attached() == OK and emitted
 *      no pulse. Confirmed on the bench: exactly servos 7 and 8 dead.
 *
 * Driving LEDC directly is about ten lines and nothing is hidden. The core
 * 3.x API assigns channels itself and shares timers across channels at the
 * same frequency, and ledcAttach() returns a real bool we can trust.
 *
 * RESOLUTION: 50Hz, 14-bit (the S3's hardware max, SOC_LEDC_TIMER_BIT_WIDTH).
 *   20000us / 16384 = 1.22us per step = 0.116 deg = 0.505mm at a 250mm tip.
 *   For reference an RC receiver runs about 1us, so this is the same league.
 *
 * Sequence: LOWER all together -> RAISE servo 1..8 one at a time, alone ->
 * SWAY all together, continuously.
 *
 * WIRING: GPIO 1,3,4,5,6,7,8,10 -> servo 1..8. GPIO 2,9,11 unused.
 */

#define VERSION "3.0"

// ---------------- CONFIG ----------------

const int SERVO_COUNT = 8;
const int SERVO_PINS[SERVO_COUNT] = {1, 3, 4, 5, 6, 7, 8, 10};

const float SHAFT_TO_TIP_MM = 250.0;

const int   SERVO_HZ   = 50;
const int   RES_BITS   = 14;     // ESP32-S3 hardware maximum
const int   MIN_US     = 500;    // pulse at 0 deg
const int   MAX_US     = 2400;   // pulse at 180 deg

const float LOWER_DEG = 0.0;
const unsigned long LOWER_HOLD_MS = 2500;

const float RAISE_TO_DEG  = 75.0;
const float RISE_STEP_DEG = 1.0;   // 1.0 = 4.4mm at the tip; 0.1 glides
const int   RISE_STEP_MS  = 25;
const int   PAUSE_BETWEEN_SERVOS_MS = 400;

const float OSC_LOW_DEG   = 75.0;
const float OSC_HIGH_DEG  = 105.0;
const float SWAY_PERIOD_S = 6.0;

float angleTrim[SERVO_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};

// -----------------------------------------

bool attachedOk[SERVO_COUNT];

// 20ms frame split into 2^RES_BITS ticks; duty is simply the fraction of
// the frame the pulse is high.
const uint32_t TICKS = 1UL << RES_BITS;

uint32_t usToDuty(float us) {
  float periodUs = 1000000.0 / SERVO_HZ;
  return (uint32_t)lroundf(us * (float)TICKS / periodUs);
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
  // one line, not a burst: the S3's USB CDC drops multi-line bursts when
  // the host isn't already reading, which truncated every earlier report.
  Serial.print("attached:");
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.printf(" s%d/GPIO%d=%s", i + 1, SERVO_PINS[i],
                  attachedOk[i] ? "OK" : "FAIL");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.printf("\nTuft 3x3 native LEDC, firmware v%s\n", VERSION);

  for (int i = 0; i < SERVO_COUNT; i++) {
    attachedOk[i] = ledcAttach(SERVO_PINS[i], SERVO_HZ, RES_BITS);
  }
  for (int i = 0; i < SERVO_COUNT; i++) {
    writeDeg(i, LOWER_DEG);
  }

  printStatusLine();

  float usPerTick = (1000000.0 / SERVO_HZ) / (float)TICKS;
  float degPerTick = usPerTick / ((float)(MAX_US - MIN_US) / 180.0);
  Serial.printf("%d-bit: %.3f us/step, %.4f deg/step, %.3f mm at tip\n",
                RES_BITS, usPerTick, degPerTick,
                SHAFT_TO_TIP_MM * sinf(radians(degPerTick)));

  Serial.printf("LOWER: all to %.0f deg, holding %lums\n", LOWER_DEG, LOWER_HOLD_MS);
  delay(LOWER_HOLD_MS);

  Serial.println("RAISE: one at a time, in order.");
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.printf("  >> servo%d GPIO%d\n", i + 1, SERVO_PINS[i]);
    Serial.flush();
    for (float d = LOWER_DEG; d <= RAISE_TO_DEG; d += RISE_STEP_DEG) {
      writeDeg(i, d);
      delay(RISE_STEP_MS);
    }
    writeDeg(i, RAISE_TO_DEG);
    delay(PAUSE_BETWEEN_SERVOS_MS);
  }

  Serial.println("SWAY: all together, continuous.");
}

void loop() {
  const float mid = (OSC_LOW_DEG + OSC_HIGH_DEG) / 2.0;
  const float amp = (OSC_HIGH_DEG - OSC_LOW_DEG) / 2.0;

  float t = millis() / 1000.0;
  float angleDeg = mid + amp * sinf((2.0 * PI / SWAY_PERIOD_S) * t);

  for (int i = 0; i < SERVO_COUNT; i++) {
    writeDeg(i, angleDeg);
  }

  static unsigned long lastReport = 0;
  if (millis() - lastReport > 5000) {
    lastReport = millis();
    printStatusLine();
  }

  delay(1000 / SERVO_HZ);
}
