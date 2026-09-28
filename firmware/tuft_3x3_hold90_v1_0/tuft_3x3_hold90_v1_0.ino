/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  3x3 rig, hold at 90 for assembly, ESP32-S3 SuperMini
 *
 * A utility, not an effect. Every servo is parked at exactly 90 degrees and
 * held there so the horns can be fitted square. Nothing moves, ever.
 *
 * NO TRIM IS APPLIED HERE, deliberately. angleTrim normally carries -15 deg
 * on servos 2,5,8 to compensate for horns that were seated one spline tooth
 * high. During fitting that correction is exactly wrong: it would put those
 * three shafts at a different true angle from the rest, and the old offset
 * would get built straight back in. With trim off, all eight shafts sit at
 * the same real position, so horns fitted to match each other are actually
 * matched, and the trim can be zeroed in the effect sketches afterwards.
 *
 * FITTING NOTE: these are nylon-gear servos and they are actively holding
 * position. Do not twist an arm against that holding torque to line it up,
 * it strips the gear train. If a horn lands at the wrong angle, pull it off
 * the spline and re-seat it, don't force it round. A 25-tooth spline steps
 * in 14.4 degree increments, so the closest you can get by re-seating alone
 * is within about 7 degrees; the remainder is what angleTrim is for.
 *
 * Position is re-asserted every second, so a glitch or brownout can't leave
 * a servo parked somewhere unexpected mid-assembly.
 *
 * WIRING: GPIO 1,3,4,5,6,7,8,10 -> servo 1..8. GPIO 2,9,11 unused.
 */

#define VERSION "1.0"

// ---------------- CONFIG ----------------

const int SERVO_COUNT = 8;
const int SERVO_PINS[SERVO_COUNT] = {1, 3, 4, 5, 6, 7, 8, 10};

const float HOLD_DEG = 90.0;   // true shaft angle, no trim

const int SERVO_HZ = 50;
const int RES_BITS = 14;
const int MIN_US   = 500;
const int MAX_US   = 2400;

// -----------------------------------------

bool attachedOk[SERVO_COUNT];
const uint32_t TICKS = 1UL << RES_BITS;

uint32_t usToDuty(float us) {
  return (uint32_t)lroundf(us * (float)TICKS / (1000000.0f / SERVO_HZ));
}

float degToUs(float deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  return MIN_US + (deg / 180.0f) * (MAX_US - MIN_US);
}

void holdAll() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    if (attachedOk[i]) ledcWrite(SERVO_PINS[i], usToDuty(degToUs(HOLD_DEG)));
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.printf("\nTuft 3x3 assembly hold, firmware v%s\n", VERSION);

  for (int i = 0; i < SERVO_COUNT; i++) {
    attachedOk[i] = ledcAttach(SERVO_PINS[i], SERVO_HZ, RES_BITS);
  }
  holdAll();

  Serial.print("attached:");
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.printf(" s%d/GPIO%d=%s", i + 1, SERVO_PINS[i],
                  attachedOk[i] ? "OK" : "FAIL");
  }
  Serial.println();
  Serial.printf("HOLDING all servos at %.0f deg, NO trim applied.\n", HOLD_DEG);
  Serial.println("Fit horns square. Do not force an arm against the servo,");
  Serial.println("pull it off the spline and re-seat it instead.");
}

void loop() {
  holdAll();          // re-assert, so a glitch can't move anything mid-fit
  delay(1000);
}
