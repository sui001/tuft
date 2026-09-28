/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  Standalone 3x3 ripple rig, single ESP32-S3 SuperMini, no PCA9685
 *
 * Supersedes tuft_3x3_fs90r_v1_0.ino now that the rig is wired with SG92R
 * positional servos instead of FS90R continuous-rotation ones, same pins,
 * same 3x3 layout, same wiring. This version is simpler and more honest to
 * the console: SG92R holds a real angle, so the ripple wave from
 * https://sui001.github.io/tuft/ is applied directly as a bend angle
 * around a centre position, no speed-command workaround, no per-unit
 * stop-point drift to fight.
 *
 * Exported console fields still NOT used here, and why: stickLengthCm/
 * stickThicknessCm are the console's visual horn geometry, this rig has no
 * physical horn/arm attached, just the servo itself moving. mountPanDeg is
 * the console's second (pan) servo stage, this rig is single-servo per
 * cell, no pan axis exists in hardware. Only swingRangeDeg (amplitude),
 * wavelengthCm and speedMultiplier (the ripple's shape) carry over.
 *
 * As before, the console never exports a ripple origin (no click point in
 * the JSON), so this defaults to the centre servo. Change
 * RIPPLE_ORIGIN_ROW/COL below to move it.
 *
 * CONFIGURATION:
 *   All tuning in the CONFIG block below. Single standalone rig, not a
 *   mesh node, no DEVICE_ID needed.
 *
 * WIRING (unchanged from the FS90R build, same rig, same wiring, 2026-09-09):
 *   GPIO 1, 3, 4, 5, 6, 7, 8, 9, 10  ->  servo 1..9 signal
 *   Servo layout (viewed from the front):
 *     7  8  9   <- top row
 *     4  5  6   <- middle row
 *     1  2  3   <- bottom row
 *   GPIO2 deliberately skipped (SuperMini strapping pin, do not use).
 *   Servo power is a separate supply from the ESP32's own rail, its ground
 *   is tied to the ESP32's GND (confirmed common 2026-09-09).
 */

#include <ESP32Servo.h>

#define VERSION "1.0"

// ---------------- CONFIG ----------------

// GPIO per servo, index 0 = servo1 (bottom-left) ... index 8 = servo9 (top-right)
const int SERVO_PINS[9] = {1, 3, 4, 5, 6, 7, 8, 9, 10};

// Grid position per servo index, row 0 = bottom, col 0 = left.
const int SERVO_ROW[9] = {0, 0, 0, 1, 1, 1, 2, 2, 2};
const int SERVO_COL[9] = {0, 1, 2, 0, 1, 2, 0, 1, 2};

// Ripple origin cell, defaults to the centre servo (index 4, servo5), the
// console export has no click-origin field to read this from.
const float RIPPLE_ORIGIN_ROW = 1.0;
const float RIPPLE_ORIGIN_COL = 1.0;

// Straight from the console export (rig.json):
const float WAVELENGTH_CELLS = 2.0;   // wavelengthCm: 2, read as cells (no real
                                       // physical spacing to convert against)
const float ANIM_SPEED       = 0.9;   // speedMultiplier 0.3 * the console's internal x3
const float SWING_DEG        = 11.0;  // swingRangeDeg: 11, now a real bend angle

const int CENTER_DEG = 90;  // rest position, swing happens either side of this

// Per-servo mounting trim in degrees. Horns rarely sit dead-square across
// 9 units, nudge individually if a servo's "centre" visibly leans.
int angleTrim[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

const int UPDATE_HZ = 50;                 // standard servo refresh rate
const unsigned long ARM_DELAY_MS = 800;   // settle at centre before the wave starts

// -----------------------------------------

Servo servos[9];

void setup() {
  Serial.begin(115200);
  Serial.printf("Tuft 3x3 SG92R ripple rig, firmware v%s\n", VERSION);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < 9; i++) {
    servos[i].setPeriodHertz(UPDATE_HZ);
    servos[i].attach(SERVO_PINS[i], 500, 2400); // SG92R full-range pulse span
    servos[i].write(CENTER_DEG + angleTrim[i]);
  }

  Serial.println("Settling at centre...");
  delay(ARM_DELAY_MS);
  Serial.println("Rippling.");
}

void loop() {
  float t = millis() / 1000.0;

  for (int i = 0; i < 9; i++) {
    float dRow = SERVO_ROW[i] - RIPPLE_ORIGIN_ROW;
    float dCol = SERVO_COL[i] - RIPPLE_ORIGIN_COL;
    float dist = sqrt(dRow * dRow + dCol * dCol);

    // same shape as the console's ripple: sin(distance/wavelength - t*speed)
    float value = sin(dist / WAVELENGTH_CELLS - t * ANIM_SPEED);

    int angle = CENTER_DEG + angleTrim[i] + (int)round(value * SWING_DEG);
    angle = constrain(angle, 0, 180);

    servos[i].write(angle);
  }

  delay(1000 / UPDATE_HZ);
}

/*
 * FLASH INSTRUCTIONS:
 *   Board:            ESP32S3 Dev Module
 *   USB CDC On Boot:  Enabled
 *   Flash Size:       4MB (32Mb)
 *   Partition Scheme: Default 4MB with spiffs
 *   Upload Mode:      UART0 / Hardware CDC
 *   Upload Speed:     921600
 *
 *   Library needed: "ESP32Servo" by Kevin Harrington / John K. Bennett.
 *
 *   After flashing, reselect the COM port before opening Serial Monitor,
 *   it changes on every flash.
 */
