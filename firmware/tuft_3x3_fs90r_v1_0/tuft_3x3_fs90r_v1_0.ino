/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  Standalone 3x3 ripple rig, single ESP32-S3 SuperMini, no PCA9685
 *
 * Drives 9x FS90R continuous-rotation servos directly off SuperMini GPIOs
 * (no I2C expander, this board is small enough to run straight off the
 * chip's own PWM). Reproduces the "ripple" effect from the Tuft console
 * (https://sui001.github.io/tuft/), reinterpreted for continuous rotation:
 * FS90R has no absolute angle, a pulse width sets speed and direction
 * around a per-unit stop point, so the wave that the console draws as a
 * tilt angle is applied here as a speed command instead.
 *
 * Exported console fields that do NOT carry over and are intentionally
 * ignored: "type" (sg90 in the export, real hardware is fs90r), stickLengthCm,
 * stickThicknessCm, stickOrientationDeg, mountTiltDeg, mountPanDeg. Those are
 * all position/geometry properties for a positional servo, meaningless for a
 * continuous-rotation one. Only the ripple's shape (wavelength, speed,
 * swing-as-amplitude) made it across. The console also never exported a
 * ripple origin (click point), it isn't tracked in the JSON, so this
 * defaults to the centre servo, change RIPPLE_ORIGIN_ROW/COL below to move it.
 *
 * CONFIGURATION:
 *   All tuning below in the CONFIG block. No per-device ID, this is a
 *   single standalone rig, not a mesh node.
 *
 * WIRING (confirmed on the physical rig, 2026-09-09):
 *   GPIO 1, 3, 4, 5, 6, 7, 8, 9, 10  ->  servo 1..9 signal
 *   Servo layout (viewed from the front):
 *     7  8  9   <- top row
 *     4  5  6   <- middle row
 *     1  2  3   <- bottom row
 *   GPIO2 deliberately skipped (SuperMini strapping pin, do not use).
 *   Servo power is a SEPARATE supply from the ESP32's own rail, but its
 *   ground MUST be tied to the ESP32's GND, or the PWM signal has no shared
 *   reference and the servos will jitter or ignore it entirely. If servos
 *   are dead or twitchy on first power-up, check that ground tie first,
 *   before touching any code.
 */

#include <ESP32Servo.h>

#define VERSION "1.0"

// ---------------- CONFIG ----------------

// GPIO per servo, index 0 = servo1 (bottom-left) ... index 8 = servo9 (top-right)
const int SERVO_PINS[9] = {1, 3, 4, 5, 6, 7, 8, 9, 10};

// Grid position per servo index, row 0 = bottom, col 0 = left, matches the
// physical layout Sui described (bottom-left=1, bottom-right=3, top-left=7,
// top-right=9).
const int SERVO_ROW[9] = {0, 0, 0, 1, 1, 1, 2, 2, 2};
const int SERVO_COL[9] = {0, 1, 2, 0, 1, 2, 0, 1, 2};

// Ripple origin cell, defaults to the centre servo (index 4, servo5) since
// the console export doesn't carry a click origin. Change to re-centre.
const float RIPPLE_ORIGIN_ROW = 1.0;
const float RIPPLE_ORIGIN_COL = 1.0;

// From the console export (rig.json), reinterpreted for continuous rotation:
const float WAVELENGTH_CELLS = 2.0;    // was wavelengthCm: 2 -> read as cells, there's
                                        // no real physical spacing to convert against
const float ANIM_SPEED       = 0.9;    // was speedMultiplier 0.3 * the console's internal x3
const float MAX_SPEED_PCT    = 0.12;   // was swingRangeDeg 11 degrees, re-based as
                                        // "12% of full speed" since degrees don't apply here

// Per-servo stop-point trim in microseconds. FS90R units drift from the
// textbook 1500us neutral, sometimes by 20-50us, and drift differently per
// unit. Nudge these individually if a servo creeps at "stop".
int neutralUs[9] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

const int SPEED_RANGE_US = 400;   // full MAX_SPEED_PCT=1.0 deviation from neutral
const int UPDATE_HZ      = 50;    // standard servo refresh rate
const unsigned long ARM_DELAY_MS = 1000; // hold all servos at neutral this long at boot

// -----------------------------------------

Servo servos[9];

void setup() {
  Serial.begin(115200);
  Serial.printf("Tuft 3x3 FS90R ripple rig, firmware v%s\n", VERSION);

  // ESP32Servo needs its PWM timers claimed up front when driving this many
  // channels at once, otherwise later attach() calls can silently fail.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < 9; i++) {
    servos[i].setPeriodHertz(UPDATE_HZ);
    servos[i].attach(SERVO_PINS[i], 1000, 2000);
    servos[i].writeMicroseconds(neutralUs[i]); // hold stop while arming
  }

  Serial.println("Holding neutral, arming...");
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

    float speedFraction = value * MAX_SPEED_PCT; // -MAX_SPEED_PCT .. +MAX_SPEED_PCT
    int pulseUs = neutralUs[i] + (int)(speedFraction * SPEED_RANGE_US);

    servos[i].writeMicroseconds(pulseUs);
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
 *   Library needed: "ESP32Servo" by Kevin Harrington / John K. Bennett,
 *   install via Arduino IDE Library Manager before compiling.
 *
 *   After flashing, reselect the COM port before opening Serial Monitor,
 *   it changes on every flash.
 */
