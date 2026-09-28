/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  3x3 rig, gentle synchronized sway test, ESP32-S3 SuperMini
 *
 * Not the ripple effect, a calibration/feel check after re-seating the
 * horns: stand every servo up to 90 degrees (same rest position as
 * tuft_3x3_sg92r_v1_0.ino), hold, then sway all of them together, smoothly
 * and slowly, no per-servo phase offset, no wave travelling across the
 * grid, just everything moving as one.
 *
 * The swing amount is specified as a physical tip displacement, not an
 * angle: horn is 250mm shaft-to-tip, and the ask was "sway the tip about
 * 5cm either side". Converted geometrically: for a horn of length L
 * swinging through angle theta from centre, the tip's lateral displacement
 * is L*sin(theta), so theta = asin(displacement / L). At L=25cm and a
 * displacement of 5cm that's asin(5/25) = ~11.5 degrees, worked out once
 * in setup() rather than hardcoded, so changing TIP_SWAY_CM or
 * SHAFT_TO_TIP_CM keeps it correct.
 *
 * WIRING: same rig as tuft_3x3_sg92r_v1_0.ino, GPIO 1,3,4,5,6,7,8,9,10 ->
 * servo 1..9. Servo8 (GPIO9) was confirmed dead on the bench 2026-09-09
 * (wiring tested good, servo itself faulty), still wired here so it just
 * picks up commands again once swapped.
 */

#include <ESP32Servo.h>

#define VERSION "1.0"

// ---------------- CONFIG ----------------

const int SERVO_PINS[9] = {1, 3, 4, 5, 6, 7, 8, 9, 10};

const float SHAFT_TO_TIP_CM = 25.0;  // horn length, shaft to tip
const float TIP_SWAY_CM     = 5.0;   // how far the tip should sway either side

const int CENTER_DEG = 90;
const float SWAY_PERIOD_S = 6.0;     // seconds for one full sway cycle, slow and gentle

int angleTrim[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // per-servo mounting trim, degrees

const int UPDATE_HZ = 50;
const unsigned long ARM_DELAY_MS = 1000; // stand up and hold before swaying

// -----------------------------------------

Servo servos[9];
float swingDeg;

void setup() {
  Serial.begin(115200);
  delay(1500);

  swingDeg = degrees(asin(TIP_SWAY_CM / SHAFT_TO_TIP_CM));
  Serial.printf("Tuft 3x3 sway test, firmware v%s\n", VERSION);
  Serial.printf(
    "tip sway target %.1fcm at %.1fcm horn length -> +/-%.2f degrees\n",
    TIP_SWAY_CM, SHAFT_TO_TIP_CM, swingDeg
  );

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  Serial.println("Standing up to 90 degrees...");
  for (int i = 0; i < 9; i++) {
    servos[i].setPeriodHertz(UPDATE_HZ);
    servos[i].attach(SERVO_PINS[i], 500, 2400);
    servos[i].write(CENTER_DEG + angleTrim[i]);
  }

  delay(ARM_DELAY_MS);
  Serial.println("Swaying.");
}

void loop() {
  float t = millis() / 1000.0;
  float phase = (2.0 * PI / SWAY_PERIOD_S) * t;
  int angle = CENTER_DEG + (int)round(swingDeg * sin(phase));

  for (int i = 0; i < 9; i++) {
    servos[i].write(constrain(angle + angleTrim[i], 0, 180));
  }

  delay(1000 / UPDATE_HZ);
}
