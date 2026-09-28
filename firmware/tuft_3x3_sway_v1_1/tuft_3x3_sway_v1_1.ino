/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  3x3 rig, gentle synchronized sway test, ESP32-S3 SuperMini
 *
 * v1.1: fixes visible jerkiness in v1.0. That version computed the target
 * as an integer degree (`int angle = ...round(...)`) before handing it to
 * Servo::write(), so across a small +/-11.5 degree sway at 50Hz the motion
 * was really a staircase of whole-degree steps, not a curve. This version
 * keeps the angle as a float all the way through and writes microseconds
 * directly (writeMicroseconds instead of write), which gives roughly 10x
 * finer resolution (~10.5us per degree over the 500-2400us range) than
 * quantizing to whole degrees first. Motion should read as continuous now.
 *
 * If it's still visibly jerky after this, the likely cause has moved from
 * software to power: syncing all 8 servos to reverse direction at the same
 * instant means they all demand peak current at the same instant too, a
 * supply that sags under that (voltage dip, not enough smoothing) will
 * show up as a stutter right at the sway's direction changes specifically,
 * worth checking with a meter on the servo rail if the jerk lines up with
 * the turnaround points rather than happening throughout the sway.
 *
 * WIRING: same rig as tuft_3x3_sg92r_v1_0.ino, GPIO 1,3,4,5,6,7,8,9,10 ->
 * servo 1..9. Servo8 (GPIO9) confirmed dead 2026-09-09, still wired here.
 */

#include <ESP32Servo.h>

#define VERSION "1.1"

// ---------------- CONFIG ----------------

const int SERVO_PINS[9] = {1, 3, 4, 5, 6, 7, 8, 9, 10};

const float SHAFT_TO_TIP_CM = 25.0;
const float TIP_SWAY_CM     = 5.0;

const int CENTER_DEG = 90;
const float SWAY_PERIOD_S = 6.0;

const int MIN_US = 500, MAX_US = 2400; // must match attach() range below

float angleTrim[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // per-servo mounting trim, degrees

const int UPDATE_HZ = 50;
const unsigned long ARM_DELAY_MS = 1000;

// -----------------------------------------

Servo servos[9];
float swingDeg;

int degToUs(float deg) {
  deg = constrain(deg, 0.0, 180.0);
  return (int)round(MIN_US + (deg / 180.0) * (MAX_US - MIN_US));
}

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
    servos[i].attach(SERVO_PINS[i], MIN_US, MAX_US);
    servos[i].writeMicroseconds(degToUs(CENTER_DEG + angleTrim[i]));
  }

  delay(ARM_DELAY_MS);
  Serial.println("Swaying.");
}

void loop() {
  float t = millis() / 1000.0;
  float phase = (2.0 * PI / SWAY_PERIOD_S) * t;
  float angleDeg = CENTER_DEG + swingDeg * sin(phase); // stays float, never rounded early

  for (int i = 0; i < 9; i++) {
    servos[i].writeMicroseconds(degToUs(angleDeg + angleTrim[i]));
  }

  delay(1000 / UPDATE_HZ);
}
