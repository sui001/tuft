/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  3x3 rig, gentle synchronized sway test, ESP32-S3 SuperMini
 *
 * v1.4: new hypothesis, third attempt at the jerk.
 *
 *   v1.1: continuous 50Hz updates, sub-degree deltas -> chattery jabs.
 *   v1.3: infrequent (350ms) big jumps -> clean but visibly "8 chunky steps".
 *
 * v1.1's sine wave has near-zero velocity right at each extreme of the
 * sway, the target barely moves for a stretch of time exactly when it's
 * sitting there, that's the condition (tiny residual error, held for a
 * while) that lets an analog servo's control loop start dithering. This
 * version keeps v1.1's continuous 50Hz streaming (so it CAN look fluid,
 * no visible discrete steps) but swaps the sine for a triangle wave:
 * constant velocity the whole traverse, instant direction reversal at the
 * ends, never lingering near-static. If the chatter was really about
 * dwell-time-near-target rather than raw per-step delta size, this should
 * look meaningfully different from both v1.1 and v1.3. If it's still
 * chattery, that's a real signal we're at this servo class's genuine
 * limit for slow, small-amplitude motion, not a code problem left to solve.
 *
 * WIRING: GPIO 1,3,4,5,6,7,8,10,11 -> servo 1..9 (servo8/9 remapped off
 * the retired GPIO9, see v1.2's header for why).
 */

#include <ESP32Servo.h>

#define VERSION "1.4"

// ---------------- CONFIG ----------------

const int SERVO_PINS[9] = {1, 3, 4, 5, 6, 7, 8, 10, 11};

const float SHAFT_TO_TIP_CM = 25.0;
const float TIP_SWAY_CM     = 5.0;

const int CENTER_DEG = 90;
const float SWAY_PERIOD_S = 6.0;

const int MIN_US = 500, MAX_US = 2400;

float angleTrim[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

const int UPDATE_HZ = 50;
const unsigned long ARM_DELAY_MS = 1000;

// -----------------------------------------

Servo servos[9];
float swingDeg;

int degToUs(float deg) {
  deg = constrain(deg, 0.0, 180.0);
  return (int)round(MIN_US + (deg / 180.0) * (MAX_US - MIN_US));
}

// -1..+1 triangle wave, constant slope, instant reversal at the peaks,
// unlike sin() this never slows down approaching +/-1.
float triangleWave(float t, float periodS) {
  float x = t / periodS;
  x = x - floor(x); // fractional part, 0..1
  return (x < 0.5) ? (-1.0 + 4.0 * x) : (3.0 - 4.0 * x);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  swingDeg = degrees(asin(TIP_SWAY_CM / SHAFT_TO_TIP_CM));
  Serial.printf("Tuft 3x3 sway test, firmware v%s (triangle wave)\n", VERSION);
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
  Serial.println("Swaying, triangle wave.");
}

void loop() {
  float t = millis() / 1000.0;
  float angleDeg = CENTER_DEG + swingDeg * triangleWave(t, SWAY_PERIOD_S);

  for (int i = 0; i < 9; i++) {
    servos[i].writeMicroseconds(degToUs(angleDeg + angleTrim[i]));
  }

  delay(1000 / UPDATE_HZ);
}
