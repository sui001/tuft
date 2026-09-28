/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  3x3 rig, gentle synchronized sway test, ESP32-S3 SuperMini
 *
 * v1.3: different theory of the jerk, opposite fix from v1.1's.
 *
 * v1.1 sent a new target every 20ms with a tiny (~0.2 degree) delta,
 * assuming the fix was finer signal resolution, it wasn't, still jerky.
 * But 2026-09-09 bench testing found the servos move perfectly smoothly
 * on the fast stand-up-to-90 move, they only chatter on the slow sway.
 * That's the signature of a cheap analog servo's control loop: it drives
 * confidently toward a target with real position error, but dithers/
 * buzzes when the error is tiny, exactly what a continuous slow creep
 * feeds it every 20ms.
 *
 * So this version does the opposite of v1.1: instead of streaming
 * sub-degree corrections constantly, it samples the same sine trajectory
 * only every STEP_INTERVAL_MS and sends ONE confident target each time,
 * a real multi-degree move the servo can execute the way it executes the
 * stand-up move, not a correction it has to dither over. Motion will read
 * as distinct steps rather than one continuous glide, but each step
 * should itself be clean, worth comparing against v1.1/v1.2 by feel.
 *
 * WIRING: GPIO 1,3,4,5,6,7,8,10,11 -> servo 1..9 (servo8/9 remapped off
 * the retired GPIO9, see v1.2's header for why).
 */

#include <ESP32Servo.h>

#define VERSION "1.3"

// ---------------- CONFIG ----------------

const int SERVO_PINS[9] = {1, 3, 4, 5, 6, 7, 8, 10, 11};

const float SHAFT_TO_TIP_CM = 25.0;
const float TIP_SWAY_CM     = 5.0;

const int CENTER_DEG = 90;
const float SWAY_PERIOD_S = 6.0;

const int MIN_US = 500, MAX_US = 2400;

float angleTrim[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

// how far apart in time each confident step is sent. Bigger = fewer, more
// decisive moves (try 250-600ms), smaller drifts back toward v1.1's chatter.
const unsigned long STEP_INTERVAL_MS = 350;

const unsigned long ARM_DELAY_MS = 1000;

// -----------------------------------------

Servo servos[9];
float swingDeg;
unsigned long lastStepMs = 0;

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
    "tip sway target %.1fcm at %.1fcm horn length -> +/-%.2f degrees, "
    "stepped every %lums\n",
    TIP_SWAY_CM, SHAFT_TO_TIP_CM, swingDeg, STEP_INTERVAL_MS
  );

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  Serial.println("Standing up to 90 degrees...");
  for (int i = 0; i < 9; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], MIN_US, MAX_US);
    servos[i].writeMicroseconds(degToUs(CENTER_DEG + angleTrim[i]));
  }

  delay(ARM_DELAY_MS);
  Serial.println("Swaying, stepped.");
}

void loop() {
  if (millis() - lastStepMs < STEP_INTERVAL_MS) return;
  lastStepMs = millis();

  float t = millis() / 1000.0;
  float phase = (2.0 * PI / SWAY_PERIOD_S) * t;
  float angleDeg = CENTER_DEG + swingDeg * sin(phase);

  for (int i = 0; i < 9; i++) {
    servos[i].writeMicroseconds(degToUs(angleDeg + angleTrim[i]));
  }
}
