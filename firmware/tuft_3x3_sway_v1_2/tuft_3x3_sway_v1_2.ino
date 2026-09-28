/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  3x3 rig, gentle synchronized sway test, ESP32-S3 SuperMini
 *
 * v1.2: pin remap only, motion code unchanged from v1.1. GPIO9 (servo8)
 * was confirmed faulty on 2026-09-09 (wiring tested good, diagnostic sketch
 * showed everything else attaching fine, servo itself dead). Retiring
 * GPIO9 rather than trusting it, shifted the last two servos one slot
 * along the pin pool instead:
 *   servo8: GPIO9  -> GPIO10
 *   servo9: GPIO10 -> GPIO11
 * GPIO11 is safe on the SuperMini (it's in the confirmed-usable GPIO1-13
 * pool, just unused until now, it doubles as native FSPI MOSI but nothing
 * here uses SPI so that's not a conflict).
 *
 * On the persistent jerkiness (v1.0/v1.1, still visible after switching to
 * float-precision microsecond writes and after ruling out power sag, 5V
 * 40A supply confirmed on 2026-09-09): the "10-11 discrete jabs" across
 * the sway most likely means the SG92R's own internal position resolution
 * is the bottleneck now, not the PWM signal, cheap analog servos commonly
 * can't track a slow continuously-moving target smoothly, their pot +
 * comparator loop only resolves roughly 1 degree steps. Software is
 * already sending sub-degree precision, there isn't more headroom here,
 * this would need finer-resolution/digital servos to actually fix.
 *
 * WIRING: GPIO 1,3,4,5,6,7,8,10,11 -> servo 1..9. GPIO2 and now GPIO9
 * deliberately unused.
 */

#include <ESP32Servo.h>

#define VERSION "1.2"

// ---------------- CONFIG ----------------

const int SERVO_PINS[9] = {1, 3, 4, 5, 6, 7, 8, 10, 11}; // servo8->GPIO10, servo9->GPIO11

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
  float angleDeg = CENTER_DEG + swingDeg * sin(phase);

  for (int i = 0; i < 9; i++) {
    servos[i].writeMicroseconds(degToUs(angleDeg + angleTrim[i]));
  }

  delay(1000 / UPDATE_HZ);
}
