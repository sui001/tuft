/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  3x3 rig diagnostic build, single ESP32-S3 SuperMini, no PCA9685
 *
 * Not the ripple effect, a bring-up check: attach all 9 servos, report over
 * Serial whether each one actually attached (a channel/timer allocation
 * failure attaches silently otherwise, it doesn't throw), then park every
 * servo at 0 degrees and hold, so a suspect unit (servo8 / GPIO9 as of
 * 2026-09-09, wiring already re-checked) can be visually confirmed against
 * what the firmware thinks happened.
 *
 * WIRING: same rig as tuft_3x3_sg92r_v1_0.ino, GPIO 1,3,4,5,6,7,8,9,10 ->
 * servo 1..9, bottom row 1-2-3, top row 7-8-9.
 */

#include <ESP32Servo.h>

#define VERSION "1.0"

const int SERVO_PINS[9] = {1, 3, 4, 5, 6, 7, 8, 9, 10};
Servo servos[9];

void setup() {
  Serial.begin(115200);
  delay(1500); // give the USB CDC serial time to enumerate before we print
  Serial.printf("\nTuft 3x3 diagnostic, firmware v%s\n", VERSION);
  Serial.println("Attaching all 9 servos...");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < 9; i++) {
    servos[i].setPeriodHertz(50);
    int ch = servos[i].attach(SERVO_PINS[i], 500, 2400);
    Serial.printf(
      "  servo%d  pin=GPIO%-2d  attach()=%-3d  attached()=%s\n",
      i + 1, SERVO_PINS[i], ch, servos[i].attached() ? "YES" : "NO"
    );
    Serial.flush();
    delay(80); // earlier run dropped mid-burst lines over USB CDC, space them out
  }

  Serial.println("Parking all servos at 0 degrees and holding.");
  for (int i = 0; i < 9; i++) {
    servos[i].write(0);
  }
}

void loop() {
  static unsigned long lastBeat = 0;
  if (millis() - lastBeat > 3000) {
    lastBeat = millis();
    Serial.println("holding at 0...");
    for (int i = 0; i < 9; i++) {
      servos[i].write(0); // re-assert every beat in case anything glitched
    }
  }
}
