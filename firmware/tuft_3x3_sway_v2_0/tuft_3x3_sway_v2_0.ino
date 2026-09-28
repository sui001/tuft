/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  3x3 rig sway, ESP32-S3 SuperMini, direct GPIO (no PCA9685)
 *
 * v2.0 fixes the two real faults. Both were in this firmware, neither was
 * the servos, and v1.1 through v1.4 all chased the wrong thing.
 *
 * FAULT 1, the jerk: ESP32Servo defaults to a 10-bit LEDC timer
 * (ESP32Servo.h, DEFAULT_TIMER_WIDTH 10). That splits the 20ms servo frame
 * into 1024 ticks = 19.5us per achievable pulse width. At 10.56us per
 * degree that is a 1.85 degree minimum step, which on a 250mm horn is an
 * 8.1mm jump at the tip. The sway is 50mm either side, so the whole motion
 * only had about 12 reachable positions, hence the counted "10-11 jabs".
 * Every earlier version computed lovely sub-degree floats and then handed
 * them to a peripheral that rounded them to the nearest 8mm.
 *   Fix: setTimerWidth(14), the ESP32-S3's actual hardware maximum
 *   (SOC_LEDC_TIMER_BIT_WIDTH = 14; note ESP32Servo.h's MAXIMUM_TIMER_WIDTH
 *   of 20 is wrong for this chip and would clamp past the hardware limit).
 *   14-bit gives 1.22us per step, ~0.116 degrees, ~0.5mm at the tip.
 *   That is RC-receiver territory, which is the answer to "how does RC gear
 *   move servos so smoothly": dedicated timers at roughly 1us resolution.
 *
 *   ORDERING TRAP: attach() resets timer_width back to the 10-bit default
 *   (ESP32Servo.cpp lines 94-99), so setTimerWidth MUST be called AFTER
 *   attach(), not before, or it is silently discarded. Also always write a
 *   fresh value afterwards: setTimerWidth's internal tick rescale
 *   (lines 234-243) divides by the bit difference instead of 2^difference,
 *   so the carried-over tick value is garbage until overwritten.
 *
 * FAULT 2, the wandering dead servo: the ESP32-S3's LEDC has 8 channels
 * (SOC_LEDC_CHANNEL_NUM = 8), not the 16 the classic ESP32 has. Driving 9
 * servos means one can never get a channel, and which one loses depends on
 * allocation order, which is why it was GPIO9 one day and GPIO10 the next,
 * and why moving that servo to another pin made it spring to life.
 *   Fix here is a stopgap: drive 8 and leave servo9 idle, purely so the
 *   smoothness fix can be judged without a phantom fault muddying it. The
 *   real fix is a PCA9685, which is what Tuft was designed around in the
 *   first place, 16 channels per board over I2C and none of this contention.
 *
 * WIRING (unchanged, no rewiring needed for this version):
 *   GPIO 1,3,4,5,6,7,8,10 -> servo 1..8
 *   servo9 on GPIO11 is left unattached this round.
 *   GPIO2 and GPIO9 unused.
 */

#include <ESP32Servo.h>

#define VERSION "2.0"

// ---------------- CONFIG ----------------

// 8 servos, not 9: the S3 has only 8 LEDC channels. See FAULT 2 above.
const int SERVO_COUNT = 8;
const int SERVO_PINS[SERVO_COUNT] = {1, 3, 4, 5, 6, 7, 8, 10};

const float SHAFT_TO_TIP_MM = 250.0;
const float TIP_SWAY_MM     = 50.0;

const int CENTER_DEG = 90;
const float SWAY_PERIOD_S = 6.0;

const int MIN_US = 500, MAX_US = 2400;
const int TIMER_WIDTH_BITS = 14; // ESP32-S3 hardware max, do not raise
const int SERVO_HZ = 50;

float angleTrim[SERVO_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};

const unsigned long ARM_DELAY_MS = 1000;

// -----------------------------------------

Servo servos[SERVO_COUNT];
float swingDeg;

int degToUs(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  return (int)lround(MIN_US + (deg / 180.0) * (MAX_US - MIN_US));
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  swingDeg = degrees(asin(TIP_SWAY_MM / SHAFT_TO_TIP_MM));

  Serial.printf("\nTuft 3x3 sway, firmware v%s\n", VERSION);
  Serial.printf("sway %.0fmm either side at %.0fmm horn -> +/-%.2f deg\n",
                TIP_SWAY_MM, SHAFT_TO_TIP_MM, swingDeg);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < SERVO_COUNT; i++) {
    servos[i].setPeriodHertz(SERVO_HZ);
    int ch = servos[i].attach(SERVO_PINS[i], MIN_US, MAX_US);
    servos[i].setTimerWidth(TIMER_WIDTH_BITS); // AFTER attach, see header
    servos[i].writeMicroseconds(degToUs(CENTER_DEG + angleTrim[i]));
    Serial.printf("  servo%d GPIO%-2d attach=%-3d %s\n",
                  i + 1, SERVO_PINS[i], ch,
                  servos[i].attached() ? "OK" : "FAILED");
    Serial.flush();
    delay(60); // space the prints, a fast burst drops lines over USB CDC
  }

  // report the resolution actually achieved, rather than trusting the comment
  float usPerTick   = 1000000.0 / SERVO_HZ / (float)(1UL << TIMER_WIDTH_BITS);
  float usPerDeg    = (float)(MAX_US - MIN_US) / 180.0;
  float degPerTick  = usPerTick / usPerDeg;
  float tipMmPerTick = SHAFT_TO_TIP_MM * sin(radians(degPerTick));
  Serial.printf("resolution: %d-bit -> %.3f us/step, %.4f deg/step, %.3f mm at the tip\n",
                TIMER_WIDTH_BITS, usPerTick, degPerTick, tipMmPerTick);
  Serial.printf("(10-bit default was 19.531 us, 1.850 deg, 8.07 mm)\n");

  delay(ARM_DELAY_MS);
  Serial.println("Swaying.");
}

void loop() {
  float t = millis() / 1000.0;
  float angleDeg = CENTER_DEG + swingDeg * sin((2.0 * PI / SWAY_PERIOD_S) * t);

  for (int i = 0; i < SERVO_COUNT; i++) {
    servos[i].writeMicroseconds(degToUs(angleDeg + angleTrim[i]));
  }

  delay(1000 / SERVO_HZ);
}
