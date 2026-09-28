/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  3x3 rig, staged startup + wider sway, ESP32-S3 SuperMini
 *
 * v2.1, built on v2.0's two fixes (14-bit LEDC timer set AFTER attach, and
 * 8 servos rather than 9 because the S3's LEDC has only 8 channels, see
 * v2_0's header for the full reasoning).
 *
 * Sequence:
 *   1. RESET   - all 8 servos to 0 degrees, hold.
 *   2. RAISE   - servo 1..8 in turn, each stepping 0 -> 75 degrees, so each
 *                one can be watched individually. Doubles as a liveness
 *                check: anything that doesn't move on its turn is either
 *                unplugged or dead, no guessing which.
 *   3. SWAY    - all together, 75 <-> 105 degrees, continuously.
 *
 * ON THE REMAINING JITTER. v2.0 fixed the big one (10-bit PWM = 8.1mm jumps
 * at the tip). What's likely left is not fixable in firmware:
 *   - Deadband. Cheap analog servos ignore pulse changes below roughly
 *     5-10us. We now step at 1.22us, well under that, so the servo can sit
 *     through several updates and then move once accumulated error crosses
 *     its threshold: perhaps 2-4mm at a 250mm tip. Digital servos spec
 *     deadband down around 1-2us, which is the real fix if it matters.
 *   - Backlash. A 250mm horn is ~20x the arm these are designed for, so
 *     gear lash is magnified ~20x at the tip too.
 * Both shrink as a proportion of travel when the motion is bigger, which is
 * why the wider 75-105 sway here should read smoother than the old +/-11.5.
 *
 * NOTE ON RISE_STEP_DEG: 1.0 degree per step is 4.4mm at the tip, nearly 9x
 * coarser than the 0.505mm the hardware can now resolve. If the raise looks
 * steppy, that is why, drop it to 0.1 and it will glide.
 *
 * WIRING (unchanged): GPIO 1,3,4,5,6,7,8,10 -> servo 1..8.
 *   servo9 / GPIO11 idle (no 9th LEDC channel exists). GPIO2, GPIO9 unused.
 */

#include <ESP32Servo.h>

#define VERSION "2.1"

// ---------------- CONFIG ----------------

const int SERVO_COUNT = 8;
const int SERVO_PINS[SERVO_COUNT] = {1, 3, 4, 5, 6, 7, 8, 10};

const float SHAFT_TO_TIP_MM = 250.0;

// stage 1: reset
const float RESET_DEG      = 0.0;
const unsigned long RESET_HOLD_MS = 2000;

// stage 2: sequential raise
const float RAISE_TO_DEG   = 75.0;
const float RISE_STEP_DEG  = 1.0;   // 1.0 = "one by one" as asked; 0.1 is far smoother
const int   RISE_STEP_MS   = 25;    // pace of the climb
const int   PAUSE_BETWEEN_SERVOS_MS = 250;

// stage 3: continuous sway
const float OSC_LOW_DEG    = 75.0;
const float OSC_HIGH_DEG   = 105.0;
const float SWAY_PERIOD_S  = 6.0;

const int MIN_US = 500, MAX_US = 2400;
const int TIMER_WIDTH_BITS = 14; // ESP32-S3 hardware max, do not raise
const int SERVO_HZ = 50;

float angleTrim[SERVO_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};

// -----------------------------------------

Servo servos[SERVO_COUNT];

int degToUs(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  return (int)lround(MIN_US + (deg / 180.0) * (MAX_US - MIN_US));
}

void writeDeg(int i, float deg) {
  servos[i].writeMicroseconds(degToUs(deg + angleTrim[i]));
}

float tipMmForDeg(float deg) {
  return SHAFT_TO_TIP_MM * sin(radians(deg));
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.printf("\nTuft 3x3, firmware v%s\n", VERSION);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // ---- attach, then set timer width (attach resets it to 10-bit) ----
  for (int i = 0; i < SERVO_COUNT; i++) {
    servos[i].setPeriodHertz(SERVO_HZ);
    int ch = servos[i].attach(SERVO_PINS[i], MIN_US, MAX_US);
    servos[i].setTimerWidth(TIMER_WIDTH_BITS);
    writeDeg(i, RESET_DEG);
    Serial.printf("  servo%d GPIO%-2d attach=%-3d %s\n", i + 1, SERVO_PINS[i],
                  ch, servos[i].attached() ? "OK" : "FAILED");
    Serial.flush();
    delay(60);
  }

  float usPerTick = 1000000.0 / SERVO_HZ / (float)(1UL << TIMER_WIDTH_BITS);
  float degPerTick = usPerTick / ((float)(MAX_US - MIN_US) / 180.0);
  Serial.printf("resolution %d-bit: %.3f us = %.4f deg = %.3f mm at tip\n",
                TIMER_WIDTH_BITS, usPerTick, degPerTick, tipMmForDeg(degPerTick));
  Serial.printf("rise step %.2f deg = %.2f mm at tip\n",
                RISE_STEP_DEG, tipMmForDeg(RISE_STEP_DEG));
  Serial.printf("sway %.0f-%.0f deg = %.0f mm of travel at tip\n",
                OSC_LOW_DEG, OSC_HIGH_DEG,
                tipMmForDeg(OSC_HIGH_DEG - 90.0) - tipMmForDeg(OSC_LOW_DEG - 90.0));

  // ---- stage 1: reset and hold ----
  Serial.printf("RESET: all servos to %.0f deg, holding %lums\n",
                RESET_DEG, RESET_HOLD_MS);
  delay(RESET_HOLD_MS);

  // ---- stage 2: raise each servo in turn ----
  Serial.printf("RAISE: 1..%d in sequence, %.0f -> %.0f deg\n",
                SERVO_COUNT, RESET_DEG, RAISE_TO_DEG);
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.printf("  raising servo%d (GPIO%d)\n", i + 1, SERVO_PINS[i]);
    Serial.flush();
    for (float d = RESET_DEG; d <= RAISE_TO_DEG; d += RISE_STEP_DEG) {
      writeDeg(i, d);
      delay(RISE_STEP_MS);
    }
    writeDeg(i, RAISE_TO_DEG); // land exactly on target
    delay(PAUSE_BETWEEN_SERVOS_MS);
  }

  Serial.println("SWAY: all together, continuous.");
}

void loop() {
  float mid = (OSC_LOW_DEG + OSC_HIGH_DEG) / 2.0;
  float amp = (OSC_HIGH_DEG - OSC_LOW_DEG) / 2.0;

  float t = millis() / 1000.0;
  float angleDeg = mid + amp * sin((2.0 * PI / SWAY_PERIOD_S) * t);

  for (int i = 0; i < SERVO_COUNT; i++) {
    writeDeg(i, angleDeg);
  }

  delay(1000 / SERVO_HZ);
}
