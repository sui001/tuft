/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  3x3 rig, staged startup, ESP32-S3 SuperMini
 *
 * v2.2 fixes a bug I introduced in v2.1, which showed up on the bench as
 * servos dropping in two groups (1,3,5 then 2,4,6) and then "coming up
 * weird".
 *
 * CAUSE: setTimerWidth() reconfigures the LEDC *timer*, and a timer is
 * shared by several channels. v2.1 called attach() -> setTimerWidth() ->
 * write() per servo inside one loop, so every setTimerWidth call
 * re-resolutioned a timer that already-attached servos were using. Their
 * previously written duty values were scaled for the old resolution and
 * were never rewritten, so they jumped, and only the servos sharing that
 * particular timer jumped together, hence the grouping.
 *
 * FIX: do it in three passes. Attach everything, then set every timer
 * width, and only then write any positions. All timer reconfiguration is
 * finished before a single position is committed, so nothing gets scaled
 * against a resolution that is about to change underneath it.
 *
 * Sequence: LOWER all together -> RAISE servo 1, then 2, then 3 ... each
 * fully and alone -> SWAY all together 75-105 continuously.
 *
 * The attach table is printed twice, once at boot and again before the
 * sway, because a servo that doesn't move on its turn during RAISE is
 * either unplugged or on a pin this firmware doesn't drive. Only these
 * pins are driven: 1,3,4,5,6,7,8,10. Anything on GPIO9 or GPIO11 will sit
 * still no matter how healthy it is (GPIO9 retired, GPIO11 has no channel
 * left, the S3's LEDC only has 8).
 *
 * WIRING: GPIO 1,3,4,5,6,7,8,10 -> servo 1..8. GPIO 2,9,11 unused.
 */

#include <ESP32Servo.h>

#define VERSION "2.2"

// ---------------- CONFIG ----------------

const int SERVO_COUNT = 8;
const int SERVO_PINS[SERVO_COUNT] = {1, 3, 4, 5, 6, 7, 8, 10};

const float SHAFT_TO_TIP_MM = 250.0;

const float LOWER_DEG = 0.0;
const unsigned long LOWER_HOLD_MS = 2500;

const float RAISE_TO_DEG  = 75.0;
const float RISE_STEP_DEG = 1.0;  // 1.0 = 4.4mm at the tip; 0.1 glides
const int   RISE_STEP_MS  = 25;
const int   PAUSE_BETWEEN_SERVOS_MS = 400;

const float OSC_LOW_DEG   = 75.0;
const float OSC_HIGH_DEG  = 105.0;
const float SWAY_PERIOD_S = 6.0;

const int MIN_US = 500, MAX_US = 2400;
const int TIMER_WIDTH_BITS = 14; // ESP32-S3 hardware max
const int SERVO_HZ = 50;

float angleTrim[SERVO_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};

// -----------------------------------------

Servo servos[SERVO_COUNT];
int attachResult[SERVO_COUNT];

int degToUs(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  return (int)lround(MIN_US + (deg / 180.0) * (MAX_US - MIN_US));
}

void writeDeg(int i, float deg) {
  servos[i].writeMicroseconds(degToUs(deg + angleTrim[i]));
}

void printAttachTable(const char *when) {
  Serial.printf("-- attach table (%s) --\n", when);
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.printf("  servo%d  GPIO%-2d  attach=%-4d  %s\n",
                  i + 1, SERVO_PINS[i], attachResult[i],
                  servos[i].attached() ? "OK" : "FAILED");
    Serial.flush();
    delay(40); // a fast burst drops lines over USB CDC
  }
  Serial.println("  not driven: GPIO2, GPIO9, GPIO11");
  Serial.flush();
}

void setup() {
  Serial.begin(115200);
  delay(3000); // long enough that the boot report can actually be captured
  Serial.printf("\nTuft 3x3, firmware v%s\n", VERSION);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // PASS 1: attach everything. No widths, no positions yet.
  for (int i = 0; i < SERVO_COUNT; i++) {
    servos[i].setPeriodHertz(SERVO_HZ);
    attachResult[i] = servos[i].attach(SERVO_PINS[i], MIN_US, MAX_US);
  }

  // PASS 2: set every timer width. Shared timers get reconfigured here,
  // while no position has been committed yet, so nothing can be corrupted.
  for (int i = 0; i < SERVO_COUNT; i++) {
    servos[i].setTimerWidth(TIMER_WIDTH_BITS);
  }

  // PASS 3: now commit positions, all against the final resolution.
  for (int i = 0; i < SERVO_COUNT; i++) {
    writeDeg(i, LOWER_DEG);
  }

  printAttachTable("boot");

  float usPerTick = 1000000.0 / SERVO_HZ / (float)(1UL << TIMER_WIDTH_BITS);
  float degPerTick = usPerTick / ((float)(MAX_US - MIN_US) / 180.0);
  Serial.printf("resolution %d-bit: %.3f us = %.4f deg = %.3f mm at tip\n",
                TIMER_WIDTH_BITS, usPerTick, degPerTick,
                SHAFT_TO_TIP_MM * sin(radians(degPerTick)));
  Serial.printf("rise step %.2f deg = %.2f mm at tip\n", RISE_STEP_DEG,
                SHAFT_TO_TIP_MM * sin(radians(RISE_STEP_DEG)));

  // STAGE 1: everything down together, hold.
  Serial.printf("LOWER: all to %.0f deg, holding %lums\n", LOWER_DEG, LOWER_HOLD_MS);
  delay(LOWER_HOLD_MS);

  // STAGE 2: one servo at a time, all the way up, alone.
  Serial.println("RAISE: one at a time, in order.");
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.printf("  >> servo%d (GPIO%d) rising\n", i + 1, SERVO_PINS[i]);
    Serial.flush();
    for (float d = LOWER_DEG; d <= RAISE_TO_DEG; d += RISE_STEP_DEG) {
      writeDeg(i, d);
      delay(RISE_STEP_MS);
    }
    writeDeg(i, RAISE_TO_DEG);
    delay(PAUSE_BETWEEN_SERVOS_MS);
  }

  printAttachTable("after raise");
  Serial.println("SWAY: all together, continuous.");
}

void loop() {
  const float mid = (OSC_LOW_DEG + OSC_HIGH_DEG) / 2.0;
  const float amp = (OSC_HIGH_DEG - OSC_LOW_DEG) / 2.0;

  float t = millis() / 1000.0;
  float angleDeg = mid + amp * sin((2.0 * PI / SWAY_PERIOD_S) * t);

  for (int i = 0; i < SERVO_COUNT; i++) {
    writeDeg(i, angleDeg);
  }

  // Repeat the attach status forever, as ONE line. Multi-line bursts get
  // dropped by the S3's USB CDC when the host isn't already reading, which
  // is why the boot table kept arriving truncated.
  static unsigned long lastReport = 0;
  if (millis() - lastReport > 5000) {
    lastReport = millis();
    Serial.print("attached:");
    for (int i = 0; i < SERVO_COUNT; i++) {
      Serial.printf(" s%d/GPIO%d=%s", i + 1, SERVO_PINS[i],
                    servos[i].attached() ? "OK" : "NO");
    }
    Serial.println();
  }

  delay(1000 / SERVO_HZ);
}
