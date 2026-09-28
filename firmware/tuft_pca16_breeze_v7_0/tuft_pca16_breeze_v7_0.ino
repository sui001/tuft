/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  4x4 grid, 16 SG92R servos on one PCA9685, ESP32-S3 SuperMini
 *
 * v7.0. Same breeze and startup raise as v6.0, but every servo now runs off a
 * PCA9685 over I2C instead of direct GPIO. v6 used every LEDC and MCPWM
 * channel on the chip and needed wires soldered to the underside pads (GPIO
 * 14-18, 21, 47). A replacement board has no pads soldered, so the top row
 * went dead. Two wires to a PCA9685 replaces all of that, and more patches
 * are just more boards at 0x41, 0x42...
 *
 * WIRING
 *   PCA9685 SDA -> GPIO9, SCL -> GPIO10, VCC -> 3V3, GND -> GND, OE unconnected
 *   Servo power into the PCA9685 V+ screw terminal ONLY, ground shared with
 *   the ESP. Never feed servos from the SuperMini 5V pin.
 *
 * CHANNELS, numbered bottom-left first, servo N on PCA channel N-1:
 *      13 14 15 16    <- row 3 (top)
 *       9 10 11 12
 *       5  6  7  8
 *       1  2  3  4    <- row 0 (bottom)
 *
 * PULSE TIMING. The PCA9685's internal oscillator is nominally 25MHz but
 * real chips run 23-27MHz, which scales every pulse by the same error. OSC_HZ
 * is the correction. 27MHz is Adafruit's measured typical. If the whole grid
 * rests visibly off 100 degrees, that's the number to change, not the trims.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define VERSION "7.0"

// ---------------- CONFIG ----------------

const int PIN_SDA = 9;
const int PIN_SCL = 10;
const uint8_t PCA_ADDR = 0x40;
const uint32_t OSC_HZ = 27000000;

const int ACTIVE_COUNT = 16;
const int GRID_COLS    = 4;

const int SERVO_HZ = 50;
const int MIN_US   = 500;
const int MAX_US   = 2400;

const float REST_DEG = 100.0;

// --- breeze, unchanged from v4.1 ---
const float SWAY_DEG = 5.0;
const float FLUTTER_A_S = 2.5;
const float FLUTTER_B_S = 3.7;
const float GUST_PERIOD_A_S = 11.0;
const float GUST_PERIOD_B_S = 17.0;
const float GUST_FLOOR = 0.18;
const float LAG_PER_COL = 1.10;
const float LAG_PER_ROW = 0.35;

float angleTrim[16] = {0};

// --- startup ---
const bool  DO_STARTUP_RAISE = true;
const float LOWER_DEG = 0.0;
const unsigned long LOWER_HOLD_MS = 1500;
const float RISE_STEP_DEG = 1.0;
const int   RISE_STEP_MS  = 15;
const int   PAUSE_BETWEEN_SERVOS_MS = 200;

// -----------------------------------------

Adafruit_PWMServoDriver pca(PCA_ADDR);
bool pcaOk = false;
float spatialPhase[16];
float phaseJitter[16];

float degToUs(float deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  return MIN_US + (deg / 180.0f) * (MAX_US - MIN_US);
}

void writeDeg(int i, float deg) {
  if (i >= ACTIVE_COUNT || !pcaOk) return;
  // 4096 ticks per 20ms frame -> ~4.9us per step
  uint16_t off = (uint16_t)lroundf(degToUs(deg + angleTrim[i]) * 4096.0f * SERVO_HZ / 1000000.0f);
  pca.setPWM(i, 0, off);
}

bool pcaPresent() {
  Wire.beginTransmission(PCA_ADDR);
  return Wire.endTransmission() == 0;
}

float gust(float t) {
  float a = 0.5f + 0.5f * sinf(2.0f * PI * t / GUST_PERIOD_A_S);
  float b = 0.5f + 0.5f * sinf(2.0f * PI * t / GUST_PERIOD_B_S + 1.7f);
  return GUST_FLOOR + (1.0f - GUST_FLOOR) * (0.45f * a + 0.55f * b);
}

float breezeOffset(int i, float t) {
  float p = spatialPhase[i] + phaseJitter[i];
  float s = 0.62f * sinf(2.0f * PI * t / FLUTTER_A_S + p)
          + 0.38f * sinf(2.0f * PI * t / FLUTTER_B_S + p * 1.3f);
  return SWAY_DEG * gust(t) * s;
}

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println();
  Serial.println("=== Tuft PCA9685 breeze v" VERSION " ===");
  Serial.println("4x4 SG92R servo grid on one PCA9685 over I2C, breeze effect");
  Serial.println("https://github.com/sui001/tuft/tree/master/firmware/tuft_pca16_breeze_v7_0");
  randomSeed(esp_random());

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  pcaOk = pcaPresent();
  if (pcaOk) {
    pca.begin();
    pca.setOscillatorFrequency(OSC_HZ);
    pca.setPWMFreq(SERVO_HZ);
    delay(10);
  }
  Serial.printf("PCA9685 at 0x%02X (SDA %d, SCL %d): %s\n", PCA_ADDR, PIN_SDA, PIN_SCL,
                pcaOk ? "OK" : "NOT FOUND");

  for (int i = 0; i < ACTIVE_COUNT; i++) {
    int row = i / GRID_COLS, col = i % GRID_COLS;
    spatialPhase[i] = -(col * LAG_PER_COL + row * LAG_PER_ROW);
    phaseJitter[i] = (float)random(-300, 301) / 1000.0f;
  }

  if (!pcaOk) return;   // loop() keeps shouting about it

  if (DO_STARTUP_RAISE) {
    for (int i = 0; i < ACTIVE_COUNT; i++) writeDeg(i, LOWER_DEG);
    Serial.println("LOWER");
    delay(LOWER_HOLD_MS);
    Serial.println("RAISE: one at a time.");
    for (int i = 0; i < ACTIVE_COUNT; i++) {
      Serial.printf("  >> s%d ch%d\n", i + 1, i);
      Serial.flush();
      for (float d = LOWER_DEG; d <= REST_DEG; d += RISE_STEP_DEG) {
        writeDeg(i, d);
        delay(RISE_STEP_MS);
      }
      writeDeg(i, REST_DEG);
      delay(PAUSE_BETWEEN_SERVOS_MS);
    }
  } else {
    for (int i = 0; i < ACTIVE_COUNT; i++) writeDeg(i, REST_DEG);
    delay(1500);
  }

  Serial.printf("BREEZE, all %d.\n", ACTIVE_COUNT);
}

void loop() {
  static unsigned long lastReport = 0;
  bool report = millis() - lastReport > 6000;
  if (report) lastReport = millis();

  if (!pcaOk) {
    if (report) Serial.printf("PCA9685 NOT FOUND at 0x%02X, check SDA %d / SCL %d / 3V3\n",
                              PCA_ADDR, PIN_SDA, PIN_SCL);
    delay(100);
    return;
  }

  float t = millis() / 1000.0;
  for (int i = 0; i < ACTIVE_COUNT; i++) {
    writeDeg(i, REST_DEG + breezeOffset(i, t));
  }

  // one status line forever, the boot table is easy to miss on USB CDC
  if (report) Serial.printf("v%s breeze, %d servos, PCA 0x%02X %s\n", VERSION,
                            ACTIVE_COUNT, PCA_ADDR, pcaPresent() ? "OK" : "LOST");

  delay(1000 / SERVO_HZ);
}
