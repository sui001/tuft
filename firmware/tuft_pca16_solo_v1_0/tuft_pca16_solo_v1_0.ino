/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  4x4 grid, 16 SG92R servos on one PCA9685, ESP32-S3 SuperMini
 *
 * Solo test. Runs the v6 breeze on ONE channel at a time, 50Hz, v6 timing,
 * moving to the next channel every SOLO_S seconds. Every other channel gets
 * no pulse at all (limp, near-zero current). Separates the question the v7
 * breezes couldn't answer:
 *   one servo smooth on its own, violent in the full grid -> power under load
 *   one servo violent on its own too                      -> that servo, or
 *                                                            the signal path
 * Serial announces each channel as it starts.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define VERSION "1.0"

const int PIN_SDA = 9;
const int PIN_SCL = 10;
const uint8_t PCA_ADDR = 0x40;
const uint32_t OSC_HZ = 27000000;

const int CHANNELS = 16;
const int SERVO_HZ = 50;          // SG92R is an analog 50Hz servo
const int MIN_US = 500;
const int MAX_US = 2400;

const float WAKE_DEG = 90.0;
const float REST_DEG = 100.0;
const float SOLO_S   = 20.0;

// v6 breeze, unchanged, full speed
const float SWAY_DEG = 5.0;
const float FLUTTER_A_S = 2.5;
const float FLUTTER_B_S = 3.7;
const float GUST_PERIOD_A_S = 11.0;
const float GUST_PERIOD_B_S = 17.0;
const float GUST_FLOOR = 0.18;

Adafruit_PWMServoDriver pca(PCA_ADDR);
bool pcaOk = false;

uint16_t ticksFor(float deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  float us = MIN_US + (deg / 180.0f) * (MAX_US - MIN_US);
  return (uint16_t)lroundf(us * 4096.0f * SERVO_HZ / 1000000.0f);
}

float breeze(float t) {
  float a = 0.5f + 0.5f * sinf(2.0f * PI * t / GUST_PERIOD_A_S);
  float b = 0.5f + 0.5f * sinf(2.0f * PI * t / GUST_PERIOD_B_S + 1.7f);
  float gust = GUST_FLOOR + (1.0f - GUST_FLOOR) * (0.45f * a + 0.55f * b);
  float s = 0.62f * sinf(2.0f * PI * t / FLUTTER_A_S)
          + 0.38f * sinf(2.0f * PI * t / FLUTTER_B_S);
  return SWAY_DEG * gust * s;
}

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println();
  Serial.println("=== Tuft PCA9685 solo test v" VERSION " ===");
  Serial.println("v6 breeze on one channel at a time, 50Hz, others limp");
  Serial.println("https://github.com/sui001/tuft/tree/master/firmware/tuft_pca16_solo_v1_0");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Wire.beginTransmission(PCA_ADDR);
  pcaOk = Wire.endTransmission() == 0;
  if (pcaOk) {
    pca.begin();
    pca.setOscillatorFrequency(OSC_HZ);
    pca.setPWMFreq(SERVO_HZ);
    for (int i = 0; i < CHANNELS; i++) pca.setPWM(i, 0, 0);   // all limp
  }
  Serial.printf("PCA9685 0x%02X: %s\n", PCA_ADDR, pcaOk ? "OK" : "NOT FOUND");
}

void loop() {
  static int ch = -1;
  static unsigned long chStart = 0;
  static uint16_t last = 0;

  if (!pcaOk) { delay(1000); Serial.println("PCA9685 NOT FOUND"); return; }

  if (ch < 0 || millis() - chStart > (unsigned long)(SOLO_S * 1000)) {
    if (ch >= 0) pca.setPWM(ch, 0, 0);          // previous one goes limp
    ch = (ch + 1) % CHANNELS;
    chStart = millis();
    Serial.printf(">> servo %d (ch %d) solo for %.0fs\n", ch + 1, ch, SOLO_S);
    // wake at 90, ease to 100 over ~1s
    for (float d = WAKE_DEG; d <= REST_DEG; d += 0.5f) {
      pca.setPWM(ch, 0, ticksFor(d));
      delay(50);
    }
    last = 0;
  }

  float t = (millis() - chStart) / 1000.0f;
  float fade = t / 3.0f;
  if (fade > 1.0f) fade = 1.0f;
  uint16_t off = ticksFor(REST_DEG + fade * breeze(t));
  if (off != last) { pca.setPWM(ch, 0, off); last = off; }
  delay(1000 / SERVO_HZ);
}
