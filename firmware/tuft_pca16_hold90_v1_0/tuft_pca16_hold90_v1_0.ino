/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  4x4 grid, 16 SG92R servos on one PCA9685, ESP32-S3 SuperMini
 *
 * Assembly utility: drives all 16 channels to 90 degrees and holds them
 * there, so horns can be fitted square. Flash the breeze back afterwards.
 * Same wiring and pulse maths as tuft_pca16_breeze_v7_0.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define VERSION "1.0"

const int PIN_SDA = 9;
const int PIN_SCL = 10;
const uint8_t PCA_ADDR = 0x40;
const uint32_t OSC_HZ = 27000000;

const int ACTIVE_COUNT = 16;
const int SERVO_HZ = 50;
const int MIN_US   = 500;
const int MAX_US   = 2400;
const float HOLD_DEG = 90.0;

Adafruit_PWMServoDriver pca(PCA_ADDR);
bool pcaOk = false;

bool pcaPresent() {
  Wire.beginTransmission(PCA_ADDR);
  return Wire.endTransmission() == 0;
}

void holdAll() {
  float us = MIN_US + (HOLD_DEG / 180.0f) * (MAX_US - MIN_US);
  uint16_t off = (uint16_t)lroundf(us * 4096.0f * SERVO_HZ / 1000000.0f);
  for (int i = 0; i < ACTIVE_COUNT; i++) pca.setPWM(i, 0, off);
}

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println();
  Serial.println("=== Tuft PCA9685 hold90 v" VERSION " ===");
  Serial.println("Holds all 16 servos at 90 degrees for fitting horns");
  Serial.println("https://github.com/sui001/tuft/tree/master/firmware/tuft_pca16_hold90_v1_0");

  Wire.begin(PIN_SDA, PIN_SCL);
  pcaOk = pcaPresent();
  if (pcaOk) {
    pca.begin();
    pca.setOscillatorFrequency(OSC_HZ);
    pca.setPWMFreq(SERVO_HZ);
    delay(10);
    holdAll();
  }
}

void loop() {
  static unsigned long lastReport = 0;
  if (millis() - lastReport > 5000) {
    lastReport = millis();
    if (pcaOk) Serial.printf("hold90 v%s: all %d at %.0f deg\n", VERSION, ACTIVE_COUNT, HOLD_DEG);
    else Serial.printf("PCA9685 NOT FOUND at 0x%02X, check SDA %d / SCL %d / 3V3\n",
                       PCA_ADDR, PIN_SDA, PIN_SCL);
  }
  delay(100);
}
