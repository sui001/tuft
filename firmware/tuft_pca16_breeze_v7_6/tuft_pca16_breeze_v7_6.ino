/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  4x4 grid, 16 SG92R servos on one PCA9685, ESP32-S3 SuperMini
 *
 * v7.6. Gentler acceleration. The grid now hangs from a box lid, not
 * cardboard on the floor, and a lid is a drum skin: when many 1.1m rods
 * reverse together on a deeper gust, the reaction bounces the lid and that
 * bounce feeds back into every servo. Each servo now follows the breeze
 * through a smooth follower (spring-damper with a hard cap on acceleration
 * and speed), so reversals are rounded off instead of snapped. The breeze
 * shape and depth are unchanged. MAX_ACCEL is the knob: lower = gentler,
 * but too low and the deepest sways get clipped short.
 *
 * v7.5. Back to 50Hz and v6 speed (the solo test showed the breeze itself
 * is gentle; 100Hz only made the analog SG92Rs hunt). Softer start: the
 * violent moment was each servo's FIRST pulse after being limp, when it
 * drives full speed to wherever it's told. The PCA keeps pulsing through an
 * ESP reset, so on a warm start the old positions are read back off the
 * chip and every servo glides from there. Only a true cold power-up still
 * gets a first pulse, sent straight to REST one servo at a time.
 *
 * v7.4. I2C hygiene, and a way to see if the bus is the problem. hold90 is
 * calm because it writes once and goes quiet; the breeze writes 16 channels
 * every frame, and a corrupted write sends a servo somewhere random. So:
 *   - bus at 100kHz, not 400kHz, far more tolerant of long leads and noise
 *   - a channel is only written when its tick value actually changes
 *   - one channel is read back per frame and compared; mismatches are
 *     counted and reported as "i2c bad N/M". Non-zero means the bus is
 *     corrupting writes: shorten the SDA/SCL leads, route them away from
 *     servo wires, twist each with a ground.
 *
 * v7.3. SPEED knob. The breeze was too fast and violent on this rig with
 * the refitted horns. SPEED scales breeze time: 1.0 = v6 timing, 0.5 = half
 * as fast. Amplitude is unchanged, so lower SPEED is purely gentler.
 *
 * v7.2. 100Hz instead of 50Hz. The PCA9685 is 12-bit, so at 50Hz one step
 * is 4.88us = ~0.46 deg = ~9mm at a 1.1m tip, 4x coarser than v6's LEDC.
 * The +/-5 deg breeze became ~11 jerks each way and the springy rods rang
 * on every one. Step size scales with frame length, so 100Hz halves it
 * (2.44us). v3.3 showed these SG92Rs tolerate 100Hz. If still steppy, try
 * 200Hz (1.22us, v6 parity) but watch for servos buzzing or warming.
 *
 * v7.1. Gentle start. v7.0 began by driving all 16 servos to 0 degrees AT
 * ONCE, which with 1.1m rods is 16 stalls together. After the horns were
 * refitted at 90 that spike sagged the supply and the whole grid went into
 * hyper jitter. Now: wake one servo at a time at 90 (where hold90 leaves
 * them), ease the grid together to REST_DEG, then fade the breeze in.
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

#define VERSION "7.6"

// ---------------- CONFIG ----------------

const int PIN_SDA = 9;
const int PIN_SCL = 10;
const uint8_t PCA_ADDR = 0x40;
const uint32_t OSC_HZ = 27000000;

const int ACTIVE_COUNT = 16;
const int GRID_COLS    = 4;

const int SERVO_HZ = 50;    // SG92R is an analog 50Hz servo; 100Hz made it hunt
const int MIN_US   = 500;
const int MAX_US   = 2400;

const float REST_DEG = 100.0;

// --- breeze ---
const float SPEED = 1.0;   // 1.0 = v6 timing; lower = slower, gentler

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
const int   WAKE_GAP_MS = 250;            // cold start: one servo at a time
const unsigned long EASE_MS = 2000;       // warm start: glide from last pos
const float FADE_IN_S = 8.0;              // breeze amplitude ramps 0 -> full

unsigned long breezeStartMs = 0;

// --- smooth follower ---
const float FOLLOW_HZ = 1.2;     // how tightly each servo tracks the breeze
const float MAX_ACCEL = 30.0;    // deg/s^2, the gentleness knob
const float MAX_VEL   = 12.0;    // deg/s
float pos[16], vel[16];

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

uint16_t lastOff[16];                // 0 = never written
unsigned long busChecks = 0, busBad = 0;

void writeDeg(int i, float deg) {
  if (i >= ACTIVE_COUNT || !pcaOk) return;
  // 4096 ticks per frame
  uint16_t off = (uint16_t)lroundf(degToUs(deg + angleTrim[i]) * 4096.0f * SERVO_HZ / 1000000.0f);
  if (off == lastOff[i]) return;     // unchanged, keep the bus quiet
  pca.setPWM(i, 0, off);
  lastOff[i] = off;
}

// read one channel's OFF register back and compare with what we sent
void checkBus() {
  static int ch = 0;
  if (lastOff[ch]) {
    busChecks++;
    if (pca.getPWM(ch, true) != lastOff[ch]) busBad++;
  }
  ch = (ch + 1) % ACTIVE_COUNT;
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

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(PCA_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

// Where each servo is right now, from what the PCA is still outputting. The
// PCA keeps pulsing on its own while the ESP reboots, so after a reflash or
// reset the servos never went limp. Returns -1 for a channel with no pulse
// (cold power-up, MODE1 sleeping, or out of range).
void readLivePositions(float *deg) {
  uint8_t mode1 = readReg(0x00);
  uint8_t pre   = readReg(0xFE);
  bool asleep = mode1 & 0x10;
  float usPerTick = (pre + 1) * 1000000.0f / OSC_HZ;
  for (int i = 0; i < ACTIVE_COUNT; i++) {
    uint16_t off = readReg(0x08 + 4 * i) | ((readReg(0x09 + 4 * i) & 0x0F) << 8);
    float us = off * usPerTick;
    deg[i] = (!asleep && us >= MIN_US && us <= MAX_US)
             ? (us - MIN_US) / (MAX_US - MIN_US) * 180.0f : -1;
  }
}

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println();
  Serial.println("=== Tuft PCA9685 breeze v" VERSION " ===");
  Serial.println("4x4 SG92R servo grid on one PCA9685 over I2C, breeze effect");
  Serial.println("https://github.com/sui001/tuft/tree/master/firmware/tuft_pca16_breeze_v7_6");
  randomSeed(esp_random());

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  pcaOk = pcaPresent();
  float liveDeg[16];
  if (pcaOk) readLivePositions(liveDeg);   // before begin() touches anything
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

  // Warm channels glide from where they are. Cold ones (no pulse yet) have
  // no known position, so their first pulse is a full-speed move no firmware
  // can soften; send them straight to REST one at a time so only one servo
  // lurches at once.
  int warm = 0;
  for (int i = 0; i < ACTIVE_COUNT; i++) if (liveDeg[i] >= 0) warm++;
  Serial.printf("START: %d warm (glide), %d cold (one at a time)\n",
                warm, ACTIVE_COUNT - warm);

  for (int i = 0; i < ACTIVE_COUNT; i++) {
    if (liveDeg[i] >= 0) writeDeg(i, liveDeg[i]);   // re-assert, no motion
  }
  for (int i = 0; i < ACTIVE_COUNT; i++) {
    if (liveDeg[i] < 0) { writeDeg(i, REST_DEG); delay(WAKE_GAP_MS); }
  }
  if (warm) {
    unsigned long t0 = millis();
    while (millis() - t0 < EASE_MS) {
      float k = (millis() - t0) / (float)EASE_MS;
      k = k * k * (3 - 2 * k);                        // smoothstep
      for (int i = 0; i < ACTIVE_COUNT; i++)
        if (liveDeg[i] >= 0) writeDeg(i, liveDeg[i] + k * (REST_DEG - liveDeg[i]));
      delay(1000 / SERVO_HZ);
    }
  }
  for (int i = 0; i < ACTIVE_COUNT; i++) writeDeg(i, REST_DEG);

  Serial.printf("BREEZE, all %d, fading in over %.0fs\n", ACTIVE_COUNT, FADE_IN_S);
  for (int i = 0; i < ACTIVE_COUNT; i++) { pos[i] = REST_DEG; vel[i] = 0; }
  breezeStartMs = millis();
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

  float t = SPEED * millis() / 1000.0f;
  float fade = (millis() - breezeStartMs) / 1000.0f / FADE_IN_S;
  if (fade > 1.0f) fade = 1.0f;
  static unsigned long lastMs = millis();
  float dt = (millis() - lastMs) / 1000.0f;
  lastMs = millis();
  if (dt > 0.1f) dt = 0.1f;

  const float w = 2.0f * PI * FOLLOW_HZ;
  for (int i = 0; i < ACTIVE_COUNT; i++) {
    float target = REST_DEG + fade * breezeOffset(i, t);
    float acc = w * w * (target - pos[i]) - 2.0f * w * vel[i];   // critically damped
    acc = constrain(acc, -MAX_ACCEL, MAX_ACCEL);
    vel[i] = constrain(vel[i] + acc * dt, -MAX_VEL, MAX_VEL);
    pos[i] += vel[i] * dt;
    writeDeg(i, pos[i]);
  }
  checkBus();

  // one status line forever, the boot table is easy to miss on USB CDC
  if (report) Serial.printf("v%s breeze, PCA 0x%02X %s, i2c bad %lu/%lu\n",
                            VERSION, PCA_ADDR, pcaPresent() ? "OK" : "LOST",
                            busBad, busChecks);

  delay(1000 / SERVO_HZ);
}
