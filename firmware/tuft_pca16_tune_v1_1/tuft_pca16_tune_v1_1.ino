/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  4x4 grid, 16 SG92R servos on one PCA9685, ESP32-S3 SuperMini
 *
 * tune v1.1. Any set of servos, swaying together, in step or as a wave.
 * v1.0 showed one servo is smooth at 9-12 deg over 4-5 s, which is about
 * what the jerky v7.9 grid was doing. So the PCA9685 can do it and the
 * question is what breaks when many move together:
 *   - worse with count whatever the pattern -> power (supply sag)
 *   - worse in step, fine as a wave         -> the lid (reaction bounce)
 *
 * Commands (one per line, 115200), driven by tune.html over Web Serial:
 *   M n     which servos sway: 16-bit mask, bit 0 = servo 1. Servos
 *           joining or leaving ease in or out on their own.
 *   A deg   sway amplitude, +- degrees around centre (0-30)
 *   P s     sway period in seconds (0.5-60)
 *   C deg   centre for all servos (60-120). Glides there.
 *   W n     pattern: 0 in step, 1 wave, 2 scattered (fixed random phases)
 *   L deg   wave lag between neighbouring servos, degrees of the cycle
 *   G / X   go / stop. Stop eases everything out to centre.
 *   ?       print state
 * A pattern or lag change eases every servo to centre, swaps the phases,
 * then eases back in, so nothing jumps.
 *
 * Status line for the page:
 *   st n 4 amp 9.00/9.00 period 4.5 centre 90.0 run 1 pattern 1 lag 90 frame 21 i2c 0/812
 * "frame" is the loop time in ms; above ~25 the bus is the bottleneck.
 *
 * WIRING as v7: PCA9685 SDA GPIO9, SCL GPIO10, 0x40, servo N on channel N-1.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define VERSION "1.1"

const int PIN_SDA = 9;
const int PIN_SCL = 10;
const uint8_t PCA_ADDR = 0x40;
const uint32_t OSC_HZ = 27000000;

const int ACTIVE_COUNT = 16;
const int GRID_COLS = 4;
const int SERVO_HZ = 50;
const int FRAME_MS = 1000 / SERVO_HZ;
const int MIN_US   = 500;
const int MAX_US   = 2400;

const float AMP_RATE   = 2.0;    // deg/s that each servo's amplitude eases at
const float GLIDE_RATE = 10.0;   // deg/s for centre changes and start-up
const int   WAKE_GAP_MS = 250;   // cold start: one servo at a time

uint16_t mask = 1;               // bit i = servo i+1 sways
float ampTarget = 3.0, amp[16];
float period = 6.0;
float centre = 90.0, centreTarget = 90.0;
bool  running = false;
float phase = 0;
int   pattern = 0, patternNext = 0;
float lagDeg = 90, lagNext = 90;
float offset[16], scatter[16];
unsigned long frameMs = 0;

Adafruit_PWMServoDriver pca(PCA_ADDR);
bool pcaOk = false;
uint16_t lastOff[16];
unsigned long busChecks = 0, busBad = 0;

uint16_t degToTick(float deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  float us = MIN_US + (deg / 180.0f) * (MAX_US - MIN_US);
  return (uint16_t)lroundf(us * 4096.0f * SERVO_HZ / 1000000.0f);
}

void writeDeg(int i, float deg) {
  if (!pcaOk) return;
  uint16_t off = degToTick(deg);
  if (off == lastOff[i]) return;
  pca.setPWM(i, 0, off);
  lastOff[i] = off;
}

void checkBus() {
  static int ch = 0;
  if (lastOff[ch]) {
    busChecks++;
    if (pca.getPWM(ch, true) != lastOff[ch]) busBad++;
  }
  ch = (ch + 1) % ACTIVE_COUNT;
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(PCA_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

// what the PCA is still outputting from the last sketch; -1 = no pulse
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

void applyPattern() {
  pattern = patternNext;
  lagDeg = lagNext;
  for (int i = 0; i < ACTIVE_COUNT; i++) {
    int row = i / GRID_COLS, col = i % GRID_COLS;
    if (pattern == 1)      offset[i] = -(col + row) * lagDeg * PI / 180.0f;
    else if (pattern == 2) offset[i] = scatter[i];
    else                   offset[i] = 0;
  }
}

bool patternPending() { return patternNext != pattern || lagNext != lagDeg; }

void printState() {
  int n = 0;
  float a = 0;
  for (int i = 0; i < ACTIVE_COUNT; i++) {
    if (mask >> i & 1) n++;
    if (amp[i] > a) a = amp[i];
  }
  Serial.printf("st n %d amp %.2f/%.2f period %.1f centre %.1f run %d pattern %d lag %.0f frame %lu i2c %lu/%lu\n",
                n, a, ampTarget, period, centreTarget, running ? 1 : 0,
                patternNext, lagNext, frameMs, busBad, busChecks);
}

void handleLine(char *line) {
  char c = toupper(line[0]);
  float v = atof(line + 1);
  switch (c) {
    case 'M': mask = (uint16_t)strtoul(line + 1, NULL, 10); break;
    case 'S': if (v >= 1 && v <= ACTIVE_COUNT) mask = 1 << ((int)v - 1); break;  // v1.0 compat
    case 'A': ampTarget = constrain(v, 0.0f, 30.0f); break;
    case 'P': period = constrain(v, 0.5f, 60.0f); break;
    case 'C': centreTarget = constrain(v, 60.0f, 120.0f); break;
    case 'W': if (v >= 0 && v <= 2) patternNext = (int)v; break;
    case 'L': lagNext = constrain(v, 0.0f, 180.0f); break;
    case 'G': running = true; break;
    case 'X': running = false; break;
    case '?': break;
    default: Serial.printf("err unknown '%s'\n", line); return;
  }
  printState();
}

void readSerial() {
  static char buf[32];
  static int n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      buf[n] = 0;
      if (n) handleLine(buf);
      n = 0;
    } else if (n < (int)sizeof(buf) - 1) {
      buf[n++] = ch;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println();
  Serial.println("=== Tuft PCA9685 tune v" VERSION " ===");
  Serial.println("Sway tuner for the 4x4 grid: any set of servos, in step or as a wave (see tune.html)");
  Serial.println("https://github.com/sui001/tuft/tree/master/firmware/tuft_pca16_tune_v1_1");
  randomSeed(esp_random());
  for (int i = 0; i < ACTIVE_COUNT; i++) {
    scatter[i] = random(0, 6283) / 1000.0f;
    amp[i] = 0;
  }
  applyPattern();

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Wire.beginTransmission(PCA_ADDR);
  pcaOk = Wire.endTransmission() == 0;
  float live[16];
  if (pcaOk) {
    readLivePositions(live);
    pca.begin();
    pca.setOscillatorFrequency(OSC_HZ);
    pca.setPWMFreq(SERVO_HZ);
    delay(10);
  }
  Serial.printf("PCA9685 at 0x%02X: %s\n", PCA_ADDR, pcaOk ? "OK" : "NOT FOUND");
  if (!pcaOk) return;

  // warm channels glide to centre, cold ones go one at a time
  for (int i = 0; i < ACTIVE_COUNT; i++) if (live[i] >= 0) writeDeg(i, live[i]);
  for (int i = 0; i < ACTIVE_COUNT; i++)
    if (live[i] < 0) { writeDeg(i, centre); live[i] = centre; delay(WAKE_GAP_MS); }
  bool moving = true;
  while (moving) {
    moving = false;
    for (int i = 0; i < ACTIVE_COUNT; i++) {
      float d = centre - live[i];
      float step = GLIDE_RATE / SERVO_HZ;
      if (fabsf(d) > step) { live[i] += d > 0 ? step : -step; moving = true; }
      else live[i] = centre;
      writeDeg(i, live[i]);
    }
    delay(FRAME_MS);
  }
  Serial.println("ok ready, all at centre, send G to sway");
  printState();
}

void loop() {
  unsigned long frameStart = millis();
  readSerial();
  if (!pcaOk) {
    static unsigned long last = 0;
    if (millis() - last > 3000) { last = millis(); Serial.println("PCA9685 NOT FOUND, check SDA 9 / SCL 10 / 3V3"); }
    delay(50);
    return;
  }

  static unsigned long lastMs = millis();
  float dt = (millis() - lastMs) / 1000.0f;
  lastMs = millis();
  if (dt > 0.1f) dt = 0.1f;

  float dc = centreTarget - centre, cs = GLIDE_RATE * dt;
  centre = fabsf(dc) > cs ? centre + (dc > 0 ? cs : -cs) : centreTarget;

  // each servo's amplitude eases on its own; a pattern change eases all out first
  bool pending = patternPending();
  bool allZero = true;
  for (int i = 0; i < ACTIVE_COUNT; i++) {
    float want = (running && !pending && (mask >> i & 1)) ? ampTarget : 0;
    float da = want - amp[i], as = AMP_RATE * dt;
    amp[i] = fabsf(da) > as ? amp[i] + (da > 0 ? as : -as) : want;
    if (amp[i] != 0) allZero = false;
  }
  if (pending && allZero) { applyPattern(); printState(); }

  phase += 2.0f * PI * dt / period;
  if (phase > 2.0f * PI) phase -= 2.0f * PI;

  for (int i = 0; i < ACTIVE_COUNT; i++)
    writeDeg(i, centre + amp[i] * sinf(phase + offset[i]));
  checkBus();

  static unsigned long lastReport = 0;
  if (millis() - lastReport > 1000) { lastReport = millis(); printState(); }

  // hold a steady 50Hz frame; keep listening while we wait
  while (millis() - frameStart < FRAME_MS) readSerial();
  frameMs = millis() - frameStart;
}
