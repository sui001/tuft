/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  4x4 grid, 16 SG92R servos on one PCA9685, ESP32-S3 SuperMini
 *
 * tune v1.0. One servo, hand-tuned sway, to settle whether the PCA9685 can
 * do smooth gentle motion at all before ripping it out. Every other servo
 * holds at CENTRE and is never rewritten. Driven from tune.html over
 * Web Serial (Chrome/Edge), or by typing the commands below into any
 * serial monitor at 115200.
 *
 *   S n     sway servo n (1-16, bottom-left first). The old one eases home.
 *   A deg   sway amplitude, +- degrees around centre (0-30)
 *   P s     sway period in seconds (0.5-60)
 *   C deg   centre for all servos (60-120). Glides there.
 *   G / X   go / stop. Stop eases the sway out to centre, never snaps.
 *   ?       print state
 *
 * Changes never jump: amplitude eases toward its target at AMP_RATE deg/s,
 * the phase is accumulated so a period change doesn't skip, and switching
 * servo eases the old one to zero before the new one fades in.
 *
 * Status lines start with "st " so the page can parse them:
 *   st servo 4 amp 3.00/3.00 period 6.0 centre 90.0 run 1 tick 307 i2c 0/812
 * "tick" is the PCA9685 OFF value on the swaying channel. One tick at 50Hz
 * is 4.88us = 0.46 deg = ~9mm at a 1.1m tip, which is what we're judging.
 *
 * WIRING as v7: PCA9685 SDA GPIO9, SCL GPIO10, 0x40, servo N on channel N-1.
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

const float AMP_RATE  = 2.0;     // deg/s that amplitude eases at
const float GLIDE_RATE = 10.0;   // deg/s for centre changes and start-up
const int   WAKE_GAP_MS = 250;   // cold start: one servo at a time

int   servo = 1;                 // 1-based, the one swaying
int   nextServo = 1;
float ampTarget = 3.0, amp = 0;
float period = 6.0;
float centre = 90.0, centreTarget = 90.0;
bool  running = false;
float phase = 0;

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

void checkBus(int ch) {
  if (!lastOff[ch]) return;
  busChecks++;
  if (pca.getPWM(ch, true) != lastOff[ch]) busBad++;
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

void printState() {
  Serial.printf("st servo %d amp %.2f/%.2f period %.1f centre %.1f run %d tick %u i2c %lu/%lu\n",
                servo, amp, ampTarget, period, centreTarget, running ? 1 : 0,
                lastOff[servo - 1], busBad, busChecks);
}

void handleLine(char *line) {
  char c = toupper(line[0]);
  float v = atof(line + 1);
  switch (c) {
    case 'S': if (v >= 1 && v <= ACTIVE_COUNT) nextServo = (int)v; break;
    case 'A': ampTarget = constrain(v, 0.0f, 30.0f); break;
    case 'P': period = constrain(v, 0.5f, 60.0f); break;
    case 'C': centreTarget = constrain(v, 60.0f, 120.0f); break;
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
  Serial.println("One-servo sway tuner for the 4x4 grid, commands over serial (see tune.html)");
  Serial.println("https://github.com/sui001/tuft/tree/master/firmware/tuft_pca16_tune_v1_0");

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
    delay(1000 / SERVO_HZ);
  }
  Serial.println("ok ready, all at centre, send G to sway");
  printState();
}

void loop() {
  if (!pcaOk) {
    static unsigned long last = 0;
    if (millis() - last > 3000) { last = millis(); Serial.println("PCA9685 NOT FOUND, check SDA 9 / SCL 10 / 3V3"); }
    readSerial();
    delay(50);
    return;
  }
  readSerial();

  static unsigned long lastMs = millis();
  float dt = (millis() - lastMs) / 1000.0f;
  lastMs = millis();
  if (dt > 0.1f) dt = 0.1f;

  // centre glides for every servo
  float centreBefore = centre;
  float dc = centreTarget - centre, cs = GLIDE_RATE * dt;
  centre = fabsf(dc) > cs ? centre + (dc > 0 ? cs : -cs) : centreTarget;

  // amplitude eases; a servo switch eases out first, then the new one fades in
  float want = (running && nextServo == servo) ? ampTarget : 0;
  float da = want - amp, as = AMP_RATE * dt;
  amp = fabsf(da) > as ? amp + (da > 0 ? as : -as) : want;
  if (nextServo != servo && amp == 0) {
    writeDeg(servo - 1, centre);
    servo = nextServo;
    phase = 0;
    printState();
  }

  phase += 2.0f * PI * dt / period;
  if (phase > 2.0f * PI) phase -= 2.0f * PI;

  if (centre != centreBefore)
    for (int i = 0; i < ACTIVE_COUNT; i++) if (i != servo - 1) writeDeg(i, centre);
  writeDeg(servo - 1, centre + amp * sinf(phase));
  checkBus(servo - 1);

  static unsigned long lastReport = 0;
  if (millis() - lastReport > 1000) { lastReport = millis(); printState(); }

  delay(1000 / SERVO_HZ);
}
