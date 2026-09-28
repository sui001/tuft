/*
 * PROJECT: Tuft (notwled servo console)
 * DEVICE:  3x3 rig, MECHANICAL diagnostic, ESP32-S3 SuperMini
 *
 * Not an effect. A measurement.
 *
 * The firmware causes are exhausted. v2.0/v3.0 fixed the real ones
 * (10-bit PWM -> 8.1mm steps, and a channel allocator that silently killed
 * two servos). After that: raising the frame rate 50->100Hz made no visible
 * difference, and dither from 0 to +/-10us made no visible difference. A
 * +/-10us command wobble is 4.2mm at a 250mm tip and should have visibly
 * shaken the arm if the servo were tracking it. It didn't. That points away
 * from signal precision entirely and towards mechanical slop, which no
 * amount of signal work can fix.
 *
 * This sketch separates the three remaining candidates so the next spend
 * (better servos? shorter arm? stiffer arm?) is decided by evidence.
 *
 * PHASE 1  HOLD STILL (10s)
 *   Every servo commanded to a fixed 90 degrees and left there. Nothing
 *   moves in software at all.
 *   -> If the tips still twitch, drift or hunt, that is the servo's own
 *      control loop oscillating around its target. Better (digital) servos
 *      fix this. A shorter arm reduces how much you see it.
 *   -> If the tips are dead still, the servos are fine at rest and the
 *      problem only exists in motion. Go to phase 2 and 3.
 *
 * PHASE 2  BACKLASH (approach the same point from both sides)
 *   Approaches 90 degrees from below (60 -> 90), stops, holds 4s. Then
 *   approaches the SAME 90 degrees from above (120 -> 90), stops, holds 4s.
 *   Repeats.
 *   -> If the tip rests in two different places, the gap between them IS
 *      the backlash, and you can measure it with a ruler. This is slop in
 *      the gear train, amplified by arm length. No firmware fixes it.
 *      Metal-gear servos reduce it; a shorter arm reduces what you see.
 *   -> If it lands in the same spot both times, backlash isn't your problem.
 *
 * PHASE 3  SLOW CREEP, ONE SERVO (servo5, the centre)
 *   A single servo creeps 85 -> 95 degrees over 20 seconds, alone, so the
 *   stepping can be watched in isolation without seven others distracting.
 *   Commanded resolution here is 0.116 deg (0.5mm at the tip), so anything
 *   chunkier than that is the servo or the mechanics, not the signal.
 *
 * WHAT TO DO WITH THE ANSWER:
 *   The single biggest lever is ARM LENGTH, and it is free. Every error
 *   source here (deadband, backlash, hunting, resonance, quantisation) is
 *   amplified linearly by how far out you measure it. At 250mm you are
 *   magnifying all of them ~20x versus the ~12mm horn these servos are
 *   designed around. Halving the arm halves every millimetre of visible
 *   error, without changing a servo or a line of code.
 *
 * WIRING: GPIO 1,3,4,5,6,7,8,10 -> servo 1..8. GPIO 2,9,11 unused.
 */

#define VERSION "3.5"

// ---------------- CONFIG ----------------

const int SERVO_COUNT = 8;
const int SERVO_PINS[SERVO_COUNT] = {1, 3, 4, 5, 6, 7, 8, 10};
const int CREEP_SERVO = 4;   // index 4 = servo5, the centre of the grid

const float SHAFT_TO_TIP_MM = 250.0;

const int SERVO_HZ = 50;
const int RES_BITS = 14;
const int MIN_US   = 500;
const int MAX_US   = 2400;

const float HOLD_DEG = 90.0;
const unsigned long HOLD_STILL_MS = 10000;

const float BACKLASH_LOW  = 60.0;
const float BACKLASH_HIGH = 120.0;
const float BACKLASH_TARGET = 90.0;
const unsigned long BACKLASH_SETTLE_MS = 4000;
const int BACKLASH_REPS = 2;

const float CREEP_FROM = 85.0;
const float CREEP_TO   = 95.0;
const unsigned long CREEP_MS = 20000;

// servos 2,5,8 sit one spline tooth high, see v3.2
float angleTrim[SERVO_COUNT] = {0.0, -15.0, 0.0, 0.0, -15.0, 0.0, 0.0, -15.0};

// -----------------------------------------

bool attachedOk[SERVO_COUNT];
const uint32_t TICKS = 1UL << RES_BITS;

uint32_t usToDuty(float us) {
  return (uint32_t)lroundf(us * (float)TICKS / (1000000.0f / SERVO_HZ));
}

float degToUs(float deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  return MIN_US + (deg / 180.0f) * (MAX_US - MIN_US);
}

void writeDeg(int i, float deg) {
  if (!attachedOk[i]) return;
  ledcWrite(SERVO_PINS[i], usToDuty(degToUs(deg + angleTrim[i])));
}

void writeAll(float deg) {
  for (int i = 0; i < SERVO_COUNT; i++) writeDeg(i, deg);
}

// glide all servos between two angles over a set duration
void glideAll(float from, float to, unsigned long ms) {
  unsigned long t0 = millis();
  unsigned long elapsed;
  while ((elapsed = millis() - t0) < ms) {
    float k = (float)elapsed / (float)ms;
    writeAll(from + (to - from) * k);
    delay(1000 / SERVO_HZ);
  }
  writeAll(to);
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.printf("\nTuft mechanical diagnostic, firmware v%s\n", VERSION);

  for (int i = 0; i < SERVO_COUNT; i++) {
    attachedOk[i] = ledcAttach(SERVO_PINS[i], SERVO_HZ, RES_BITS);
  }
  Serial.print("attached:");
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.printf(" s%d/GPIO%d=%s", i + 1, SERVO_PINS[i],
                  attachedOk[i] ? "OK" : "FAIL");
  }
  Serial.println();

  writeAll(HOLD_DEG);
  delay(2000);
}

void loop() {
  // ---------- PHASE 1: hold dead still ----------
  Serial.println("\n== PHASE 1: HOLD STILL ==");
  Serial.println("Nothing is being commanded to move. Watch the tips.");
  Serial.println("Any twitch/drift/hum = the servo's own loop hunting.");
  writeAll(HOLD_DEG);
  delay(HOLD_STILL_MS);

  // ---------- PHASE 2: backlash, same point from both sides ----------
  Serial.println("\n== PHASE 2: BACKLASH ==");
  Serial.printf("Approaching %.0f deg from BELOW then from ABOVE.\n", BACKLASH_TARGET);
  Serial.println("If the tips rest in two different spots, that gap is backlash.");
  Serial.println("Measure it with a ruler, it is the number that decides");
  Serial.println("whether metal-gear servos are worth buying.");

  for (int rep = 0; rep < BACKLASH_REPS; rep++) {
    Serial.printf("  rep %d: from below (%.0f -> %.0f), settling...\n",
                  rep + 1, BACKLASH_LOW, BACKLASH_TARGET);
    writeAll(BACKLASH_LOW);
    delay(1500);
    glideAll(BACKLASH_LOW, BACKLASH_TARGET, 2500);
    delay(BACKLASH_SETTLE_MS);   // <-- mark where the tip sits

    Serial.printf("  rep %d: from above (%.0f -> %.0f), settling...\n",
                  rep + 1, BACKLASH_HIGH, BACKLASH_TARGET);
    writeAll(BACKLASH_HIGH);
    delay(1500);
    glideAll(BACKLASH_HIGH, BACKLASH_TARGET, 2500);
    delay(BACKLASH_SETTLE_MS);   // <-- compare against the mark
  }

  // ---------- PHASE 3: one servo, very slow ----------
  Serial.println("\n== PHASE 3: SLOW CREEP, ONE SERVO ==");
  Serial.printf("servo%d (GPIO%d) alone, %.0f -> %.0f deg over %lus.\n",
                CREEP_SERVO + 1, SERVO_PINS[CREEP_SERVO],
                CREEP_FROM, CREEP_TO, CREEP_MS / 1000);
  Serial.printf("Commanded step is 0.116 deg (%.2fmm at the tip). Anything\n",
                SHAFT_TO_TIP_MM * sinf(radians(0.1156f)));
  Serial.println("chunkier than that is mechanical, not signal.");

  writeAll(HOLD_DEG);
  delay(1000);
  writeDeg(CREEP_SERVO, CREEP_FROM);
  delay(1500);

  unsigned long t0 = millis();
  unsigned long elapsed;
  while ((elapsed = millis() - t0) < CREEP_MS) {
    float k = (float)elapsed / (float)CREEP_MS;
    writeDeg(CREEP_SERVO, CREEP_FROM + (CREEP_TO - CREEP_FROM) * k);
    delay(1000 / SERVO_HZ);
  }
  writeDeg(CREEP_SERVO, CREEP_TO);
  delay(2000);

  Serial.println("\n-- cycle complete, repeating --");
}
