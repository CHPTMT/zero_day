/*
 * MECN3066A Mechatronics II, Group 31
 * TEST: base servo only (D2)
 *
 * Only the base servo is driven; the other servos can stay unplugged. Use it
 * to fit the turntable at the true middle, check the full swing, and find
 * SERVO_ZERO_BASE. Moves are eased and slow (18 degrees per second, the same
 * as the base in arm_controller), and the servo creeps to the middle at
 * power up, so nothing jerks.
 *
 * Fitting the turntable:
 *   1. Take the turntable (or gear) off the servo spline.
 *   2. Upload this, power the servo. It creeps to the middle (90) and holds.
 *   3. Fit the turntable facing straight forward, on the nearest tooth.
 *   4. Nudge with + and - until it faces exactly forward, then type z and
 *      paste the line it prints into every sketch.
 *
 * Checking the base turns as far as it is told (BASE_GAIN):
 *   5. Type a: the base turns 80 degrees left, to where A and B should be.
 *   6. If it stops short of (or past) A and B, nudge with + and - until the
 *      arm points straight at them, then type k.
 *   7. Do the same with e (80 right, Point E) and k again.
 *   8. Paste the BASE_GAIN line it prints into every sketch.
 *
 * Commands (Serial Monitor, 115200 baud, line ending "New Line"):
 *   c        go to the middle and hold
 *   l  r     go slowly to the far left / far right (85 degrees)
 *   a  e     go to the A and B side (80 left) / the E side (80 right)
 *   s        sweep: far left, far right, then the middle
 *   + -      nudge 1 degree left / right      ++ --  nudge 5 degrees
 *   z        make the current position the middle and print SERVO_ZERO_BASE
 *   k        the arm now points at A/B (after a) or E (after e): work out BASE_GAIN
 *   f        free: let go of the servo so the turntable turns by hand
 *   h        hold again where it is told to be
 *   p        print where it is
 */

#include <Servo.h>

const uint8_t PIN_BASE   = 2;
int   SERVO_ZERO_BASE    = 90;     // copy from arm_controller.ino if it is already set
const int8_t BASE_DIR    = 1;      // same as arm_controller.ino: + is the arm's left (towards A and B)
const float BASE_SPEED_DEG_S = 18;
const float SWEEP_DEG    = 85;
const float STATION_DEG  = 80;
const uint8_t STEP_MS    = 20;
const int BASE_US_MIN    = 500;    // SG92R full travel, same as arm_controller.ino
const int BASE_US_MAX    = 2500;

Servo base;
float rel = 0;          // relative angle: 0 = middle, + = left, - = right
bool attached = false;
float gainSum = 0;      // BASE_GAIN measurements so far
uint8_t gainCount = 0;

// Same scale as arm_controller, without the gain (this sketch measures it).
int pulseFor(float deg) {
  int us = (int)(544 + deg * (2400 - 544) / 180.0 + 0.5);
  return constrain(us, BASE_US_MIN, BASE_US_MAX);
}

void writeBase(float r) {
  rel = constrain(r, -100.0, 100.0);
  base.writeMicroseconds(pulseFor(SERVO_ZERO_BASE + BASE_DIR * rel));
}

// Creep to the angle already written, in short bursts, instead of jumping.
void softAttach() {
  for (uint8_t i = 0; i < 30; i++) {
    base.attach(PIN_BASE, BASE_US_MIN, BASE_US_MAX);
    delay(22);
    base.detach();
    delay(110);
  }
  base.attach(PIN_BASE, BASE_US_MIN, BASE_US_MAX);
  attached = true;
}

// Eased move at no more than BASE_SPEED_DEG_S.
void glide(float to) {
  if (!attached) { Serial.println(F("The servo is free (f). Type h to hold it first.")); return; }
  float from = rel;
  unsigned long dur = max(300UL, (unsigned long)(fabs(to - from) * 1571.0 / BASE_SPEED_DEG_S));
  uint16_t steps = max(1UL, dur / STEP_MS);
  for (uint16_t s = 1; s <= steps; s++) {
    writeBase(from + (to - from) * (0.5 - 0.5 * cos(PI * s / steps)));
    delay(STEP_MS);
  }
}

void printWhere() {
  Serial.print(F("Base at "));
  if (fabs(rel) < 0.5) Serial.print(F("the middle"));
  else { Serial.print(fabs(rel), 0); Serial.print(rel > 0 ? F(" degrees left") : F(" degrees right")); }
  Serial.print(F(" (servo value ")); Serial.print(SERVO_ZERO_BASE + BASE_DIR * rel, 0);
  Serial.print(F(", middle is ")); Serial.print(SERVO_ZERO_BASE); Serial.println(F(")."));
}

void help() {
  Serial.println(F("Commands: c middle, l / r far left / right, a / e A-B side / E side, s sweep,"));
  Serial.println(F("          + - nudge 1 deg, ++ -- nudge 5 deg, z set this as the middle, k work out BASE_GAIN, f free, h hold, p where."));
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("\nBase servo test (D2). Creeping to the middle, about 4 seconds..."));
  writeBase(0);
  softAttach();
  printWhere();
  help();
}

void loop() {
  static char buf[8];
  static uint8_t n = 0;
  if (!Serial.available()) return;
  char ch = Serial.read();
  if (ch != '\n' && ch != '\r') { if (n < 7) buf[n++] = ch; return; }
  if (n == 0) return;
  buf[n] = '\0';
  n = 0;

  if      (!strcmp(buf, "c"))  { Serial.println(F("Going to the middle.")); glide(0); }
  else if (!strcmp(buf, "l"))  { Serial.println(F("Going to the far left.")); glide(SWEEP_DEG); }
  else if (!strcmp(buf, "r"))  { Serial.println(F("Going to the far right.")); glide(-SWEEP_DEG); }
  else if (!strcmp(buf, "a"))  { Serial.println(F("Going to the A and B side (80 left). Nudge onto A and B, then type k.")); glide(STATION_DEG); }
  else if (!strcmp(buf, "e"))  { Serial.println(F("Going to the E side (80 right). Nudge onto E, then type k.")); glide(-STATION_DEG); }
  else if (!strcmp(buf, "s"))  {
    Serial.println(F("Sweep: far left, far right, then the middle."));
    glide(SWEEP_DEG); delay(500); glide(-SWEEP_DEG); delay(500); glide(0);
  }
  else if (!strcmp(buf, "+"))  glide(rel + 1);
  else if (!strcmp(buf, "-"))  glide(rel - 1);
  else if (!strcmp(buf, "++")) glide(rel + 5);
  else if (!strcmp(buf, "--")) glide(rel - 5);
  else if (!strcmp(buf, "z"))  {
    SERVO_ZERO_BASE = (int)round(SERVO_ZERO_BASE + BASE_DIR * rel);
    rel = 0;
    writeBase(0);
    Serial.println(F("This is now the middle. Paste this line into arm_controller.ino and the other sketches:"));
    Serial.print(F("const int SERVO_ZERO_BASE  = ")); Serial.print(SERVO_ZERO_BASE); Serial.println(';');
  }
  else if (!strcmp(buf, "k"))  {
    if (fabs(rel) < 40) { Serial.println(F("Type a or e first, nudge the arm onto A/B or E, then k.")); return; }
    gainSum += fabs(rel) / STATION_DEG;
    gainCount++;
    Serial.print(F("To point at ")); Serial.print(rel > 0 ? F("A and B") : F("E"));
    Serial.print(F(" the base needed ")); Serial.print(fabs(rel), 0); Serial.println(F(" degrees instead of 80."));
    Serial.print(F("Paste this line into arm_controller.ino and the other sketches (average of ")); Serial.print(gainCount);
    Serial.println(F(" reading(s)):"));
    Serial.print(F("const float BASE_GAIN   = ")); Serial.print(gainSum / gainCount, 2); Serial.println(';');
  }
  else if (!strcmp(buf, "f"))  { base.detach(); attached = false; Serial.println(F("Servo free: the turntable can be turned by hand. Type h to hold again.")); }
  else if (!strcmp(buf, "h"))  { if (!attached) { writeBase(rel); softAttach(); } Serial.println(F("Holding.")); }
  else if (!strcmp(buf, "p"))  { }
  else { Serial.print(F("Unknown command: ")); Serial.println(buf); help(); return; }
  printWhere();
}
