/*
 * TEST: all servos (Group 31 arm)
 *
 * 1. Moves every joint to relative 0 and holds for 5 s:
 *      base facing forward, main arm upright, second arm level.
 *    If any joint is not at that position, change its SERVO_ZERO_xxx value.
 * 2. Small direction check: each joint moves +10 then back. Base should turn
 *    LEFT (towards A and B), main arm should lean FORWARD, second arm should tilt UP. If one goes
 *    the other way, flip its xxx_DIR.
 * 3. Full sweep of each joint through its whole allowed range:
 *      base -90 to +90, main arm -15 to +50, second arm -90 to +60.
 *    The two main arm servos always move in opposite directions.
 * 4. Then manual control from the Serial Monitor (115200, Newline):
 *      b30   base to +30        m20   main arm to +20     e-40   2nd arm to -40
 *      b+    nudge base +1      m-    nudge main -1
 *      b++   nudge base +5      e--   nudge 2nd -5
 *      z     back to zero       s     repeat the sweep    p     print angles
 *
 * FINDING THE ZERO POINTS (do this first, with z):
 *   Nudge each joint with b+ b- m+ m- e+ e- (1 degree, ++ / -- for 5) until the
 *   base faces straight forward, the main arm stands upright and the second
 *   arm is level (use a set square or a phone level). If the two main arm
 *   servos buzz or strain against each other at rest, nudge just one of them
 *   with l+ l- (left servo) or r+ r- (right servo) until the buzzing stops.
 *   Then type c: it prints the four SERVO_ZERO lines to paste into
 *   arm_controller.ino (and the test sketches), replacing the old ones.
 *
 * Pins: base D2, main left D3, main right D4, second arm D5.
 * Copy any changed SERVO_ZERO and DIR values into arm_controller.ino.
 */

#include <Servo.h>

const uint8_t PIN_BASE = 2, PIN_MAIN_LEFT = 3, PIN_MAIN_RIGHT = 4, PIN_ELBOW = 5;

const int SERVO_ZERO_BASE  = 90;
const int SERVO_ZERO_LEFT  = 90;
const int SERVO_ZERO_RIGHT = 90;
const int SERVO_ZERO_ELBOW = 90;
const float BASE_GAIN   = 1.0;    // same as arm_controller.ino (from tests/test_base)
const int   BASE_US_MIN = 500, BASE_US_MAX = 2500;   // SG92R full travel
const int8_t BASE_DIR = 1, MAIN_DIR = -1, ELBOW_DIR = 1;   // MAIN_DIR -1: same as arm_controller

enum Joint : uint8_t { J_BASE = 0, J_MAIN = 1, J_ELBOW = 2, NJ = 3 };
const float REL_MIN[NJ] = {-90, -15, -90};
const float REL_MAX[NJ] = { 90,  50,  60};   // same as arm_controller.ino
const char *NAMES[NJ] = {"base", "main arm", "second arm"};
const char KEYS[NJ] = {'b', 'm', 'e'};

Servo sBase, sLeft, sRight, sElbow;
float cur[NJ] = {0, 0, 0};
int offLeft = 0, offRight = 0;   // single-servo nudges on the main arm (l and r commands)

int rawClamp(float v) { return constrain((int)round(v), 0, 180); }

// Servo pulse for an angle, keeping fractions of a degree. write() only moves
// in whole degrees, which makes slow moves step visibly; writeMicroseconds()
// moves in about 0.1 degree steps. 544 and 2400 us are the Servo library's own
// 0 and 180 degree pulses, so the angles match what write() would give.
int pulseFor(float deg) {
  deg = constrain(deg, 0.0, 180.0);
  return (int)(544 + deg * (2400 - 544) / 180.0 + 0.5);
}

// Base pulse: same scale, with the gain, and the SG92R's wider pulse range.
int basePulse(float rel) {
  int us = (int)(544 + (SERVO_ZERO_BASE + BASE_DIR * rel * BASE_GAIN) * (2400 - 544) / 180.0 + 0.5);
  return constrain(us, BASE_US_MIN, BASE_US_MAX);
}

void writeJoint(uint8_t j, float rel) {
  rel = constrain(rel, REL_MIN[j], REL_MAX[j]);
  cur[j] = rel;
  if (j == J_BASE) {
    sBase.writeMicroseconds(basePulse(rel));
  } else if (j == J_MAIN) {
    sLeft.writeMicroseconds(pulseFor(SERVO_ZERO_LEFT + offLeft + MAIN_DIR * rel));
    sRight.writeMicroseconds(pulseFor(SERVO_ZERO_RIGHT + offRight - MAIN_DIR * rel));
  } else {
    sElbow.writeMicroseconds(pulseFor(SERVO_ZERO_ELBOW + ELBOW_DIR * rel));
  }
}

// Half a degree every 20 ms (25 deg/s), in fine pulse steps, so the sweep is
// slow and smooth.
void glide(uint8_t j, float target) {
  target = constrain(target, REL_MIN[j], REL_MAX[j]);
  while (fabs(cur[j] - target) > 0.25) {
    writeJoint(j, cur[j] + (target > cur[j] ? 0.5 : -0.5));
    delay(20);
  }
  writeJoint(j, target);
}

void printAngles() {
  Serial.print(F("Now: base ")); Serial.print(cur[J_BASE], 0);
  Serial.print(F(" degrees, main arm ")); Serial.print(cur[J_MAIN], 0);
  Serial.print(F(" degrees (left servo at ")); Serial.print(rawClamp(SERVO_ZERO_LEFT + MAIN_DIR * cur[J_MAIN]));
  Serial.print(F(", right servo at ")); Serial.print(rawClamp(SERVO_ZERO_RIGHT - MAIN_DIR * cur[J_MAIN]));
  Serial.print(F("), second arm ")); Serial.print(cur[J_ELBOW], 0); Serial.println(F(" degrees."));
}

// The SERVO_ZERO values that make the current position the new rest position.
void printZeros() {
  Serial.println(F("\nPaste these over the SERVO_ZERO lines in arm_controller.ino (and the test sketches):"));
  Serial.print(F("const int SERVO_ZERO_BASE  = ")); Serial.print(rawClamp(SERVO_ZERO_BASE + BASE_DIR * cur[J_BASE] * BASE_GAIN)); Serial.println(';');
  Serial.print(F("const int SERVO_ZERO_LEFT  = ")); Serial.print(rawClamp(SERVO_ZERO_LEFT + offLeft + MAIN_DIR * cur[J_MAIN])); Serial.println(';');
  Serial.print(F("const int SERVO_ZERO_RIGHT = ")); Serial.print(rawClamp(SERVO_ZERO_RIGHT + offRight - MAIN_DIR * cur[J_MAIN])); Serial.println(';');
  Serial.print(F("const int SERVO_ZERO_ELBOW = ")); Serial.print(rawClamp(SERVO_ZERO_ELBOW + ELBOW_DIR * cur[J_ELBOW])); Serial.println(';');
  Serial.println(F("Only the whole-degree part is kept, so finish with 1 degree nudges."));
}

void goZero() {
  for (uint8_t j = 0; j < NJ; j++) glide(j, 0);
  printAngles();
}

void directionCheck() {
  Serial.println(F("\nDIRECTION CHECK: each joint moves 10 degrees one way and back. Watch which way it goes."));
  const char *expect[NJ] = {"should turn LEFT, towards A and B", "should lean FORWARD", "should tilt UP"};
  for (uint8_t j = 0; j < NJ; j++) {
    Serial.print(F("Moving the ")); Serial.print(NAMES[j]); Serial.print(F(" 10 degrees. It ")); Serial.print(expect[j]);
    Serial.println(F(". If not, flip its DIR value."));
    glide(j, 10);
    delay(1500);
    glide(j, 0);
    delay(500);
  }
}

void sweepAll() {
  Serial.println(F("\nFULL SWEEP: each joint goes slowly to both ends of its range and back to the middle."));
  for (uint8_t j = 0; j < NJ; j++) {
    Serial.print(F("Sweeping the ")); Serial.print(NAMES[j]);
    Serial.print(F(" from ")); Serial.print(REL_MIN[j], 0);
    Serial.print(F(" to ")); Serial.print(REL_MAX[j], 0); Serial.println(F(" degrees."));
    glide(j, REL_MIN[j]); printAngles(); delay(800);
    glide(j, REL_MAX[j]); printAngles(); delay(800);
    glide(j, 0);          printAngles(); delay(500);
  }
  Serial.println(F("Sweep finished. You can now move joints by typing commands:"));
  Serial.println(F("  b30 = base to 30 degrees, m-10 = main arm to -10, e20 = second arm to 20"));
  Serial.println(F("  b+ / b- = nudge 1 degree, b++ / b-- = nudge 5 degrees (same for m and e)"));
  Serial.println(F("  z = back to rest, s = sweep again, p = print the angles"));
  Serial.println(F("  l+ / r+ = nudge only the left / right main arm servo, c = print the SERVO_ZERO lines to paste"));
}

void handle(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = tolower(cmd.charAt(0));
  if (c == 'z') { goZero(); return; }
  if (c == 's') { sweepAll(); return; }
  if (c == 'p') { printAngles(); return; }
  if (c == 'c') { printZeros(); return; }
  if (c == 'l' || c == 'r') {   // one main arm servo on its own, to stop the pair fighting
    String rest = cmd.substring(1);
    int step = rest == "+" ? 1 : rest == "-" ? -1 : rest == "++" ? 5 : rest == "--" ? -5 : 0;
    if (step == 0) { Serial.println(F("Use l+ l- l++ l-- or r+ r- r++ r--.")); return; }
    if (c == 'l') offLeft += step; else offRight += step;
    writeJoint(J_MAIN, cur[J_MAIN]);
    Serial.print(F("Single-servo nudge: left ")); Serial.print(offLeft);
    Serial.print(F(", right ")); Serial.print(offRight); Serial.println(F(" degrees. Type c for the zero lines."));
    return;
  }
  for (uint8_t j = 0; j < NJ; j++) {
    if (c != KEYS[j]) continue;
    String rest = cmd.substring(1);
    if (rest == "+") glide(j, cur[j] + 1);
    else if (rest == "-") glide(j, cur[j] - 1);
    else if (rest == "++") glide(j, cur[j] + 5);
    else if (rest == "--") glide(j, cur[j] - 5);
    else glide(j, rest.toInt());   // absolute angle, e.g. 30 or -40
    printAngles();
    return;
  }
  Serial.println(F("Command not recognised. Try b30, m-10, e+, l+, r-, z, s, p or c."));
}

// Soft start: short bursts of pulses with pauses, so each servo creeps to rest
// instead of jumping there (same as arm_controller.ino). Base last and slowest.
void softAttach(Servo &s, uint8_t pin, uint8_t bursts) {
  for (uint8_t i = 0; i < bursts; i++) { s.attach(pin, BASE_US_MIN, BASE_US_MAX); delay(22); s.detach(); delay(110); }
  s.attach(pin, BASE_US_MIN, BASE_US_MAX);
}
void softAttachAll() {
  softAttach(sElbow, PIN_ELBOW, 15);
  for (uint8_t i = 0; i < 15; i++) {   // the two main arm servos together, so they never fight
    sLeft.attach(PIN_MAIN_LEFT); sRight.attach(PIN_MAIN_RIGHT); delay(22);
    sLeft.detach(); sRight.detach(); delay(110);
  }
  sLeft.attach(PIN_MAIN_LEFT); sRight.attach(PIN_MAIN_RIGHT);
  softAttach(sBase, PIN_BASE, 30);
}

void setup() {
  Serial.begin(115200);
  for (uint8_t j = 0; j < NJ; j++) writeJoint(j, 0);
  softAttachAll();
  for (uint8_t j = 0; j < NJ; j++) writeJoint(j, 0);

  Serial.println(F("\nSERVO TEST"));
  Serial.println(F("Holding the rest position for 5 seconds: base facing forward, main arm upright, second arm level."));
  Serial.println(F("If any joint is not in that position, adjust its SERVO_ZERO value."));
  printAngles();
  delay(5000);

  directionCheck();
  sweepAll();
}

void loop() {
  if (Serial.available()) handle(Serial.readStringUntil('\n'));
}
