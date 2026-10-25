/*
 * TEST: all servos (Group 21 arm)
 *
 * 1. Moves every joint to relative 0 and holds for 5 s:
 *      base facing forward, main arm upright, second arm level.
 *    If any joint is not at that position, set the rest position with the
 *    ZERO command of arm_controller (it is saved in EEPROM and used here too).
 * 2. Small direction check: each joint moves +10 then back. Base should turn
 *    LEFT (towards A and B), main arm should lean FORWARD, second arm should tilt UP. If one goes
 *    the other way, flip its xxx_DIR.
 * 3. Full sweep of each joint through its whole allowed range:
 *      base -90 to +90, main arm -15 to +50, second arm -90 to +45.
 *    The two main arm servos always move in opposite directions.
 * 4. Then manual control from the Serial Monitor (115200, Newline):
 *      b30   base to +30        m20   main arm to +20     e-40   2nd arm to -40
 *      b+    nudge base +1      m-    nudge main -1
 *      b++   nudge base +5      e--   nudge 2nd -5
 *      z     back to zero       s     repeat the sweep    p     print angles
 *
 * Pins: base D2, main left D3, main right D4, second arm D5.
 * Copy any changed DIR values into arm_controller.ino.
 */

#include <Servo.h>
#include <EEPROM.h>

const uint8_t PIN_BASE = 2, PIN_MAIN_LEFT = 3, PIN_MAIN_RIGHT = 4, PIN_ELBOW = 5;

const int8_t BASE_DIR = 1, MAIN_DIR = 1, ELBOW_DIR = 1;
// Rest positions (pulse width in microseconds, 1500 = middle of the servo's
// travel). Read from the EEPROM values saved by arm_controller's ZERO set-up,
// so every sketch uses the same rest position. Same as arm_controller.ino.
enum ServoId : uint8_t { S_BASE = 0, S_LEFT = 1, S_RIGHT = 2, S_ELBOW = 3, NS = 4 };
const int   ZERO_US_DEFAULT[NS] = {1500, 1500, 1500, 1500};
const float US_PER_DEG = 1000.0 / 90.0;
const int   SERVO_MIN_US = 500, SERVO_MAX_US = 2500;
const float BASE_GEAR_RATIO = 1.0;
int zeroUs[NS];

void loadZeros() {
  bool saved = EEPROM.read(8) == 0x5A;
  for (uint8_t i = 0; i < NS; i++) {
    int16_t v = ZERO_US_DEFAULT[i];
    if (saved) EEPROM.get(10 + 2 * i, v);
    zeroUs[i] = (v >= SERVO_MIN_US && v <= SERVO_MAX_US) ? v : ZERO_US_DEFAULT[i];
  }
}

int pulseAt(uint8_t s, float deg) {
  return constrain((int)round(zeroUs[s] + deg * US_PER_DEG), SERVO_MIN_US, SERVO_MAX_US);
}


enum Joint : uint8_t { J_BASE = 0, J_MAIN = 1, J_ELBOW = 2, NJ = 3 };
const float REL_MIN[NJ] = {-90, -15, -90};
const float REL_MAX[NJ] = { 90,  50,  45};
const char *NAMES[NJ] = {"base", "main arm", "second arm"};
const char KEYS[NJ] = {'b', 'm', 'e'};

Servo sBase, sLeft, sRight, sElbow;
float cur[NJ] = {0, 0, 0};


void writeJoint(uint8_t j, float rel) {
  rel = constrain(rel, REL_MIN[j], REL_MAX[j]);
  cur[j] = rel;
  if (j == J_BASE) {
    sBase.writeMicroseconds(pulseAt(S_BASE, BASE_DIR * BASE_GEAR_RATIO * rel));
  } else if (j == J_MAIN) {
    sLeft.writeMicroseconds(pulseAt(S_LEFT, MAIN_DIR * rel));
    sRight.writeMicroseconds(pulseAt(S_RIGHT, -MAIN_DIR * rel));
  } else {
    sElbow.writeMicroseconds(pulseAt(S_ELBOW, ELBOW_DIR * rel));
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
  Serial.print(F(" degrees (left servo ")); Serial.print(pulseAt(S_LEFT, MAIN_DIR * cur[J_MAIN]));
  Serial.print(F(" us, right servo ")); Serial.print(pulseAt(S_RIGHT, -MAIN_DIR * cur[J_MAIN])); Serial.print(F(" us"));
  Serial.print(F("), second arm ")); Serial.print(cur[J_ELBOW], 0); Serial.println(F(" degrees."));
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
}

void handle(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = tolower(cmd.charAt(0));
  if (c == 'z') { goZero(); return; }
  if (c == 's') { sweepAll(); return; }
  if (c == 'p') { printAngles(); return; }
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
  Serial.println(F("Command not recognised. Try b30, m-10, e+, z, s or p."));
}

void setup() {
  Serial.begin(115200);
  loadZeros();
  for (uint8_t j = 0; j < NJ; j++) writeJoint(j, 0);
  sBase.attach(PIN_BASE, SERVO_MIN_US, SERVO_MAX_US);
  sLeft.attach(PIN_MAIN_LEFT, SERVO_MIN_US, SERVO_MAX_US);
  sRight.attach(PIN_MAIN_RIGHT, SERVO_MIN_US, SERVO_MAX_US);
  sElbow.attach(PIN_ELBOW, SERVO_MIN_US, SERVO_MAX_US);
  for (uint8_t j = 0; j < NJ; j++) writeJoint(j, 0);

  Serial.println(F("\nSERVO TEST"));
  Serial.println(F("Holding the rest position for 5 seconds: base facing forward, main arm upright, second arm level."));
  Serial.println(F("If any joint is not in that position, set it with ZERO in arm_controller (saved in EEPROM)."));
  printAngles();
  delay(5000);

  directionCheck();
  sweepAll();
}

void loop() {
  if (Serial.available()) handle(Serial.readStringUntil('\n'));
}
