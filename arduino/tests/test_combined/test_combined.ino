/*
 * TEST: combined movements (Group 21 arm)
 *
 * Runs every movement of the demonstration without making any decisions,
 * and prints the joint angles, the calculated sensor position and every
 * sensor reading at each stop. Use it to check the arm reaches each station
 * and does not hit a stand, before running arm_controller.
 *
 *   1. Zero pose, hold 5 s
 *   2. Point A, read distance, YES gesture
 *   3. Point B, read distance, NO gesture
 *   4. BOX gesture
 *   5. Point C, read colour
 *   6. Point D, read sound level, then indicator low, middle, high
 *   7. Point E hover, one dot press, one dash press
 *   8. Back to zero
 * Type any key in the Serial Monitor (115200) to run it again.
 *
 * Every value (pins, zeros, directions, dimensions, stations) is a copy of
 * arm_controller.ino. If you change one there, change it here too.
 */

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// ---------------- pins ----------------
const uint8_t PIN_BASE = 2, PIN_MAIN_LEFT = 3, PIN_MAIN_RIGHT = 4, PIN_ELBOW = 5;
const uint8_t PIN_US_ECHO = 6, PIN_US_TRIG = 7, PIN_SOUND = A1;

// ---------------- servo calibration ----------------
const int SERVO_ZERO_BASE = 90, SERVO_ZERO_LEFT = 90, SERVO_ZERO_RIGHT = 90, SERVO_ZERO_ELBOW = 90;
const int8_t BASE_DIR = 1, MAIN_DIR = 1, ELBOW_DIR = 1;

enum Joint : uint8_t { J_BASE = 0, J_MAIN = 1, J_ELBOW = 2, NJ = 3 };
const float REL_MIN[NJ] = {-90, -15, -90};
const float REL_MAX[NJ] = { 90,  50,  45};

// ---------------- dimensions (mm) ----------------
const float SHOULDER_HEIGHT_MM = 60, LINK1_MM = 225, LINK2_MM = 160;
const char *JOINT_NAMES[NJ] = {"base", "main", "2nd"};

// ---------------- sensor board (offsets from the end of the second arm) ----------------
// along = out along the arm, down = square to the arm, side + = your right facing the machine
struct Tool {
  const char *name;
  float along, down, side;
  bool pressDown;   // true: approach along the "down" direction (pusher)
};
const Tool TOOL_US     = {"ultrasonic",  0,  0,   0, false};  // centre of the board
const Tool TOOL_RGB    = {"colour",      0, 83,  45, false};  // tip of the right hand stick
const Tool TOOL_SOUND  = {"sound",       0, 42, -45, false};  // left hand side
const Tool TOOL_PUSHER = {"pusher",     20, 50,   0, true};   // 50 mm bar, fixed 20 mm out
// Pusher note: fixed right at the end of the arm (along 0), Point E needs the
// second arm at +49, past its +45 limit. 20 mm further out it needs +38.


// ---------------- stations ----------------
struct Station { const char *name; float theta, r, z, standoff; float trim[NJ]; const Tool *tool; };
Station ST_A       = {"A",       10, 350, 320, 100, {0, 0, 0}, &TOOL_US};
Station ST_B       = {"B",       10, 350, 150, 100, {0, 0, 0}, &TOOL_US};
Station ST_C       = {"C",       55, 200,  70,  15, {0, 0, 0}, &TOOL_RGB};
Station ST_D       = {"D",      100, 275, 250,  40, {0, 0, 0}, &TOOL_SOUND};
Station ST_IND_LO  = {"ind low", 110, 275, 150,  60, {0, 0, 0}, &TOOL_US};
Station ST_IND_MID = {"ind mid", 110, 275, 250,  60, {0, 0, 0}, &TOOL_US};
Station ST_IND_HI  = {"ind high",110, 275, 350,  60, {0, 0, 0}, &TOOL_US};
Station ST_E_HOVER = {"E hover",170, 250, 300,  15, {0, 0, 0}, &TOOL_PUSHER};
Station ST_E_PRESS = {"E press",170, 250, 300,  -8, {0, 0, 0}, &TOOL_PUSHER};

const uint16_t MOVE_MS = 1200, BASE_TURN_MS = 1500, SETTLE_MS = 400;
const uint8_t STEP_MS = 15;
const float GESTURE_DEG = 12;

Servo sBase, sLeft, sRight, sElbow;
struct Pose { float a[NJ]; };
Pose cur = {{0, 0, 0}};
const Pose ZERO_POSE = {{0, 0, 0}};
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
bool tcsOk = false;

// ---------------- servos ----------------
int rawClamp(float v) { return constrain((int)round(v), 0, 180); }

void writeJoint(uint8_t j, float rel) {
  rel = constrain(rel, REL_MIN[j], REL_MAX[j]);
  cur.a[j] = rel;
  if (j == J_BASE) sBase.write(rawClamp(SERVO_ZERO_BASE + BASE_DIR * rel));
  else if (j == J_MAIN) {
    sLeft.write(rawClamp(SERVO_ZERO_LEFT + MAIN_DIR * rel));
    sRight.write(rawClamp(SERVO_ZERO_RIGHT - MAIN_DIR * rel));
  } else sElbow.write(rawClamp(SERVO_ZERO_ELBOW + ELBOW_DIR * rel));
}

void writePose(const Pose &p) { for (uint8_t j = 0; j < NJ; j++) writeJoint(j, p.a[j]); }

void moveTo(const Pose &target, uint16_t ms) {
  Pose start = cur;
  uint16_t steps = max(1, ms / STEP_MS);
  for (uint16_t s = 1; s <= steps; s++) {
    float e = 0.5 - 0.5 * cos(PI * s / steps);
    for (uint8_t j = 0; j < NJ; j++) writeJoint(j, start.a[j] + (target.a[j] - start.a[j]) * e);
    delay(STEP_MS);
  }
}

void moveJoint(uint8_t j, float rel, uint16_t ms) {
  Pose p = cur;
  p.a[j] = rel;
  moveTo(p, ms);
}

// ---------------- kinematics (same as arm_controller.ino) ----------------
// Where a tool is for a pose: theta (deg), r and z (mm) in the brief's
// coordinates. The side offset moves it out of the arm's plane, which shows
// up as a small change in theta and r.
void toolPoint(const Pose &p, const Tool &t, float &theta, float &r, float &z) {
  float phi = radians(p.a[J_MAIN]);                 // main arm from vertical
  float psi = radians(p.a[J_ELBOW] - p.a[J_MAIN]);  // second arm from horizontal
  float a = LINK2_MM + t.along;
  float rin = LINK1_MM * sin(phi) + a * cos(psi) + t.down * sin(psi);
  z = SHOULDER_HEIGHT_MM + LINK1_MM * cos(phi) + a * sin(psi) - t.down * cos(psi);
  r = sqrt(rin * rin + t.side * t.side);
  theta = p.a[J_BASE] + 90 + degrees(atan2(t.side, rin));
}

// Joint angles that put a tool `standoff` mm from the station point, facing
// it. The arm end and the tool form one rigid link, so this is a two link
// solution in the arm's plane; the base angle is corrected for the side
// offset. The tool direction depends on the answer, so it is refined.
bool toolIK(const Station &s, Pose &out) {
  const Tool &t = *s.tool;
  float a = LINK2_MM + t.along;
  float l2 = sqrt(a * a + t.down * t.down);
  float delta = atan2(t.down, a);
  if (s.r <= fabs(t.side)) return false;
  float rin = sqrt(s.r * s.r - t.side * t.side);    // distance in the arm's plane
  float baseRel = s.theta - degrees(atan2(t.side, rin)) - 90;
  float psi = 0;
  for (uint8_t it = 0; it < 6; it++) {
    float ax = t.pressDown ? sin(psi) : cos(psi);    // direction the tool points
    float az = t.pressDown ? -cos(psi) : sin(psi);
    float tr = rin - ax * s.standoff;
    float tz = s.z - az * s.standoff;
    float dz = tz - SHOULDER_HEIGHT_MM;
    float D = sqrt(tr * tr + dz * dz);
    if (D < 1 || D > LINK1_MM + l2 || D < fabs(LINK1_MM - l2)) return false;
    float c = (LINK1_MM * LINK1_MM + D * D - l2 * l2) / (2 * LINK1_MM * D);
    float e = atan2(dz, tr) + acos(constrain(c, -1.0, 1.0));   // elbow up solution
    float ex = LINK1_MM * cos(e);
    float ez = SHOULDER_HEIGHT_MM + LINK1_MM * sin(e);
    psi = atan2(tz - ez, tr - ex) + delta;
    out.a[J_BASE]  = baseRel;
    out.a[J_MAIN]  = 90 - degrees(e);
    out.a[J_ELBOW] = degrees(psi) + out.a[J_MAIN];
  }
  return true;
}

// Station pose with trims. Prints a warning and clamps if a limit is hit.
bool stationPose(const Station &s, Pose &out) {
  if (!toolIK(s, out)) {
    Serial.print(F("[ik] ")); Serial.print(s.name); Serial.println(F(" OUT OF REACH"));
    return false;
  }
  for (uint8_t j = 0; j < NJ; j++) {
    out.a[j] += s.trim[j];
    if (out.a[j] < REL_MIN[j] || out.a[j] > REL_MAX[j]) {
      Serial.print(F("[ik] ")); Serial.print(s.name); Serial.print(F(" ")); Serial.print(JOINT_NAMES[j]);
      Serial.print(F(" limit, wanted ")); Serial.println(out.a[j]);
      out.a[j] = constrain(out.a[j], REL_MIN[j], REL_MAX[j]);
    }
  }
  return true;
}

void printTool(const Tool &t) {
  float th, r, z;
  toolPoint(cur, t, th, r, z);
  Serial.print(F("  ")); Serial.print(t.name);
  Serial.print(F(" (")); Serial.print(th, 0);
  Serial.print(F(", ")); Serial.print(r, 0);
  Serial.print(F(", ")); Serial.print(z, 0); Serial.print(')');
}

// Joint angles, then where every tool is as (theta, r, z).
void printPosition(const char *label) {
  Serial.print(F("[pos] ")); Serial.print(label);
  Serial.print(F("  base ")); Serial.print(cur.a[J_BASE], 1);
  Serial.print(F("  main ")); Serial.print(cur.a[J_MAIN], 1);
  Serial.print(F("  2nd ")); Serial.println(cur.a[J_ELBOW], 1);
  Serial.print(F("     "));
  printTool(TOOL_US); printTool(TOOL_RGB); printTool(TOOL_SOUND); printTool(TOOL_PUSHER);
  Serial.println();
}

void goToPose(const Pose &p, const char *label) {
  if (fabs(cur.a[J_BASE] - p.a[J_BASE]) > 1) {
    Pose lift = {{cur.a[J_BASE], 0, 15}};
    moveTo(lift, MOVE_MS);
    lift.a[J_BASE] = p.a[J_BASE];
    moveTo(lift, BASE_TURN_MS);
  }
  moveTo(p, MOVE_MS);
  delay(SETTLE_MS);
  printPosition(label);
}

bool goToStation(const Station &s) {
  Pose p;
  if (!stationPose(s, p)) return false;
  goToPose(p, s.name);
  return true;
}

// ---------------- sensors ----------------
float readDistanceCm() {
  float v[5];
  uint8_t n = 0;
  for (uint8_t i = 0; i < 5; i++) {
    digitalWrite(PIN_US_TRIG, LOW); delayMicroseconds(2);
    digitalWrite(PIN_US_TRIG, HIGH); delayMicroseconds(10);
    digitalWrite(PIN_US_TRIG, LOW);
    unsigned long us = pulseIn(PIN_US_ECHO, HIGH, 25000UL);
    if (us) v[n++] = us * 0.0343 / 2.0;
    delay(30);
  }
  if (n == 0) return -1;
  for (uint8_t i = 1; i < n; i++) {
    float x = v[i];
    int8_t k = i - 1;
    while (k >= 0 && v[k] > x) { v[k + 1] = v[k]; k--; }
    v[k + 1] = x;
  }
  return v[n / 2];
}

void printDistance(const char *at) {
  float d = readDistanceCm();
  Serial.print(F("[")); Serial.print(at); Serial.print(F("] distance "));
  if (d < 0) Serial.println(F("no echo"));
  else { Serial.print(d, 1); Serial.println(F(" cm")); }
}

void printColour() {
  if (!tcsOk) { Serial.println(F("[C] colour sensor missing")); return; }
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  float s = (float)r + g + b;
  if (s == 0) s = 1;
  Serial.print(F("[C] colour raw ")); Serial.print(r); Serial.print('/'); Serial.print(g);
  Serial.print('/'); Serial.print(b); Serial.print(F(" clear ")); Serial.print(c);
  Serial.print(F("  normalised ")); Serial.print(r / s, 2); Serial.print(' ');
  Serial.print(g / s, 2); Serial.print(' '); Serial.println(b / s, 2);
  Serial.print(F("[C] temp ")); Serial.print(tcs.calculateColorTemperature_dn40(r, g, b, c));
  Serial.print(F(" K  lux ")); Serial.print(tcs.calculateLux(r, g, b));
  Serial.print(F("  primary "));
  if (r >= g && r >= b) Serial.println(F("RED"));
  else Serial.println(g >= b ? F("GREEN") : F("BLUE"));
}

void printSound() {
  int lo = 1023, hi = 0;
  unsigned long end = millis() + 2000;
  Serial.println(F("[D] listening 2 s, tap now"));
  while ((long)(end - millis()) > 0) {
    int s = analogRead(PIN_SOUND);
    if (s < lo) lo = s;
    if (s > hi) hi = s;
  }
  Serial.print(F("[D] sound peak to peak ")); Serial.println(hi - lo);
}

// ---------------- gestures and presses ----------------
void nod(uint8_t j, const __FlashStringHelper *name) {
  Serial.print(F("[gesture] ")); Serial.println(name);
  float c = cur.a[j];
  for (uint8_t i = 0; i < 3; i++) {
    moveJoint(j, c + GESTURE_DEG, 400);
    moveJoint(j, c - GESTURE_DEG, 400);
  }
  moveJoint(j, c, 400);
}

void box() {
  Serial.println(F("[gesture] BOX"));
  float cb = cur.a[J_BASE], ce = cur.a[J_ELBOW];
  const int8_t k[5][2] = {{-1, 1}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
  for (uint8_t i = 0; i < 5; i++) {
    Pose p = cur;
    p.a[J_BASE] = cb + k[i][0] * GESTURE_DEG;
    p.a[J_ELBOW] = ce + k[i][1] * GESTURE_DEG;
    moveTo(p, 800);
  }
  Pose back = cur;
  back.a[J_BASE] = cb;
  back.a[J_ELBOW] = ce;
  moveTo(back, 800);
}

void runAll() {
  Serial.println(F("\n=== COMBINED MOVEMENT TEST ==="));
  moveTo(ZERO_POSE, MOVE_MS);
  printPosition("zero");
  Serial.println(F("[zero] holding 5 s"));
  delay(5000);

  if (goToStation(ST_A)) { printDistance("A"); nod(J_ELBOW, F("YES (up / down)")); }
  if (goToStation(ST_B)) { printDistance("B"); nod(J_BASE, F("NO (left / right)")); }
  box();
  if (goToStation(ST_C)) printColour();
  if (goToStation(ST_D)) printSound();
  goToStation(ST_IND_LO);  delay(1000);
  goToStation(ST_IND_MID); delay(1000);
  goToStation(ST_IND_HI);  delay(1000);

  Pose hover, press;
  if (stationPose(ST_E_HOVER, hover) && stationPose(ST_E_PRESS, press)) {
    goToPose(hover, "E hover");
    Serial.println(F("[E] dot press (300 ms)"));
    writePose(press); delay(300); writePose(hover); delay(600);
    Serial.println(F("[E] dash press (900 ms)"));
    writePose(press); delay(900); writePose(hover); delay(600);
    printPosition("E hover");
  }

  goToPose(ZERO_POSE, "zero");
  Serial.println(F("=== DONE. Type any key to run again. ==="));
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  writePose(ZERO_POSE);
  sBase.attach(PIN_BASE);
  sLeft.attach(PIN_MAIN_LEFT);
  sRight.attach(PIN_MAIN_RIGHT);
  sElbow.attach(PIN_ELBOW);
  writePose(ZERO_POSE);
  tcsOk = tcs.begin();
  runAll();
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    runAll();
  }
}
