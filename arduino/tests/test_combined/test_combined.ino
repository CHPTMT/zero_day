/*
 * TEST: combined movements (Group 31 arm)
 *
 * Runs every movement of the demonstration without making any decisions,
 * and prints the joint angles, the calculated sensor position and every
 * sensor reading at each stop. Use it to check the arm reaches each station
 * and does not hit a stand, before running arm_controller.
 *
 *   1. Zero pose, hold 5 s
 *   2. Point A, read distance, YES gesture (sensor straight up and down)
 *   3. Point B, read distance, NO gesture (base left and right)
 *   4. BOX gesture (sensor traces a square)
 *   5. Point C, read colour
 *   6. Point D, read sound level, then indicator low, middle, high
 *   7. Point E, one dot press and one dash press (600 / 1800 ms), each the
 *      pusher pushing straight into the button along its axis and back
 *   8. Back to zero
 * Type any key in the Serial Monitor (115200) to run it again.
 *
 * Every value (pins, zeros, directions, dimensions, stations) is a copy of
 * arm_controller.ino. If you change one there, change it here too.
 */

#include <Servo.h>
#include <Wire.h>
#include "src/Adafruit_TCS34725.h"   // bundled in src/, nothing to install

// ---------------- pins ----------------
const uint8_t PIN_BASE = 2, PIN_MAIN_LEFT = 3, PIN_MAIN_RIGHT = 4, PIN_ELBOW = 5;
const uint8_t PIN_US_ECHO = 6, PIN_US_TRIG = 7, PIN_SOUND = A1;

// ---------------- servo calibration ----------------
const int SERVO_ZERO_BASE = 90, SERVO_ZERO_LEFT = 90, SERVO_ZERO_RIGHT = 90, SERVO_ZERO_ELBOW = 90;
const int8_t BASE_DIR = 1, MAIN_DIR = -1, ELBOW_DIR = 1;   // MAIN_DIR -1: same as arm_controller

enum Joint : uint8_t { J_BASE = 0, J_MAIN = 1, J_ELBOW = 2, NJ = 3 };
const float REL_MIN[NJ] = {-90, -15, -90};
const float REL_MAX[NJ] = { 90,  50,  45};

// ---------------- dimensions (mm) ----------------
const float SHOULDER_HEIGHT_MM = 130, LINK1_MM = 225, LINK2_MM = 160;
const char *JOINT_NAMES[NJ] = {"base", "main arm", "second arm"};

// ---------------- sensor board (offsets from the end of the second arm) ----------------
// along = out along the arm axis, down = square to the arm, side + = your right facing the machine
struct Tool {
  const char *name;
  float along, down, side;
  bool pressDown;   // true: approach along the "down" direction (pusher)
  float pitch;      // degrees the sensor is tilted up from the arm's axis (0 = looks straight along the arm)
};
const Tool TOOL_US     = {"ultrasonic",  0,  0,   0, false, 0};  // centre, end of the second arm
const Tool TOOL_SOUND  = {"sound",      42,  0, -45, false, 0};  // 42 mm out, 45 mm to your left
// Point C is a flat vertical face (blue paper on the end of the stand's bar, facing
// back towards the arm), 70 mm up. The arm cannot get low enough to look at it
// level: at C the second arm always points about 75 degrees down. With the colour
// sensor along the arm it looks steeply down at the face from a few mm away. Tilt
// the sensor up on the board by COLOUR_PITCH_DEG (a 60 to 70 degree wedge) and it
// looks at the face almost level, 15 mm away. Measure the angle you glue it at.
const float COLOUR_PITCH_DEG = 0;
const Tool TOOL_RGB    = {"colour",     83,  0,  45, false, COLOUR_PITCH_DEG};  // 83 mm out (the tip), 45 mm to your right
const Tool TOOL_PUSHER = {"pusher",      0, 90,   0, true, 0};   // 70 mm bar under the arm end: tip 20 + 70 = 90 mm below the arm's centre line
// Pusher note: if a pose ever needs the second arm past +45 at Point E, the
// arm holds +45 and sets the pusher height instead (see pressAtLimit).


// ---------------- stations ----------------
struct Station { const char *name; float theta, r, z, standoff; float trim[NJ]; const Tool *tool; };
Station ST_A       = {"Point A",       10, 350, 320, 100, {0, 0, 0}, &TOOL_US};
Station ST_B       = {"Point B",       10, 350, 150,  70, {0, 0, 0}, &TOOL_US};
Station ST_C       = {"Point C",       55, 200,  70,  15, {0, 0, 0}, &TOOL_RGB};
Station ST_D       = {"Point D",      100, 275, 250,  40, {0, 0, 0}, &TOOL_SOUND};
Station ST_IND_LO  = {"the volume indicator (quiet end)", 110, 275, 150,  60, {0, 0, 0}, &TOOL_US};
Station ST_IND_MID = {"the volume indicator (middle)", 110, 275, 250,  60, {0, 0, 0}, &TOOL_US};
Station ST_IND_HI  = {"the volume indicator (loud end)",110, 275, 350,  60, {0, 0, 0}, &TOOL_US};
Station ST_E       = {"Point E (the button)",170, 250, 300,   0, {0, 0, 0}, &TOOL_PUSHER};
const float PRESS_TRAVEL_MM   = 36;   // the pusher tip waits this far above the button (more puts the second arm past +45)
const float PRESS_DEPTH_MM    = 8;    // and pushes this far down past contact

const uint16_t MOVE_MS = 1200, BASE_TURN_MS = 1500, SETTLE_MS = 400;
const uint16_t POINT_HOLD_MS = 2000;   // stays still at each point, same as arm_controller.ino
const uint8_t STEP_MS = 20;
const float MAX_SPEED_DEG_S = 30;      // same as arm_controller.ino
const float PRESS_SPEED_DEG_S = 60, PRESS_CLOSE_AT = 0.75;
const float LIFT_TURN_DEG = 25;
const float GESTURE_UP_MM = 70;          // same as arm_controller.ino
const float GESTURE_DEG = 15;
const float BOX_LIFT_MM = 60;
const uint8_t GESTURE_REPEATS = 2;
const uint16_t GESTURE_STEP_MS = 900;
const uint16_t GESTURE_PAUSE_MS = 400;
const float US_MIN_CM = 3, US_MAX_CM = 15;   // detection window at A and B in arm_controller.ino

Servo sBase, sLeft, sRight, sElbow;
struct Pose { float a[NJ]; };
Pose cur = {{0, 0, 0}};
const Pose ZERO_POSE = {{0, 0, 0}};
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
bool tcsOk = false;

// ---------------- servos ----------------
int rawClamp(float v) { return constrain((int)round(v), 0, 180); }

// Servo pulse for an angle, keeping fractions of a degree. write() only moves
// in whole degrees, which makes slow moves step visibly; writeMicroseconds()
// moves in about 0.1 degree steps. 544 and 2400 us are the Servo library's own
// 0 and 180 degree pulses, so the angles match what write() would give.
int pulseFor(float deg) {
  deg = constrain(deg, 0.0, 180.0);
  return (int)(544 + deg * (2400 - 544) / 180.0 + 0.5);
}

void writeJoint(uint8_t j, float rel) {
  rel = constrain(rel, REL_MIN[j], REL_MAX[j]);
  cur.a[j] = rel;
  if (j == J_BASE) sBase.writeMicroseconds(pulseFor(SERVO_ZERO_BASE + BASE_DIR * rel));
  else if (j == J_MAIN) {
    sLeft.writeMicroseconds(pulseFor(SERVO_ZERO_LEFT + MAIN_DIR * rel));
    sRight.writeMicroseconds(pulseFor(SERVO_ZERO_RIGHT - MAIN_DIR * rel));
  } else sElbow.writeMicroseconds(pulseFor(SERVO_ZERO_ELBOW + ELBOW_DIR * rel));
}

void writePose(const Pose &p) { for (uint8_t j = 0; j < NJ; j++) writeJoint(j, p.a[j]); }

unsigned long moveAtSpeed(const Pose &target, uint16_t ms, float topSpeed) {
  Pose start = cur;
  // ms is the shortest time allowed. Long moves take longer, so that no joint
  // ever goes faster than topSpeed (cosine easing peaks at pi/2 times the
  // average speed, hence the 1.571).
  float far = 0;
  for (uint8_t j = 0; j < NJ; j++) far = max(far, fabs(target.a[j] - start.a[j]));
  unsigned long dur = max((unsigned long)ms, (unsigned long)(far * 1571.0 / topSpeed));
  uint16_t steps = max(1UL, dur / STEP_MS);
  for (uint16_t s = 1; s <= steps; s++) {
    float e = 0.5 - 0.5 * cos(PI * s / steps);
    for (uint8_t j = 0; j < NJ; j++) writeJoint(j, start.a[j] + (target.a[j] - start.a[j]) * e);
    delay(STEP_MS);
  }
  return dur;
}

unsigned long moveTo(const Pose &target, uint16_t ms) { return moveAtSpeed(target, ms, MAX_SPEED_DEG_S); }

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
  // Base + turns the arm left, towards A and B (low theta). A tool on the
  // arm's left (side +) also sits at lower theta.
  theta = 90 - p.a[J_BASE] - degrees(atan2(t.side, rin));
}

// Point E: the point d mm straight above the button top (negative = pushed
// down into it), for the pusher tip to go to. Presses run straight down.
Station buttonPoint(float d) {
  Station s = ST_E;
  s.z += d;
  s.standoff = 0;
  return s;
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
  float baseRel = 90 - s.theta - degrees(atan2(t.side, rin));   // A and B: +80, E: -80
  float psi = 0;
  for (uint8_t it = 0; it < 6; it++) {
    float look = t.pressDown ? psi - HALF_PI : psi + radians(t.pitch);   // direction the tool points
    float ax = cos(look), az = sin(look);
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

// Lean only the main arm until the pusher tip is at height wantZ (mm above
// the table). The tip drops as the main arm leans forward.
void setTipHeight(Pose &p, float wantZ) {
  float lo = REL_MIN[J_MAIN], hi = REL_MAX[J_MAIN], th, r, z;
  for (uint8_t it = 0; it < 30; it++) {
    p.a[J_MAIN] = (lo + hi) / 2;
    toolPoint(p, TOOL_PUSHER, th, r, z);
    if (z > wantZ) lo = p.a[J_MAIN]; else hi = p.a[J_MAIN];
  }
}

// Point E fallback. If pressing needs the second arm past its limit, hold
// the second arm at the limit and move only the main arm until the pusher
// tip is at the right height (button top + gap). The tip then lands a little
// short of the button centre instead of pressing early or not at all.
bool pressAtLimit(const Station &s, Pose &out) {
  out.a[J_ELBOW] = REL_MAX[J_ELBOW];
  setTipHeight(out, s.z + s.standoff);     // pusher points straight down near the limit
  float th, r, z;
  toolPoint(out, *s.tool, th, r, z);
  Serial.print(F("Note: ")); Serial.print(s.name);
  Serial.print(F(" needs the second arm past its limit, so it is held at the limit. The pusher will land "));
  Serial.print(s.r - r, 0); Serial.println(F(" mm short of the button centre."));
  return true;
}

// Station pose with trims. Prints a warning and clamps if a limit is hit.
bool stationPose(const Station &s, Pose &out) {
  if (!toolIK(s, out)) {
    Serial.print(F("WARNING: ")); Serial.print(s.name); Serial.println(F(" is out of the arm's reach. Skipping this move."));
    return false;
  }
  if (s.tool->pressDown && out.a[J_ELBOW] > REL_MAX[J_ELBOW]) pressAtLimit(s, out);
  for (uint8_t j = 0; j < NJ; j++) {
    out.a[j] += s.trim[j];
    if (out.a[j] < REL_MIN[j] || out.a[j] > REL_MAX[j]) {
      Serial.print(F("WARNING: ")); Serial.print(s.name); Serial.print(F(" needs the ")); Serial.print(JOINT_NAMES[j]);
      Serial.print(F(" at ")); Serial.print(out.a[j], 1); Serial.println(F(" degrees, past its limit. Using the limit instead."));
      out.a[j] = constrain(out.a[j], REL_MIN[j], REL_MAX[j]);
    }
  }
  return true;
}

// One joint in words, e.g. "turned 80.0 degrees left" or "facing forward".
void printJoint(float v, const __FlashStringHelper *verb, const __FlashStringHelper *plus,
                const __FlashStringHelper *minus, const __FlashStringHelper *atZero) {
  if (fabs(v) < 0.5) { Serial.print(atZero); return; }
  Serial.print(verb); Serial.print(' '); Serial.print(fabs(v), 1);
  Serial.print(F(" degrees ")); Serial.print(v > 0 ? plus : minus);
}

// Where one sensor is: how far out from the base and how high above the table.
void printTool(const __FlashStringHelper *name, const Tool &t) {
  float th, r, z;
  toolPoint(cur, t, th, r, z);
  Serial.print(name); Serial.print(' '); Serial.print(r, 0);
  Serial.print(F(" mm out and ")); Serial.print(z, 0); Serial.print(F(" mm up"));
}

// Plain description of the arm after a move: each joint, then each sensor.
void printPosition(const char *label) {
  Serial.print(F("Arrived at ")); Serial.print(label); Serial.print(F(". Base "));
  printJoint(cur.a[J_BASE], F("turned"), F("left"), F("right"), F("facing forward"));
  Serial.print(F(", main arm "));
  printJoint(cur.a[J_MAIN], F("leaning"), F("forward"), F("back"), F("upright"));
  Serial.print(F(", second arm "));
  printJoint(cur.a[J_ELBOW], F("tilted"), F("up"), F("down"), F("level"));
  Serial.println('.');
  Serial.print(F("   Sensors: "));
  printTool(F("ultrasonic"), TOOL_US);   Serial.print(F("; "));
  printTool(F("colour"), TOOL_RGB);      Serial.print(F("; "));
  printTool(F("sound"), TOOL_SOUND);     Serial.print(F("; "));
  printTool(F("pusher tip"), TOOL_PUSHER);
  Serial.println('.');
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
  delay(POINT_HOLD_MS);
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
  if (n < 3) return -1;   // fewer than 3 of 5 echoes counts as nothing, as in arm_controller
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
  Serial.print(at);
  if (d < 0) Serial.println(F(": no echo came back to the ultrasonic sensor (counts as nothing there)."));
  else {
    Serial.print(F(": the ultrasonic sensor measures ")); Serial.print(d, 1); Serial.print(F(" cm, which the demo counts as "));
    Serial.println(d >= US_MIN_CM && d <= US_MAX_CM ? F("an object.") : F("nothing there."));
  }
}

void printColour() {
  if (!tcsOk) { Serial.println(F("Colour sensor not connected, no reading.")); return; }
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  float s = (float)r + g + b;
  if (s == 0) s = 1;
  Serial.print(F("Colour sensor readings: red ")); Serial.print(r); Serial.print(F(", green ")); Serial.print(g);
  Serial.print(F(", blue ")); Serial.print(b); Serial.print(F(", overall brightness ")); Serial.print(c); Serial.println('.');
  Serial.print(F("Share of each colour: red ")); Serial.print((int)(r * 100 / s)); Serial.print(F("%, green "));
  Serial.print((int)(g * 100 / s)); Serial.print(F("%, blue ")); Serial.print((int)(b * 100 / s)); Serial.println(F("%."));
  Serial.print(F("Colour temperature ")); Serial.print(tcs.calculateColorTemperature_dn40(r, g, b, c));
  Serial.print(F(" K, light level ")); Serial.print(tcs.calculateLux(r, g, b));
  Serial.print(F(" lux, strongest colour channel: "));
  if (r >= g && r >= b) Serial.println(F("RED."));
  else Serial.println(g >= b ? F("GREEN.") : F("BLUE."));
}

void printSound() {
  int lo = 1023, hi = 0;
  unsigned long end = millis() + 2000;
  Serial.println(F("Listening for 2 seconds. Tap now."));
  while ((long)(end - millis()) > 0) {
    int s = analogRead(PIN_SOUND);
    if (s < lo) lo = s;
    if (s > hi) hi = s;
  }
  Serial.print(F("Loudest sound heard: ")); Serial.print(hi - lo); Serial.println(F(" out of 1023."));
}

// ---------------- gestures and presses (same as arm_controller.ino) ----------------
// Pose that puts the station's sensor dz mm above (or below) its usual spot,
// still aimed the same way, with the base at baseRel.
bool shiftedPose(const Station &s, float dz, float baseRel, Pose &out) {
  Station t = s;
  t.z += dz;
  if (!toolIK(t, out)) return false;
  for (uint8_t j = 0; j < NJ; j++) out.a[j] += s.trim[j];
  out.a[J_BASE] = baseRel;
  for (uint8_t j = 0; j < NJ; j++) out.a[j] = constrain(out.a[j], REL_MIN[j], REL_MAX[j]);
  return true;
}

float baseCentre() {
  return constrain(cur.a[J_BASE], REL_MIN[J_BASE] + GESTURE_DEG + 3, REL_MAX[J_BASE] - GESTURE_DEG - 3);   // 3 deg clear of the servo end stop
}

void gestureStep(const Pose &p, uint16_t ms) {
  moveTo(p, ms);
  delay(GESTURE_PAUSE_MS);
}

void gestureYes(const Station &s) {
  Serial.println(F("Gesture: YES, nodding (sensor moves straight up and down twice)."));
  Pose home = cur, up, down;
  if (!shiftedPose(s, GESTURE_UP_MM, cur.a[J_BASE], up) || !shiftedPose(s, -GESTURE_UP_MM, cur.a[J_BASE], down)) return;
  for (uint8_t i = 0; i < GESTURE_REPEATS; i++) {
    gestureStep(up, GESTURE_STEP_MS);
    gestureStep(down, GESTURE_STEP_MS);
  }
  gestureStep(home, GESTURE_STEP_MS);
}

void gestureNo() {
  Serial.println(F("Gesture: NO, shaking (base turns left and right twice)."));
  Pose home = cur, left = cur, right = cur;
  float c = baseCentre();
  left.a[J_BASE] = c + GESTURE_DEG;
  right.a[J_BASE] = c - GESTURE_DEG;
  for (uint8_t i = 0; i < GESTURE_REPEATS; i++) {
    gestureStep(left, GESTURE_STEP_MS);
    gestureStep(right, GESTURE_STEP_MS);
  }
  gestureStep(home, GESTURE_STEP_MS);
}

void gestureBox(const Station &s) {
  Serial.println(F("Gesture: BOX (sensor traces a square: up, across, down, back)."));
  Pose home = cur, corner[4];
  float c = baseCentre();
  const int8_t k[4][2] = {{1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
  for (uint8_t i = 0; i < 4; i++)
    if (!shiftedPose(s, BOX_LIFT_MM + k[i][1] * GESTURE_UP_MM, c + k[i][0] * GESTURE_DEG, corner[i])) return;
  for (uint8_t i = 0; i < 4; i++) gestureStep(corner[i], GESTURE_STEP_MS * 2);
  gestureStep(corner[0], GESTURE_STEP_MS * 2);
  gestureStep(home, GESTURE_STEP_MS * 2);
}

void runAll() {
  Serial.println(F("\nCOMBINED MOVEMENT TEST: visiting every station and doing every movement, without making decisions."));
  moveTo(ZERO_POSE, MOVE_MS);
  printPosition("the rest position");
  Serial.println(F("Holding the rest position for 5 seconds: base facing forward, main arm upright, second arm level."));
  delay(5000);

  if (goToStation(ST_A)) { printDistance("Point A"); gestureYes(ST_A); }
  Serial.println(F("Leaving the point: lifting up and turning towards the front."));
  { Pose lift = {{cur.a[J_BASE], 0, 15}}; moveTo(lift, MOVE_MS);
    lift.a[J_BASE] -= (cur.a[J_BASE] > 0 ? 1 : -1) * LIFT_TURN_DEG; moveTo(lift, BASE_TURN_MS); delay(POINT_HOLD_MS / 2); }
  Serial.println(F("Going down to Point B, the lower point."));
  if (goToStation(ST_B)) { printDistance("Point B"); gestureNo(); gestureBox(ST_B); }
  if (goToStation(ST_C)) printColour();
  if (goToStation(ST_D)) printSound();
  goToStation(ST_IND_LO);  delay(1000);
  goToStation(ST_IND_MID); delay(1000);
  goToStation(ST_IND_HI);  delay(1000);

  Pose hover, press;
  if (stationPose(buttonPoint(-PRESS_DEPTH_MM), press) && stationPose(buttonPoint(PRESS_TRAVEL_MM), hover)) {
    goToPose(hover, "Point E (above the button)");
    Serial.println(F("Each press pushes the pusher tip straight down onto the button and back up."));
    // Same timing as pressFor() in arm_controller.ino
    Serial.println(F("Short press on the button (600 ms, a Morse dot)."));
    unsigned long ramp = moveAtSpeed(press, 300, PRESS_SPEED_DEG_S);
    delay(max(0L, 600L - (long)(2 * (1 - PRESS_CLOSE_AT) * ramp))); moveAtSpeed(hover, 300, PRESS_SPEED_DEG_S); delay(1500);
    Serial.println(F("Long press on the button (1800 ms, a Morse dash)."));
    ramp = moveAtSpeed(press, 300, PRESS_SPEED_DEG_S);
    delay(max(0L, 1800L - (long)(2 * (1 - PRESS_CLOSE_AT) * ramp))); moveAtSpeed(hover, 300, PRESS_SPEED_DEG_S); delay(1500);
    printPosition("Point E (above the button)");
  }

  goToPose(ZERO_POSE, "the rest position");
  Serial.println(F("Test finished. Type any key in the Serial Monitor and press Enter to run it again."));
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
