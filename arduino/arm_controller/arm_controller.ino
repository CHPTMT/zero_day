/*
 * MECN3066A Mechatronics II, Group 21
 * Recycled robotic arm: full demonstration (Arduino Uno R3)
 *
 * Sequence (state machine, fully autonomous after reset):
 *   ZERO    all joints at relative 0 for 5 s (also the window to type the word)
 *   TASK 1  Points A and B: ultrasonic presence, YES / NO / BOX gesture
 *   TASK 2  Point C: colour sensor, light the matching LED
 *   TASK 3  Point D: capture a tap, point at a height on the volume indicator
 *   TASK 4  Point E: press the button in Morse for the 3 letter word
 *   FINISH  back to zero, print a summary
 * Everything the arm does and senses is printed to the Serial Monitor
 * (115200 baud), so the run can be followed without the LCD.
 *
 * ANGLES
 *   All joint angles in this file are RELATIVE, with 0 at the zero pose:
 *     Base      0 = facing forward (middle of the half circle, theta 90)
 *               +90 = the arm's left (towards A and B), -90 = its right (towards E),
 *               as on the base map seen from behind the arm
 *     Main arm  0 = upright, + = forward (max +50), - = back (max -15)
 *               Driven by two servos facing each other (left D3, right D4),
 *               always moved in opposite directions.
 *     2nd arm   0 = level with the ground when the main arm is upright,
 *               + = up (max +45), - = down (max -90)
 *   SERVO_ZERO_xxx holds the raw servo value that equals relative 0.
 *   If a joint moves the wrong way, flip its xxx_DIR from 1 to -1.
 *
 * POSITIONING
 *   Joint angles are calculated on board from the station coordinates in the
 *   brief and the arm dimensions below (two link inverse kinematics). The
 *   sensor board sits at the end of the second arm; each sensor's own offset
 *   on the board (see SENSOR BOARD) is used, so the right sensor is aimed at
 *   each station with a standoff (gap between the sensor and the target). Fine tune a station with
 *   its trim values instead of changing the maths.
 *
 * QUICK EDITS ON THE DAY
 *   Word: type it in the Serial Monitor during the 5 s zero hold (e.g. NET).
 *         Saved in EEPROM. Or change DEFAULT_WORD.
 *   Fallback: set RUN_TASK_x to false to skip a task that keeps failing.
 *   Re-run: press the Uno reset button.
 *
 * Libraries: Servo, Adafruit TCS34725
 */

#include <Servo.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_TCS34725.h>

// ======================================================================
// TASK SWITCHES
// ======================================================================
const bool RUN_TASK_1 = true;   // Points A and B
const bool RUN_TASK_2 = true;   // Point C
const bool RUN_TASK_3 = true;   // Point D
const bool RUN_TASK_4 = true;   // Point E

const char DEFAULT_WORD[] = "NET";
const unsigned long ZERO_HOLD_MS = 5000;   // hold at zero before the run

// ======================================================================
// PINS
// ======================================================================
const uint8_t PIN_BASE       = 2;
const uint8_t PIN_MAIN_LEFT  = 3;
const uint8_t PIN_MAIN_RIGHT = 4;
const uint8_t PIN_ELBOW      = 5;    // second arm
const uint8_t PIN_US_ECHO    = 6;
const uint8_t PIN_US_TRIG    = 7;
const uint8_t PIN_SOUND      = A1;   // sound sensor AO
// Colour sensor on I2C: SDA = A4, SCL = A5
// External colour LEDs (each through 220 ohm to GND). Not wired yet:
// connect them to these pins, or change the numbers.
const uint8_t PIN_LED_RED    = 8;
const uint8_t PIN_LED_GREEN  = 9;
const uint8_t PIN_LED_BLUE   = 10;
const uint8_t PIN_STATUS     = 13;   // onboard LED

// ======================================================================
// SERVO CALIBRATION
// ======================================================================
// Raw servo value (0 to 180) at relative 0. Adjust until test_servos shows
// the base facing forward, the main arm upright and the second arm level.
const int SERVO_ZERO_BASE  = 90;
const int SERVO_ZERO_LEFT  = 90;
const int SERVO_ZERO_RIGHT = 90;
const int SERVO_ZERO_ELBOW = 90;

// Direction of each joint. Flip to -1 if the joint moves the wrong way.
// MAIN_DIR = 1 means: forward = left servo value goes up, right servo goes down.
const int8_t BASE_DIR  = 1;
const int8_t MAIN_DIR  = 1;
const int8_t ELBOW_DIR = 1;

// Relative limits in degrees
enum Joint : uint8_t { J_BASE = 0, J_MAIN = 1, J_ELBOW = 2, NJ = 3 };
const float REL_MIN[NJ] = {-90, -15, -90};
const float REL_MAX[NJ] = { 90,  50,  45};
const char *JOINT_NAMES[NJ] = {"base", "main", "2nd"};

// ======================================================================
// ARM DIMENSIONS (mm)
// ======================================================================
const float SHOULDER_HEIGHT_MM = 130;   // table to main arm pivot: 20 mm platform + 110 mm.
                                        // Station heights in the brief are from the table.
const float LINK1_MM           = 225;   // main arm, pivot to pivot
const float LINK2_MM           = 160;   // second arm, pivot to the sensor board

// ======================================================================
// SENSOR BOARD (mm, measured from where the second arm ends)
// ======================================================================
//   along = further out along the axis of the second arm
//   down  = square to the arm, towards the ground when the second arm is level
//   side  = sideways: + is to YOUR RIGHT when you stand in front of the
//           machine facing it (that is the arm's own left)
// The sensor array runs along the axis of the second arm and every sensor
// faces outward along it. The button pusher hangs below the array at the end
// of the arm and presses along its own length (straight down when level).
struct Tool {
  const char *name;
  float along, down, side;
  bool pressDown;   // true: approach along the "down" direction (pusher)
};
const Tool TOOL_US     = {"ultrasonic",  0,  0,   0, false};  // centre, end of the second arm
const Tool TOOL_SOUND  = {"sound",      42,  0, -45, false};  // 42 mm out, 45 mm to your left
const Tool TOOL_RGB    = {"colour",     83,  0,  45, false};  // 83 mm out (the tip), 45 mm to your right
const Tool TOOL_PUSHER = {"pusher",      0, 70,   0, true};   // 50 mm bar under the arm end: tip 20 + 50 = 70 mm below the arm's centre line
// Pusher note: if a pose ever needs the second arm past +45 at Point E, the
// arm holds +45 and sets the pusher height instead (see pressAtLimit).

// Sensor ranges used for the standoffs below
//   HC-SR04 ultrasonic: 20 mm to 4 m, about 3 mm resolution, 15 deg cone.
//                       Below about 20 mm it reads nothing.
//   TCS34725 colour:    reads best 5 to 20 mm from the paper. Further away
//                       room light swamps the colour; closer it shadows itself.
//   Sound module (KY-037/038): sensitivity set by its screw pot. A tap a few
//                       cm away gives a clear peak; across the room barely registers.

// ======================================================================
// STATIONS (theta deg, R mm, Z mm from the brief) and standoffs
// ======================================================================
// standoff = gap from the tool to the target along its pointing line
//            (negative = pushed in past contact, used for the button).
// trim = extra degrees added to the calculated base / main / 2nd angles.
struct Station {
  const char *name;
  float theta, r, z, standoff;
  float trim[NJ];
  const Tool *tool;
};

Station ST_A       = {"A",       10, 350, 320, 100, {0, 0, 0}, &TOOL_US};      // 10 cm from the plate
Station ST_B       = {"B",       10, 350, 150, 100, {0, 0, 0}, &TOOL_US};
Station ST_C       = {"C",       55, 200,  70,  15, {0, 0, 0}, &TOOL_RGB};     // 15 mm from the paper
Station ST_D       = {"D",      100, 275, 250,  40, {0, 0, 0}, &TOOL_SOUND};   // 4 cm from the source
Station ST_E_HOVER = {"E hover",170, 250, 300,  15, {0, 0, 0}, &TOOL_PUSHER};  // 15 mm off the button
Station ST_E_PRESS = {"E press",170, 250, 300,  -8, {0, 0, 0}, &TOOL_PUSHER};  // 8 mm past contact

// Volume indicator: the pointer height goes from IND_Z_LOW (quiet) to
// IND_Z_HIGH (loud) at theta 110, R 275.
const float IND_THETA    = 110;
const float IND_R        = 275;
const float IND_Z_LOW    = 150;
const float IND_Z_HIGH   = 350;
const float IND_STANDOFF = 60;

// ======================================================================
// MOTION
// ======================================================================
const uint16_t MOVE_MS      = 1200;
const uint16_t BASE_TURN_MS = 1500;
const uint8_t  STEP_MS      = 15;
const uint16_t SETTLE_MS    = 400;
const bool     USE_TRAVEL   = true;          // lift before turning the base
const float    TRAVEL_MAIN  = 0;             // main arm upright while turning
const float    TRAVEL_ELBOW = 15;            // head slightly up while turning

// Gestures are kept small and slow so they read clearly and do not shake
// the arm or knock it off its taught positions.
const float    GESTURE_DEG     = 6;     // swing each way from the station pose
const uint8_t  GESTURE_REPEATS = 2;     // nods / shakes
const uint16_t GESTURE_STEP_MS = 900;   // time for each half swing

// ======================================================================
// SENSOR THRESHOLDS
// ======================================================================
const float    US_DETECT_MIN_CM = 2.0;
const float    US_DETECT_MAX_CM = 20.0;      // standoff 10 cm, plus 10 cm margin
const uint8_t  US_SAMPLES       = 5;
const unsigned long US_TIMEOUT_US = 25000;

const float REF_RED[3]   = {0.58, 0.23, 0.19};   // calibrate with test_sensors
const float REF_GREEN[3] = {0.28, 0.44, 0.28};
const float REF_BLUE[3]  = {0.20, 0.32, 0.48};
const uint16_t COLOUR_MIN_CLEAR = 30;
const uint8_t  COLOUR_SAMPLES   = 3;

const uint16_t SOUND_TRIGGER   = 40;
const uint16_t SOUND_AMP_MIN   = 40;
const uint16_t SOUND_AMP_MAX   = 600;
const uint16_t SOUND_WINDOW_MS = 150;
const unsigned long SOUND_TIMEOUT_MS = 3000;
const uint16_t SOUND_HOLD_MS   = 3000;

const uint16_t MORSE_UNIT_MS  = 300;   // must match morse_receiver.ino
const int16_t  PRESS_EXTRA_MS = 0;

// ======================================================================
// GLOBALS
// ======================================================================
Servo sBase, sLeft, sRight, sElbow;
struct Pose { float a[NJ]; };
Pose cur = {{0, 0, 0}};
const Pose ZERO_POSE = {{0, 0, 0}};

Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
bool tcsOk = false;
char targetWord[4];

enum State : uint8_t { ST_START, ST_TASK1, ST_TASK2, ST_TASK3, ST_TASK4, ST_FINISH, ST_IDLE };
State state = ST_START;

bool resultA = false, resultB = false;
float distA = -1, distB = -1;
char resultColour = '-';
int  resultSound = -1;
int  resultLevel = 0;

// ======================================================================
// SERVO OUTPUT
// ======================================================================
int rawClamp(float v) { return constrain((int)round(v), 0, 180); }

// Set one joint to a relative angle. The main arm writes both servos in
// opposite directions so they twist the shaft together.
void writeJoint(uint8_t j, float rel) {
  rel = constrain(rel, REL_MIN[j], REL_MAX[j]);
  cur.a[j] = rel;
  switch (j) {
    case J_BASE:
      sBase.write(rawClamp(SERVO_ZERO_BASE + BASE_DIR * rel));
      break;
    case J_MAIN:
      sLeft.write(rawClamp(SERVO_ZERO_LEFT + MAIN_DIR * rel));
      sRight.write(rawClamp(SERVO_ZERO_RIGHT - MAIN_DIR * rel));
      break;
    case J_ELBOW:
      sElbow.write(rawClamp(SERVO_ZERO_ELBOW + ELBOW_DIR * rel));
      break;
  }
}

void writePose(const Pose &p) {
  for (uint8_t j = 0; j < NJ; j++) writeJoint(j, p.a[j]);
}

// Smooth move of all joints together with cosine easing.
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

// ======================================================================
// KINEMATICS
// ======================================================================
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

// Point E fallback. If pressing needs the second arm past its limit, hold
// the second arm at the limit and move only the main arm until the pusher
// tip is at the right height (button top + gap). The tip then lands a little
// short of the button centre instead of pressing early or not at all.
bool pressAtLimit(const Station &s, Pose &out) {
  out.a[J_ELBOW] = REL_MAX[J_ELBOW];
  float wantZ = s.z + s.standoff;          // pusher points straight down near the limit
  float lo = REL_MIN[J_MAIN], hi = REL_MAX[J_MAIN], th, r, z;
  for (uint8_t it = 0; it < 30; it++) {    // tip height falls as the main arm leans forward
    out.a[J_MAIN] = (lo + hi) / 2;
    toolPoint(out, *s.tool, th, r, z);
    if (z > wantZ) lo = out.a[J_MAIN]; else hi = out.a[J_MAIN];
  }
  Serial.print(F("[ik] ")); Serial.print(s.name);
  Serial.print(F(": 2nd arm held at its limit, pusher tip ")); Serial.print(s.r - r, 0);
  Serial.println(F(" mm short of the button centre"));
  return true;
}

// Station pose with trims. Prints a warning and clamps if a limit is hit.
bool stationPose(const Station &s, Pose &out) {
  if (!toolIK(s, out)) {
    Serial.print(F("[ik] ")); Serial.print(s.name); Serial.println(F(" OUT OF REACH"));
    return false;
  }
  if (s.tool->pressDown && out.a[J_ELBOW] > REL_MAX[J_ELBOW]) pressAtLimit(s, out);
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

// Lift, swing the base, then settle onto the pose.
void goToPose(const Pose &p, const char *label) {
  if (USE_TRAVEL && fabs(cur.a[J_BASE] - p.a[J_BASE]) > 1) {
    Pose lift = {{cur.a[J_BASE], TRAVEL_MAIN, TRAVEL_ELBOW}};
    moveTo(lift, MOVE_MS);
    lift.a[J_BASE] = p.a[J_BASE];
    moveTo(lift, BASE_TURN_MS);
  }
  moveTo(p, MOVE_MS);
  delay(SETTLE_MS);
  printPosition(label);
}

// Go to a station. If the maths fails the arm stays put and the task
// carries on with whatever it can sense.
void goToStation(const Station &s) {
  Pose p;
  if (stationPose(s, p)) goToPose(p, s.name);
}

// ======================================================================
// GESTURES
// ======================================================================
void gestureYes() {   // vertical: second arm up and down
  Serial.println(F("[gesture] YES (up / down)"));
  float c = cur.a[J_ELBOW];
  for (uint8_t i = 0; i < GESTURE_REPEATS; i++) {
    moveJoint(J_ELBOW, c + GESTURE_DEG, GESTURE_STEP_MS);
    moveJoint(J_ELBOW, c - GESTURE_DEG, GESTURE_STEP_MS);
  }
  moveJoint(J_ELBOW, c, GESTURE_STEP_MS);
}

void gestureNo() {    // horizontal: base left and right
  Serial.println(F("[gesture] NO (left / right)"));
  float c = cur.a[J_BASE];
  for (uint8_t i = 0; i < GESTURE_REPEATS; i++) {
    moveJoint(J_BASE, c + GESTURE_DEG, GESTURE_STEP_MS);
    moveJoint(J_BASE, c - GESTURE_DEG, GESTURE_STEP_MS);
  }
  moveJoint(J_BASE, c, GESTURE_STEP_MS);
}

void gestureBox() {   // square traced with base and second arm
  Serial.println(F("[gesture] BOX"));
  float cb = cur.a[J_BASE], ce = cur.a[J_ELBOW];
  const int8_t k[5][2] = {{-1, 1}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
  for (uint8_t i = 0; i < 5; i++) {
    Pose p = cur;
    p.a[J_BASE] = cb + k[i][0] * GESTURE_DEG;
    p.a[J_ELBOW] = ce + k[i][1] * GESTURE_DEG;
    moveTo(p, GESTURE_STEP_MS * 2);
  }
  Pose back = cur;
  back.a[J_BASE] = cb;
  back.a[J_ELBOW] = ce;
  moveTo(back, GESTURE_STEP_MS * 2);
}

// ======================================================================
// ULTRASONIC
// ======================================================================
float pingCm() {
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);
  unsigned long us = pulseIn(PIN_US_ECHO, HIGH, US_TIMEOUT_US);
  return us == 0 ? -1 : us * 0.0343 / 2.0;
}

// Median of several pings, -1 if none came back.
float readDistanceCm() {
  float v[US_SAMPLES];
  uint8_t n = 0;
  for (uint8_t i = 0; i < US_SAMPLES; i++) {
    float d = pingCm();
    if (d > 0) v[n++] = d;
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

bool objectPresent(float d) { return d >= US_DETECT_MIN_CM && d <= US_DETECT_MAX_CM; }

// ======================================================================
// COLOUR
// ======================================================================
void setColourLed(char c) {
  digitalWrite(PIN_LED_RED,   c == 'R');
  digitalWrite(PIN_LED_GREEN, c == 'G');
  digitalWrite(PIN_LED_BLUE,  c == 'B');
}

float dist2(const float a[3], const float b[3]) {
  float s = 0;
  for (uint8_t i = 0; i < 3; i++) s += (a[i] - b[i]) * (a[i] - b[i]);
  return s;
}

// Strongest raw channel. Shown next to the calibrated result as a cross check.
char primaryColour(uint32_t r, uint32_t g, uint32_t b) {
  if (r >= g && r >= b) return 'R';
  return g >= b ? 'G' : 'B';
}

const __FlashStringHelper *colourName(char c);

// Colour temperature (K), light level (lux) and strongest channel.
void printColourExtras(uint32_t r, uint32_t g, uint32_t b, uint32_t c) {
  uint16_t temp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  uint16_t lux = tcs.calculateLux(r, g, b);
  Serial.print(F("[colour] temp ")); Serial.print(temp);
  Serial.print(F(" K  lux ")); Serial.print(lux);
  Serial.print(F("  primary ")); Serial.println(colourName(primaryColour(r, g, b)));
}

// Always returns R, G or B, so an LED is always lit.
char readColour() {
  if (!tcsOk) { Serial.println(F("[colour] sensor missing, default R")); return 'R'; }
  uint32_t r = 0, g = 0, b = 0, c = 0;
  for (uint8_t i = 0; i < COLOUR_SAMPLES; i++) {
    uint16_t rr, gg, bb, cc;
    tcs.getRawData(&rr, &gg, &bb, &cc);
    r += rr; g += gg; b += bb; c += cc;
  }
  r /= COLOUR_SAMPLES; g /= COLOUR_SAMPLES; b /= COLOUR_SAMPLES; c /= COLOUR_SAMPLES;
  float sum = r + g + b;
  Serial.print(F("[colour] raw r ")); Serial.print(r);
  Serial.print(F(" g ")); Serial.print(g);
  Serial.print(F(" b ")); Serial.print(b);
  Serial.print(F(" clear ")); Serial.println(c);
  printColourExtras(r, g, b, c);
  if (c < COLOUR_MIN_CLEAR || sum == 0) {
    Serial.println(F("[colour] weak reading, using strongest channel"));
    return primaryColour(r, g, b);
  }
  float n[3] = {r / sum, g / sum, b / sum};
  Serial.print(F("[colour] normalised ")); Serial.print(n[0], 3); Serial.print(' ');
  Serial.print(n[1], 3); Serial.print(' '); Serial.println(n[2], 3);
  float dr = dist2(n, REF_RED), dg = dist2(n, REF_GREEN), db = dist2(n, REF_BLUE);
  if (dr <= dg && dr <= db) return 'R';
  return dg <= db ? 'G' : 'B';
}

const __FlashStringHelper *colourName(char c) {
  if (c == 'R') return F("RED");
  if (c == 'G') return F("GREEN");
  if (c == 'B') return F("BLUE");
  return F("none");
}

// ======================================================================
// SOUND
// ======================================================================
// Peak to peak amplitude of the first tap within the timeout, or -1.
int captureSound() {
  long sum = 0;
  for (uint8_t i = 0; i < 64; i++) sum += analogRead(PIN_SOUND);
  int baseline = sum / 64;
  unsigned long deadline = millis() + SOUND_TIMEOUT_MS;
  while ((long)(deadline - millis()) > 0) {
    int v = analogRead(PIN_SOUND);
    if (abs(v - baseline) >= SOUND_TRIGGER) {
      int lo = v, hi = v;
      unsigned long end = millis() + SOUND_WINDOW_MS;
      while ((long)(end - millis()) > 0) {
        int s = analogRead(PIN_SOUND);
        if (s < lo) lo = s;
        if (s > hi) hi = s;
      }
      return max(hi - lo, abs(v - baseline));
    }
  }
  return -1;
}

// ======================================================================
// MORSE
// ======================================================================
struct MorseChar { char letter; const char *code; };
const MorseChar MORSE[] = {
  {'A', ".-"},   {'B', "-..."}, {'C', "-.-."}, {'D', "-.."},  {'E', "."},
  {'F', "..-."}, {'G', "--."},  {'H', "...."}, {'I', ".."},   {'J', ".---"},
  {'K', "-.-"},  {'L', ".-.."}, {'M', "--"},   {'N', "-."},   {'O', "---"},
  {'P', ".--."}, {'Q', "--.-"}, {'R', ".-."},  {'S', "..."},  {'T', "-"},
  {'U', "..-"},  {'V', "...-"}, {'W', ".--"},  {'X', "-..-"}, {'Y', "-.--"},
  {'Z', "--.."},
};

const char *morseFor(char c) {
  for (uint8_t i = 0; i < sizeof(MORSE) / sizeof(MORSE[0]); i++)
    if (MORSE[i].letter == c) return MORSE[i].code;
  return nullptr;
}

Pose pressPose, hoverPose;

// Snap (no easing) so the contact time matches the commanded time.
void pressFor(uint16_t ms) {
  writePose(pressPose);
  digitalWrite(PIN_STATUS, HIGH);
  delay(max(0, (int)ms + PRESS_EXTRA_MS));
  writePose(hoverPose);
  digitalWrite(PIN_STATUS, LOW);
}

void sendMorseWord(const char *word) {
  Serial.print(F("[morse] word ")); Serial.println(word);
  for (uint8_t i = 0; word[i]; i++) {
    const char *code = morseFor(word[i]);
    if (!code) continue;
    Serial.print(F("[morse] ")); Serial.print(word[i]); Serial.print(F("  "));
    for (uint8_t k = 0; code[k]; k++) {
      Serial.print(code[k] == '.' ? F("dot ") : F("DASH "));
      pressFor(code[k] == '.' ? MORSE_UNIT_MS : 3 * MORSE_UNIT_MS);
      if (code[k + 1]) delay(MORSE_UNIT_MS);
    }
    Serial.println();
    if (word[i + 1]) delay(3 * MORSE_UNIT_MS);
  }
  delay(8 * MORSE_UNIT_MS);   // receiver ends the word after the silence
  Serial.print(F("[morse] sent ")); Serial.println(word);
}

// ======================================================================
// WORD (EEPROM and Serial Monitor)
// ======================================================================
bool validWord(const char *w) {
  for (uint8_t i = 0; i < 3; i++) if (w[i] < 'A' || w[i] > 'Z') return false;
  return true;
}

void loadWord() {
  for (uint8_t i = 0; i < 3; i++) targetWord[i] = EEPROM.read(i);
  targetWord[3] = '\0';
  if (!validWord(targetWord)) strcpy(targetWord, DEFAULT_WORD);
}

// Hold the zero pose for ZERO_HOLD_MS. Any 3 letter word typed in that
// time becomes the target word.
void holdZeroAndReadWord() {
  Serial.print(F("[zero] holding all joints at 0 for "));
  Serial.print(ZERO_HOLD_MS / 1000);
  Serial.println(F(" s"));
  Serial.print(F("[word] current word ")); Serial.print(targetWord);
  Serial.println(F(". Type a new 3 letter word now to change it."));
  char buf[8];
  uint8_t n = 0;
  unsigned long start = millis();
  while (millis() - start < ZERO_HOLD_MS) {
    if (!Serial.available()) continue;
    char c = toupper(Serial.read());
    if (c == '\n' || c == '\r') {
      buf[min(n, (uint8_t)7)] = '\0';
      if (n == 3 && validWord(buf)) {
        strcpy(targetWord, buf);
        for (uint8_t i = 0; i < 3; i++) EEPROM.update(i, targetWord[i]);
        Serial.print(F("[word] saved ")); Serial.println(targetWord);
      }
      n = 0;
    } else if (n < 7) {
      buf[n++] = c;
    }
  }
}

// ======================================================================
// TASKS
// ======================================================================
void task1() {
  Serial.println(F("\n=== TASK 1: Points A and B (ultrasonic) ==="));
  goToStation(ST_A);
  distA = readDistanceCm();
  resultA = objectPresent(distA);
  Serial.print(F("[A] distance ")); Serial.print(distA, 1);
  Serial.println(resultA ? F(" cm -> OBJECT") : F(" cm -> empty"));
  if (resultA) gestureYes();

  goToStation(ST_B);
  distB = readDistanceCm();
  resultB = objectPresent(distB);
  Serial.print(F("[B] distance ")); Serial.print(distB, 1);
  Serial.println(resultB ? F(" cm -> OBJECT") : F(" cm -> empty"));
  if (resultB) gestureNo();

  if (!resultA && !resultB) gestureBox();
}

void task2() {
  Serial.println(F("\n=== TASK 2: Point C (colour) ==="));
  goToStation(ST_C);
  resultColour = readColour();
  Serial.print(F("[C] colour -> ")); Serial.println(colourName(resultColour));
  setColourLed(resultColour);   // stays lit to the end of the run
}

void task3() {
  Serial.println(F("\n=== TASK 3: Point D (sound) ==="));
  goToStation(ST_D);
  Serial.println(F("[D] listening for a tap (3 s)"));
  digitalWrite(PIN_STATUS, HIGH);
  resultSound = captureSound();
  digitalWrite(PIN_STATUS, LOW);

  float f = 0;
  if (resultSound >= 0) {
    f = (float)(resultSound - (int)SOUND_AMP_MIN) / (SOUND_AMP_MAX - SOUND_AMP_MIN);
    f = constrain(f, 0.0, 1.0);
  }
  resultLevel = (int)(f * 100);
  Serial.print(F("[D] amplitude ")); Serial.print(resultSound);
  Serial.print(F(" -> level ")); Serial.print(resultLevel); Serial.println('%');

  float zPoint = IND_Z_LOW + f * (IND_Z_HIGH - IND_Z_LOW);
  Station ind = {"indicator", IND_THETA, IND_R, zPoint, IND_STANDOFF, {0, 0, 0}, &TOOL_US};
  Pose p;
  if (stationPose(ind, p)) {
    moveTo(p, MOVE_MS);   // indicator is next to D, no travel lift needed
    printPosition("indicator");
  }
  Serial.print(F("[D] pointing at height ")); Serial.print(zPoint, 0); Serial.println(F(" mm"));
  delay(SOUND_HOLD_MS);
}

void task4() {
  Serial.println(F("\n=== TASK 4: Point E (Morse) ==="));
  if (!stationPose(ST_E_HOVER, hoverPose) || !stationPose(ST_E_PRESS, pressPose)) {
    Serial.println(F("[E] cannot reach, skipping"));
    return;
  }
  goToPose(hoverPose, "E hover");
  sendMorseWord(targetWord);
}

void printSummary() {
  Serial.println(F("\n=========== RUN SUMMARY ==========="));
  Serial.print(F("A: ")); Serial.print(resultA ? F("object") : F("empty"));
  Serial.print(F(" (")); Serial.print(distA, 1); Serial.println(F(" cm)"));
  Serial.print(F("B: ")); Serial.print(resultB ? F("object") : F("empty"));
  Serial.print(F(" (")); Serial.print(distB, 1); Serial.println(F(" cm)"));
  Serial.print(F("Gesture: "));
  if (resultA && resultB) Serial.println(F("YES then NO"));
  else if (resultA) Serial.println(F("YES"));
  else if (resultB) Serial.println(F("NO"));
  else Serial.println(F("BOX"));
  Serial.print(F("C colour: ")); Serial.println(colourName(resultColour));
  Serial.print(F("D sound: amplitude ")); Serial.print(resultSound);
  Serial.print(F(", level ")); Serial.print(resultLevel); Serial.println('%');
  Serial.print(F("E word: ")); Serial.println(targetWord);
  Serial.println(F("===================================="));
  Serial.println(F("Press reset to run again."));
}

// ======================================================================
// SETUP AND LOOP
// ======================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_STATUS, OUTPUT);
  setColourLed('-');

  // Write zero before attaching so nothing jumps on power up.
  writePose(ZERO_POSE);
  sBase.attach(PIN_BASE);
  sLeft.attach(PIN_MAIN_LEFT);
  sRight.attach(PIN_MAIN_RIGHT);
  sElbow.attach(PIN_ELBOW);
  writePose(ZERO_POSE);

  Serial.println(F("\nGroup 21 robotic arm, demonstration"));
  tcsOk = tcs.begin();
  Serial.println(tcsOk ? F("[init] colour sensor found") : F("[init] colour sensor NOT found, Task 2 will default"));

  loadWord();
  printPosition("zero");
  holdZeroAndReadWord();
  Serial.print(F("[word] target word ")); Serial.println(targetWord);
}

void loop() {
  switch (state) {
    case ST_START:
      Serial.println(F("\n=== RUN START ==="));
      state = ST_TASK1;
      break;
    case ST_TASK1:
      if (RUN_TASK_1) task1();
      state = ST_TASK2;
      break;
    case ST_TASK2:
      if (RUN_TASK_2) task2();
      state = ST_TASK3;
      break;
    case ST_TASK3:
      if (RUN_TASK_3) task3();
      state = ST_TASK4;
      break;
    case ST_TASK4:
      if (RUN_TASK_4) task4();
      state = ST_FINISH;
      break;
    case ST_FINISH:
      goToPose(ZERO_POSE, "zero");
      printSummary();
      state = ST_IDLE;
      break;
    case ST_IDLE:
      digitalWrite(PIN_STATUS, (millis() / 500) % 2);
      break;
  }
}
