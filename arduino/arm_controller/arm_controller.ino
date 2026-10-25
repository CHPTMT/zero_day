/*
 * MECN3066A Mechatronics II, Group 21
 * Recycled robotic arm: full demonstration (Arduino Uno R3)
 *
 * Sequence (state machine, fully autonomous after reset):
 *   ZERO    all joints at relative 0 for 5 s (also the window to type the word)
 *   TASK 1  Points A and B: ultrasonic presence, then one gesture:
 *           YES  sensor moves straight up and down (object at A)
 *           NO   base turns left and right (object at B)
 *           BOX  sensor traces a square, both together (nothing at A or B)
 *   TASK 2  Point C: colour sensor, the red, green or blue LED lights
 *   TASK 3  Point D: capture a tap, point at a height on the volume indicator
 *   TASK 4  Point E: press the button in Morse for the 3 letter word; each
 *           press pushes the pusher straight down onto the button and back up
 *   FINISH  back to zero, print a summary
 * Beeper on D11: task N beeps N times when it starts and N times when it
 * ends, then a 2 s pause before the next task.
 * Everything the arm does and senses is printed to the Serial Monitor
 * (115200 baud). A 16x2 I2C LCD on A2 (SDA) and A3 (SCL) shows the task in
 * progress, the distances at A and B and each task's result, then cycles
 * through a summary at the end.
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
 *   each station with a standoff (gap between the sensor and the target).
 *   Fine tune a station with its trim values instead of changing the maths.
 *
 * QUICK EDITS ON THE DAY
 *   Word: type it in the Serial Monitor during the 5 s zero hold (e.g. NET).
 *         Saved in EEPROM. Or change DEFAULT_WORD.
 *   Fallback: set RUN_TASK_x to false to skip a task that keeps failing.
 *   Re-run: press the Uno reset button.
 *
 * Libraries: Servo, Wire and EEPROM come with the Arduino IDE. The colour sensor
 * library (Adafruit TCS34725) is bundled in the src folder, so nothing needs installing.
 */

#include <Servo.h>
#include <Wire.h>
#include <EEPROM.h>
#include "src/Adafruit_TCS34725.h"   // bundled in src/, nothing to install

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
// Three separate LEDs: pin -> 220 ohm -> long leg, short leg -> GND.
// A pin set HIGH lights that LED.
const uint8_t PIN_LED_RED    = 8;
const uint8_t PIN_LED_GREEN  = 9;
const uint8_t PIN_LED_BLUE   = 10;
const bool LEDS_ACTIVE_HIGH  = true;   // false only if the LEDs' short legs go to a pin and long legs to 5 V
const uint8_t PIN_BUZZER     = 11;   // beeper: + to D11, - to GND
const uint8_t PIN_STATUS     = 13;   // onboard LED
// LCD monitor: 16x2 LCD with an I2C backpack. SDA -> A2, SCL -> A3, VCC -> 5 V, GND -> GND.
const uint8_t PIN_LCD_SDA    = A2;
const uint8_t PIN_LCD_SCL    = A3;

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
const char *JOINT_NAMES[NJ] = {"base", "main arm", "second arm"};

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
const Tool TOOL_PUSHER = {"pusher",      0, 90,   0, true, 0};  // 70 mm bar under the arm end: tip 20 + 70 = 90 mm below the arm's centre line
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
// To set the trims on the real stands, run tests/capture_points: it visits
// every point, lets you nudge the arm onto it, and prints these lines to paste.
struct Station {
  const char *name;
  float theta, r, z, standoff;
  float trim[NJ];
  const Tool *tool;
};

Station ST_A       = {"Point A",       10, 350, 320, 100, {0, 0, 0}, &TOOL_US};      // 10 cm from the plate
Station ST_B       = {"Point B",       10, 350, 150,  70, {0, 0, 0}, &TOOL_US};      // 7 cm, so the arm leans clearly lower than at A
Station ST_C       = {"Point C",       55, 200,  70,  15, {0, 0, 0}, &TOOL_RGB};     // 15 mm from the paper
Station ST_D       = {"Point D",      100, 275, 250,  40, {0, 0, 0}, &TOOL_SOUND};   // 4 cm from the source
Station ST_E       = {"Point E (the button)",170, 250, 300,   0, {0, 0, 0}, &TOOL_PUSHER};  // button top; presses go straight down (see buttonPoint)

// Volume indicator: the pointer height goes from IND_Z_LOW (quiet) to
// IND_Z_HIGH (loud) at theta 110, R 275.
const float IND_THETA    = 110;
const float IND_R        = 275;
const float IND_Z_LOW    = 150;
const float IND_Z_HIGH   = 350;
const float IND_STANDOFF = 60;
float IND_TRIM[NJ] = {0, 0, 0};   // trim for the whole indicator scale, from capture_points

// ======================================================================
// MOTION
// ======================================================================
const uint16_t MOVE_MS      = 1200;
const uint16_t BASE_TURN_MS = 1500;
const uint8_t  STEP_MS      = 20;    // one servo update per 20 ms servo frame
const float    MAX_SPEED_DEG_S = 30;  // top joint speed. Lower it for slower, gentler moves
const uint16_t SETTLE_MS    = 400;
const uint16_t POINT_HOLD_MS = 2000;   // stays still at each point this long before doing anything there
const bool     USE_TRAVEL   = true;          // lift before turning the base
const float    TRAVEL_MAIN  = 0;             // main arm upright while turning
const float    TRAVEL_ELBOW = 15;            // head slightly up while turning
const float    LIFT_TURN_DEG = 25;           // between A and B the base turns this far towards the front and back

// Gestures, each a different shape so they cannot be confused:
//   YES  the sensor goes straight up and down (vertical line), 2 nods
//   NO   the base turns left and right (horizontal line), 2 shakes
//   BOX  the sensor traces a square, up/down and left/right together
const float    GESTURE_UP_MM   = 70;    // YES and BOX: aim this far above and below the point (sensor moves about 40 mm)
const float    GESTURE_DEG     = 15;    // NO and BOX: base turns this far each way
const float    BOX_LIFT_MM     = 60;    // BOX is drawn this far above B, so its bottom stays clear of the main arm limit
const uint8_t  GESTURE_REPEATS = 2;     // nods / shakes
const uint16_t GESTURE_STEP_MS = 900;   // time for each half swing
const uint16_t GESTURE_PAUSE_MS = 400;  // short stop at each end of a swing, so the shape reads clearly

// Point E press. Each press moves the pusher tip straight down, from
// PRESS_TRAVEL_MM above the button to PRESS_DEPTH_MM into it, and straight
// back up. The main arm and second arm move together to keep it vertical. Presses run at
// PRESS_SPEED_DEG_S, faster than other moves, so dots and dashes stay apart.
const float    PRESS_SPEED_DEG_S  = 60;
const float PRESS_TRAVEL_MM   = 36;   // the pusher tip waits this far above the button (more puts the second arm past +45)
const float PRESS_DEPTH_MM    = 8;    // and pushes this far down past contact

// Beeper
const bool     BUZZER_PASSIVE  = true;  // passive buzzer (needs a tone). Set false for an active buzzer
const uint16_t BEEP_HZ         = 2500;
const uint16_t BEEP_MS         = 120;   // one quick beep
const uint16_t BEEP_GAP_MS     = 180;   // silence between beeps
const uint16_t TASK_GAP_MS     = 2000;  // pause after a task's end beeps

// ======================================================================
// SENSOR THRESHOLDS
// ======================================================================
// Detection window for each point, in cm from the ultrasonic face. The arm
// stops 10 cm from A and 7 cm from B, so an object reads about 10 or 7 cm.
//   MIN 3: the HC-SR04 cannot measure closer than about 2 to 3 cm; nearer
//          readings are noise.
//   MAX 15: the standoff plus a margin for positioning error. Going further
//          (20 cm or more) lets the 15 degree cone reach the stand or plate
//          behind the point and report an object that is not there.
// Check on the day: run test_combined with nothing at A and B and note the
// distances. Each empty reading must be at least 3 cm above that point's MAX.
const float    US_A_MIN_CM      = 3.0;
const float    US_A_MAX_CM      = 15.0;
const float    US_B_MIN_CM      = 3.0;
const float    US_B_MAX_CM      = 15.0;
const uint8_t  US_SAMPLES       = 5;
const uint8_t  US_MIN_ECHOES    = 3;         // at least this many of the 5 pings must come back
const unsigned long US_TIMEOUT_US = 25000;

const float REF_RED[3]   = {0.58, 0.23, 0.19};   // calibrate with test_sensors
const float REF_GREEN[3] = {0.28, 0.44, 0.28};
const float REF_BLUE[3]  = {0.20, 0.32, 0.48};
const uint16_t COLOUR_MIN_CLEAR = 30;
const uint8_t  COLOUR_SAMPLES   = 3;
const uint16_t COLOUR_LIVE_MS   = 2000;   // LED follows the sensor live at C for this long
const uint8_t  COLOUR_STEADY    = 3;      // same colour this many readings in a row before the LED changes
const uint16_t LED_CHECK_MS     = 1000;   // each colour shown alone at start up

const uint16_t SOUND_TRIGGER   = 40;
const uint16_t SOUND_AMP_MIN   = 40;
const uint16_t SOUND_AMP_MAX   = 600;
const uint16_t SOUND_WINDOW_MS = 150;
const unsigned long SOUND_TIMEOUT_MS = 3000;
const uint16_t SOUND_HOLD_MS   = 4000;   // time spent pointing at the indicator

// Morse timing, two times slower than before to go easy on the arm. A dot is
// 1 unit, a dash 3, the gap inside a letter 1 and between letters 3. The
// receivers split dots from dashes at 1100 ms and end a letter after 1300 ms
// of silence, halfway between what the button actually sees.
const uint16_t MORSE_UNIT_MS  = 600;   // dot 600 ms, dash 1800 ms
const uint16_t PRESS_MOVE_MS  = 300;   // shortest move onto and off the button
const int16_t  PRESS_EXTRA_MS = 0;
// The button only closes in the last 8 mm of the 44 mm push, about 75% of the
// way through each eased move. The press the receiver sees is the hold plus
// the end of the move down and the start of the move up; the gap is the pause
// plus the rest of both moves. Holds and pauses are cut by those amounts.
const float    PRESS_CLOSE_AT = 0.75;

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
float resultPointZ = -1;   // height pointed at on the indicator

// ======================================================================
// LCD MONITOR
// ======================================================================
// A2 and A3 are not the Uno's hardware I2C pins (A4 and A5 carry the colour
// sensor), so the LCD is driven by this small software I2C. Nothing to
// install. It finds the backpack at 0x27 or 0x3F; with no LCD the run goes on
// as normal. Backpack wiring assumed: P0 RS, P1 RW, P2 E, P3 backlight, P4 to
// P7 = D4 to D7 (the common PCF8574 board).
class SoftLcd : public Print {
 public:
  bool ok = false;
  uint8_t addr = 0;

  bool begin() {
    line(PIN_LCD_SDA, true);
    line(PIN_LCD_SCL, true);
    const uint8_t found[2] = {0x27, 0x3F};
    for (uint8_t i = 0; i < 2 && !ok; i++) {
      addr = found[i];
      ok = startTx();
      stop();
    }
    if (!ok) return false;
    delay(50);
    for (uint8_t i = 0; i < 3; i++) { nibble(0x30); delay(5); }   // into 8 bit mode from any state
    nibble(0x20);                                                  // then 4 bit mode
    command(0x28);   // 2 lines, 5x8 dots
    command(0x0C);   // display on, no cursor
    command(0x06);   // move right after each character
    clear();
    return true;
  }
  void clear() { command(0x01); delay(2); }
  void setCursor(uint8_t col, uint8_t row) { command(0x80 | (col + (row ? 0x40 : 0))); }
  size_t write(uint8_t c) { send(c, RS); return 1; }

 private:
  static const uint8_t RS = 0x01, EN = 0x04, LIGHT = 0x08;

  // Open drain: a line is pulled low, or let go and pulled high.
  void line(uint8_t pin, bool high) {
    if (high) pinMode(pin, INPUT_PULLUP);
    else { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }
    delayMicroseconds(4);
  }
  bool writeByte(uint8_t b) {
    for (int8_t i = 7; i >= 0; i--) {
      line(PIN_LCD_SDA, (b >> i) & 1);
      line(PIN_LCD_SCL, true);
      line(PIN_LCD_SCL, false);
    }
    line(PIN_LCD_SDA, true);
    line(PIN_LCD_SCL, true);
    bool ack = digitalRead(PIN_LCD_SDA) == LOW;
    line(PIN_LCD_SCL, false);
    return ack;
  }
  bool startTx() {
    line(PIN_LCD_SDA, true); line(PIN_LCD_SCL, true);
    line(PIN_LCD_SDA, false); line(PIN_LCD_SCL, false);
    return writeByte(addr << 1);
  }
  void stop() { line(PIN_LCD_SDA, false); line(PIN_LCD_SCL, true); line(PIN_LCD_SDA, true); }

  // One 4 bit transfer, latched by a pulse on E (used only while starting up).
  void nibble(uint8_t hi) {
    if (startTx()) { writeByte(hi | LIGHT | EN); writeByte(hi | LIGHT); }
    stop();
    delayMicroseconds(50);
  }
  // A whole byte as two 4 bit halves in one I2C transfer.
  void send(uint8_t v, uint8_t mode) {
    if (!ok) return;
    uint8_t hi = (v & 0xF0) | mode | LIGHT, lo = (uint8_t)(v << 4) | mode | LIGHT;
    if (startTx()) { writeByte(hi | EN); writeByte(hi); writeByte(lo | EN); writeByte(lo); }
    stop();
    delayMicroseconds(50);
  }
  void command(uint8_t c) { send(c, 0); }
};
SoftLcd lcd;

// ======================================================================
// SERVO OUTPUT
// ======================================================================
int rawClamp(float v) { return constrain((int)round(v), 0, 180); }

// Servo pulse for an angle, keeping fractions of a degree. write() only moves
// in whole degrees, which makes slow moves step visibly; writeMicroseconds()
// moves in about 0.1 degree steps. 544 and 2400 us are the Servo library's own
// 0 and 180 degree pulses, so the angles match what write() would give.
int pulseFor(float deg) {
  deg = constrain(deg, 0.0, 180.0);
  return (int)(544 + deg * (2400 - 544) / 180.0 + 0.5);
}

// Set one joint to a relative angle. The main arm writes both servos in
// opposite directions so they twist the shaft together.
void writeJoint(uint8_t j, float rel) {
  rel = constrain(rel, REL_MIN[j], REL_MAX[j]);
  cur.a[j] = rel;
  switch (j) {
    case J_BASE:
      sBase.writeMicroseconds(pulseFor(SERVO_ZERO_BASE + BASE_DIR * rel));
      break;
    case J_MAIN:
      sLeft.writeMicroseconds(pulseFor(SERVO_ZERO_LEFT + MAIN_DIR * rel));
      sRight.writeMicroseconds(pulseFor(SERVO_ZERO_RIGHT - MAIN_DIR * rel));
      break;
    case J_ELBOW:
      sElbow.writeMicroseconds(pulseFor(SERVO_ZERO_ELBOW + ELBOW_DIR * rel));
      break;
  }
}

void writePose(const Pose &p) {
  for (uint8_t j = 0; j < NJ; j++) writeJoint(j, p.a[j]);
}

// Smooth move of all joints together with cosine easing, no joint faster
// than topSpeed (deg/s). Returns how long it took.
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
  delay(POINT_HOLD_MS);
}

// Leave a point completely: lift to the travel pose (main arm upright, second
// arm slightly up) and turn the base LIFT_TURN_DEG towards the front, then
// hold. The next goToStation turns back and comes down, so A and B, which sit
// above each other, are two clearly separate visits.
void liftClear() {
  Serial.println(F("Leaving the point: lifting up and turning towards the front."));
  Pose lift = {{cur.a[J_BASE], TRAVEL_MAIN, TRAVEL_ELBOW}};
  moveTo(lift, MOVE_MS);
  lift.a[J_BASE] -= (cur.a[J_BASE] > 0 ? 1 : -1) * LIFT_TURN_DEG;
  moveTo(lift, BASE_TURN_MS);
  delay(POINT_HOLD_MS / 2);
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
// Pose that puts the station's sensor dz mm above (or below, if negative) its
// usual spot, still aimed the same way, with the base at baseRel.
bool shiftedPose(const Station &s, float dz, float baseRel, Pose &out) {
  Station t = s;
  t.z += dz;
  if (!toolIK(t, out)) return false;
  for (uint8_t j = 0; j < NJ; j++) out.a[j] += s.trim[j];
  out.a[J_BASE] = baseRel;
  for (uint8_t j = 0; j < NJ; j++) out.a[j] = constrain(out.a[j], REL_MIN[j], REL_MAX[j]);
  return true;
}

// Middle of the base swing, moved in from a limit so both ends are reachable
// (at A and B the base is 80 left, so NO swings about 72).
float baseCentre() {
  return constrain(cur.a[J_BASE], REL_MIN[J_BASE] + GESTURE_DEG + 3, REL_MAX[J_BASE] - GESTURE_DEG - 3);   // 3 deg clear of the servo end stop
}

// Move to a gesture pose and stop briefly, so each end of the shape is clear.
void gestureStep(const Pose &p, uint16_t ms) {
  moveTo(p, ms);
  delay(GESTURE_PAUSE_MS);
}

void gestureYes(const Station &s) {   // vertical line: sensor up and down
  Serial.println(F("Gesture: YES, nodding (sensor moves straight up and down twice)."));
  Pose home = cur, up, down;
  if (!shiftedPose(s, GESTURE_UP_MM, cur.a[J_BASE], up) || !shiftedPose(s, -GESTURE_UP_MM, cur.a[J_BASE], down)) return;
  for (uint8_t i = 0; i < GESTURE_REPEATS; i++) {
    gestureStep(up, GESTURE_STEP_MS);
    gestureStep(down, GESTURE_STEP_MS);
  }
  gestureStep(home, GESTURE_STEP_MS);
}

void gestureNo() {    // horizontal line: base left and right
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

void gestureBox(const Station &s) {   // square: up/down and left/right
  Serial.println(F("Gesture: BOX, nothing at A or B (sensor traces a square: up, across, down, back)."));
  Pose home = cur, corner[4];
  float c = baseCentre();
  const int8_t k[4][2] = {{1, 1}, {-1, 1}, {-1, -1}, {1, -1}};   // {left/right, up/down}
  for (uint8_t i = 0; i < 4; i++)
    if (!shiftedPose(s, BOX_LIFT_MM + k[i][1] * GESTURE_UP_MM, c + k[i][0] * GESTURE_DEG, corner[i])) return;
  for (uint8_t i = 0; i < 4; i++) gestureStep(corner[i], GESTURE_STEP_MS * 2);
  gestureStep(corner[0], GESTURE_STEP_MS * 2);   // close the square
  gestureStep(home, GESTURE_STEP_MS * 2);
}

// ======================================================================
// BEEPER
// ======================================================================
void beepOn()  { if (BUZZER_PASSIVE) tone(PIN_BUZZER, BEEP_HZ); else digitalWrite(PIN_BUZZER, HIGH); }
void beepOff() { if (BUZZER_PASSIVE) noTone(PIN_BUZZER); else digitalWrite(PIN_BUZZER, LOW); }

// n quick beeps
void beeps(uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    beepOn();
    delay(BEEP_MS);
    beepOff();
    delay(BEEP_GAP_MS);
  }
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

// Median of several pings, -1 if fewer than US_MIN_ECHOES came back (one
// stray echo is not enough to call an object).
float readDistanceCm() {
  float v[US_SAMPLES];
  uint8_t n = 0;
  for (uint8_t i = 0; i < US_SAMPLES; i++) {
    float d = pingCm();
    if (d > 0) v[n++] = d;
    delay(30);
  }
  if (n < US_MIN_ECHOES) return -1;
  for (uint8_t i = 1; i < n; i++) {
    float x = v[i];
    int8_t k = i - 1;
    while (k >= 0 && v[k] > x) { v[k + 1] = v[k]; k--; }
    v[k + 1] = x;
  }
  return v[n / 2];
}

bool inWindow(float d, float lo, float hi) { return d >= lo && d <= hi; }

// ======================================================================
// COLOUR
// ======================================================================
// Lights only the LED for that colour (R, G or B); any other letter turns all off.
void setColourLed(char c) {
  digitalWrite(PIN_LED_RED,   (c == 'R') == LEDS_ACTIVE_HIGH);
  digitalWrite(PIN_LED_GREEN, (c == 'G') == LEDS_ACTIVE_HIGH);
  digitalWrite(PIN_LED_BLUE,  (c == 'B') == LEDS_ACTIVE_HIGH);
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
  Serial.print(F("Colour temperature ")); Serial.print(temp);
  Serial.print(F(" K, light level ")); Serial.print(lux);
  Serial.print(F(" lux, strongest colour channel: ")); Serial.print(colourName(primaryColour(r, g, b)));
  Serial.println('.');
}

// Nearest calibrated colour. A reading too dark to trust uses the strongest channel.
char classifyColour(uint32_t r, uint32_t g, uint32_t b, uint32_t c) {
  float sum = r + g + b;
  if (c < COLOUR_MIN_CLEAR || sum == 0) return primaryColour(r, g, b);
  float n[3] = {r / sum, g / sum, b / sum};
  float dr = dist2(n, REF_RED), dg = dist2(n, REF_GREEN), db = dist2(n, REF_BLUE);
  if (dr <= dg && dr <= db) return 'R';
  return dg <= db ? 'G' : 'B';
}

// The LEDs follow what the sensor sees, a fresh reading every 50 ms. The lit
// LED only changes after COLOUR_STEADY matching readings, so one LED stays on
// instead of two flickering.
void showColourLive(uint16_t ms) {
  if (!tcsOk) return;
  Serial.println(F("The LEDs now follow what the colour sensor sees for 2 seconds."));
  char shown = '-', last = '-';
  uint8_t same = 0;
  unsigned long end = millis() + ms;
  while ((long)(end - millis()) > 0) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    char k = classifyColour(r, g, b, c);
    same = (k == last) ? same + 1 : 1;
    last = k;
    if (same >= COLOUR_STEADY && k != shown) { shown = k; setColourLed(k); }
  }
}

// Start up check: red, green and blue LEDs one at a time, one second each,
// with a beep, so a wrong wire or a dead LED or beeper shows straight away.
void ledCheck() {
  const char order[3] = {'R', 'G', 'B'};
  const __FlashStringHelper *pin[3] = {F("D8"), F("D9"), F("D10")};
  for (uint8_t i = 0; i < 3; i++) {
    Serial.print(F("LED check: only ")); Serial.print(colourName(order[i]));
    Serial.print(F(" should be lit now (")); Serial.print(pin[i]); Serial.println(F(")."));
    setColourLed(order[i]);
    beeps(1);
    delay(LED_CHECK_MS);
  }
  setColourLed('-');
  Serial.println(F("LED check done. Each LED should have lit alone, in the order red, green, blue, with a beep each."));
}

// Always returns R, G or B, so the LED always shows a colour.
char readColour() {
  if (!tcsOk) { Serial.println(F("The colour sensor is not connected, so the red LED is used by default.")); return 'R'; }
  uint32_t r = 0, g = 0, b = 0, c = 0;
  for (uint8_t i = 0; i < COLOUR_SAMPLES; i++) {
    uint16_t rr, gg, bb, cc;
    tcs.getRawData(&rr, &gg, &bb, &cc);
    r += rr; g += gg; b += bb; c += cc;
  }
  r /= COLOUR_SAMPLES; g /= COLOUR_SAMPLES; b /= COLOUR_SAMPLES; c /= COLOUR_SAMPLES;
  float sum = r + g + b;
  Serial.print(F("Colour sensor readings: red ")); Serial.print(r);
  Serial.print(F(", green ")); Serial.print(g);
  Serial.print(F(", blue ")); Serial.print(b);
  Serial.print(F(", overall brightness ")); Serial.print(c); Serial.println('.');
  printColourExtras(r, g, b, c);
  if (c < COLOUR_MIN_CLEAR || sum == 0) {
    Serial.println(F("The reading is too dark to trust, so the strongest channel decides the colour."));
  } else {
    Serial.print(F("Share of each colour: red ")); Serial.print((int)(r * 100 / sum));
    Serial.print(F("%, green ")); Serial.print((int)(g * 100 / sum));
    Serial.print(F("%, blue ")); Serial.print((int)(b * 100 / sum)); Serial.println(F("%."));
  }
  return classifyColour(r, g, b, c);
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
unsigned long pressRamp = PRESS_MOVE_MS;   // time one move onto or off the button really takes

// One press: ease down onto the button, hold, ease back up, timed so the
// press the receiver sees is close to `ms`.
void pressFor(uint16_t ms) {
  digitalWrite(PIN_STATUS, HIGH);
  unsigned long ramp = moveAtSpeed(pressPose, PRESS_MOVE_MS, PRESS_SPEED_DEG_S);
  pressRamp = ramp;
  delay(max(0L, (long)ms + PRESS_EXTRA_MS - (long)(2 * (1 - PRESS_CLOSE_AT) * ramp)));
  moveAtSpeed(hoverPose, PRESS_MOVE_MS, PRESS_SPEED_DEG_S);
  digitalWrite(PIN_STATUS, LOW);
}

// Silence between presses, cut by the parts of the moves that are off the button.
void pauseFor(uint16_t ms) {
  delay(max(0L, (long)ms - (long)(2 * PRESS_CLOSE_AT * pressRamp)));
}

void sendMorseWord(const char *word) {
  Serial.print(F("Sending the word ")); Serial.print(word); Serial.println(F(" in Morse code on the button."));
  for (uint8_t i = 0; word[i]; i++) {
    const char *code = morseFor(word[i]);
    if (!code) continue;
    Serial.print(F("   Letter ")); Serial.print(word[i]); Serial.print(F(":"));
    lcd.setCursor(13 + i, 1); lcd.write(word[i]);   // each letter appears as it is sent
    for (uint8_t k = 0; code[k]; k++) {
      Serial.print(code[k] == '.' ? F(" short press (dot)") : F(" long press (dash)"));
      pressFor(code[k] == '.' ? MORSE_UNIT_MS : 3 * MORSE_UNIT_MS);
      if (code[k + 1]) pauseFor(MORSE_UNIT_MS);
    }
    Serial.println();
    if (word[i + 1]) pauseFor(3 * MORSE_UNIT_MS);
  }
  delay(6 * MORSE_UNIT_MS);   // 3.6 s of silence: the receiver ends the word after 3 s
  Serial.print(F("Finished sending ")); Serial.print(word); Serial.println(F(". The receiver should now show it on its screen."));
}

// ======================================================================
// LCD SCREENS
// ======================================================================
// Clear one row and leave the cursor at its start.
void lcdRow(uint8_t row) {
  lcd.setCursor(0, row);
  for (uint8_t i = 0; i < 16; i++) lcd.write(' ');
  lcd.setCursor(0, row);
}

// New screen: the top line, with the cursor left at the start of the bottom line.
void lcdShow(const __FlashStringHelper *top) {
  lcd.clear();
  lcd.print(top);
  lcd.setCursor(0, 1);
}

// "A12.0cm" or "A none" (at most 7 characters), starting at a column of a row.
void lcdDist(uint8_t col, uint8_t row, char tag, float d) {
  lcd.setCursor(col, row);
  lcd.write(tag);
  if (d < 0) lcd.print(F(" none"));
  else { lcd.print(d, d < 100 ? 1 : 0); lcd.print(F("cm")); }
}

const __FlashStringHelper *gestureName() {
  if (resultA && resultB) return F("YES+NO");
  if (resultA) return F("YES");
  if (resultB) return F("NO");
  return F("BOX");
}

// End of the run: two summary screens, swapped every 3 s.
void lcdSummary(uint8_t page) {
  lcd.clear();
  if (page == 0) {
    lcdDist(0, 0, 'A', distA);
    lcdDist(8, 0, 'B', distB);
    lcd.setCursor(0, 1); lcd.print(F("Gesture ")); lcd.print(gestureName());
  } else {
    lcd.print(F("C ")); lcd.print(colourName(resultColour));
    lcd.print(F(" D ")); if (resultSound < 0) lcd.print(F("none")); else { lcd.print(resultLevel); lcd.print('%'); }
    lcd.setCursor(0, 1); lcd.print(F("E sent ")); lcd.print(targetWord);
  }
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
  Serial.print(F("Holding the rest position for "));
  Serial.print(ZERO_HOLD_MS / 1000);
  Serial.println(F(" seconds: base facing forward, main arm upright, second arm level."));
  Serial.print(F("The Morse word for Point E is ")); Serial.print(targetWord);
  Serial.println(F(". To change it, type a new 3-letter word now and press Enter."));
  lcdShow(F("Rest position 5s"));
  lcd.print(F("Word: ")); lcd.print(targetWord);
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
        Serial.print(F("New word saved: ")); Serial.print(targetWord); Serial.println('.');
        lcd.setCursor(6, 1); lcd.print(targetWord);
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
// e.g. "Point A: object found, 10.2 cm away." or "Point B: nothing there (no echo)."
void printDistance(const __FlashStringHelper *where, float d, bool found, float lo, float hi) {
  Serial.print(where);
  if (found) { Serial.print(F(": object found, ")); Serial.print(d, 1); Serial.println(F(" cm away.")); }
  else if (d < 0) Serial.println(F(": nothing there (no echo came back)."));
  else {
    Serial.print(F(": nothing there (nearest echo ")); Serial.print(d, 1);
    Serial.print(F(" cm, outside the ")); Serial.print(lo, 0); Serial.print(F(" to ")); Serial.print(hi, 0);
    Serial.println(F(" cm window)."));
  }
}

void task1() {
  Serial.println(F("\nTASK 1 of 4: checking Points A and B for an object with the ultrasonic sensor."));
  lcdShow(F("Task 1: A and B"));
  lcd.print(F("Going to A"));
  goToStation(ST_A);
  distA = readDistanceCm();
  resultA = inWindow(distA, US_A_MIN_CM, US_A_MAX_CM);
  printDistance(F("Point A"), distA, resultA, US_A_MIN_CM, US_A_MAX_CM);
  lcdRow(1);
  lcdDist(0, 1, 'A', distA);
  if (resultA) { lcdRow(0); lcd.print(F("A found: YES")); gestureYes(ST_A); }

  liftClear();   // A and B are above each other, so make B a visibly separate visit
  Serial.println(F("Going down to Point B, the lower point."));
  lcdRow(0); lcd.print(F("Going to B"));
  goToStation(ST_B);
  distB = readDistanceCm();
  resultB = inWindow(distB, US_B_MIN_CM, US_B_MAX_CM);
  printDistance(F("Point B"), distB, resultB, US_B_MIN_CM, US_B_MAX_CM);
  lcdDist(8, 1, 'B', distB);
  if (resultB) { lcdRow(0); lcd.print(F("B found: NO")); gestureNo(); }

  if (!resultA && !resultB) { lcdRow(0); lcd.print(F("None found: BOX")); gestureBox(ST_B); }
  lcdRow(0); lcd.print(F("T1 done: ")); lcd.print(gestureName());
}

void task2() {
  Serial.println(F("\nTASK 2 of 4: reading the colour of the paper at Point C."));
  lcdShow(F("Task 2: colour C"));
  lcd.print(F("Going to C"));
  goToStation(ST_C);
  lcdRow(1); lcd.print(F("Reading..."));
  showColourLive(COLOUR_LIVE_MS);
  resultColour = readColour();
  Serial.print(F("Point C: the paper is ")); Serial.print(colourName(resultColour));
  Serial.print(F(". The ")); Serial.print(colourName(resultColour)); Serial.println(F(" LED is now on."));
  setColourLed(resultColour);   // stays lit to the end of the run
  lcdRow(1); lcd.print(F("C ")); lcd.print(colourName(resultColour)); lcd.print(F(", LED on"));
}

void task3() {
  Serial.println(F("\nTASK 3 of 4: measuring a tap at Point D and showing its loudness on the indicator."));
  lcdShow(F("Task 3: sound D"));
  lcd.print(F("Going to D"));
  goToStation(ST_D);
  Serial.println(F("Listening for a tap (up to 3 seconds)..."));
  lcdRow(1); lcd.print(F("Tap now..."));
  digitalWrite(PIN_STATUS, HIGH);
  resultSound = captureSound();
  digitalWrite(PIN_STATUS, LOW);

  float f = 0;
  if (resultSound >= 0) {
    f = (float)(resultSound - (int)SOUND_AMP_MIN) / (SOUND_AMP_MAX - SOUND_AMP_MIN);
    f = constrain(f, 0.0, 1.0);
  }
  resultLevel = (int)(f * 100);
  if (resultSound < 0) Serial.println(F("No tap heard in 3 seconds. Pointing at the bottom of the scale."));
  else {
    Serial.print(F("Heard a tap: loudness ")); Serial.print(resultSound);
    Serial.print(F(" out of 1023, which is ")); Serial.print(resultLevel); Serial.println(F("% of the scale."));
  }

  float zPoint = IND_Z_LOW + f * (IND_Z_HIGH - IND_Z_LOW);
  resultPointZ = zPoint;
  lcdRow(1);
  if (resultSound < 0) lcd.print(F("No tap, "));
  else { lcd.print(F("Tap ")); lcd.print(resultLevel); lcd.print(F("% ")); }
  lcd.print(zPoint, 0); lcd.print(F("mm"));
  Station ind = {"the volume indicator", IND_THETA, IND_R, zPoint, IND_STANDOFF, {IND_TRIM[0], IND_TRIM[1], IND_TRIM[2]}, &TOOL_US};
  Pose p;
  if (stationPose(ind, p)) {
    moveTo(p, MOVE_MS);   // indicator is next to D, no travel lift needed
    printPosition("the volume indicator");
  }
  Serial.print(F("Pointing at the indicator ")); Serial.print(zPoint, 0); Serial.println(F(" mm above the table."));
  delay(SOUND_HOLD_MS);
}

void task4() {
  Serial.println(F("\nTASK 4 of 4: pressing the word in Morse code on the button at Point E."));
  lcdShow(F("Task 4: Morse E"));
  lcd.print(F("Going to E"));
  if (!stationPose(buttonPoint(-PRESS_DEPTH_MM), pressPose) || !stationPose(buttonPoint(PRESS_TRAVEL_MM), hoverPose)) {
    Serial.println(F("Point E is out of reach. Skipping the Morse task."));
    return;
  }
  goToPose(hoverPose, "Point E (above the button)");
  Serial.print(F("Each press pushes the pusher tip ")); Serial.print(PRESS_TRAVEL_MM + PRESS_DEPTH_MM, 0);
  Serial.print(F(" mm straight down, ending ")); Serial.print(PRESS_DEPTH_MM, 0); Serial.println(F(" mm into the button."));
  lcdRow(1); lcd.print(F("Sending ")); lcd.print(targetWord); lcd.print(':');
  sendMorseWord(targetWord);
  lcdRow(0); lcd.print(F("T4 done: sent"));
}

// Task n: n beeps, the task, n beeps, then a 2 s pause.
void runTask(uint8_t n, void (*task)()) {
  Serial.print(F("Beep x")); Serial.print(n); Serial.print(F(": start of Task ")); Serial.print(n); Serial.println('.');
  beeps(n);
  task();
  Serial.print(F("Beep x")); Serial.print(n); Serial.print(F(": end of Task ")); Serial.print(n); Serial.println('.');
  beeps(n);
  delay(TASK_GAP_MS);
}

void printSummary() {
  Serial.println(F("\n========== RUN SUMMARY =========="));
  printDistance(F("Point A"), distA, resultA, US_A_MIN_CM, US_A_MAX_CM);
  printDistance(F("Point B"), distB, resultB, US_B_MIN_CM, US_B_MAX_CM);
  Serial.print(F("Gesture made: "));
  if (resultA && resultB) Serial.println(F("YES, then NO."));
  else if (resultA) Serial.println(F("YES."));
  else if (resultB) Serial.println(F("NO."));
  else Serial.println(F("BOX."));
  Serial.print(F("Point C colour: ")); Serial.print(colourName(resultColour)); Serial.println('.');
  Serial.print(F("Point D tap: "));
  if (resultSound < 0) Serial.println(F("none heard."));
  else { Serial.print(F("loudness ")); Serial.print(resultSound); Serial.print(F(", ")); Serial.print(resultLevel); Serial.println(F("% of the scale.")); }
  Serial.print(F("Point E word sent: ")); Serial.print(targetWord); Serial.println('.');
  Serial.println(F("================================="));
  Serial.println(F("Run finished. Press the reset button on the Uno to run again."));
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
  pinMode(PIN_BUZZER, OUTPUT);
  setColourLed('-');

  // Write zero before attaching so nothing jumps on power up.
  writePose(ZERO_POSE);
  sBase.attach(PIN_BASE);
  sLeft.attach(PIN_MAIN_LEFT);
  sRight.attach(PIN_MAIN_RIGHT);
  sElbow.attach(PIN_ELBOW);
  writePose(ZERO_POSE);

  Serial.println(F("\nGroup 21 robotic arm: demonstration program starting."));
  if (lcd.begin()) {
    Serial.print(F("LCD monitor: found at 0x")); Serial.print(lcd.addr, HEX); Serial.println('.');
    lcdShow(F("Group 21 arm")); lcd.print(F("Starting..."));
  } else {
    Serial.println(F("LCD monitor: NOT FOUND. Check SDA (A2), SCL (A3), 5 V and GND. The run carries on without it."));
  }
  tcsOk = tcs.begin();
  Serial.println(tcsOk ? F("Colour sensor: connected.")
                       : F("Colour sensor: NOT FOUND. Check its SDA (A4) and SCL (A5) wires. Point C will light red by default."));

  ledCheck();
  loadWord();
  printPosition("the rest position");
  holdZeroAndReadWord();
  Serial.print(F("Using the Morse word ")); Serial.print(targetWord); Serial.println('.');
}

void loop() {
  switch (state) {
    case ST_START:
      Serial.println(F("\n---------- Starting the run ----------"));
      state = ST_TASK1;
      break;
    case ST_TASK1:
      if (RUN_TASK_1) runTask(1, task1);
      state = ST_TASK2;
      break;
    case ST_TASK2:
      if (RUN_TASK_2) runTask(2, task2);
      state = ST_TASK3;
      break;
    case ST_TASK3:
      if (RUN_TASK_3) runTask(3, task3);
      state = ST_TASK4;
      break;
    case ST_TASK4:
      if (RUN_TASK_4) runTask(4, task4);
      state = ST_FINISH;
      break;
    case ST_FINISH:
      lcdShow(F("Run finished"));
      lcd.print(F("Back to rest"));
      goToPose(ZERO_POSE, "the rest position");
      printSummary();
      state = ST_IDLE;
      break;
    case ST_IDLE: {
      digitalWrite(PIN_STATUS, (millis() / 500) % 2);
      static uint8_t page = 2;   // summary screens on the LCD, swapped every 3 s
      uint8_t now = (millis() / 3000) % 2;
      if (now != page) { page = now; lcdSummary(page); }
      break;
    }
  }
}
