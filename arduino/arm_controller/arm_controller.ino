/*
 * MECN3066A Mechatronics II, Group 21
 * Recycled robotic arm: main controller (Arduino Uno R3)
 *
 * Runs the full autonomous demonstration as a state machine:
 *   Task 1  Points A and B  ultrasonic presence check, YES / NO / BOX gesture
 *   Task 2  Point C         TCS34725 colour read, light red / green / blue LED
 *   Task 3  Point D         capture a sound impulse, point at a height on the
 *                           volume indicator proportional to loudness
 *   Task 4  Point E         press the push button in Morse code for the
 *                           assigned three letter word
 *
 * Station coordinates from the brief (theta deg, R mm, Z mm):
 *   A (10, 350, 320)   B (10, 350, 150)   C (55, 200, 70)
 *   D (100, 275, 250)  Volume indicator (110, 275, 250)   E (170, 250, 300)
 *
 * Every sensor wait has a timeout and every bad reading has a safe default,
 * so the arm never stalls and always moves on to the next task.
 *
 * QUICK EDITS ON THE DAY
 *   1. Target word: open the Serial Monitor (115200 baud, newline ending)
 *      within WORD_WINDOW_MS of reset and type the word, e.g. NET. It is saved
 *      in EEPROM and survives power cycles. Or change DEFAULT_WORD below.
 *   2. Station poses: section "STATION POSES". Tune them with test_servos.
 *   3. Thresholds: section "SENSOR THRESHOLDS".
 *   4. Fallback: set any RUN_TASK_x to false to skip a task that is failing.
 *
 * Re-run: press the Uno reset button. The run starts automatically.
 *
 * Libraries (Library Manager): Servo, Adafruit TCS34725
 */

#include <Servo.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_TCS34725.h>

// ======================================================================
// TASK SWITCHES (fallback: turn off a task that misbehaves on the day)
// ======================================================================
const bool RUN_TASK_1 = true;   // Points A and B, ultrasonic and gestures
const bool RUN_TASK_2 = true;   // Point C, colour
const bool RUN_TASK_3 = true;   // Point D, sound and height
const bool RUN_TASK_4 = true;   // Point E, Morse code

// ======================================================================
// TARGET WORD
// ======================================================================
const char DEFAULT_WORD[] = "NET";          // used if nothing valid is in EEPROM
const unsigned long WORD_WINDOW_MS = 4000;  // time after reset to type a new word
const unsigned long START_DELAY_MS = 1000;  // pause before the run starts

// ======================================================================
// PINS
// ======================================================================
// Servos. Power them from an external 5 V supply (2 A or more) with the
// supply ground joined to the Uno GND. Do not run them from the Uno 5 V pin.
const uint8_t PIN_SERVO_BASE     = 3;
const uint8_t PIN_SERVO_SHOULDER = 5;   // main arm
const uint8_t PIN_SERVO_ELBOW    = 6;   // second arm
const uint8_t PIN_SERVO_WRIST    = 9;   // head pitch, only used if USE_WRIST

// Ultrasonic HC-SR04
const uint8_t PIN_US_TRIG = 7;
const uint8_t PIN_US_ECHO = 8;

// External colour LEDs on the off-board breadboard (each through 220 ohm)
const uint8_t PIN_LED_RED   = 10;
const uint8_t PIN_LED_GREEN = 11;
const uint8_t PIN_LED_BLUE  = 12;

// TCS34725 on I2C: SDA = A4, SCL = A5. Its LED pin goes to PIN_TCS_LED.
const uint8_t PIN_TCS_LED = 4;

// Sound sensor analogue output (KY-038 / KY-037 AO pin)
const uint8_t PIN_SOUND = A0;

// Onboard LED for status blinks
const uint8_t PIN_STATUS = 13;

// ======================================================================
// SERVOS
// ======================================================================
// Set USE_WRIST to 1 if the head has its own pitch servo. With 0 the arm
// uses three servos and the YES nod is done with the elbow.
#define USE_WRIST 0

#if USE_WRIST
const uint8_t NUM_SERVOS = 4;
#else
const uint8_t NUM_SERVOS = 3;
#endif

enum Joint : uint8_t { BASE = 0, SHOULDER = 1, ELBOW = 2, WRIST = 3 };

const uint8_t SERVO_PINS[4] = {PIN_SERVO_BASE, PIN_SERVO_SHOULDER, PIN_SERVO_ELBOW, PIN_SERVO_WRIST};

// Software limits in raw servo degrees. Tighten these if a link can hit the
// base or a stand. Every write is clamped to these.
const uint8_t SERVO_MIN[4] = {0,   10,  0,   0};
const uint8_t SERVO_MAX[4] = {180, 170, 180, 180};

// ======================================================================
// STATION POSES (raw servo degrees: base, shoulder, elbow, wrist)
// ======================================================================
// PLACEHOLDERS. Base values follow theta from the brief, assuming the base
// servo reads 0 deg along theta = 0. Shoulder and elbow values must be taught
// on the real arm with test_servos, which prints lines ready to paste here.
struct Pose { uint8_t a[4]; };

Pose POSE_HOME    = {{ 90,  90,  90, 90}};   // parked, safe start and end
Pose POSE_TRAVEL  = {{ 90, 110,  70, 90}};   // lifted pose used while the base turns (base value ignored)

Pose POSE_A       = {{ 10, 120,  80, 90}};   // sensor level with A, facing the plate
Pose POSE_B       = {{ 10,  80,  70, 90}};   // sensor level with B, facing the plate
Pose POSE_C       = {{ 55,  60,  40, 90}};   // colour sensor 10 to 20 mm above the paper, no contact
Pose POSE_D       = {{100, 110,  80, 90}};   // sound sensor next to the source at D
Pose POSE_D_LOW   = {{110,  90,  60, 90}};   // pointer at the bottom of the indicator (quiet)
Pose POSE_D_HIGH  = {{110, 130, 100, 90}};   // pointer at the top of the indicator (loud)
Pose POSE_E_HOVER = {{170, 120,  90, 90}};   // a few mm off the button, not touching
Pose POSE_E_PRESS = {{170, 115,  84, 90}};   // button fully pressed

// Turn the base only after lifting to POSE_TRAVEL. Set false if the arm
// clears every stand without lifting.
const bool USE_TRAVEL_POSE = true;

// Motion timing
const uint16_t MOVE_MS       = 1200;  // typical joint move duration
const uint16_t BASE_TURN_MS  = 1500;  // base swing between stations
const uint8_t  STEP_MS       = 15;    // servo update interval while moving
const uint16_t SETTLE_MS     = 400;   // wait after a move before reading sensors

// Gestures
const uint8_t  GESTURE_PITCH_DEG = 15;  // YES nod size
const uint8_t  GESTURE_YAW_DEG   = 15;  // NO shake size
const uint8_t  GESTURE_REPEATS   = 3;
const uint16_t GESTURE_STEP_MS   = 400;

// ======================================================================
// SENSOR THRESHOLDS
// ======================================================================
// Ultrasonic: an object is present if the distance is inside this band.
// Tune with test_ultrasonic: measure with the plate in place and with it removed.
const float    US_DETECT_MIN_CM = 2.0;
const float    US_DETECT_MAX_CM = 25.0;
const uint8_t  US_SAMPLES       = 5;       // median of this many pings
const unsigned long US_TIMEOUT_US = 25000; // about 4 m, so pulseIn never hangs

// Colour: normalised reference values r/(r+g+b), g/(...), b/(...).
// Calibrate with test_colour on the actual paper at the actual distance.
const float REF_RED[3]   = {0.58, 0.23, 0.19};
const float REF_GREEN[3] = {0.28, 0.44, 0.28};
const float REF_BLUE[3]  = {0.20, 0.32, 0.48};
const uint16_t COLOUR_MIN_CLEAR = 30;      // below this the reading is treated as invalid
const uint8_t  COLOUR_SAMPLES   = 3;

// Sound: amplitudes are peak to peak ADC counts over SOUND_WINDOW_MS.
// Calibrate with test_sound: note a soft tap and a hard tap.
const uint16_t SOUND_TRIGGER     = 40;     // deviation from baseline that starts a capture
const uint16_t SOUND_AMP_MIN     = 40;     // maps to the bottom of the indicator
const uint16_t SOUND_AMP_MAX     = 600;    // maps to the top of the indicator
const uint16_t SOUND_WINDOW_MS   = 150;
const unsigned long SOUND_TIMEOUT_MS = 3000; // brief asks for 2 to 3 s timeouts
const uint16_t SOUND_HOLD_MS     = 3000;   // hold the height so the marker sees it

// Morse: must match MORSE_UNIT_MS in morse_receiver.ino
const uint16_t MORSE_UNIT_MS     = 300;    // dot = 1 unit, dash = 3, letter gap = 3
const int16_t  PRESS_EXTRA_MS    = 0;      // add or remove time if dots read as dashes

// ======================================================================
// GLOBALS
// ======================================================================
Servo servos[4];
uint8_t current[4];  // last angle written to each servo

Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
bool tcsOk = false;

char targetWord[4];

enum State : uint8_t {
  ST_START, ST_TASK1, ST_TASK2, ST_TASK3, ST_TASK4, ST_FINISH, ST_IDLE
};
State state = ST_START;

// Results kept for the end of run summary
bool resultA = false, resultB = false;
char resultColour = '-';
int  resultSound = -1;

// ======================================================================
// SERVO MOTION
// ======================================================================

// Write one servo, clamped to its software limits.
void writeJoint(uint8_t j, int angle) {
  angle = constrain(angle, SERVO_MIN[j], SERVO_MAX[j]);
  servos[j].write(angle);
  current[j] = angle;
}

// Move all servos together from where they are to the target pose over
// durationMs, with cosine easing so the arm starts and stops gently.
void moveTo(const Pose &target, uint16_t durationMs) {
  uint8_t start[4];
  for (uint8_t j = 0; j < NUM_SERVOS; j++) start[j] = current[j];
  uint16_t steps = max(1, durationMs / STEP_MS);
  for (uint16_t s = 1; s <= steps; s++) {
    float t = (float)s / steps;
    float e = 0.5 - 0.5 * cos(t * PI);
    for (uint8_t j = 0; j < NUM_SERVOS; j++) {
      writeJoint(j, start[j] + (int)round((target.a[j] - start[j]) * e));
    }
    delay(STEP_MS);
  }
}

// Move a single joint smoothly, leaving the others where they are.
void moveJoint(uint8_t j, int angle, uint16_t durationMs) {
  Pose p;
  for (uint8_t k = 0; k < 4; k++) p.a[k] = current[k];
  p.a[j] = constrain(angle, SERVO_MIN[j], SERVO_MAX[j]);
  moveTo(p, durationMs);
}

// Go to a station: lift, swing the base, then lower onto the station pose.
// Lifting first keeps the head clear of the stands while the base turns.
void goToStation(const Pose &station, const char *name) {
  Serial.print(F("[move] -> "));
  Serial.println(name);
  if (USE_TRAVEL_POSE && current[BASE] != station.a[BASE]) {
    Pose lift = POSE_TRAVEL;
    lift.a[BASE] = current[BASE];
    moveTo(lift, MOVE_MS);
    Pose swing = POSE_TRAVEL;
    swing.a[BASE] = station.a[BASE];
    moveTo(swing, BASE_TURN_MS);
  }
  moveTo(station, MOVE_MS);
  delay(SETTLE_MS);
}

// ======================================================================
// GESTURES (Task 1)
// ======================================================================
// The joint that tilts the head up and down.
uint8_t pitchJoint() { return USE_WRIST ? WRIST : ELBOW; }

// YES: vertical pitch, up and down.
void gestureYes() {
  Serial.println(F("[gesture] YES"));
  uint8_t j = pitchJoint();
  int c = current[j];
  for (uint8_t i = 0; i < GESTURE_REPEATS; i++) {
    moveJoint(j, c + GESTURE_PITCH_DEG, GESTURE_STEP_MS);
    moveJoint(j, c - GESTURE_PITCH_DEG, GESTURE_STEP_MS);
  }
  moveJoint(j, c, GESTURE_STEP_MS);
}

// NO: horizontal yaw, left and right, using the base.
void gestureNo() {
  Serial.println(F("[gesture] NO"));
  int c = current[BASE];
  for (uint8_t i = 0; i < GESTURE_REPEATS; i++) {
    moveJoint(BASE, c + GESTURE_YAW_DEG, GESTURE_STEP_MS);
    moveJoint(BASE, c - GESTURE_YAW_DEG, GESTURE_STEP_MS);
  }
  moveJoint(BASE, c, GESTURE_STEP_MS);
}

// BOX: trace the four corners of a square with yaw and pitch together.
void gestureBox() {
  Serial.println(F("[gesture] BOX"));
  uint8_t pj = pitchJoint();
  int cb = current[BASE], cp = current[pj];
  const int8_t corners[5][2] = {{-1, 1}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
  for (uint8_t i = 0; i < 5; i++) {
    Pose p;
    for (uint8_t k = 0; k < 4; k++) p.a[k] = current[k];
    p.a[BASE] = constrain(cb + corners[i][0] * GESTURE_YAW_DEG, SERVO_MIN[BASE], SERVO_MAX[BASE]);
    p.a[pj]   = constrain(cp + corners[i][1] * GESTURE_PITCH_DEG, SERVO_MIN[pj], SERVO_MAX[pj]);
    moveTo(p, GESTURE_STEP_MS * 2);
  }
  Pose back;
  for (uint8_t k = 0; k < 4; k++) back.a[k] = current[k];
  back.a[BASE] = cb;
  back.a[pj] = cp;
  moveTo(back, GESTURE_STEP_MS * 2);
}

// ======================================================================
// ULTRASONIC (Task 1)
// ======================================================================
// One ping in cm, or -1 if no echo came back in time.
float pingCm() {
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);
  unsigned long us = pulseIn(PIN_US_ECHO, HIGH, US_TIMEOUT_US);
  if (us == 0) return -1;
  return us * 0.0343 / 2.0;
}

// Median of several pings. Invalid pings are dropped. Returns -1 if all failed.
float readDistanceCm() {
  float v[US_SAMPLES];
  uint8_t n = 0;
  for (uint8_t i = 0; i < US_SAMPLES; i++) {
    float d = pingCm();
    if (d > 0) v[n++] = d;
    delay(30);  // let echoes die out between pings
  }
  if (n == 0) return -1;
  for (uint8_t i = 1; i < n; i++) {  // insertion sort, n is tiny
    float x = v[i];
    int8_t k = i - 1;
    while (k >= 0 && v[k] > x) { v[k + 1] = v[k]; k--; }
    v[k + 1] = x;
  }
  return v[n / 2];
}

bool objectPresent(float d) {
  return d >= US_DETECT_MIN_CM && d <= US_DETECT_MAX_CM;
}

// ======================================================================
// COLOUR (Task 2)
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

// Returns 'R', 'G' or 'B'. Always returns a colour so an LED is always lit.
char readColour() {
  if (!tcsOk) {
    Serial.println(F("[colour] sensor missing, default R"));
    return 'R';
  }
  digitalWrite(PIN_TCS_LED, HIGH);
  delay(100);
  uint32_t r = 0, g = 0, b = 0, c = 0;
  for (uint8_t i = 0; i < COLOUR_SAMPLES; i++) {
    uint16_t rr, gg, bb, cc;
    tcs.getRawData(&rr, &gg, &bb, &cc);  // blocks for one integration period
    r += rr; g += gg; b += bb; c += cc;
  }
  digitalWrite(PIN_TCS_LED, LOW);
  r /= COLOUR_SAMPLES; g /= COLOUR_SAMPLES; b /= COLOUR_SAMPLES; c /= COLOUR_SAMPLES;

  Serial.print(F("[colour] raw r g b c = "));
  Serial.print(r); Serial.print(' ');
  Serial.print(g); Serial.print(' ');
  Serial.print(b); Serial.print(' ');
  Serial.println(c);

  float sum = r + g + b;
  if (c < COLOUR_MIN_CLEAR || sum == 0) {
    // Too dark to classify properly: fall back to the strongest channel.
    Serial.println(F("[colour] weak reading, using strongest channel"));
    if (r >= g && r >= b) return 'R';
    return g >= b ? 'G' : 'B';
  }
  float n[3] = {r / sum, g / sum, b / sum};
  float dr = dist2(n, REF_RED), dg = dist2(n, REF_GREEN), db = dist2(n, REF_BLUE);
  if (dr <= dg && dr <= db) return 'R';
  return dg <= db ? 'G' : 'B';
}

// ======================================================================
// SOUND (Task 3)
// ======================================================================
// Waits up to SOUND_TIMEOUT_MS for an impulse. Returns the peak to peak
// amplitude in ADC counts, or -1 if nothing loud enough was heard.
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
      // Peak to peak, but at least the trigger deviation from baseline.
      int amp = max(hi - lo, abs(v - baseline));
      return amp;
    }
  }
  return -1;
}

// Linear interpolation between two poses, f from 0 to 1.
Pose lerpPose(const Pose &a, const Pose &b, float f) {
  Pose p;
  for (uint8_t j = 0; j < 4; j++) p.a[j] = a.a[j] + (int)round((b.a[j] - a.a[j]) * f);
  return p;
}

// ======================================================================
// MORSE (Task 4)
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
  for (uint8_t i = 0; i < sizeof(MORSE) / sizeof(MORSE[0]); i++) {
    if (MORSE[i].letter == c) return MORSE[i].code;
  }
  return nullptr;
}

// Snap straight to a pose with no easing. Used for button presses so the
// contact time matches the commanded time as closely as possible.
void snapTo(const Pose &p) {
  for (uint8_t j = 0; j < NUM_SERVOS; j++) writeJoint(j, p.a[j]);
}

void pressFor(uint16_t ms) {
  snapTo(POSE_E_PRESS);
  digitalWrite(PIN_STATUS, HIGH);
  delay(max(0, (int)ms + PRESS_EXTRA_MS));
  snapTo(POSE_E_HOVER);
  digitalWrite(PIN_STATUS, LOW);
}

void sendMorseWord(const char *word) {
  Serial.print(F("[morse] sending "));
  Serial.println(word);
  for (uint8_t i = 0; word[i]; i++) {
    const char *code = morseFor(word[i]);
    if (!code) continue;  // skip anything that is not A to Z
    Serial.print(F("[morse] "));
    Serial.print(word[i]);
    Serial.print(' ');
    Serial.println(code);
    for (uint8_t k = 0; code[k]; k++) {
      pressFor(code[k] == '.' ? MORSE_UNIT_MS : 3 * MORSE_UNIT_MS);
      if (code[k + 1]) delay(MORSE_UNIT_MS);   // gap inside a letter
    }
    if (word[i + 1]) delay(3 * MORSE_UNIT_MS); // gap between letters
  }
  // The receiver ends the word after a long silence, so just wait here.
  delay(8 * MORSE_UNIT_MS);
}

// ======================================================================
// TARGET WORD (EEPROM and Serial Monitor)
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

// Gives WORD_WINDOW_MS to type a word in the Serial Monitor before the run.
void askForWord() {
  Serial.print(F("Word is "));
  Serial.print(targetWord);
  Serial.print(F(". Type a new 3 letter word within "));
  Serial.print(WORD_WINDOW_MS / 1000);
  Serial.println(F(" s to change it."));
  char buf[8];
  uint8_t n = 0;
  unsigned long deadline = millis() + WORD_WINDOW_MS;
  while ((long)(deadline - millis()) > 0) {
    if (!Serial.available()) continue;
    char c = toupper(Serial.read());
    if (c == '\n' || c == '\r') {
      if (n == 3) {
        buf[3] = '\0';
        if (validWord(buf)) {
          strcpy(targetWord, buf);
          for (uint8_t i = 0; i < 3; i++) EEPROM.update(i, targetWord[i]);
          Serial.print(F("Saved word "));
          Serial.println(targetWord);
          return;
        }
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
  Serial.println(F("=== Task 1: Points A and B ==="));
  goToStation(POSE_A, "A");
  float dA = readDistanceCm();
  resultA = objectPresent(dA);
  Serial.print(F("[A] distance cm = ")); Serial.print(dA);
  Serial.println(resultA ? F("  object") : F("  empty"));
  if (resultA) gestureYes();

  goToStation(POSE_B, "B");
  float dB = readDistanceCm();
  resultB = objectPresent(dB);
  Serial.print(F("[B] distance cm = ")); Serial.print(dB);
  Serial.println(resultB ? F("  object") : F("  empty"));
  if (resultB) gestureNo();

  if (!resultA && !resultB) gestureBox();
}

void task2() {
  Serial.println(F("=== Task 2: Point C ==="));
  goToStation(POSE_C, "C");
  resultColour = readColour();
  Serial.print(F("[C] colour = "));
  Serial.println(resultColour);
  setColourLed(resultColour);  // stays lit for the rest of the run
}

void task3() {
  Serial.println(F("=== Task 3: Point D ==="));
  goToStation(POSE_D, "D");
  Serial.println(F("[D] listening"));
  digitalWrite(PIN_STATUS, HIGH);
  resultSound = captureSound();
  digitalWrite(PIN_STATUS, LOW);

  float f = 0;  // no sound: point at the bottom of the scale
  if (resultSound >= 0) {
    f = (float)(resultSound - (int)SOUND_AMP_MIN) / (SOUND_AMP_MAX - SOUND_AMP_MIN);
    f = constrain(f, 0.0, 1.0);
  }
  Serial.print(F("[D] amplitude = ")); Serial.print(resultSound);
  Serial.print(F("  level = ")); Serial.print((int)(f * 100)); Serial.println('%');

  // The indicator is close to D, so slide there without the travel lift.
  moveTo(lerpPose(POSE_D_LOW, POSE_D_HIGH, f), MOVE_MS);
  delay(SOUND_HOLD_MS);
}

void task4() {
  Serial.println(F("=== Task 4: Point E ==="));
  goToStation(POSE_E_HOVER, "E");
  sendMorseWord(targetWord);
}

void printSummary() {
  Serial.println(F("=== Run complete ==="));
  Serial.print(F("A object: ")); Serial.println(resultA ? F("yes") : F("no"));
  Serial.print(F("B object: ")); Serial.println(resultB ? F("yes") : F("no"));
  Serial.print(F("C colour: ")); Serial.println(resultColour);
  Serial.print(F("D amplitude: ")); Serial.println(resultSound);
  Serial.print(F("E word: ")); Serial.println(targetWord);
  Serial.println(F("Press reset to run again."));
}

// ======================================================================
// SETUP AND MAIN LOOP
// ======================================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_TCS_LED, OUTPUT);
  pinMode(PIN_STATUS, OUTPUT);
  setColourLed('-');

  // Write the home angle before attach so the servos do not jump to 90 first.
  for (uint8_t j = 0; j < NUM_SERVOS; j++) {
    servos[j].write(POSE_HOME.a[j]);
    servos[j].attach(SERVO_PINS[j]);
    current[j] = POSE_HOME.a[j];
  }

  tcsOk = tcs.begin();
  Serial.println(tcsOk ? F("TCS34725 found") : F("TCS34725 NOT found, Task 2 will use a default"));

  loadWord();
  askForWord();
  Serial.print(F("Target word: "));
  Serial.println(targetWord);
  delay(START_DELAY_MS);
}

void loop() {
  switch (state) {
    case ST_START:
      Serial.println(F("=== Run start ==="));
      moveTo(POSE_HOME, MOVE_MS);
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
      goToStation(POSE_HOME, "HOME");
      printSummary();
      state = ST_IDLE;
      break;
    case ST_IDLE:
      // Slow blink so the tutors can see the run has finished.
      digitalWrite(PIN_STATUS, (millis() / 500) % 2);
      break;
  }
}
