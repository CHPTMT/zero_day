/*
 * TEST: servos, gestures and pose teaching (arm controller Uno)
 *
 * Use this to check every servo moves, find safe limits, and teach the
 * station poses. Open the Serial Monitor at 115200 baud, line ending
 * "Newline", and type commands:
 *
 *   s              sweep each servo slowly through 60..120 deg, one at a time
 *   b90            set base to 90 deg       (b = base)
 *   m120           set main arm to 120 deg  (m = shoulder / main arm)
 *   e80            set second arm to 80 deg (e = elbow / second arm)
 *   w90            set wrist to 90 deg      (w = wrist, only if USE_WRIST)
 *   b+ / b-        nudge base by +1 / -1 deg (works for m, e, w too)
 *   b+5 / m-3      nudge by any amount
 *   p NAME         print the current pose as a line to paste into
 *                  arm_controller.ino, e.g. "p POSE_A"
 *   h              go to home (90 on every joint)
 *   y / n / x      YES, NO and BOX gestures at the current pose
 *
 * Teaching a station: jog the arm until the sensor is where it must be,
 * then type "p POSE_C" and paste the printed line into arm_controller.ino.
 *
 * Pins match arm_controller.ino. Power the servos from an external 5 V
 * supply with a common ground.
 */

#include <Servo.h>

#define USE_WRIST 0
#if USE_WRIST
const uint8_t NUM_SERVOS = 4;
#else
const uint8_t NUM_SERVOS = 3;
#endif

const uint8_t SERVO_PINS[4] = {3, 5, 6, 9};          // base, shoulder, elbow, wrist
const char    SERVO_KEYS[4] = {'b', 'm', 'e', 'w'};
const char   *SERVO_NAMES[4] = {"base", "main arm", "second arm", "wrist"};
const uint8_t SERVO_MIN[4] = {0, 10, 0, 0};          // same limits as arm_controller.ino
const uint8_t SERVO_MAX[4] = {180, 170, 180, 180};

const uint8_t GESTURE_DEG = 15;

Servo servos[4];
int current[4] = {90, 90, 90, 90};

void writeJoint(uint8_t j, int a) {
  a = constrain(a, SERVO_MIN[j], SERVO_MAX[j]);
  servos[j].write(a);
  current[j] = a;
}

// Smooth move of one joint, 1 deg every 15 ms.
void glide(uint8_t j, int target) {
  target = constrain(target, SERVO_MIN[j], SERVO_MAX[j]);
  while (current[j] != target) {
    writeJoint(j, current[j] + (target > current[j] ? 1 : -1));
    delay(15);
  }
}

void printPose(const char *name) {
  Serial.print(F("Pose "));
  Serial.print(name);
  Serial.print(F(" = {{"));
  for (uint8_t j = 0; j < 4; j++) {
    Serial.print(j < NUM_SERVOS ? current[j] : 90);
    if (j < 3) Serial.print(F(", "));
  }
  Serial.println(F("}};"));
}

void printAll() {
  for (uint8_t j = 0; j < NUM_SERVOS; j++) {
    Serial.print(SERVO_NAMES[j]);
    Serial.print('=');
    Serial.print(current[j]);
    Serial.print(F("  "));
  }
  Serial.println();
}

void sweepAll() {
  for (uint8_t j = 0; j < NUM_SERVOS; j++) {
    Serial.print(F("Sweeping "));
    Serial.println(SERVO_NAMES[j]);
    int start = current[j];
    glide(j, 60);
    glide(j, 120);
    glide(j, start);
    delay(300);
  }
  Serial.println(F("Sweep done"));
}

uint8_t pitchJoint() { return USE_WRIST ? 3 : 2; }

void gesture(char g) {
  uint8_t p = pitchJoint();
  int cb = current[0], cp = current[p];
  if (g == 'y') {
    for (uint8_t i = 0; i < 3; i++) { glide(p, cp + GESTURE_DEG); glide(p, cp - GESTURE_DEG); }
    glide(p, cp);
  } else if (g == 'n') {
    for (uint8_t i = 0; i < 3; i++) { glide(0, cb + GESTURE_DEG); glide(0, cb - GESTURE_DEG); }
    glide(0, cb);
  } else {
    glide(0, cb - GESTURE_DEG); glide(p, cp + GESTURE_DEG);
    glide(0, cb + GESTURE_DEG); glide(p, cp - GESTURE_DEG);
    glide(0, cb - GESTURE_DEG); glide(p, cp + GESTURE_DEG);
    glide(0, cb); glide(p, cp);
  }
}

void handle(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = cmd.charAt(0);

  if (c == 's') { sweepAll(); return; }
  if (c == 'h') { for (uint8_t j = 0; j < NUM_SERVOS; j++) glide(j, 90); printAll(); return; }
  if (c == 'y' || c == 'n' || c == 'x') { gesture(c); return; }
  if (c == 'p') {
    String name = cmd.length() > 2 ? cmd.substring(2) : String("POSE_X");
    name.trim();
    printPose(name.c_str());
    return;
  }

  for (uint8_t j = 0; j < NUM_SERVOS; j++) {
    if (c != SERVO_KEYS[j]) continue;
    String rest = cmd.substring(1);
    if (rest == "+") glide(j, current[j] + 1);
    else if (rest == "-") glide(j, current[j] - 1);
    else if (rest.startsWith("+") || rest.startsWith("-")) glide(j, current[j] + rest.toInt());
    else glide(j, rest.toInt());
    printAll();
    return;
  }
  Serial.println(F("Unknown command"));
}

void setup() {
  Serial.begin(115200);
  for (uint8_t j = 0; j < NUM_SERVOS; j++) {
    servos[j].write(current[j]);
    servos[j].attach(SERVO_PINS[j]);
  }
  Serial.println(F("Servo test ready. Commands: s, h, b90, m+5, e-, p POSE_A, y, n, x"));
  printAll();
}

void loop() {
  if (Serial.available()) handle(Serial.readStringUntil('\n'));
}
