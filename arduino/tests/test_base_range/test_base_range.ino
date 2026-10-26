/*
 * MECN3066A Mechatronics II, Group 31
 * TEST: base servo left and right range (D2 only)
 *
 * Finds how far the base servo really turns each way. It starts in the
 * middle (the same 1472 us pulse arm_controller sends for the middle), then
 * turns slowly LEFT a little at a time. Press Enter in the Serial Monitor the
 * moment the base stops turning or starts to buzz. It goes back to the
 * middle and does the same to the RIGHT. Then it prints both ends, says
 * whether the arm can reach A and B (80 left) and E (80 right), and swings
 * slowly between the two ends so you can watch. Type c and Enter to stop in
 * the middle.
 *
 * Serial Monitor: 115200 baud, line ending "New Line".
 * Power the servo from 5 V with enough current (not the Uno's 5V pin if you
 * can avoid it), GND shared with the Uno, signal on D2.
 */

#include <Servo.h>

const uint8_t PIN_BASE  = 2;
const int CENTRE_US     = 1472;   // arm_controller's middle (90 degrees)
const int MIN_US        = 500;    // widest pulses tried; most servos stop before these
const int MAX_US        = 2500;
const int CODE_MIN_US   = 544;    // what arm_controller can send (0 and 180 degrees)
const int CODE_MAX_US   = 2400;
const int STEP_US       = 10;     // about 1 degree
const uint16_t STEP_MS  = 150;    // slow: about 6.5 degrees per second
const float DEG_PER_US  = 180.0 / (2400 - 544);
const float NEED_DEG    = 80;     // A and B are 80 left, E is 80 right

Servo base;
int pulse = CENTRE_US;

float degFromMiddle(int us) { return (us - CENTRE_US) * DEG_PER_US; }

bool enterPressed() {
  bool got = false;
  while (Serial.available()) { char c = Serial.read(); if (c == '\n' || c == '\r') got = true; }
  return got;
}

void flushInput() { delay(20); while (Serial.available()) Serial.read(); }

// Eased move to a pulse at about 18 degrees per second.
void glideTo(int target) {
  int from = pulse;
  unsigned long dur = max(300UL, (unsigned long)(abs(target - from) * DEG_PER_US * 1571.0 / 18.0));
  uint16_t steps = max(1UL, dur / 20);
  for (uint16_t s = 1; s <= steps; s++) {
    pulse = from + (int)((target - from) * (0.5 - 0.5 * cos(PI * s / steps)));
    base.writeMicroseconds(pulse);
    delay(20);
  }
  pulse = target;
  base.writeMicroseconds(pulse);
}

// Turn one way in small steps until Enter is pressed or the pulse limit is hit.
int findEnd(int dir, const __FlashStringHelper *side) {
  Serial.print(F("\nTurning slowly to the ")); Serial.print(side);
  Serial.println(F(". Press Enter as soon as the base STOPS turning or starts to BUZZ."));
  flushInput();
  while (true) {
    int next = pulse + dir * STEP_US;
    if (next < MIN_US || next > MAX_US) {
      Serial.println(F("Reached the widest pulse tested without Enter."));
      break;
    }
    pulse = next;
    base.writeMicroseconds(pulse);
    if ((pulse - CENTRE_US) % 50 == 0) {
      Serial.print(F("   pulse ")); Serial.print(pulse); Serial.print(F(" us, about "));
      Serial.print(fabs(degFromMiddle(pulse)), 0); Serial.print(F(" degrees ")); Serial.println(side);
    }
    delay(STEP_MS);
    if (enterPressed()) break;
  }
  int end = pulse - dir * 2 * STEP_US;   // back off a little from where it was stopped
  Serial.print(F("End to the ")); Serial.print(side); Serial.print(F(": pulse ")); Serial.print(end);
  Serial.print(F(" us, about ")); Serial.print(fabs(degFromMiddle(end)), 0); Serial.println(F(" degrees from the middle."));
  glideTo(CENTRE_US);
  delay(800);
  return end;
}

void report(int leftUs, int rightUs) {
  float left = degFromMiddle(leftUs), right = -degFromMiddle(rightUs);
  Serial.println(F("\n========== BASE RANGE =========="));
  Serial.print(F("Left:  ")); Serial.print(left, 0); Serial.print(F(" degrees (pulse ")); Serial.print(leftUs); Serial.println(F(" us)"));
  Serial.print(F("Right: ")); Serial.print(right, 0); Serial.print(F(" degrees (pulse ")); Serial.print(rightUs); Serial.println(F(" us)"));
  Serial.print(F("Total: ")); Serial.print(left + right, 0); Serial.println(F(" degrees. The arm needs 80 each way, 160 in all."));
  if (left >= NEED_DEG && right >= NEED_DEG) {
    Serial.println(F("OK: the base can reach A and B (80 left) and E (80 right)."));
  } else if (left + right >= 2 * NEED_DEG) {
    float shift = (left - right) / 2;
    Serial.println(F("The total range is enough, but the middle is off to one side."));
    Serial.print(F("Either refit the turntable one tooth towards the short side, or set SERVO_ZERO_BASE to "));
    Serial.print(90 + shift, 0); Serial.println(F(" in arm_controller.ino (then refit the turntable so it faces forward)."));
  } else {
    Serial.println(F("NOT ENOUGH: this servo cannot turn 80 degrees both ways."));
    Serial.println(F("Use a servo with a full 180 degree range, or move the stands closer to the front."));
  }
  if (leftUs > CODE_MAX_US || rightUs < CODE_MIN_US)
    Serial.println(F("Note: this servo goes past the 544 to 2400 us the arm code sends. Tell us the numbers above and the code can be widened."));
  Serial.println(F("================================"));
  Serial.println(F("Now swinging slowly between the two ends. Type c and Enter to stop in the middle."));
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("\nBase range test (D2). Creeping to the middle..."));
  base.writeMicroseconds(CENTRE_US);
  // Wide limits: by default the Servo library will not send pulses outside 544 to 2400 us.
  for (uint8_t i = 0; i < 30; i++) { base.attach(PIN_BASE, MIN_US, MAX_US); delay(22); base.detach(); delay(110); }   // soft start
  base.attach(PIN_BASE, MIN_US, MAX_US);
  base.writeMicroseconds(CENTRE_US);
  delay(1000);
  Serial.println(F("In the middle now. Make sure nothing (cables, stands) blocks the swing."));
  Serial.println(F("Press Enter to start."));
  while (!enterPressed()) {}

  int leftUs = findEnd(+1, F("LEFT"));     // + is the arm's left, as in arm_controller (BASE_DIR = 1)
  int rightUs = findEnd(-1, F("RIGHT"));
  report(leftUs, rightUs);

  while (true) {   // swing between the ends until c is typed
    glideTo(leftUs);  delay(500);
    if (Serial.available() && Serial.read() == 'c') break;
    glideTo(rightUs); delay(500);
    if (Serial.available() && Serial.read() == 'c') break;
  }
  glideTo(CENTRE_US);
  Serial.println(F("Stopped in the middle. Press reset to run the test again."));
}

void loop() {}
