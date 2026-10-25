/*
 * TEST: one servo on its own (Group 21 arm)
 *
 * Moves a single servo slowly between two angles, forever, and prints
 * what it is doing. Nothing else runs, so if this servo does not move
 * here, the problem is its wiring, its power or the servo itself, not
 * the arm code.
 *
 * Set SERVO_PIN to the servo you want to check:
 *   2 = base, 3 = main arm left, 4 = main arm right, 5 = second arm
 *
 * Serial Monitor: 115200 baud.
 */

#include <Servo.h>

const uint8_t SERVO_PIN = 5;      // second arm
const int LOW_ANGLE  = 70;        // 20 degrees each side of the middle (90)
const int HIGH_ANGLE = 110;

Servo s;

// Move one degree every 25 ms, so the move is slow and easy to watch.
void glideTo(int from, int to) {
  int step = (to > from) ? 1 : -1;
  for (int a = from; a != to; a += step) {
    s.write(a);
    delay(25);
  }
  s.write(to);
}

void setup() {
  Serial.begin(115200);
  s.write(90);
  s.attach(SERVO_PIN);
  Serial.print(F("One-servo test on pin D"));
  Serial.print(SERVO_PIN);
  Serial.println(F(". The servo should now sit in the middle (90 degrees)."));
  delay(2000);
}

void loop() {
  Serial.print(F("Moving from 90 to ")); Serial.print(LOW_ANGLE); Serial.println(F(" degrees..."));
  glideTo(90, LOW_ANGLE);
  delay(1000);
  Serial.print(F("Moving to ")); Serial.print(HIGH_ANGLE); Serial.println(F(" degrees..."));
  glideTo(LOW_ANGLE, HIGH_ANGLE);
  delay(1000);
  Serial.println(F("Back to 90 degrees."));
  glideTo(HIGH_ANGLE, 90);
  delay(2000);
}
