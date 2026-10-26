/*
 * MECN3066A Mechatronics II, Group 31
 * Ultrasonic fault finder (HC-SR04 only, nothing else needed)
 *
 * Wiring, same as arm_controller: VCC -> 5 V, GND -> GND, Trig -> D7, Echo -> D6.
 * Hold a flat object (a book or your hand) about 10 cm in front of the sensor,
 * square to it, and open the Serial Monitor at 115200 baud.
 *
 * Every second it checks three things and says which one fails:
 *   1. Echo idles LOW. If it reads HIGH all the time, Echo is not on D6,
 *      or the module has no power.
 *   2. Echo goes HIGH after a trigger. If not, the module is not being
 *      triggered (Trig not on D7) or has no power.
 *   3. The pulse is a sensible length, which gives the distance.
 */

const uint8_t PIN_ECHO = 6;
const uint8_t PIN_TRIG = 7;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  Serial.println(F("\nUltrasonic fault finder. Trig D7, Echo D6. Hold something flat 10 cm away."));
}

void loop() {
  delay(1000);
  bool idleHigh = digitalRead(PIN_ECHO) == HIGH;

  // 10 us trigger pulse
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // Wait up to 30 ms for Echo to rise, then time how long it stays high.
  unsigned long t0 = micros();
  bool rose = false;
  while (micros() - t0 < 30000UL) {
    if (digitalRead(PIN_ECHO) == HIGH) { rose = true; break; }
  }
  unsigned long start = micros(), width = 0;
  if (rose) {
    while (digitalRead(PIN_ECHO) == HIGH && micros() - start < 40000UL) {}
    width = micros() - start;
  }

  if (idleHigh) {
    Serial.println(F("FAULT: Echo (D6) is HIGH before the ping. Check the Echo wire goes to D6, and that VCC and GND reach the sensor."));
  } else if (!rose) {
    Serial.println(F("FAULT: no echo pulse at all. The sensor is not being triggered or has no power. Check Trig -> D7, and measure 5 V between VCC and GND on the sensor's own pins."));
  } else if (width >= 38000UL) {
    Serial.println(F("Sensor works, but heard nothing back (timed out). Point it at something flat within 2 m, square on."));
  } else {
    Serial.print(F("OK: echo pulse ")); Serial.print(width);
    Serial.print(F(" us = ")); Serial.print(width * 0.0343 / 2.0, 1); Serial.println(F(" cm."));
  }
}
