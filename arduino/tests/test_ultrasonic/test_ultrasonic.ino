/*
 * TEST: HC-SR04 ultrasonic sensor (arm controller Uno)
 *
 * Prints single pings and the 5 sample median every half second, plus
 * whether an object would count as present with the current thresholds.
 *
 * Calibration: put the arm at POSE_A, read the distance with the 5 x 5 cm
 * plate in place, then with it removed. Set US_DETECT_MAX_CM in
 * arm_controller.ino between the two (closer to the "plate" value plus a few cm).
 *
 * Wiring: VCC 5 V, GND, TRIG pin 7, ECHO pin 8.
 */

const uint8_t PIN_US_TRIG = 7;
const uint8_t PIN_US_ECHO = 8;
const unsigned long US_TIMEOUT_US = 25000;
const float US_DETECT_MIN_CM = 2.0;    // same values as arm_controller.ino
const float US_DETECT_MAX_CM = 25.0;

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

float medianCm() {
  float v[5];
  uint8_t n = 0;
  for (uint8_t i = 0; i < 5; i++) {
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

void setup() {
  Serial.begin(115200);
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  Serial.println(F("single_cm\tmedian_cm\tresult"));
}

void loop() {
  float single = pingCm();
  float med = medianCm();
  Serial.print(single);
  Serial.print('\t');
  Serial.print(med);
  Serial.print('\t');
  if (med < 0) Serial.println(F("no echo (counts as empty)"));
  else if (med >= US_DETECT_MIN_CM && med <= US_DETECT_MAX_CM) Serial.println(F("OBJECT"));
  else Serial.println(F("empty"));
  delay(500);
}
