/*
 * TEST: all sensors at once (Group 21 arm). Servos are not moved.
 *
 * Every half second prints one line:
 *   distance (cm) and present / empty | colour raw, normalised and class |
 *   sound peak to peak amplitude and indicator level
 * The external LED for the detected colour lights if the LEDs are wired.
 *
 * Calibration from the Serial Monitor (115200, Newline):
 *   r / g / b   hold that colour paper under the sensor, then type the letter.
 *               Prints a REF_xxx line to paste into arm_controller.ino.
 *   t           3 s tap test: tap once, prints the amplitude used by Task 3.
 *               Do a soft and a hard tap, set SOUND_AMP_MIN and SOUND_AMP_MAX.
 *
 * Pins: ultrasonic echo D6, trig D7, sound AO A1, colour SDA A4 / SCL A5,
 *       LEDs red D8, green D9, blue D10.
 * Library: Adafruit TCS34725
 */

#include <Wire.h>
#include <Adafruit_TCS34725.h>

const uint8_t PIN_US_ECHO = 6, PIN_US_TRIG = 7, PIN_SOUND = A1;
const uint8_t PIN_LED_RED = 8, PIN_LED_GREEN = 9, PIN_LED_BLUE = 10;

// Same values as arm_controller.ino
const float US_DETECT_MIN_CM = 2.0, US_DETECT_MAX_CM = 20.0;
float REF_RED[3]   = {0.58, 0.23, 0.19};
float REF_GREEN[3] = {0.28, 0.44, 0.28};
float REF_BLUE[3]  = {0.20, 0.32, 0.48};
const uint16_t SOUND_TRIGGER = 40, SOUND_AMP_MIN = 40, SOUND_AMP_MAX = 600;
const uint16_t SOUND_WINDOW_MS = 150;

Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
bool tcsOk = false;

float pingCm() {
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);
  unsigned long us = pulseIn(PIN_US_ECHO, HIGH, 25000UL);
  return us == 0 ? -1 : us * 0.0343 / 2.0;
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

float dist2(const float a[3], const float b[3]) {
  float s = 0;
  for (uint8_t i = 0; i < 3; i++) s += (a[i] - b[i]) * (a[i] - b[i]);
  return s;
}

void readNorm(float n[3], uint16_t raw[4]) {
  tcs.getRawData(&raw[0], &raw[1], &raw[2], &raw[3]);
  float s = (float)raw[0] + raw[1] + raw[2];
  if (s == 0) s = 1;
  for (uint8_t i = 0; i < 3; i++) n[i] = raw[i] / s;
}

char classify(const float n[3]) {
  float dr = dist2(n, REF_RED), dg = dist2(n, REF_GREEN), db = dist2(n, REF_BLUE);
  if (dr <= dg && dr <= db) return 'R';
  return dg <= db ? 'G' : 'B';
}

void showLed(char c) {
  digitalWrite(PIN_LED_RED, c == 'R');
  digitalWrite(PIN_LED_GREEN, c == 'G');
  digitalWrite(PIN_LED_BLUE, c == 'B');
}

// Peak to peak level over a short window (live meter).
int soundLevelNow() {
  int lo = 1023, hi = 0;
  unsigned long end = millis() + 50;
  while ((long)(end - millis()) > 0) {
    int s = analogRead(PIN_SOUND);
    if (s < lo) lo = s;
    if (s > hi) hi = s;
  }
  return hi - lo;
}

// Same capture as Task 3: wait up to 3 s for a tap.
void tapTest() {
  Serial.println(F("[tap] tap once within 3 s"));
  long sum = 0;
  for (uint8_t i = 0; i < 64; i++) sum += analogRead(PIN_SOUND);
  int baseline = sum / 64;
  unsigned long deadline = millis() + 3000;
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
      int amp = max(hi - lo, abs(v - baseline));
      float f = constrain((float)(amp - (int)SOUND_AMP_MIN) / (SOUND_AMP_MAX - SOUND_AMP_MIN), 0.0, 1.0);
      Serial.print(F("[tap] amplitude ")); Serial.print(amp);
      Serial.print(F(" -> level ")); Serial.print((int)(f * 100)); Serial.println('%');
      return;
    }
  }
  Serial.println(F("[tap] timeout, nothing heard"));
}

void calibrate(char which) {
  if (!tcsOk) { Serial.println(F("colour sensor missing")); return; }
  float acc[3] = {0, 0, 0};
  for (uint8_t i = 0; i < 10; i++) {
    float n[3];
    uint16_t raw[4];
    readNorm(n, raw);
    for (uint8_t k = 0; k < 3; k++) acc[k] += n[k] / 10;
  }
  float *ref = which == 'r' ? REF_RED : (which == 'g' ? REF_GREEN : REF_BLUE);
  for (uint8_t k = 0; k < 3; k++) ref[k] = acc[k];
  Serial.print(F(">>> const float "));
  Serial.print(which == 'r' ? F("REF_RED") : (which == 'g' ? F("REF_GREEN") : F("REF_BLUE")));
  Serial.print(F("[3] = {"));
  Serial.print(acc[0], 3); Serial.print(F(", "));
  Serial.print(acc[1], 3); Serial.print(F(", "));
  Serial.print(acc[2], 3); Serial.println(F("};"));
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  tcsOk = tcs.begin();
  Serial.println(F("\n=== SENSOR TEST ==="));
  Serial.println(tcsOk ? F("colour sensor found") : F("colour sensor NOT found (check A4, A5, power)"));
  Serial.println(F("Commands: r, g, b (calibrate colour), t (tap test)"));
}

void loop() {
  if (Serial.available()) {
    char c = tolower(Serial.read());
    if (c == 'r' || c == 'g' || c == 'b') calibrate(c);
    if (c == 't') tapTest();
  }

  float d = medianCm();
  Serial.print(F("dist "));
  if (d < 0) Serial.print(F("no echo"));
  else { Serial.print(d, 1); Serial.print(F(" cm")); }
  Serial.print((d >= US_DETECT_MIN_CM && d <= US_DETECT_MAX_CM) ? F(" OBJECT") : F(" empty"));

  Serial.print(F(" | colour "));
  if (tcsOk) {
    float n[3];
    uint16_t raw[4];
    readNorm(n, raw);
    char col = classify(n);
    Serial.print(raw[0]); Serial.print('/'); Serial.print(raw[1]); Serial.print('/');
    Serial.print(raw[2]); Serial.print(F(" clr ")); Serial.print(raw[3]);
    Serial.print(F(" n ")); Serial.print(n[0], 2); Serial.print(' ');
    Serial.print(n[1], 2); Serial.print(' '); Serial.print(n[2], 2);
    Serial.print(F(" -> ")); Serial.print(col == 'R' ? F("RED") : (col == 'G' ? F("GREEN") : F("BLUE")));
    showLed(col);
  } else {
    Serial.print(F("missing"));
  }

  int s = soundLevelNow();
  Serial.print(F(" | sound ")); Serial.println(s);
  delay(300);
}
