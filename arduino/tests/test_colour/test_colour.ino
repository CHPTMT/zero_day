/*
 * TEST: TCS34725 colour sensor and calibration (arm controller Uno)
 *
 * Prints raw r, g, b, clear and the normalised values r/(r+g+b) etc. every
 * half second, with the colour the main sketch would pick and the matching
 * LED lit.
 *
 * Calibration (do it with the arm at POSE_C, at the real height, under lab
 * lighting, with your own paper):
 *   1. Put the red sheet under the sensor. Type r and press enter. The
 *      averaged normalised values are printed as a REF_RED line.
 *   2. Same with green (type g) and blue (type b).
 *   3. Paste the three lines into arm_controller.ino.
 *
 * Wiring: VIN 5 V, GND, SDA A4, SCL A5, LED pin 4. External LEDs on 10 / 11 / 12.
 * Library: Adafruit TCS34725
 */

#include <Wire.h>
#include <Adafruit_TCS34725.h>

const uint8_t PIN_TCS_LED   = 4;
const uint8_t PIN_LED_RED   = 10;
const uint8_t PIN_LED_GREEN = 11;
const uint8_t PIN_LED_BLUE  = 12;

// Start with the same references as arm_controller.ino
float REF_RED[3]   = {0.58, 0.23, 0.19};
float REF_GREEN[3] = {0.28, 0.44, 0.28};
float REF_BLUE[3]  = {0.20, 0.32, 0.48};

Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void readNorm(float n[3], uint16_t &clear) {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  clear = c;
  float s = (float)r + g + b;
  if (s == 0) s = 1;
  n[0] = r / s; n[1] = g / s; n[2] = b / s;
  Serial.print(F("raw ")); Serial.print(r); Serial.print(' '); Serial.print(g);
  Serial.print(' '); Serial.print(b); Serial.print(F(" clear ")); Serial.print(c);
}

float dist2(const float a[3], const float b[3]) {
  float s = 0;
  for (uint8_t i = 0; i < 3; i++) s += (a[i] - b[i]) * (a[i] - b[i]);
  return s;
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

// Average 10 readings and print them as a line to paste.
void calibrate(char which) {
  float acc[3] = {0, 0, 0};
  for (uint8_t i = 0; i < 10; i++) {
    float n[3];
    uint16_t c;
    readNorm(n, c);
    Serial.println();
    for (uint8_t k = 0; k < 3; k++) acc[k] += n[k] / 10;
  }
  float *ref = which == 'r' ? REF_RED : (which == 'g' ? REF_GREEN : REF_BLUE);
  for (uint8_t k = 0; k < 3; k++) ref[k] = acc[k];
  const char *name = which == 'r' ? "REF_RED" : (which == 'g' ? "REF_GREEN" : "REF_BLUE");
  Serial.print(F(">>> const float "));
  Serial.print(name);
  Serial.print(F("[3] = {"));
  Serial.print(acc[0], 3); Serial.print(F(", "));
  Serial.print(acc[1], 3); Serial.print(F(", "));
  Serial.print(acc[2], 3); Serial.println(F("};"));
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TCS_LED, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  digitalWrite(PIN_TCS_LED, HIGH);
  if (!tcs.begin()) {
    Serial.println(F("TCS34725 not found. Check SDA A4, SCL A5, power."));
    while (true) { showLed('R'); delay(200); showLed('-'); delay(200); }
  }
  Serial.println(F("Colour test ready. Type r, g or b to calibrate that colour."));
}

void loop() {
  if (Serial.available()) {
    char c = tolower(Serial.read());
    if (c == 'r' || c == 'g' || c == 'b') calibrate(c);
  }
  float n[3];
  uint16_t clear;
  readNorm(n, clear);
  char col = classify(n);
  Serial.print(F("  norm "));
  Serial.print(n[0], 3); Serial.print(' ');
  Serial.print(n[1], 3); Serial.print(' ');
  Serial.print(n[2], 3);
  Serial.print(F("  -> "));
  Serial.println(col);
  showLed(col);
  delay(500);
}
