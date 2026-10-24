/*
 * TEST: external colour LEDs (arm controller Uno)
 *
 * Cycles red, green, blue, all on, all off, one second each, and prints
 * which should be lit. If an LED stays dark, check its polarity (long leg
 * to the pin side, short leg to GND) and its 220 ohm resistor.
 *
 * Wiring: red pin 10, green pin 11, blue pin 12, each through 220 ohm to GND.
 */

const uint8_t PIN_LED_RED   = 10;
const uint8_t PIN_LED_GREEN = 11;
const uint8_t PIN_LED_BLUE  = 12;

void show(bool r, bool g, bool b, const __FlashStringHelper *label) {
  digitalWrite(PIN_LED_RED, r);
  digitalWrite(PIN_LED_GREEN, g);
  digitalWrite(PIN_LED_BLUE, b);
  Serial.println(label);
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
}

void loop() {
  show(1, 0, 0, F("RED"));
  show(0, 1, 0, F("GREEN"));
  show(0, 0, 1, F("BLUE"));
  show(1, 1, 1, F("ALL ON"));
  show(0, 0, 0, F("ALL OFF"));
}
