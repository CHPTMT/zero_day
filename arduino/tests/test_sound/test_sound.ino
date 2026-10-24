/*
 * TEST: sound sensor impulse capture (arm controller Uno)
 *
 * Uses the same capture method as Task 3. Each time a tap is heard it prints
 * the peak to peak amplitude and the indicator level (0 to 100 %) the arm
 * would point at. If nothing is heard for 3 s it prints a timeout, which is
 * what the main sketch treats as "quiet".
 *
 * Calibration: do a soft tap and a hard tap at the distance used at Point D.
 * Set SOUND_AMP_MIN a little below the soft value and SOUND_AMP_MAX a little
 * above the hard value in arm_controller.ino. If quiet room noise triggers
 * captures, raise SOUND_TRIGGER, or turn the sensor's potentiometer.
 *
 * Wiring: sound module AO to A0, VCC 5 V, GND. DO is not used.
 */

const uint8_t PIN_SOUND = A0;
const uint16_t SOUND_TRIGGER   = 40;    // same values as arm_controller.ino
const uint16_t SOUND_AMP_MIN   = 40;
const uint16_t SOUND_AMP_MAX   = 600;
const uint16_t SOUND_WINDOW_MS = 150;
const unsigned long SOUND_TIMEOUT_MS = 3000;

int captureSound(int &baselineOut) {
  long sum = 0;
  for (uint8_t i = 0; i < 64; i++) sum += analogRead(PIN_SOUND);
  int baseline = sum / 64;
  baselineOut = baseline;
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

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println(F("Sound test ready. Tap near the sensor."));
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // LED on while listening
  int baseline;
  int amp = captureSound(baseline);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print(F("baseline "));
  Serial.print(baseline);
  if (amp < 0) {
    Serial.println(F("  timeout, nothing heard (arm would point at 0 %)"));
  } else {
    float f = (float)(amp - (int)SOUND_AMP_MIN) / (SOUND_AMP_MAX - SOUND_AMP_MIN);
    f = constrain(f, 0.0, 1.0);
    Serial.print(F("  amplitude "));
    Serial.print(amp);
    Serial.print(F("  level "));
    Serial.print((int)(f * 100));
    Serial.println('%');
  }
  delay(500);  // let the tap ring out before listening again
}
