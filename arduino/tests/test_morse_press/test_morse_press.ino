/*
 * TEST: Point E button pressing in Morse (arm controller Uno)
 *
 * Moves the arm to POSE_E_HOVER and lets you test pressing on its own, with
 * the receiver Uno and LCD running alongside. Serial Monitor at 115200,
 * line ending "Newline":
 *
 *   NET          send any word (letters only) in Morse, then wait
 *   d            one dot press, to check contact
 *   l            one long 2 s press, to check the button really closes
 *   all          send all 10 words from the brief, one after the other
 *   u300         change the unit time (must match the receiver)
 *   x+50 / x-50  change PRESS_EXTRA_MS (lengthen or shorten every press)
 *
 * Copy POSE_E_HOVER, POSE_E_PRESS, MORSE_UNIT_MS and PRESS_EXTRA_MS from
 * arm_controller.ino before testing, and copy them back once tuned.
 */

#include <Servo.h>

#define USE_WRIST 0
#if USE_WRIST
const uint8_t NUM_SERVOS = 4;
#else
const uint8_t NUM_SERVOS = 3;
#endif

const uint8_t SERVO_PINS[4] = {3, 5, 6, 9};

struct Pose { uint8_t a[4]; };
Pose POSE_E_HOVER = {{170, 120, 90, 90}};
Pose POSE_E_PRESS = {{170, 115, 84, 90}};

uint16_t MORSE_UNIT_MS = 300;
int16_t  PRESS_EXTRA_MS = 0;

const char *WORDS[10] = {"NET", "PET", "SET", "TAP", "PAN", "TAN", "TEN", "PEN", "RAT", "SAT"};

struct MorseChar { char letter; const char *code; };
const MorseChar MORSE[] = {
  {'A', ".-"},   {'B', "-..."}, {'C', "-.-."}, {'D', "-.."},  {'E', "."},
  {'F', "..-."}, {'G', "--."},  {'H', "...."}, {'I', ".."},   {'J', ".---"},
  {'K', "-.-"},  {'L', ".-.."}, {'M', "--"},   {'N', "-."},   {'O', "---"},
  {'P', ".--."}, {'Q', "--.-"}, {'R', ".-."},  {'S', "..."},  {'T', "-"},
  {'U', "..-"},  {'V', "...-"}, {'W', ".--"},  {'X', "-..-"}, {'Y', "-.--"},
  {'Z', "--.."},
};

Servo servos[4];

const char *morseFor(char c) {
  for (uint8_t i = 0; i < sizeof(MORSE) / sizeof(MORSE[0]); i++)
    if (MORSE[i].letter == c) return MORSE[i].code;
  return nullptr;
}

void snapTo(const Pose &p) {
  for (uint8_t j = 0; j < NUM_SERVOS; j++) servos[j].write(p.a[j]);
}

void pressFor(uint16_t ms) {
  snapTo(POSE_E_PRESS);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(max(0, (int)ms + PRESS_EXTRA_MS));
  snapTo(POSE_E_HOVER);
  digitalWrite(LED_BUILTIN, LOW);
}

void sendWord(const char *w) {
  Serial.print(F("Sending "));
  Serial.println(w);
  for (uint8_t i = 0; w[i]; i++) {
    const char *code = morseFor(toupper(w[i]));
    if (!code) continue;
    for (uint8_t k = 0; code[k]; k++) {
      pressFor(code[k] == '.' ? MORSE_UNIT_MS : 3 * MORSE_UNIT_MS);
      if (code[k + 1]) delay(MORSE_UNIT_MS);
    }
    if (w[i + 1]) delay(3 * MORSE_UNIT_MS);
  }
  delay(8 * MORSE_UNIT_MS);  // let the receiver end the word
  Serial.println(F("Done. Check the LCD."));
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  for (uint8_t j = 0; j < NUM_SERVOS; j++) {
    servos[j].write(POSE_E_HOVER.a[j]);
    servos[j].attach(SERVO_PINS[j]);
  }
  Serial.println(F("Morse press test ready. Type a word, d, l, all, u300 or x+50."));
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "d") { pressFor(MORSE_UNIT_MS); return; }
  if (cmd == "l") { pressFor(2000); return; }
  if (cmd == "all") {
    for (uint8_t i = 0; i < 10; i++) sendWord(WORDS[i]);
    return;
  }
  if (cmd.charAt(0) == 'u' && isDigit(cmd.charAt(1))) {
    MORSE_UNIT_MS = cmd.substring(1).toInt();
    Serial.print(F("Unit ms = ")); Serial.println(MORSE_UNIT_MS);
    return;
  }
  if (cmd.charAt(0) == 'x' && (cmd.charAt(1) == '+' || cmd.charAt(1) == '-')) {
    PRESS_EXTRA_MS += cmd.substring(1).toInt();
    Serial.print(F("PRESS_EXTRA_MS = ")); Serial.println(PRESS_EXTRA_MS);
    return;
  }
  cmd.toUpperCase();
  sendWord(cmd.c_str());
}
