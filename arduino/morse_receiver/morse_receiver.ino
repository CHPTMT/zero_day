/*
 * MECN3066A Mechatronics II, Group 31
 * Point E Morse receiver (second, standalone Arduino Uno)
 *
 * Rule R2: this board, its push button and its LCD are completely separate
 * from the arm controller. The only input is the push button.
 *
 * How it decodes
 *   press shorter than 1250 ms  -> dot
 *   press 1250 ms or longer     -> dash
 *   release longer than 1200 ms -> end of letter
 *   release longer than 3000 ms -> end of word, word shown on the LCD
 *
 * What the arm sends (arm_controller.ino): dot 600 ms, dash 1800 ms, 600 ms
 * between the presses of a letter, 1800 ms between letters, then at least
 * 3.6 s of silence after the word. Each press drives the pusher straight
 * down fast and 20 mm past the button (58 mm stroke), so the button sees:
 *   dots 520 to 800 ms, dashes 1720 to 2000 ms,
 *   gaps inside a letter 400 to 680 ms, gaps between letters 1600 to 1880 ms
 * for a button anywhere from 30 mm higher to 32 mm lower than expected.
 * The split points sit between those bands with at least 400 ms to spare.
 *
 * Bounce: a pusher arriving fast can make the contact chatter for a few tens
 * of ms. A release shorter than BOUNCE_GAP_MS is treated as part of the same
 * press, and a press shorter than MIN_PRESS_MS is ignored as noise.
 * Hand presses at a similar pace also work, so markers can test their own words.
 *
 * Wiring
 *   Push button: one side to pin 7, other side to GND (internal pull-up).
 *   LCD: I2C backpack on SDA = A4, SCL = A5, 5 V, GND (address 0x27 or 0x3F).
 *        For a parallel 16x2 LCD set LCD_I2C to 0 and wire RS 12, E 11,
 *        D4 5, D5 4, D6 3, D7 2.
 *   Pin 13 onboard LED lights while the button is held.
 *
 * Libraries: LiquidCrystal I2C (Frank de Brabander, bundled in src/) or LiquidCrystal
 */

#define LCD_I2C 1

#include <Wire.h>
#if LCD_I2C
#include "src/LiquidCrystal_I2C.h"   // bundled in src/, nothing to install
LiquidCrystal_I2C lcd(0x27, 16, 2);   // try 0x3F if the screen stays blank
#else
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
#endif

// ----------------------------------------------------------------------
// SETTINGS (must suit MORSE_UNIT_MS = 600 in arm_controller.ino)
// ----------------------------------------------------------------------
const uint16_t DASH_MIN_MS    = 1250;   // press at least this long = dash
const uint16_t LETTER_GAP_MS  = 1200;   // silence that ends a letter
const uint16_t WORD_GAP_MS    = 3000;   // silence that ends a word
const uint16_t DEBOUNCE_MS    = 20;     // contact must be steady this long to count
const uint16_t BOUNCE_GAP_MS  = 100;    // a shorter release is the same press bouncing
const uint16_t MIN_PRESS_MS   = 100;    // a shorter press is noise

const uint8_t PIN_BUTTON = 7;
const uint8_t PIN_LED    = 13;

// ----------------------------------------------------------------------
// MORSE TABLE
// ----------------------------------------------------------------------
struct MorseChar { char letter; const char *code; };
const MorseChar MORSE[] = {
  {'A', ".-"},   {'B', "-..."}, {'C', "-.-."}, {'D', "-.."},  {'E', "."},
  {'F', "..-."}, {'G', "--."},  {'H', "...."}, {'I', ".."},   {'J', ".---"},
  {'K', "-.-"},  {'L', ".-.."}, {'M', "--"},   {'N', "-."},   {'O', "---"},
  {'P', ".--."}, {'Q', "--.-"}, {'R', ".-."},  {'S', "..."},  {'T', "-"},
  {'U', "..-"},  {'V', "...-"}, {'W', ".--"},  {'X', "-..-"}, {'Y', "-.--"},
  {'Z', "--.."},
};

// Returns the letter for a dot/dash string, or '?' if it is not valid.
char decodeSymbol(const char *code) {
  for (uint8_t i = 0; i < sizeof(MORSE) / sizeof(MORSE[0]); i++) {
    if (strcmp(MORSE[i].code, code) == 0) return MORSE[i].letter;
  }
  return '?';
}

// ----------------------------------------------------------------------
// DECODER STATE
// ----------------------------------------------------------------------
char code[8];            // dots and dashes of the letter being received
uint8_t codeLen = 0;
char rxWord[17];         // letters received so far (one LCD line)
uint8_t wordLen = 0;
bool wordShown = true;   // true once the last word is on screen

bool stablePressed = false;   // debounced button state
bool lastRaw = false;
unsigned long rawChangedAt = 0;
unsigned long pressStart = 0;
unsigned long releaseStart = 0;
bool releasePending = false;  // released, but not yet long enough to rule out a bounce
unsigned long pendingHeld = 0;

// ----------------------------------------------------------------------
// DISPLAY
// ----------------------------------------------------------------------
void showProgress() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Decoding: "));
  lcd.print(code);
  lcd.setCursor(0, 1);
  lcd.print(rxWord);
}

void showWord() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Word received:"));
  lcd.setCursor((16 - wordLen) / 2, 1);
  lcd.print(rxWord);
  Serial.print(F("WORD: "));
  Serial.println(rxWord);
}

// ----------------------------------------------------------------------
// DECODING STEPS
// ----------------------------------------------------------------------
void addSymbol(char s) {
  if (codeLen < sizeof(code) - 1) {
    code[codeLen++] = s;
    code[codeLen] = '\0';
  }
  Serial.print(s);
  showProgress();
}

void endLetter() {
  if (codeLen == 0) return;
  char c = decodeSymbol(code);
  if (wordLen < sizeof(rxWord) - 1) {
    rxWord[wordLen++] = c;
    rxWord[wordLen] = '\0';
  }
  Serial.print(F(" -> "));
  Serial.println(c);
  codeLen = 0;
  code[0] = '\0';
  showProgress();
}

void endWord() {
  endLetter();
  if (wordLen > 0) showWord();
  wordShown = true;
}

void onPressStart(unsigned long t) {
  digitalWrite(PIN_LED, HIGH);
  if (releasePending && t - releaseStart < BOUNCE_GAP_MS) {
    releasePending = false;   // contact bounced: the same press carries on
    return;
  }
  if (wordShown) {
    // First press of a new word: clear the old one.
    wordLen = 0;
    rxWord[0] = '\0';
    wordShown = false;
  }
  pressStart = t;
}

void onPressEnd(unsigned long t) {
  digitalWrite(PIN_LED, LOW);
  releaseStart = t;
  pendingHeld = t - pressStart;
  releasePending = true;      // decided once the release has lasted BOUNCE_GAP_MS
}

// ----------------------------------------------------------------------
// SETUP AND LOOP
// ----------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
#if LCD_I2C
  lcd.init();
  lcd.backlight();
#else
  lcd.begin(16, 2);
#endif
  lcd.print(F("Morse receiver"));
  lcd.setCursor(0, 1);
  lcd.print(F("Waiting..."));
  code[0] = '\0';
  rxWord[0] = '\0';
  Serial.println(F("Morse receiver ready"));
}

void loop() {
  unsigned long now = millis();
  bool raw = digitalRead(PIN_BUTTON) == LOW;  // pressed pulls the pin to GND

  // Debounce: accept a new state only after it has been steady.
  if (raw != lastRaw) {
    lastRaw = raw;
    rawChangedAt = now;
  }
  if (raw != stablePressed && now - rawChangedAt >= DEBOUNCE_MS) {
    stablePressed = raw;
    // Use the time the edge happened, not the time debounce finished.
    if (stablePressed) onPressStart(rawChangedAt);
    else onPressEnd(rawChangedAt);
  }

  // A release that lasts past the bounce window ends the press: classify it.
  if (!stablePressed && releasePending && now - releaseStart >= BOUNCE_GAP_MS) {
    releasePending = false;
    if (pendingHeld >= MIN_PRESS_MS) addSymbol(pendingHeld >= DASH_MIN_MS ? '-' : '.');
  }

  // Gaps are only measured while the button is up.
  if (!stablePressed && !releasePending && !wordShown) {
    unsigned long gap = now - releaseStart;
    if (codeLen > 0 && gap >= LETTER_GAP_MS) endLetter();
    if (gap >= WORD_GAP_MS) endWord();
  }
}
