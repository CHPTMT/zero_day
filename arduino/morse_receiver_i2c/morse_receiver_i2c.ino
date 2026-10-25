/*
  MECN3066A Task 4 - Morse receiver (standalone Arduino)
  Input : pushbutton on D7 (to GND, internal pullup)
  Output: 16x2 I2C LCD (SDA=A4, SCL=A5)
  Decodes 3 letters, then shows the word.

  Timing matches arm_controller.ino. The arm aims for dot 600 ms, dash
  1800 ms, 600 ms between the presses of a letter and 1800 ms between
  letters. Measured from the pusher's real motion, the button sees
  dots 400 to 600 ms, dashes 1600 to 1800 ms, gaps inside a letter 700 to
  900 ms and gaps between letters 1780 to 1980 ms, depending on how far down
  the button switches. Each threshold below sits halfway between the two
  bands, leaving at least about 400 ms of margin on each side.
*/

#include <Wire.h>
#include "src/LiquidCrystal_I2C.h"   // bundled in src/ ("LiquidCrystal I2C" by Frank de Brabander), nothing to install

// ---------- Pins ----------
const int BTN_PIN = 7;
LiquidCrystal_I2C lcd(0x27, 16, 2);   // if blank, try address 0x3F

// ---------- Timing (ms) - TUNE THESE ----------
const unsigned long DEBOUNCE     = 30;
const unsigned long DOT_MAX      = 1100;  // press shorter than this = dot, else dash (arm: dot 400-600, dash 1600-1800)
const unsigned long LETTER_GAP   = 1300;  // release longer than this = letter finished (arm: 700-900 inside, 1780-1980 between)
const unsigned long WORD_TIMEOUT = 4000;  // silence after a letter = word finished early (only if fewer than 3 letters)
const int WORD_LEN               = 3;

// ---------- Morse table A-Z ----------
const char* MORSE[26] = {
  ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..",
  ".---", "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.",
  "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--.."
};

// ---------- State ----------
char wordBuf[WORD_LEN + 1];
int  nLetters = 0;
char symBuf[8];
int  nSym = 0;
bool pressed = false;
bool wordDone = false;
unsigned long tChange = 0;
unsigned long tLastLetter = 0;

void showScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Word: ");
  lcd.print(wordBuf);
  lcd.setCursor(0, 1);
  if (wordDone) lcd.print("Done");
  else          lcd.print(symBuf);
}

void resetAll() {
  nLetters = 0;
  nSym = 0;
  wordBuf[0] = '\0';
  symBuf[0] = '\0';
  wordDone = false;
}

char decodeSymbols() {
  for (int i = 0; i < 26; i++) {
    if (strcmp(symBuf, MORSE[i]) == 0) return 'A' + i;
  }
  return '?';
}

void finishLetter() {
  char c = decodeSymbols();
  if (nLetters < WORD_LEN) {
    wordBuf[nLetters++] = c;
    wordBuf[nLetters] = '\0';
  }
  nSym = 0;
  symBuf[0] = '\0';
  tLastLetter = millis();
  if (nLetters >= WORD_LEN) wordDone = true;
  showScreen();
}

void addSymbol(char s) {
  if (nSym < 7) {
    symBuf[nSym++] = s;
    symBuf[nSym] = '\0';
  }
  showScreen();
}

void setup() {
  pinMode(BTN_PIN, INPUT_PULLUP);
  lcd.init();
  lcd.backlight();
  resetAll();
  lcd.print("Morse RX ready");
  delay(1000);
  showScreen();
}

void loop() {
  unsigned long now = millis();
  bool down = (digitalRead(BTN_PIN) == LOW);

  // Edge detection with debounce
  if (down != pressed && (now - tChange) > DEBOUNCE) {
    if (down) {
      // press started
      if (wordDone) { resetAll(); showScreen(); }   // new word begins
      pressed = true;
      tChange = now;
    } else {
      // released -> classify by press length
      unsigned long dur = now - tChange;
      pressed = false;
      tChange = now;
      addSymbol(dur < DOT_MAX ? '.' : '-');
    }
  }

  // Letter finished (long gap after last release)
  if (!pressed && nSym > 0 && (now - tChange) > LETTER_GAP) {
    finishLetter();
  }

  // Word finished early (fewer than 3 letters, then silence)
  if (!pressed && !wordDone && nSym == 0 && nLetters > 0 &&
      (now - tLastLetter) > WORD_TIMEOUT) {
    wordDone = true;
    showScreen();
  }
}
