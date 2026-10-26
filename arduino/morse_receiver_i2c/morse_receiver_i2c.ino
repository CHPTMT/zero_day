/*
  MECN3066A Task 4 - Morse receiver (standalone Arduino), Group 31
  Input : pushbutton on D7 (to GND, internal pullup)
  Output: 16x2 I2C LCD (SDA=A4, SCL=A5)
  Decodes 3 letters, then shows the word.

  Timing matches arm_controller.ino: dot 600 ms, dash 1800 ms, 600 ms
  between the presses of a letter, 1800 ms between letters, then at least
  3.6 s of silence. Each press drives the pusher straight down fast, from
  70 mm above the button to 20 mm past it, so the button sees dots 400 to
  800 ms, dashes 1600 to 2000 ms, gaps inside a letter 390 to 790 ms and gaps
  between letters 1590 to 1990 ms, for a button anywhere from 50 mm higher
  to 32 mm lower than expected. The thresholds below sit midway, at least
  350 ms from each band.

  A pusher arriving fast can make the contact chatter. A release shorter
  than BOUNCE_GAP counts as the same press, and a press shorter than
  MIN_PRESS is ignored.
*/

#include <Wire.h>
#include "src/LiquidCrystal_I2C.h"   // bundled in src/ ("LiquidCrystal I2C" by Frank de Brabander), nothing to install

// ---------- Pins ----------
const int BTN_PIN = 7;
LiquidCrystal_I2C lcd(0x27, 16, 2);   // if blank, try address 0x3F

// ---------- Timing (ms) ----------
const unsigned long DEBOUNCE     = 20;    // contact must be steady this long
const unsigned long BOUNCE_GAP   = 100;   // shorter release = the same press bouncing
const unsigned long MIN_PRESS    = 100;   // shorter press = noise
const unsigned long DOT_MAX      = 1250;  // press shorter than this = dot, else dash (arm: dot 400-800, dash 1600-2000)
const unsigned long LETTER_GAP   = 1200;  // release longer than this = letter finished (arm: 390-790 inside, 1590-1990 between)
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
bool pressed = false;          // debounced state
bool lastRaw = false;
unsigned long rawChangedAt = 0;
bool wordDone = false;
unsigned long tPress = 0;
unsigned long tRelease = 0;
bool releasePending = false;
unsigned long heldMs = 0;
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
  bool raw = (digitalRead(BTN_PIN) == LOW);

  // Debounce: a new state counts once it has been steady for DEBOUNCE,
  // timed from when it first changed.
  if (raw != lastRaw) { lastRaw = raw; rawChangedAt = now; }
  if (raw != pressed && now - rawChangedAt >= DEBOUNCE) {
    pressed = raw;
    unsigned long t = rawChangedAt;
    if (pressed) {
      if (releasePending && t - tRelease < BOUNCE_GAP) {
        releasePending = false;                       // bounce: same press continues
      } else {
        if (wordDone) { resetAll(); showScreen(); }   // new word begins
        tPress = t;
      }
    } else {
      tRelease = t;
      heldMs = t - tPress;
      releasePending = true;                          // decided after BOUNCE_GAP
    }
  }

  // The release has outlasted any bounce: classify the press.
  if (!pressed && releasePending && now - tRelease >= BOUNCE_GAP) {
    releasePending = false;
    if (heldMs >= MIN_PRESS) addSymbol(heldMs < DOT_MAX ? '.' : '-');
  }

  // Letter finished (long gap after last release)
  if (!pressed && !releasePending && nSym > 0 && (now - tRelease) > LETTER_GAP) {
    finishLetter();
  }

  // Word finished early (fewer than 3 letters, then silence)
  if (!pressed && !wordDone && nSym == 0 && nLetters > 0 &&
      (now - tLastLetter) > WORD_TIMEOUT) {
    wordDone = true;
    showScreen();
  }
}
