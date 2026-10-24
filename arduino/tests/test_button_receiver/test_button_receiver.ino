/*
 * TEST: receiver button and LCD (Point E receiver Uno)
 *
 * Checks the hardware on the receiver board before loading morse_receiver.
 * Line 1 shows the button state, line 2 shows how long the last press was
 * in ms and the press count. Serial prints the same.
 *
 * Use it to confirm the LCD address (0x27 or 0x3F), the button wiring, and
 * how long the arm's dot and dash presses really are. Dots should read near
 * 300 ms and dashes near 900 ms. If dots read over 600 ms, lower
 * PRESS_EXTRA_MS in arm_controller.ino.
 *
 * Wiring: button pin 7 to GND. LCD I2C on A4 / A5.
 */

#define LCD_I2C 1

#include <Wire.h>
#if LCD_I2C
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
#else
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
#endif

const uint8_t PIN_BUTTON = 7;
bool lastPressed = false;
unsigned long pressStart = 0;
unsigned int count = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
#if LCD_I2C
  lcd.init();
  lcd.backlight();
#else
  lcd.begin(16, 2);
#endif
  lcd.print(F("Button: up"));
  lcd.setCursor(0, 1);
  lcd.print(F("LCD OK"));
  Serial.println(F("Button and LCD test ready"));
}

void loop() {
  bool pressed = digitalRead(PIN_BUTTON) == LOW;
  if (pressed != lastPressed) {
    delay(15);  // simple debounce
    pressed = digitalRead(PIN_BUTTON) == LOW;
    if (pressed == lastPressed) return;
    lastPressed = pressed;
    digitalWrite(LED_BUILTIN, pressed);
    lcd.setCursor(0, 0);
    lcd.print(pressed ? F("Button: DOWN    ") : F("Button: up      "));
    if (pressed) {
      pressStart = millis();
    } else {
      unsigned long held = millis() - pressStart;
      count++;
      lcd.setCursor(0, 1);
      lcd.print(F("                "));
      lcd.setCursor(0, 1);
      lcd.print(held);
      lcd.print(F(" ms  #"));
      lcd.print(count);
      Serial.print(F("press "));
      Serial.print(count);
      Serial.print(F(": "));
      Serial.print(held);
      Serial.println(F(" ms"));
    }
  }
}
