# zero_day: MECN3066A Recycled Robotic Arm (Group 21)

Arduino code for the Mechatronics II robotic arm project. The arm runs on an Arduino Uno with SG90 servos, an HC-SR04 ultrasonic sensor, a TCS34725 colour sensor and an analogue sound sensor. A second Uno decodes the Morse code button presses at Point E and shows the word on an LCD.

Demonstration: Monday 26 October 2026, NWE Lab, Session 11 (12:00 to 12:10), Station 2. See `docs/demo_day.md`.

## Sketches

| Sketch | Board | Purpose |
|---|---|---|
| `arduino/arm_controller` | Uno 1 | Full autonomous demonstration, Tasks 1 to 4 |
| `arduino/morse_receiver` | Uno 2 | Decodes the button presses and shows the word on the LCD |
| `arduino/tests/test_servos` | Uno 1 | Sweep servos, jog joints, teach station poses, try gestures |
| `arduino/tests/test_ultrasonic` | Uno 1 | Distance readings and the present / empty decision |
| `arduino/tests/test_colour` | Uno 1 | Colour readings and red / green / blue calibration |
| `arduino/tests/test_sound` | Uno 1 | Tap amplitude and indicator level |
| `arduino/tests/test_leds` | Uno 1 | Cycle the three external LEDs |
| `arduino/tests/test_morse_press` | Uno 1 | Press the Point E button in Morse for any word |
| `arduino/tests/test_button_receiver` | Uno 2 | LCD check and press duration readout |

Each sketch sits in a folder with the same name, which the Arduino IDE requires. Open the `.ino` file directly.

Libraries (Arduino IDE, Library Manager): Servo, Adafruit TCS34725, LiquidCrystal I2C (Frank de Brabander).

## Stations (theta deg, R mm, Z mm)

| Point | Coordinates | Task |
|---|---|---|
| A | 10, 350, 320 | Ultrasonic presence, YES gesture |
| B | 10, 350, 150 | Ultrasonic presence, NO gesture |
| C | 55, 200, 70 | Colour, external LED |
| D | 100, 275, 250 | Sound capture |
| Volume indicator | 110, 275, 250 | Height proportional to loudness |
| E | 170, 250, 300 | Morse word on the push button |

## How the run works

`arm_controller` is a switch-case state machine: START, TASK1 to TASK4, FINISH, IDLE. Every sensor read has a timeout (ultrasonic 25 ms per ping, sound 3 s) and a safe default, so a bad reading never stops the run. Each task can be turned off with `RUN_TASK_x` as a fallback.

Station poses are raw servo angles, taught on the real arm with `test_servos`. The values in the sketch are placeholders until taught.

The Morse unit is 300 ms (dot 300 ms, dash 900 ms). The receiver splits dots from dashes at 600 ms and ends a word after 1.5 s of silence. A simulation of the arm timing into the receiver code decodes all 10 brief words correctly with press timing errors up to about 250 ms, including contact bounce.

Wiring: `docs/wiring.md`. Calibration order and day checklist: `docs/demo_day.md`.
