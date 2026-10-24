# zero_day: MECN3066A Recycled Robotic Arm (Group 21)

Arduino code for the Mechatronics II robotic arm project. The arm runs on an Arduino Uno with SG90 servos, an HC-SR04 ultrasonic sensor, a TCS34725 colour sensor and an analogue sound sensor. A second Uno decodes the Morse code button presses at Point E and shows the word on an LCD.

Demonstration: Monday 26 October 2026, NWE Lab, Session 11 (12:00 to 12:10), Station 2. See `docs/demo_day.md`.

## Sketches

| Sketch | Board | Purpose |
|---|---|---|
| `arduino/arm_controller` | Uno 1 | Full autonomous demonstration, Tasks 1 to 4, everything printed to Serial |
| `arduino/morse_receiver` | Uno 2 | Decodes the button presses and shows the word on the LCD |
| `arduino/tests/test_servos` | Uno 1 | Zero hold, direction check, full sweep of every joint, manual control |
| `arduino/tests/test_sensors` | Uno 1 | Distance, colour and sound together, with colour and tap calibration |
| `arduino/tests/test_combined` | Uno 1 | Every station, gesture and press in order, printing angles and readings |

Each sketch sits in a folder with the same name, which the Arduino IDE requires. Libraries: Servo, Adafruit TCS34725, LiquidCrystal I2C (receiver only).

## Arm

Main arm 225 mm, second arm 160 mm, sensor face 75 mm beyond the wrist, main arm pivot 60 mm above the base plate. Joint angles are relative, 0 at the zero pose:

| Joint | Zero | Range |
|---|---|---|
| Base (D2) | facing forward | -90 (right) to +90 (left) |
| Main arm (D3 left, D4 right, mirrored) | upright | -15 (back) to +50 (forward) |
| Second arm (D5) | level with the ground | -90 (down) to +45 (up) |

The controller works out the joint angles for each station from these dimensions and the coordinates in the brief, so the sensor head points straight at each target with a set standoff.

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

The Morse unit is 300 ms (dot 300 ms, dash 900 ms). The receiver splits dots from dashes at 600 ms and ends a word after 1.5 s of silence. A simulation of the arm timing into the receiver code decodes all 10 brief words correctly with press timing errors up to about 250 ms, including contact bounce.

Wiring: `docs/wiring.md`. Calibration order and day checklist: `docs/demo_day.md`.
