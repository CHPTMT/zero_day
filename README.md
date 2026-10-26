# zero_day: MECN3066A Recycled Robotic Arm (Group 31)

Arduino code for the Mechatronics II robotic arm project. The arm runs on an Arduino Uno with SG90 servos, an HC-SR04 ultrasonic sensor, a TCS34725 colour sensor and an analogue sound sensor. A second Uno decodes the Morse code button presses at Point E and shows the word on an LCD.

Demonstration: Monday 26 October 2026, NWE Lab, Session 11 (12:00 to 12:10), Station 2. See `docs/demo_day.md`.

## Sketches

| Sketch | Board | Purpose |
|---|---|---|
| `arduino/arm_controller` | Uno 1 | Full autonomous demonstration, Tasks 1 to 4, everything printed to Serial |
| `arduino/morse_receiver` | Uno 2 | Decodes the button presses and shows the word on the LCD |
| `arduino/morse_receiver_i2c` | Uno 2 | The team's own receiver, timing matched to the arm (use one receiver or the other) |
| `arduino/tests/test_servos` | Uno 1 | Zero hold, direction check, full sweep of every joint, manual control |
| `arduino/tests/test_sensors` | Uno 1 | Distance, colour and sound together, with colour and tap calibration |
| `arduino/tests/test_combined` | Uno 1 | Every station, gesture and press in order, printing angles and readings |
| `arduino/tests/capture_points` | Uno 1 | Goes to each point, lets you nudge the arm onto the real stand, prints the trims to paste into `arm_controller` |

Each sketch sits in a folder with the same name, which the Arduino IDE requires. Each folder also has a `sketch.yaml` that selects the board (Arduino UNO) and, for the arm sketches, the port (COM8) when the sketch is opened in Arduino IDE 2. If the laptop gives the Uno a different COM number, change `default_port` in that file or pick the port in Tools > Port. Nothing needs installing: the colour sensor library (Adafruit TCS34725 with its I2C helper) and the LCD library (LiquidCrystal I2C) are bundled in each sketch's `src` folder, and Servo, Wire and EEPROM come with the Arduino IDE. Keep the `src` folder next to the `.ino` when copying a sketch.

## Arm

Main arm 225 mm, second arm 160 mm to the sensor board, main arm pivot 130 mm above the table (20 mm platform plus 110 mm to the pivot). The sensor array runs along the axis of the second arm, measured from its end: ultrasonic at the centre; sound sensor 42 mm further out and 45 mm to the left; colour sensor 83 mm out (the tip) and 45 mm to the right (left and right as seen facing the machine from the front). A 70 mm button pusher hangs under the 40 mm second arm at its end, square to it; its tip is 90 mm below the arm's centre line. Side-face widths: main arm 50 mm, second arm 40 mm. Joint angles are relative, 0 at the zero pose:

| Joint | Zero | Range |
|---|---|---|
| Base (D2) | facing forward | +90 (left, towards A and B) to -90 (right, towards E) |
| Main arm (D3 left, D4 right, mirrored) | upright | -15 (back) to +50 (forward) |
| Second arm (D5) | level with the ground | -90 (down) to +45 (up) |

The controller works out the joint angles for each station from these dimensions and the coordinates in the brief, aiming whichever sensor each task needs at its target with a set gap: ultrasonic 100 mm from A, 70 mm from B and 60 mm from the volume indicator, colour sensor 15 mm from the blue face at C, sound sensor 40 mm from D. At E each press pushes the 70 mm pusher straight down, from 36 mm above the button to 8 mm into it.

## Stations (theta deg, R mm, Z mm)

| Point | Coordinates | Task |
|---|---|---|
| A | 10, 350, 320 | Ultrasonic presence, YES gesture |
| B | 10, 350, 150 | Ultrasonic presence, NO gesture |
| C | 55, 200, 70 | Colour, red, green or blue LED |
| D | 100, 275, 250 | Sound capture |
| Volume indicator | 110, 275, 250 | Height proportional to loudness |
| E | 170, 250, 300 | Morse word on the push button |

## How the run works

`arm_controller` is a switch-case state machine: START, TASK1 to TASK4, FINISH, IDLE. Every sensor read has a timeout (ultrasonic 25 ms per ping, sound 3 s) and a safe default, so a bad reading never stops the run. Each task can be turned off with `RUN_TASK_x` as a fallback.

The Morse timing uses a 600 ms unit: a dot is 600 ms, a dash 1800 ms, 600 ms between the presses of a letter and 1800 ms between letters. Each press drives the pusher straight down 12 mm past the button at 120 deg/s. Both receivers split dots from dashes at 1250 ms, end a letter after 1200 ms of silence and merge contact bounce shorter than 100 ms: `morse_receiver` (the repo version) and `morse_receiver_i2c` (the team's version, `DOT_MAX` 1250, `LETTER_GAP` 1200, `WORD_TIMEOUT` 4000). A simulation of the pusher's motion, fed into both receiver sketches, decodes every test word with the button anywhere from 30 mm higher to 12 mm lower than expected.

Wiring: `docs/wiring.md`. Calibration order and day checklist: `docs/demo_day.md`.

## Simulation

- `simulation/arm_bench.html`: open in a browser. Animated side and plan views of the arm using the same joint maths as the firmware, a picker that plays each sketch (demo, servo test, sensor test, combined test) with the serial output it prints, adjustable test conditions, and a breadboard wiring view with the pin map.
- `simulation/wokwi/`: files to run the real sketch on wokwi.com with simulated servos, ultrasonic, LEDs and a potentiometer in place of the sound sensor. See its README.
