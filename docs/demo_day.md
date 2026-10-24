# Demo Day: Monday 26 October, NWE Lab

Group 21 is in Session 11, 12:00 to 12:10, Testing Station 2. Changeover is 11:55 to 12:00. Arrive by 11:45 at the latest, with PPE.

Report any damaged servo or sensor to the tutors before 09:30, with photo or video, by email. After 09:30 no hardware failure claims are accepted.

## Before leaving home

- [ ] Laptop charged, Arduino IDE open, both sketches and all test sketches on the laptop
- [ ] Libraries installed: Servo, Adafruit TCS34725, LiquidCrystal I2C
- [ ] Servo supply and its cable, USB cables for both Unos, power bank for the receiver
- [ ] Own red, green and blue paper (calibrated with test_colour)
- [ ] Spare SG90 servos with horns, spare jumper wires, screwdriver, tape, cable ties
- [ ] One person assigned to film every attempt and take photos

## Calibration order (do this Sunday, with the same power supply as Monday)

1. `test_leds`: all three LEDs light.
2. `test_servos`: every servo sweeps. Teach each pose with the jog commands and `p POSE_NAME`, paste into `arm_controller.ino`.
3. `test_ultrasonic`: at POSE_A and POSE_B, read the distance with and without the plate. Set `US_DETECT_MAX_CM`.
4. `test_colour`: at POSE_C, calibrate r, g, b on your own paper. Paste the three REF lines.
5. `test_sound`: soft tap and hard tap at Point D distance. Set `SOUND_AMP_MIN` and `SOUND_AMP_MAX`.
6. `test_button_receiver` on Uno 2: check the LCD and button.
7. `test_morse_press` on Uno 1 with `morse_receiver` on Uno 2: send `all`. Every word must appear on the LCD. If dots show as dashes, lower `PRESS_EXTRA_MS`.
8. Full run of `arm_controller` at least five times in a row with conditions changed (object at A only, B only, neither; each colour; soft and loud tap; different words).

## During the 5 minute changeover

1. Place the arm on the station base and plug in the servo supply.
2. Tutors give the 3 letter word. Open the Serial Monitor on Uno 1 (115200, Newline), press reset, type the word within 4 s. It prints `Saved word XXX`.
3. Close the Serial Monitor. Uno 2 running `morse_receiver` with the LCD showing "Waiting...".
4. Hand the receiver breadboard to the tutor to mount at Point E.

## During the 10 minute slot

- Press reset on Uno 1 to start each attempt. The run takes roughly one minute.
- Run at least twice; the tutors change conditions between runs. Best attempt counts.
- If one task keeps failing, set its `RUN_TASK_x` to false and upload so the other tasks still score.

## After the slot

Upload `arm_controller.ino` and `morse_receiver.ino` with the videos to the submission portal straight away.
