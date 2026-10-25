# Demo Day: Monday 26 October, NWE Lab

Group 21 is in Session 11, 12:00 to 12:10, Testing Station 2. Changeover is 11:55 to 12:00. Arrive by 11:45 with PPE.

Report any damaged servo or sensor to the tutors before 09:30, with photo or video, by email. After 09:30 no hardware failure claims are accepted.

## Order of testing (Sunday, on the same power supply as Monday)

1. `test_servos`. Watch the 5 s zero hold: base forward, main arm upright, second arm level. Fix any joint that is off with its `SERVO_ZERO_xxx`. In the direction check the base must turn left, the main arm lean forward and the second arm tilt up; flip the `xxx_DIR` of any that do not. Copy the values into `arm_controller.ino` and `test_combined.ino`.
2. `test_sensors`. Distance, colour and sound print together. Calibrate the colours with r, g, b on your own paper and the tap levels with t. Paste the printed values into `arm_controller.ino`.
3. Pivot height is set to 130 mm above the table (20 mm platform plus 110 mm). If the arm is rebuilt, re-measure and update `SHOULDER_HEIGHT_MM` in both `arm_controller.ino` and `test_combined.ino`.
4. `test_combined`. The arm visits every station and prints where it thinks the sensor is. If it is off at a station, adjust that station's trim values.
5. `arm_controller`. Full runs with conditions changed: object at A only, B only, neither; each colour; soft and loud tap; different words.

## During the 5 minute changeover

1. Place the arm on the station base and plug in the servo supply.
2. Open the Serial Monitor (115200, Newline) and press reset. During the 5 s zero hold type the word from the tutors. It prints `[word] saved XXX`.
3. Hand the receiver breadboard to the tutor to mount at Point E.

## During the 10 minute slot

- Press reset to start each attempt. A run takes about one minute.
- Run at least twice; the tutors change conditions between runs. The best attempt counts.
- If one task keeps failing, set its `RUN_TASK_x` to false and upload so the other tasks still score.
- One person films every attempt.

## After the slot

Upload `arm_controller.ino` and `morse_receiver.ino` with the videos to the submission portal straight away.
