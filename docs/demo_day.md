# Demo Day: Monday 26 October, NWE Lab

Group 21 is in Session 11, 12:00 to 12:10, Testing Station 2. Changeover is 11:55 to 12:00. Arrive by 11:45 with PPE.

Report any damaged servo or sensor to the tutors before 09:30, with photo or video, by email. After 09:30 no hardware failure claims are accepted.

## Order of testing (Sunday, on the same power supply as Monday)

1. `test_servos`. Watch the 5 s zero hold: base forward, main arm upright, second arm level. Fix any joint that is off with its `SERVO_ZERO_xxx`. In the direction check the base must turn left, the main arm lean forward and the second arm tilt up; flip the `xxx_DIR` of any that do not. Copy the values into `arm_controller.ino` and `test_combined.ino`.
2. Base gear. Fix it mechanically before anything else: the servo mount must not flex away from the turntable, and the gear needs a lip or washer that holds it in the groove. Every time it jumps a tooth the base loses its rest position and every point is off by that much.
3. `test_sensors`. Distance, colour and sound print together. Calibrate the colours with r, g, b on your own paper and the tap levels with t. Paste the printed values into `arm_controller.ino`.
4. Pivot height is set to 130 mm above the table (20 mm platform plus 110 mm). If the arm is rebuilt, re-measure and update `SHOULDER_HEIGHT_MM` in both `arm_controller.ino` and `test_combined.ino`.
5. `test_combined`. The arm visits every station and prints where it thinks the sensor is. If it is off at a station, adjust that station's trim values.
6. `arm_controller`. Full runs with conditions changed: object at A only, B only, neither; each colour; soft and loud tap; different words.

## What each point should look like

Run `test_servos` first and do the direction check: base turns left (towards A and B), main arm leans forward, second arm tilts up. If any joint goes the other way, flip its `xxx_DIR` in `arm_controller.ino` and `test_combined.ino`. With a joint reversed, the arm goes up where it should go down (Point C is the clearest sign).

Joint angles from the rest position, and where the sensor in use ends up (height above the table):

| Point | Base | Main arm | Second arm | What you should see |
|---|---|---|---|---|
| A | 80 left | 18 forward | 14 up | Arm high, ultrasonic 12 cm from A, colour sensor tip 37 mm clear |
| B | 80 left | 34 forward | 4 up | Arm leans well forward and lower, ultrasonic looking down at B from 12 cm |
| C | 22 left | 33 forward | 41 down | Clear dip: second arm points steeply down, colour sensor 15 mm off the paper at 84 mm |
| D | 1 right | 2 forward | 19 down | Arm nearly upright, second arm tilted down, sound sensor 8 cm from the tap point |
| Indicator | 20 right | 1 back to 20 forward | 17 down to 2 down | Ultrasonic points at 150 mm (quiet) up to 350 mm (loud) from 12 cm away |
| E | 80 right | 23 forward | 43 up | Arm swung right, second arm tilted up; each press tilts it 18 degrees down so the pusher drops 52 mm onto the button |

Gestures, one per run, each a different shape. YES (object at A): the sensor moves straight up and down about 40 mm, twice. NO (object at B): the base turns 15 degrees left and right, twice. BOX (nothing at A or B): the sensor traces a square, up, across, down and back. Each swing takes 900 ms with a short stop at each end, and the Serial Monitor prints a `Gesture:` line.

Beeps: Task N beeps N times when it starts and N times when it ends, then a 2 s pause. Task 1 is A and B, Task 2 is C, Task 3 is D, Task 4 is E. At start up there are three single beeps, one with each LED.

## Detection limits at A and B

The arm stops with the ultrasonic 12 cm from A and from B, so an object reads about 12 cm. It cannot come closer: the colour sensor sits 83 mm further out along the arm than the ultrasonic, and at 12 cm its tip stays 37 mm short of the point (at 7 cm it would hit whatever is at B). The code counts an object when the reading is 3 to 17 cm (`US_A_MIN_CM`, `US_A_MAX_CM`, `US_B_MIN_CM`, `US_B_MAX_CM`), and only if at least 3 of the 5 pings echo back.

- 3 cm minimum: the HC-SR04 cannot measure closer than about 2 to 3 cm, and readings below that are noise.
- 17 cm maximum: 12 cm standoff plus 5 cm for positioning error. The sensor's cone is about 15 degrees, so at 17 cm it already covers a circle about 9 cm across. Raising the limit to 22 cm or more lets it see the stand, the plate or the table behind the point and report an object that is not there.
- At B the sensor looks downward at the point. If B is the top of a plate, the plate itself can echo at about 12 to 14 cm with nothing on it. Check this on the day.

How to set them on the day: run `test_combined` with nothing at A and B and note the two distances it prints, then again with the test objects in place. Set each point's MAX halfway between its object reading and its empty reading. The empty reading must end up at least 3 cm above MAX. If B's empty reading is close to 10 cm, lower `US_B_MAX_CM` until it is clear, or raise B's main arm trim so the sensor looks over the plate.

The arm stays still for 2 s at every point before it does anything there. Between A and B it lifts up, turns 25 degrees towards the front, then turns back and leans down to B, so the two visits are clearly separate. The arm cannot bring the sensor down level with B (150 mm) with these arm lengths, so at B it looks down at the point from 12 cm away.

Each Morse press at E is the second arm tilting 18 degrees down and back up, with the base and main arm still: the pusher drops 52 mm, from about 45 mm above the button to 8 mm into it. Presses move at 60 degrees per second, faster than the rest of the run, so dots and dashes stay apart. If the second arm servo cannot manage it, set `PRESS_MAIN_ARM_ONLY` to true and the main arm leans forward for each press instead.

Morse receiver settings, worked out from the pusher's real motion. The button sees dots of 400 to 600 ms, dashes of 1600 to 1800 ms, gaps of 700 to 900 ms inside a letter and 1780 to 1980 ms between letters (the spread depends on how far down the button switches). Both receivers now split dot from dash at 1100 ms and end a letter after 1300 ms of silence, each about halfway between the bands. `morse_receiver` ends a word after 3 s of silence; `morse_receiver_i2c` shows the word after the third letter and uses 4 s only for shorter words.

## Testing at home without the stands

Mark the base centre on the table and stand behind the arm, looking the way it faces at rest. Each point sits at:

| Point | Forward | Sideways | Height above the table |
|---|---|---|---|
| A | 61 mm | 345 mm left | 320 mm |
| B | 61 mm | 345 mm left | 150 mm |
| C | 164 mm | 115 mm left | 70 mm |
| D (tap point) | 271 mm | 48 mm right | 250 mm |
| Volume indicator | 258 mm | 94 mm right | 150 to 350 mm |
| E (button top) | 43 mm | 246 mm right | 300 mm |

Stack books or boxes to each height and tape a 5 x 5 cm card at A and B facing the arm (tilt B's card up towards the sensor). Put coloured paper at C, tap the table next to D, and put the receiver's button breadboard at E. Then:

1. `test_combined` visits every point. Check each lands where the table says, and trim any station that is off.
2. `arm_controller` with a card at A only: expect `Point A: object found, about 10 cm away` and a YES nod. Card at B only: NO shake. No cards: BOX.
3. Change the paper colour, tap soft and hard, change the word, and check the summary each time.

## During the 5 minute changeover

1. Place the arm on the station base and plug in the servo supply.
2. Open the Serial Monitor, set it to 115200 baud and New Line (at 9600 baud the text comes out as random symbols), and press reset. During the 5 s rest hold type the word from the tutors and press Enter. It prints `New word saved: XXX.`
3. Hand the receiver breadboard to the tutor to mount at Point E.

## During the 10 minute slot

- Press reset to start each attempt. A run takes about two and a half minutes.
- Run at least twice; the tutors change conditions between runs. The best attempt counts.
- If one task keeps failing, set its `RUN_TASK_x` to false and upload so the other tasks still score.
- One person films every attempt.

## After the slot

Upload `arm_controller.ino` and `morse_receiver.ino` with the videos to the submission portal straight away.
