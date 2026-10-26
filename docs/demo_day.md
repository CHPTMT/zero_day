# Demo Day: Monday 26 October, NWE Lab

Group 31 is in Session 11, 12:00 to 12:10, Testing Station 2. Changeover is 11:55 to 12:00. Arrive by 11:45 with PPE.

Report any damaged servo or sensor to the tutors before 09:30, with photo or video, by email. After 09:30 no hardware failure claims are accepted.

## Order of testing (Sunday, on the same power supply as Monday)

1. `test_servos`, zero points first. After the sweep type `z`, then nudge each joint (`b+`, `m-`, `e++` and so on) until the base faces straight forward, the main arm stands upright and the second arm is level; check with a set square or a phone level. If the two main arm servos buzz against each other at rest, nudge just one with `l+`/`l-` or `r+`/`r-` until they go quiet. Type `c` and paste the four `SERVO_ZERO` lines it prints into `arm_controller.ino`, `test_combined.ino` and `capture_points.ino`. Then press reset and watch the direction check: base turns left, main arm leans forward, second arm tilts up; flip the `xxx_DIR` of any that do not, in all four sketches.
2. Base servo (SG92R, 5 V). Run `tests/test_base_range` and press Enter when the base stops each way; it says whether 80 degrees each way is possible. Then run `tests/test_base`: `c` holds the middle while the turntable is fitted, `z` prints `SERVO_ZERO_BASE`, and `a` / `e` turn to 80 left / right. If the arm stops short of A and B or E, nudge it on with `+` / `-`, type `k`, and paste the `BASE_GAIN` line it prints into every sketch.
3. Base gear. Fix it mechanically before anything else: the servo mount must not flex away from the turntable, and the gear needs a lip or washer that holds it in the groove. Every time it jumps a tooth the base loses its rest position and every point is off by that much.
4. `test_sensors`. Distance, colour and sound print together. Calibrate the colours with r, g, b on your own paper and the tap levels with t. Paste the printed values into `arm_controller.ino`.
5. Pivot height is set to 130 mm above the table (20 mm platform plus 110 mm). If the arm is rebuilt, re-measure and update `SHOULDER_HEIGHT_MM` in both `arm_controller.ino` and `test_combined.ino`.
6. `test_combined`. The arm visits every station and prints where it thinks the sensor is.
7. `capture_points`. With the stands in place, the arm goes to each point exactly as the demo does and waits. Nudge it onto the real point (`b`, `m`, `e` with `+`, `-`, `++`, `--`; one `+` is 1 degree, `++` is 5), type `s` to read the sensor there and `n` for the next point. At E, `t` is a test dot and `tt` a test dash; `d+`/`d-` push 1 mm deeper or shallower, `u+`/`u-` make the pusher wait 5 mm higher or lower and `x+`/`x-` lean the main arm 1 degree more or less at the bottom of each press. Tune until a dot and a dash both register on the receiver, and paste the `PRESS_TRAVEL_MM`, `PRESS_DEPTH_MM` and `PRESS_MAIN_EXTRA_DEG` lines it prints as well. Keep the wait at 85 mm or less, or the second arm runs out of travel. At the end it prints the `Station` lines and `IND_TRIM` with the trims filled in: paste them into `arm_controller.ino` over the lines with the same names.
8. `arm_controller`. Full runs with conditions changed: object at A only, B only, neither; each colour; soft and loud tap; different words.
9. `arm_controller_fine` (optional). The same program with the fine search switched on: after finding the object at A or B, and after the colour wave at C, the arm makes very small moves (2, then 1, then 0.5 degrees) left, right, up and down and keeps the spot that reads best. Try it after `arm_controller` works. Use it on the day only if it reads more reliably on your stands; each run takes about 30 s longer.

## What each point should look like

Run `test_servos` first and do the direction check: base turns left (towards A and B), main arm leans forward, second arm tilts up. If any joint goes the other way, flip its `xxx_DIR` in `arm_controller.ino` and `test_combined.ino`. With a joint reversed, the arm goes up where it should go down (Point C is the clearest sign).

Joint angles from the rest position, and where the sensor in use ends up (height above the table):

| Point | Base | Main arm | Second arm | What you should see |
|---|---|---|---|---|
| A | 80 left | 24 forward | 20 up | Arm high, ultrasonic 10 cm from A (at 250 mm out, 327 mm up) |
| B | 80 left | 47 forward | 11 up | Arm leans well forward, ultrasonic looking down at B from 7 cm (294 mm out, 191 mm up). The colour sensor tip reaches 364 mm out and 143 mm up, just past B: check it clears whatever sits at B |
| C | 22 left | 33 forward | 41 down | Clear dip: second arm points about 75 degrees down, colour sensor tip just in front of the blue face, looking steeply down at it (see Point C below) |
| D | 1 right | 13 forward | 11 down | Second arm tilted down, sound sensor 4 cm from the tap point |
| Indicator | 20 right | 35 forward (quiet) to 14 forward (loud) | 13 down to 15 up | Ultrasonic 6 cm from the scale, pointing at 150 mm (quiet) up to 350 mm (loud) |
| E | 80 right | 18 forward above, 23 forward pressing | 55 up above, 30 up pressing | Arm swung right; each press pushes the pusher tip 90 mm straight down, from 70 mm above the button to 20 mm in, with the main arm leaning 3 degrees extra at the bottom |

Gestures, one per run, each a different shape. YES (object at A): the sensor moves straight up and down about 40 mm, twice. NO (object at B): the base turns 15 degrees left and right, twice. BOX (nothing at A or B): the sensor traces a square, up, across, down and back. Each swing takes 900 ms with a short stop at each end, and the Serial Monitor prints a `Gesture:` line.

Beeps: Task N beeps N times when it starts and N times when it ends, then a 2 s pause. Task 1 is A and B, Task 2 is C, Task 3 is D, Task 4 is E. At start up there are three single beeps, one with each LED.

## Detection limits at A and B

The arm stops with the ultrasonic 10 cm from A and 7 cm from B, so an object reads about 10 or 7 cm. At B the colour sensor, 83 mm further out along the arm, reaches just past the point, so make sure it clears the object or stand there. The code counts an object when the reading is 2 to 25 cm, and if nothing is straight ahead it looks a little left, right, up and down before deciding. It counted 3 to 15 cm before (`US_A_MIN_CM`, `US_A_MAX_CM`, `US_B_MIN_CM`, `US_B_MAX_CM`), and only if at least 3 of the 5 pings echo back.

- 3 cm minimum: the HC-SR04 cannot measure closer than about 2 to 3 cm, and readings below that are noise.
- 15 cm maximum: the standoff plus a margin for positioning error. The sensor's cone is about 15 degrees, so at 15 cm it already covers a circle about 8 cm across. Raising the limit to 20 cm or more lets it see the stand, the plate or the table behind the point and report an object that is not there.
- At B the sensor looks downward at the point. If B is the top of a plate, the plate itself can echo at about 12 to 14 cm with nothing on it. Check this on the day.

How to set them on the day: run `test_combined` with nothing at A and B and note the two distances it prints, then again with the test objects in place. Set each point's MAX halfway between its object reading and its empty reading. The empty reading must end up at least 3 cm above MAX. If B's empty reading is close to 10 cm, lower `US_B_MAX_CM` until it is clear, or raise B's main arm trim so the sensor looks over the plate.

The arm stays still for 2 s at every point before it does anything there. Between A and B it lifts up, turns 25 degrees towards the front, then turns back and leans down to B, so the two visits are clearly separate. The arm cannot bring the sensor down level with B (150 mm) with these arm lengths, so at B it looks down at the point from 7 cm away.

Each Morse press at E pushes the 70 mm pusher straight down, from 70 mm above the button to 20 mm into it, and straight back up; the main arm and second arm move together so the tip stays vertical. Waiting 70 mm above tilts the second arm up to 55 degrees, inside its +60 limit; check that the head clears the main arm and its cables there. At the bottom of each press the main arm leans a further 3 degrees (`PRESS_MAIN_EXTRA_DEG`), so the two main arm servos push on the button as well as the second arm; this aims the tip about 7 mm further out, so set it to 0 if the pusher slides off the button cap. Presses move at 120 degrees per second, faster than the rest of the run, so dots and dashes stay apart. The timing decodes correctly with the button anywhere from 50 mm higher to 32 mm lower than expected.

## Point C

C is a flat vertical face: blue paper on the end of the stand's bar, facing back towards the arm, centred about 70 mm up. With 225 mm and 160 mm links and the pivot 130 mm up, the arm cannot get low enough to look at it level. At C the second arm always points about 75 degrees down, so with the colour sensor along the arm it looks steeply down at the face from a few mm away. Two ways to handle it:

1. Leave the sensor as it is. Check with `capture_points` (`s` at C) that it reads the blue clearly with its own LED on, and calibrate the colours in that pose.
2. Better: tilt the colour sensor up on the board, on a 60 to 70 degree wedge, and set `COLOUR_PITCH_DEG` to the angle. The arm pose at C hardly changes, but the sensor then looks at the face almost level, 15 mm away. The colour sensor is only used at C, so the tilt affects nothing else.

Morse receiver settings, worked out from the pusher's real motion. With the button anywhere from 50 mm higher to 32 mm lower than expected, it sees dots of 400 to 800 ms, dashes of 1600 to 2000 ms, gaps of 390 to 790 ms inside a letter and 1590 to 1990 ms between letters (600, 1800, 590 and 1790 ms at the expected height). Both receivers split dot from dash at 1250 ms and end a letter after 1200 ms of silence, at least 350 ms from every band. A release shorter than 100 ms counts as contact bounce and a press shorter than 100 ms is ignored. `morse_receiver` ends a word after 3 s of silence; `morse_receiver_i2c` shows the word after the third letter and uses 4 s only for shorter words. Offline test: 1,536 runs (8 words, 8 button heights, contact chatter up to 90 ms) and 840 runs with every edge shifted randomly by up to 150 ms all decoded correctly.

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

## Start-up

After reset the servos creep to the rest position one at a time (second arm, main arm, then the base, slowest) instead of jumping there. This takes about 8 seconds; leave the arm alone until the LED check beeps. If the base still jumps, raise `SOFT_BURSTS_BASE`.

Then the base sweep: the head lifts, the base turns slowly to its far left (85 degrees), then its far right, then the middle, and the arm returns to rest (about 30 s). The servo cannot report its position, so the sweep does not measure the middle; the middle is halfway between the two ends it was sent to, which is `SERVO_ZERO_BASE`. What the sweep does is show that the whole swing is clear of cables and stands, bed the base gear in its groove before the run, and make a wrong `SERVO_ZERO_BASE` obvious: if the arm does not face straight forward after the sweep, change it. Throughout the run the base is held to 18 degrees per second (24 while shaking NO or drawing BOX), slower than the other joints, so long swings never yank the gear.

## During the 5 minute changeover

1. Place the arm on the station base and plug in the servo supply.
2. Open the Serial Monitor, set it to 115200 baud and New Line (at 9600 baud the text comes out as random symbols), and press reset. During the 5 s rest hold type the word from the tutors and press Enter. It prints `New word saved: XXX.`
3. Hand the receiver breadboard to the tutor to mount at Point E.

## During the 10 minute slot

- Press reset to start each attempt. A run takes about three and a half minutes, including the 8 s soft start and the 30 s base sweep (set `START_SWEEP` to false to skip the sweep if time is short).
- Run at least twice; the tutors change conditions between runs. The best attempt counts.
- If one task keeps failing, set its `RUN_TASK_x` to false and upload so the other tasks still score.
- One person films every attempt.

## After the slot

Upload `arm_controller.ino` and `morse_receiver.ino` with the videos to the submission portal straight away.
