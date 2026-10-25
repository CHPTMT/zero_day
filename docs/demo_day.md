# Demo Day: Monday 26 October, NWE Lab

Group 21 is in Session 11, 12:00 to 12:10, Testing Station 2. Changeover is 11:55 to 12:00. Arrive by 11:45 with PPE.

Report any damaged servo or sensor to the tutors before 09:30, with photo or video, by email. After 09:30 no hardware failure claims are accepted.

## Order of testing (Sunday, on the same power supply as Monday)

1. `test_servos`. Watch the 5 s zero hold: base forward, main arm upright, second arm level. Fix any joint that is off with its `SERVO_ZERO_xxx`. In the direction check the base must turn left, the main arm lean forward and the second arm tilt up; flip the `xxx_DIR` of any that do not. Copy the values into `arm_controller.ino` and `test_combined.ino`.
2. `test_sensors`. Distance, colour and sound print together. Calibrate the colours with r, g, b on your own paper and the tap levels with t. Paste the printed values into `arm_controller.ino`.
3. Pivot height is set to 130 mm above the table (20 mm platform plus 110 mm). If the arm is rebuilt, re-measure and update `SHOULDER_HEIGHT_MM` in both `arm_controller.ino` and `test_combined.ino`.
4. `test_combined`. The arm visits every station and prints where it thinks the sensor is. If it is off at a station, adjust that station's trim values.
5. `arm_controller`. Full runs with conditions changed: object at A only, B only, neither; each colour; soft and loud tap; different words.

## What each point should look like

Run `test_servos` first and do the direction check: base turns left (towards A and B), main arm leans forward, second arm tilts up. If any joint goes the other way, flip its `xxx_DIR` in `arm_controller.ino` and `test_combined.ino`. With a joint reversed, the arm goes up where it should go down (Point C is the clearest sign).

Joint angles from the rest position, and where the sensor in use ends up (height above the table):

| Point | Base | Main arm | Second arm | What you should see |
|---|---|---|---|---|
| A | 80 left | 24 forward | 20 up | Arm high, ultrasonic level with the top plate, 10 cm from it, at 327 mm |
| B | 80 left | 39 forward | 4 up | Arm leans further forward and lower, ultrasonic pointing down at the lower plate from 214 mm |
| C | 22 left | 33 forward | 41 down | Clear dip: second arm points steeply down, colour sensor 15 mm off the paper at 84 mm |
| D | 1 right | 13 forward | 11 down | Arm nearly straight ahead, sound sensor 4 cm from the tap point at 266 mm |
| Indicator | 20 right | 19 to 35 forward | 13 down to 38 up | Ultrasonic points at 150 mm (quiet) up to 350 mm (loud) |
| E | 80 right | 21 forward | 30 up | Arm swung right, pusher 15 mm above the button, then presses down |

Gestures: one per run. YES (nod) if an object is at A, NO (shake) if one is at B, BOX if neither. Each swing is 10 degrees each way, 900 ms per swing, and the Serial Monitor prints a `Gesture:` line.

## Testing at home without the stands

Mark the base centre on the table and stand behind the arm, looking the way it faces at rest. Each point sits at:

| Point | Forward | Sideways | Height above the table |
|---|---|---|---|
| A | 61 mm | 345 mm left | 320 mm |
| B | 61 mm | 345 mm left | 150 mm |
| C | 164 mm | 115 mm left | 70 mm |
| D | 271 mm | 48 mm right | 250 mm |
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

- Press reset to start each attempt. A run takes about one minute.
- Run at least twice; the tutors change conditions between runs. The best attempt counts.
- If one task keeps failing, set its `RUN_TASK_x` to false and upload so the other tasks still score.
- One person films every attempt.

## After the slot

Upload `arm_controller.ino` and `morse_receiver.ino` with the videos to the submission portal straight away.
