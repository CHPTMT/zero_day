# Serial Protocol

Host and controller talk over USB serial at 115200 baud, 8N1. Every command is one ASCII line ending in `\n`. Every command gets exactly one reply line. Angles are integer centidegrees (1234 means 12.34 degrees), which keeps float parsing off the microcontroller.

## Commands

| Command | Arguments | Effect |
|---|---|---|
| `ENABLE` | none | Energise the drivers. Refused while the e-stop is active. |
| `DISABLE` | none | Hold the current setpoint and de-energise the drivers. |
| `HOME` | none | Move all joints to zero. Currently assumes power-on at the home pose. |
| `MOVE` | one value per joint | Set joint targets. The whole command is rejected if any value is outside its limit. |
| `GRIP` | one value | Gripper angle, clamped to the gripper range. |
| `STOP` | none | Freeze every joint at its current setpoint. |
| `STATUS` | none | Report current setpoints. |

## Replies

| Reply | Meaning |
|---|---|
| `OK` | Command accepted. |
| `ERR <message>` | Command rejected. The message says why. |
| `POS <cdeg> ...` | Current setpoint for each joint. |

The controller also sends `ERR estop` unprompted when the e-stop opens. After the e-stop is released the arm stays disabled until the host sends `ENABLE`.

## Example

```
> ENABLE
< OK
> MOVE 0 4500 -3000 0 0
< OK
> STATUS
< POS 0 1230 -820 0 0
```
