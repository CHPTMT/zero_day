# Wiring

## Arm controller (Uno 1)

| Device | Pin on device | Uno pin |
|---|---|---|
| Base servo | signal | D2 |
| Main arm servo, left | signal | D3 |
| Main arm servo, right | signal | D4 |
| Second arm servo | signal | D5 |
| Ultrasonic (yellow) | ECHO | D6 |
| Ultrasonic (brown) | TRIG | D7 |
| Ultrasonic | VCC (red) / GND (grey) | 5 V / GND |
| Sound sensor | AO | A1 |
| Sound sensor | VCC / GND | 5 V / GND |
| Colour sensor TCS34725 | SDA / SCL | A4 / A5 |
| Colour sensor | VCC / GND | 5 V / GND |
| LCD monitor, 16x2 with I2C backpack | SDA / SCL | A2 / A3 |
| LCD monitor | VCC / GND | 5 V / GND |
| Red LED, through 220 ohm | long leg / short leg | D8 / GND |
| Green LED, through 220 ohm | long leg / short leg | D9 / GND |
| Blue LED, through 220 ohm | long leg / short leg | D10 / GND |
| Beeper (buzzer) | + / - | D11 / GND |
| Status | onboard LED | D13 |

Each LED lights when its pin is HIGH. At start up the arm lights red, green and blue alone for 1 s each with a beep, so a swapped or dead LED shows straight away. At Point C the LEDs follow what the colour sensor sees for 2 s, then the LED for the paper colour stays on to the end of the run.

The LCD monitor shows the task in progress, the distances read at A and B, and each task's result, then swaps between two summary screens at the end. A2 and A3 are not the Uno's hardware I2C pins, so `arm_controller` drives this LCD with its own software I2C (no library to install) and finds the backpack at address 0x27 or 0x3F by itself. If the screen lights but shows nothing, turn the contrast screw on the backpack. With no LCD connected the run carries on and the Serial Monitor says it was not found.

The beeper marks each task: Task N beeps N times at its start and N times at its end, then the arm waits 2 s. The code drives it with a 2.5 kHz tone, which suits a passive buzzer. With an active buzzer (one that beeps on its own from plain 5 V) set `BUZZER_PASSIVE` to false. The tone uses Timer2, which the servos do not use.

The two main arm servos face each other, so the code always drives them in opposite directions: left = zero + angle, right = zero - angle.

Servo power: four SG90s at stall can draw over 2 A. Feed the servo red wires from an external 5 V supply, join its ground to the Uno GND, and keep the sensors on the Uno 5 V. Servos on the Uno 5 V pin cause resets halfway through a run.

## Morse receiver (Uno 2, separate board, rule R2)

| Device | Pin on device | Uno pin |
|---|---|---|
| Push button | one leg / other leg | D7 / GND |
| LCD I2C backpack | SDA / SCL | A4 / A5 |
| LCD I2C backpack | VCC / GND | 5 V / GND |

Until the button and LCD are on hand, the arm controller prints every Morse dot and dash to the Serial Monitor.
