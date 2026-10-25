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
| RGB LED, common anode | long leg (anode) | 5 V |
| RGB LED | red leg, through 220 ohm | D8 |
| RGB LED | green leg, through 220 ohm | D9 |
| RGB LED | blue leg, through 220 ohm | D10 |
| Status | onboard LED | D13 |

The RGB LED is common anode, so a pin set LOW lights that colour. At Point C the LED follows what the colour sensor sees for 2 s, then holds the final colour to the end of the run. If the LED shows the opposite of the paper (everything except the right colour), it is common cathode: set `LED_COMMON_ANODE` to false and move the long leg to GND. If red and blue are swapped, swap the D8 and D10 wires.

The two main arm servos face each other, so the code always drives them in opposite directions: left = zero + angle, right = zero - angle.

Servo power: four SG90s at stall can draw over 2 A. Feed the servo red wires from an external 5 V supply, join its ground to the Uno GND, and keep the sensors on the Uno 5 V. Servos on the Uno 5 V pin cause resets halfway through a run.

## Morse receiver (Uno 2, separate board, rule R2)

| Device | Pin on device | Uno pin |
|---|---|---|
| Push button | one leg / other leg | D7 / GND |
| LCD I2C backpack | SDA / SCL | A4 / A5 |
| LCD I2C backpack | VCC / GND | 5 V / GND |

Until the button and LCD are on hand, the arm controller prints every Morse dot and dash to the Serial Monitor.
