# Wiring

## Arm controller (Uno 1)

| Device | Pin on device | Uno pin |
|---|---|---|
| Base servo | signal | D3 |
| Main arm servo (shoulder) | signal | D5 |
| Second arm servo (elbow) | signal | D6 |
| Wrist servo, only if `USE_WRIST 1` | signal | D9 |
| HC-SR04 ultrasonic | TRIG / ECHO | D7 / D8 |
| TCS34725 colour sensor | SDA / SCL | A4 / A5 |
| TCS34725 colour sensor | LED | D4 |
| Sound sensor | AO | A0 |
| Red LED (220 ohm) | anode | D10 |
| Green LED (220 ohm) | anode | D11 |
| Blue LED (220 ohm) | anode | D12 |
| Status | onboard LED | D13 |

Servo power: all servo red wires to an external 5 V supply rated 2 A or more, servo brown wires to that supply's ground, and that ground joined to an Uno GND pin. Three or four SG90s pulling stall current through the Uno's 5 V pin cause brownout resets mid-run.

Sensors (ultrasonic, colour, sound) and LEDs run from the Uno 5 V and GND.

The sound sensor DO pin stays unconnected. The analogue output gives the amplitude Task 3 needs.

## Morse receiver (Uno 2, separate board, rule R2)

| Device | Pin on device | Uno pin |
|---|---|---|
| Push button | one leg | D7 |
| Push button | other leg | GND |
| LCD I2C backpack | SDA / SCL | A4 / A5 |
| LCD I2C backpack | VCC / GND | 5 V / GND |

The button uses the internal pull-up, so no resistor is needed. For a parallel 16x2 LCD set `LCD_I2C 0` and wire RS D12, E D11, D4 D5, D5 D4, D6 D3, D7 D2, with a 10k potentiometer on V0 for contrast.

The receiver Uno must have its own power (USB from a laptop or a power bank). It shares nothing with Uno 1.
