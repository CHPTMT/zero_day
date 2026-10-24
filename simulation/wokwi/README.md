# Wokwi simulation

Wokwi (wokwi.com) runs the real Arduino sketch in the browser, with servos that turn and an ultrasonic sensor whose distance can be dragged.

1. Go to wokwi.com, start a new Arduino Uno project.
2. Replace `sketch.ino` with the contents of `arduino/arm_controller/arm_controller.ino` (or any test sketch).
3. Replace `diagram.json` with the one in this folder.
4. Open the Library Manager tab and add the libraries listed in `libraries.txt`.
5. Press the green start button and open the Serial Monitor.

What maps to what:

| Real part | In Wokwi |
|---|---|
| Base, main left, main right, second arm servos | four servos on D2, D3, D4, D5 |
| Ultrasonic | HC-SR04 on D6 (echo) and D7 (trig). Click it while running to set the distance: under 20 cm counts as an object |
| Sound sensor | potentiometer on A1. Turn it quickly during the 3 s listening window to make a "tap" |
| Colour LEDs | red, green and blue LEDs on D8, D9, D10 |
| Colour sensor | not available in Wokwi. The sketch reports it missing and Task 2 lights red by default |

The two main arm servos turn in opposite directions, which is how the real pair drives the shaft.
