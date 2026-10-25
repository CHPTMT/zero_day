Bundled libraries, so the sketch compiles without installing anything:
- Adafruit_TCS34725 1.4.4 (github.com/adafruit/Adafruit_TCS34725, BSD licence)
- Adafruit_I2CDevice from Adafruit_BusIO 1.17.4 (github.com/adafruit/Adafruit_BusIO, MIT licence)
The only change: Adafruit_TCS34725.h includes "Adafruit_I2CDevice.h" from this folder.
