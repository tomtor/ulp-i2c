# ULP I2C demo application

Example of ESP-32 ULP I2C application which reads a sensor (BMP-180) and wakes up the main processors after a significant change
of the measured values.

## I2C bit banged support
Note that this example uses a bit-banged I2C implementation, because the hardware ULP I2C support cannot read 16 bit values.
This is not essential for the BMP-180, but some sensors like the ADS-1015 **DO** require 16-bit readouts.

## Example of Macro usage and ULP stack/subroutines
In additon this example shows ULP stack handling and reusable subroutines examples, like

### waitMs

Wait some milliseconds

### abs
Compute abs value of register

## Credits
Parts of the original C code from

https://github.com/adafruit/Adafruit-BMP085-Library

were translated to ESP assembly.

## Warning

Be aware of an assembler bug:

https://www.esp32.com/viewtopic.php?f=2&t=3228
 
