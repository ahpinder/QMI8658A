# QMI8658A Arduino Library by [ShadeXR](https://shadexr.com)

Arduino Library for the QMI8658A supporting programmable range, filtering, and sample rate

Loosely based on [Ali's Qmi8658c library](https://github.com/ALICHOUCHENE/Qmi8658c)

This aims to be a clean Arduino-style library that includes data rate, low-pass filtering, and power mode setup.

# Installation -- PlatformIO

Simply add

`lib_deps = https://github.com/ahpinder/QMI8658A.git`

to your `platformio.ini`

and add

`#include "QMI8658.h"`

to your `main.cpp`

# Usage

Warning: this is still a WIP, it does not work correctly right now.

Create a `QMI8658A` object at the beginning of your file:

`QMI8658A qmi;`

In `setup()`, initialize the chip:

`qmi.begin(addr, i2cspeed);`

Note: generally one of 2 addresses are used: 0x6A (if SA0 pin is high) or 0x6B (if SA0 pin is low). The chips supports up to 400kHz I2C speeds.

After beginning, we can change parameters, e.g.

`qmi.setAccScale(acc_scale_2g)`

Once we are satisfied with our parameters, we can read the sensor values, either one at a time:

`float accx = qmi.readAccX()`

or all at once in raw 16-bit int:

```
int16_t sensorvalues[6];
qmi.readValuesRaw(sensorvalues);
```
