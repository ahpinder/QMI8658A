# QMI8658A Arduino Library by [ShadeXR](https://shadexr.com)

Arduino Library for the QMI8658A supporting programmable range, filtering, and sample rate

Loosely inspired by [Ali's Qmi8658c library](https://github.com/ALICHOUCHENE/Qmi8658c)

This aims to be a clean Arduino-style library that includes data rate, low-pass filtering, and power mode setup.

# Installation -- PlatformIO

Simply add

`lib_deps = https://github.com/ahpinder/QMI8658A.git`

to your `platformio.ini`

and add

`#include "QMI8658.h"`

to your `main.cpp`

# Basic usage

Create a `QMI8658A` object at the beginning of your file:

`QMI8658A qmi;`

In `setup()`, initialize the chip:

`qmi.begin(addr, i2cspeed);`

Note: generally one of 2 addresses are used: 0x6A (if SA0 pin is high) or 0x6B (if SA0 pin is low). The chips supports up to 400kHz I2C speeds.

After beginning, we can change parameters, e.g.

`qmi.setAccScale(acc_scale_2g);`

Once we are satisfied with our parameters, we can read the sensor values, either one at a time:

`float accx = qmi.readAccX();`

or all at once in raw 16-bit int:

```
int16_t sensorvalues[6];
qmi.getRawReadings(sensorvalues);
```

# Advanced usage / good to know

The QMI8658A has several operating modes, and this library can use two of them: normal and locked.
In normal operating mode, we can ping each sensor output register at our leisure, but the registers are not guaranteed to be synced to each other, i.e. you could get the x-axis acceleration from the last reading and the y-axis acceleration from the current reading depending on when each register is read. Or worse, you could read the least significant byte of one reading and the most significant byte of the next reading.
To fix this issue, the QMI8658A has a locking mechanism, where the sensors are read and the data is synchronized and locked in the registers until the host is done reading them. To access locking mode in this library, use `qmi.setState(sensor_locking)` after `begin()`. Continue to use the read functions as normal, but now quick successive reads within 2ms will be guaranteed to come from the same sample. (Note: the threshold for getting new data can be changed by setting `QMI8658_REFRESH_DELAY` to a new value in microseconds). This is the recommended operating mode for sensitive applications like AR/VR, drones or SLAM where high accuracy is needed.

# Speed

Internal testing shows that on an STM32duino STM32F103C8 platform using 400kHz I2C, a single `getRawReadings()` call takes about 600 microseconds.
