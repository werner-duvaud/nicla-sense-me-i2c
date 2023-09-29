# Nicla Sense ME - I2C standalone

Use the [Nicla Sense ME](https://store.arduino.cc/products/nicla-sense-me) sensor as an I2C request-reply slave sensor, similar to the [BNO055](https://www.adafruit.com/product/2472).

The [BHI260AP](https://www.bosch-sensortec.com/products/smart-sensors/bhi260ap/) combined with the [BMM150](https://www.bosch-sensortec.com/products/motion-sensors/magnetometers-bmm150/) offers a modern absolute orientation IMU sensor with onchip sensor fusion. Making the Nicla Sense ME board a good successor to the BNO055.

This repository holds the [arduino code](https://github.com/werner-duvaud/nicla-sense-me-i2c/blob/main/arduino/nicla_sens_me_i2c/nicla_sens_me_i2c.ino) to run on the Nicla Sense ME microprocesseur and a [small Python library](https://github.com/werner-duvaud/nicla-sense-me-i2c/blob/main/nicla_sense_me_i2c.py) using the [busio library](https://github.com/adafruit/Adafruit_Blinka/blob/main/src/busio.py) to talk to the sensor over I2C.

Features:

- [x] Warm start with your custom calibration config.
- [x] Absolute orientation (heading to north, pitch, roll).
- [x] Raw acceleration, gyroscope, magnetometer.
- [x] Rotation quaternion.
- [ ] Linear acceleration.
- [x] Temperature.
- [ ] Pressure.
- [ ] Gaz.
- [ ] Activity.
- [ ] BSEC.
- [x] 200Hz sensor data over i2c.
- [x] Simple code easily hackable.

## Python API

Runs on Nvidia Jetson, Raspberry Pi and MicroPython boards.

```python
import json
from nicla_sense_me_i2c import NiclaSenseMe

import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)
sensor = NiclaSenseMe(i2c)

# Calib obtained with sensor.get_calib() after calibration.
with open('calib.json') as calib_file:
    calib = json.loads(calib_file.read())

# Warm start with sensor already calibrated.
sensor.send_calib(**calib)
sensor.start()

while True:
    heading, pitch, roll = sensor.orientation()  # In degrees. [0, 360], [-180, 180], [-90, 90]
    acc_x, acc_y, acc_z = sensor.acceleration()  # In earth g.
    gyr_x, gyr_y, gyr_z = sensor.gyroscope()  # In degrees/second.
    mag_x, mag_y, mag_z = sensor.magnetometer()  # In µT.
    qx, qy, qz, qw = sensor.quaternion()
    temp = sensor.temperature()  # In degrees celsius.

    print(f"{heading=:.2f}, {pitch=:.2f}, {roll=:.2f}")
```

See also [example.py](https://github.com/werner-duvaud/nicla-sense-me-i2c/tree/main/example.py).

## Setup

### Setup the Nicla Sense ME

First we need to compile and upload the software running on the sensor microcontroller.

1. Open `nicla_sense_me.ino` with the [Arduino IDE](https://www.arduino.cc/en/software).
2. Select the "Arduino Nicla Sense ME" board.
3. Install the "Arduino_BHY2" and "ArduinoBLE" libraries from the Library Manager tab.
4. Edit the library file "BoschSensortec.h" located at "Documents/Arduino/libraries/Arduino_BHY2/src/BoschSensortec.h" and move the line `struct bhy2_dev _bhy2;` after the line `public:`.
5. Connect the Micro USB port of the device to the computer.
6. Click on the upload button in Arduino editor.

### Host setup

Install the python library on the host to communicate with the sensor:

```bash
git clone https://github.com/werner-duvaud/nicla-sense-me-i2c.git
cd nicla-sense-me-i2c
pip install .
```

## I2C hardware setup

Wire 4 pin headers: Vin (5v), Ground, SCL, SDA as follows:

![nicla_sense_me_i2c](https://github.com/werner-duvaud/nicla-sense-me-i2c/assets/40442230/1c33faf8-3e62-4110-8d22-8c42acc6f91f)

## Note

- Using the onboard LED is not recommended as it is using I2C too.

## Reference and useful links

- [BHI260AP Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bhi260ap-ds000.pdf)
- [BMM150 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm150-ds001.pdf)
- [Arduino Nicla Sense ME Cheat Sheet](https://docs.arduino.cc/tutorials/nicla-sense-me/cheat-sheet)
