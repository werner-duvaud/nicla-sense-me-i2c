import json
from nicla_sense_me_i2c import NiclaSenseMe

import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)
sensor = NiclaSenseMe(i2c)

# Calib obtained with sensor.get_calib() after calibration.
with open("calib.json") as calib_file:
    calib = json.loads(calib_file.read())

# Warm start with sensor already calibrated.
sensor.send_calib(**calib)
sensor.start()

while True:
    # In degrees. [0, 360], [-180, 180], [-90, 90]
    heading, pitch, roll = sensor.orientation()
    print(f"{heading=:.2f}°\n{pitch=:.2f}°\n{roll=:.2f}°")

    # In earth g.
    acc_x, acc_y, acc_z = sensor.acceleration()
    print(f"{acc_x=:.2f}g\n{acc_y=:.2f}g\n{acc_z=:.2f}g")

    # In degrees/second.
    gyr_x, gyr_y, gyr_z = sensor.gyroscope()
    print(f"{gyr_x=:.2f}deg/s\n{gyr_y=:.2f}deg/s\n{gyr_z=:.2f}deg/s")

    # In µT.
    mag_x, mag_y, mag_z = sensor.magnetometer()
    print(f"{mag_x=:.2f}µT\n{mag_y=:.2f}µT\n{mag_z=:.2f}µT")

    qx, qy, qz, qw = sensor.quaternion()
    print(f"rotation=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})")

    # In degrees celsius.
    temp = sensor.temperature()
    print(f"{temp=:.2f}°C\n")
