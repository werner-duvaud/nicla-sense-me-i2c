import struct
import time

import board
import busio

# I2C commands.
# Naming from the point of view of the sensor.
CMD_BEGIN = 0x60
CMD_SEND_CALIB = 0x5B
CMD_SEND_ORIENTATION = 0x1A
CMD_SEND_ACCELEROMETER = 0x08
CMD_SEND_LINEAR_ACCELERATION = 0x28
CMD_SEND_GYROSCOPE = 0x14
CMD_SEND_MAGNETOMETER = 0x0E
CMD_SEND_ROTATION = 0x20
CMD_SEND_TEMPERATURE = 0x50
CMD_FETCH_CALIB = 0x5A
CMD_RECEIVE_CALIB = 0x5D
CMD_APPLY_CALIB = 0x5E


class NiclaSenseMe:
    def __init__(self, i2c=None, address=0x28):
        if i2c is None:
            i2c = busio.I2C(board.SCL, board.SDA)

        self.i2c = i2c
        self.address = address

        self.check_device()

    def _writeto(self, data):
        if isinstance(data, int):
            data = bytes((data,))

        while not self.i2c.try_lock():
            time.sleep(0)
        try:
            self.i2c.writeto(self.address, data)
        finally:
            self.i2c.unlock()

        return self

    def _writeto_then_readfrom(self, data, read_format):
        if isinstance(data, int):
            data = bytes((data,))

        buffer = bytearray(struct.calcsize(read_format))

        while not self.i2c.try_lock():
            time.sleep(0)
        try:
            self.i2c.writeto_then_readfrom(self.address, data, buffer)
        finally:
            self.i2c.unlock()

        return struct.unpack_from(read_format, buffer)

    def start(self):
        self._writeto(CMD_BEGIN)
        # Wait for the sensor to be ready, otherwise first data are zeros.
        time.sleep(0.7)
        return self

    def orientation(self):
        heading, roll, pitch = self._writeto_then_readfrom(
            data=CMD_SEND_ORIENTATION, read_format="<hhh"
        )
        return (
            heading / 91,
            pitch / 91,
            roll / 91,
        )  # In degrees. [0, 360], [-180, 180], [-90, 90]

    def acceleration(self):
        acc_x, acc_y, acc_z = self._writeto_then_readfrom(
            data=CMD_SEND_ACCELEROMETER, read_format="<hhh"
        )
        acceleration_range = 8  # In earth g.
        scale = acceleration_range / 32767
        return acc_x * scale, acc_y * scale, acc_z * scale  # In earth g.

    def linear_acceleration(self):
        lin_acc_x, lin_acc_y, lin_acc_z = self._writeto_then_readfrom(
            data=CMD_SEND_LINEAR_ACCELERATION, read_format="<hhh"
        )
        acceleration_range = 8  # In earth g.
        scale = acceleration_range / 32767
        return lin_acc_x * scale, lin_acc_y * scale, lin_acc_z * scale  # In earth g.

    def gyroscope(self):
        gyr_x, gyr_y, gyr_z = self._writeto_then_readfrom(
            data=CMD_SEND_GYROSCOPE, read_format="<hhh"
        )
        gyroscope_range = 16.4
        scale = gyroscope_range / 32767
        return gyr_x * scale, gyr_y * scale, gyr_z * scale  # In degrees/second.

    def magnetometer(self):
        mag_x, mag_y, mag_z = self._writeto_then_readfrom(
            data=CMD_SEND_MAGNETOMETER, read_format="<hhh"
        )
        magnetometer_range = 16
        scale = magnetometer_range / 32767
        return mag_x * scale, mag_y * scale, mag_z * scale  # In µT.

    def quaternion(self):
        rot_x, rot_y, rot_z, rot_w = self._writeto_then_readfrom(
            data=CMD_SEND_ROTATION, read_format="<hhhh"
        )
        scale = 1 / 32767
        return rot_x * scale, rot_y * scale, rot_z * scale, rot_w * scale

    def temperature(self):
        temp = self._writeto_then_readfrom(data=CMD_SEND_TEMPERATURE, read_format="<h")[
            0
        ]
        scale = 1 / 100
        return temp * scale  # In degree celsius.

    def get_calib(self):
        # Ask to prepare the calib on the device.
        self._writeto(data=CMD_FETCH_CALIB)

        # The sensor needs some time to prepare the calib.
        time.sleep(0.5)

        # Receive the calib by chunks of 16 bytes.
        acc_calib = [
            list(
                self._writeto_then_readfrom(
                    data=CMD_SEND_CALIB, read_format=f"<{'B' * 16}"
                )
            )
            for _ in range(32)
        ]
        gyr_calib = [
            list(
                self._writeto_then_readfrom(
                    data=CMD_SEND_CALIB, read_format=f"<{'B' * 16}"
                )
            )
            for _ in range(32)
        ]
        mag_calib = [
            list(
                self._writeto_then_readfrom(
                    data=CMD_SEND_CALIB, read_format=f"<{'B' * 16}"
                )
            )
            for _ in range(32)
        ]
        acc_calib = sum(acc_calib, [])
        gyr_calib = sum(gyr_calib, [])
        mag_calib = sum(mag_calib, [])

        assert len(acc_calib) == len(gyr_calib) == len(mag_calib) == 512
        return {
            "acc_calib": acc_calib,
            "gyr_calib": gyr_calib,
            "mag_calib": mag_calib,
        }

    def send_calib(self, acc_calib, gyr_calib, mag_calib):
        # Need to be called before start() to be applied.
        # See datasheet 13.3.3 BSX Algorithm Parameters.
        assert len(acc_calib) == len(gyr_calib) == len(mag_calib) == 512
        self._writeto(data=CMD_RECEIVE_CALIB)

        for calib_data in (acc_calib, gyr_calib, mag_calib):
            for i in range(32):
                chunk = struct.pack(f"<{'B' * 16}", *calib_data[i * 16 : (i + 1) * 16])
                self._writeto(data=chunk)

        # Apply the calib.
        self._writeto(data=CMD_APPLY_CALIB)
        time.sleep(0.5)
        return self

    def check_device(self):
        while not self.i2c.try_lock():
            time.sleep(0)
        try:
            self.i2c.writeto(self.address, b"")
        except OSError:
            try:
                result = bytearray(1)
                self.i2c.readfrom_into(self.address, result)
            except OSError:
                raise ValueError(f"No I2C device at address {hex(self.address)}.")
        finally:
            self.i2c.unlock()

        return self
