#!/usr/bin/env python3
# encoding: utf-8
"""
MPU-6050 IMU over I2C (addr 0x68). Faithful port of `jetauto_sdk/imu.py`.

On the Nano the IMU shared I2C bus 1; on the Orin Nano the 40-pin header pins 3/5
are `/dev/i2c-7` (configurable). Returns SI units:
    accel in m/s^2, gyro in rad/s.
"""
import math
import time

import smbus2

IMU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B          # AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L
GYRO_XOUT_H = 0x43           # GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L

ACCEL_SCALE = 16384.0        # LSB/g for +/-2g (default)
GYRO_SCALE = 131.0           # LSB/(deg/s) for +/-250 dps (default)
G = 9.80665                  # m/s^2


class MPU6050:
    def __init__(self, i2c_bus=7, addr=IMU_ADDR):
        self.addr = addr
        self.bus = smbus2.SMBus(i2c_bus)
        # wake up the device (clear sleep bit)
        for _ in range(100):
            try:
                self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0)
                break
            except OSError:
                time.sleep(0.01)

    def _read_word_2c(self, reg):
        high = self.bus.read_byte_data(self.addr, reg)
        low = self.bus.read_byte_data(self.addr, reg + 1)
        val = (high << 8) + low
        return val - 65536 if val >= 0x8000 else val

    def read(self):
        """Return (ax, ay, az, gx, gy, gz): accel m/s^2, gyro rad/s."""
        ax = self._read_word_2c(ACCEL_XOUT_H) / ACCEL_SCALE
        ay = self._read_word_2c(ACCEL_XOUT_H + 2) / ACCEL_SCALE
        az = self._read_word_2c(ACCEL_XOUT_H + 4) / ACCEL_SCALE
        gx = self._read_word_2c(GYRO_XOUT_H) / GYRO_SCALE
        gy = self._read_word_2c(GYRO_XOUT_H + 2) / GYRO_SCALE
        gz = self._read_word_2c(GYRO_XOUT_H + 4) / GYRO_SCALE
        return (ax * G, ay * G, az * G,
                math.radians(gx), math.radians(gy), math.radians(gz))

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass
