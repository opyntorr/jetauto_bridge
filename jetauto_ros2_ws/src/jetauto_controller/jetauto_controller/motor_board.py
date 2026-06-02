#!/usr/bin/env python3
# encoding: utf-8
"""
Low-level driver for the Hiwonder 4-channel encoder-motor board (I2C, addr 0x34).

Faithful port of the JetAuto ROS1 SDK `jetauto_sdk/encoder_motor.py` (Jetson Nano)
to a standalone, dependency-light library for ROS2 Humble on the Jetson Orin Nano.

IMPORTANT (hardware migration note):
  On the original Jetson Nano the board lived on I2C bus 1 (40-pin header pins 3/5).
  On the Jetson Orin Nano those same physical pins (3/5) are exposed as `/dev/i2c-7`,
  so the default bus here is 7. It is configurable via the `i2c_bus` argument / ROS param.
"""
import struct
import threading

import smbus2

MOTOR_BOARD_ADDR = 0x34
REG_MOTOR_TYPE = 20          # write [motor_type]
REG_MOTOR_SPEED_BASE = 50    # speed register = 50 + motor_id (motor_id in 1..4)
REG_ENCODER = 60             # 16 bytes = 4 x int32 little-endian (cumulative pulse counts)

# Physical-wheel-index -> board-motor-id remap, copied verbatim from the original
# encoder_motor.EncoderMotorController.set_speed():
#   input index 0 -> motor id 3, index 1 -> id 4, index 2 -> id 1, index 3 -> id 2
_INDEX_TO_MOTOR_ID = {0: 3, 1: 4, 2: 1, 3: 2}

# Hiwonder motor type codes (reg 20). JetAuto uses type 3 (TT/encoder motor JGB37).
MOTOR_TYPE_JGB37 = 3


class MotorBoard:
    """Owns the I2C connection to the encoder-motor board. Thread-safe."""

    def __init__(self, i2c_bus=7, motor_type=MOTOR_TYPE_JGB37, addr=MOTOR_BOARD_ADDR):
        self.addr = addr
        self._lock = threading.Lock()
        self.bus = smbus2.SMBus(i2c_bus)
        self.set_motor_type(motor_type)

    def set_motor_type(self, motor_type):
        with self._lock:
            self.bus.write_i2c_block_data(self.addr, REG_MOTOR_TYPE, [int(motor_type) & 0xFF])

    @staticmethod
    def _to_int8_byte(value):
        """Clamp to [-100, 100] and encode as an unsigned byte (two's-complement int8).
        The board firmware interprets the speed register as a signed int8."""
        v = int(value)
        if v > 100:
            v = 100
        elif v < -100:
            v = -100
        return v & 0xFF

    def set_speeds(self, speeds):
        """Set the four wheel speeds. `speeds` is an iterable of 4 values in [-100, 100],
        in the logical wheel order [w0, w1, w2, w3] used by MecanumChassis."""
        with self._lock:
            for idx, sp in enumerate(speeds):
                motor_id = _INDEX_TO_MOTOR_ID[idx]
                self.bus.write_i2c_block_data(
                    self.addr, REG_MOTOR_SPEED_BASE + motor_id, [self._to_int8_byte(sp)])

    def stop(self):
        self.set_speeds((0, 0, 0, 0))

    def read_encoders(self):
        """Return the 4 cumulative encoder counts as signed int32 (board channel order).
        NOTE: the index<->wheel correspondence for encoders is not yet validated on
        hardware; only used once encoder-based odometry is enabled (Phase 1.5)."""
        with self._lock:
            data = self.bus.read_i2c_block_data(self.addr, REG_ENCODER, 16)
        return list(struct.unpack('<iiii', bytes(data)))

    def clear_encoders(self):
        with self._lock:
            self.bus.write_i2c_block_data(self.addr, REG_ENCODER, [0] * 16)

    def close(self):
        try:
            self.stop()
        finally:
            self.bus.close()
