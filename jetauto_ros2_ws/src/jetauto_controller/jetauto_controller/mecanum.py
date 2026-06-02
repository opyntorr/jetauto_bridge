#!/usr/bin/env python3
# encoding: utf-8
"""
Mecanum-wheel chassis kinematics. Faithful port of `jetauto_sdk/mecanum.py`.

Geometry/calibration constants come straight from the JetAuto ROS1 controller:
    a = 103 mm, b = 97 mm, wheel_diameter = 96.5 mm, pulse_per_cycle = 4320.

The original control path was:
    cmd_vel (m/s, rad/s)  ->  polar (speed mm/s, direction rad)  ->  4 wheel speeds
We keep that exact math so the robot drives identically to the ROS1 stack.
"""
import math


class MecanumChassis:
    def __init__(self, motor_board, a=103.0, b=97.0, wheel_diameter=96.5, pulse_per_cycle=4320.0):
        self.board = motor_board
        self.a = a
        self.b = b
        self.wheel_diameter = wheel_diameter
        self.pulse_per_cycle = pulse_per_cycle

    def _speed_to_pulse(self, speed_mm_s):
        """Convert mm/s into the board's unit (pulses / 10 ms)."""
        return speed_mm_s / (math.pi * self.wheel_diameter) * self.pulse_per_cycle * 0.01

    def set_velocity(self, speed, direction, angular_rate, speed_up=False):
        """Polar control (verbatim from the original SDK).
        :param speed: mm/s
        :param direction: 0..2*pi
        :param angular_rate: rad/s
        """
        vx = speed * math.sin(direction)
        vy = speed * math.cos(direction)
        vp = angular_rate * (self.a + self.b)
        v1 = vy - vx - vp
        v2 = vy + vx + vp
        v3 = vy - vx + vp
        v4 = vy + vx - vp
        v_s = [int(self._speed_to_pulse(v)) for v in [-v2, v3, v1, -v4]]
        self.board.set_speeds(v_s)

    def set_velocity_xyz(self, vx_m_s, vy_m_s, wz_rad_s, speed_up=False):
        """Cartesian convenience wrapper matching jetauto_controller_main.cmd_vel_callback:
        converts (vx, vy) m/s to the polar (speed mm/s, direction) form the SDK expects."""
        lx = vx_m_s * 1000.0   # m/s -> mm/s
        ly = vy_m_s * 1000.0
        speed = math.sqrt(lx * lx + ly * ly)
        direction = math.atan2(ly, lx)
        direction = (2.0 * math.pi + direction) if direction < 0 else direction
        self.set_velocity(speed, direction, wz_rad_s, speed_up=speed_up)

    def duties_xyz(self, vx_m_s, vy_m_s, wz_rad_s):
        """Return the 4 wheel duties (float, board units, UNclamped) for a body velocity,
        WITHOUT writing to the board. Same math/order as set_velocity, so chassis_node can
        apply its own clamp + slew-rate limit before writing. Order = [w0, w1, w2, w3]."""
        lx = vx_m_s * 1000.0   # m/s -> mm/s
        ly = vy_m_s * 1000.0
        speed = math.sqrt(lx * lx + ly * ly)
        direction = math.atan2(ly, lx)
        direction = (2.0 * math.pi + direction) if direction < 0 else direction
        vx = speed * math.sin(direction)
        vy = speed * math.cos(direction)
        vp = wz_rad_s * (self.a + self.b)
        v1 = vy - vx - vp
        v2 = vy + vx + vp
        v3 = vy - vx + vp
        v4 = vy + vx - vp
        return [self._speed_to_pulse(v) for v in (-v2, v3, v1, -v4)]

    def reset(self):
        self.board.stop()
