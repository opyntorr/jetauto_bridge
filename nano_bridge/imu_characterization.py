#!/usr/bin/env python3
# encoding: utf-8
"""Caracterizacion estatica de la IMU: junta ~30s de /imu/data_raw (del Nano via bridge)
y reporta bias (media) y ruido (std) de gyro y accel. Robot en reposo."""
import time
import statistics
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuChar(Node):
    def __init__(self):
        super().__init__('imu_char')
        self.g = {'x': [], 'y': [], 'z': []}
        self.a = {'x': [], 'y': [], 'z': []}
        self.create_subscription(Imu, '/imu/data_raw', self.cb, 50)

    def cb(self, m):
        self.g['x'].append(m.angular_velocity.x)
        self.g['y'].append(m.angular_velocity.y)
        self.g['z'].append(m.angular_velocity.z)
        self.a['x'].append(m.linear_acceleration.x)
        self.a['y'].append(m.linear_acceleration.y)
        self.a['z'].append(m.linear_acceleration.z)


def main():
    rclpy.init()
    n = ImuChar()
    t0 = time.time()
    while time.time() - t0 < 30.0 and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.2)
    nn = len(n.g['z'])
    print("muestras:", nn)
    if nn < 10:
        print("POCAS MUESTRAS — la IMU no llega por el bridge?")
    else:
        def st(a):
            return statistics.mean(a), statistics.pstdev(a)
        print("--- GYRO (rad/s) bias=media (debe ~0 en reposo), std=ruido ---")
        for ax in ('x', 'y', 'z'):
            m, s = st(n.g[ax]); print("  gyro_%s: bias=% .6f  std=%.6f" % (ax, m, s))
        print("--- ACCEL (m/s^2) — una componente debe ~9.81 (gravedad) ---")
        for ax in ('x', 'y', 'z'):
            m, s = st(n.a[ax]); print("  acc_%s : mean=% .4f  std=%.4f" % (ax, m, s))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
