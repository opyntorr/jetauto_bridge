#!/usr/bin/env python3
# encoding: utf-8
"""Caracterizacion del LiDAR: junta ~8s de /scan (best_effort) y reporta geometria,
rango, % de lecturas validas y rate. Corre en el Orin (lee del bridge)."""
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarChar(Node):
    def __init__(self):
        super().__init__('lidar_char')
        self.scans = []
        self.t = []
        self.create_subscription(LaserScan, '/scan', self.cb, qos_profile_sensor_data)

    def cb(self, m):
        self.scans.append(m)
        self.t.append(time.time())


def main():
    rclpy.init()
    n = LidarChar()
    t0 = time.time()
    while time.time() - t0 < 8.0 and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.2)
    if not n.scans:
        print("SIN /scan (no llega por el bridge?)")
    else:
        m = n.scans[-1]
        rngs = list(m.ranges)
        valid = [r for r in rngs if math.isfinite(r) and m.range_min <= r <= m.range_max]
        print("scans recibidos:", len(n.scans))
        if len(n.t) > 1:
            print("rate ~%.1f Hz" % ((len(n.t) - 1) / (n.t[-1] - n.t[0])))
        print("span: %.0f deg  (angle_min=%.3f max=%.3f rad)"
              % (math.degrees(m.angle_max - m.angle_min), m.angle_min, m.angle_max))
        print("angle_increment: %.5f rad  -> %d puntos" % (m.angle_increment, len(rngs)))
        print("range_min/max sensor: %.2f / %.2f m" % (m.range_min, m.range_max))
        print("lecturas: %d total, %d validas (%.0f%%)"
              % (len(rngs), len(valid), 100.0 * len(valid) / max(1, len(rngs))))
        if valid:
            print("distancias medidas: min=%.2f  max=%.2f m" % (min(valid), max(valid)))
        print("frame_id:", m.header.frame_id)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
