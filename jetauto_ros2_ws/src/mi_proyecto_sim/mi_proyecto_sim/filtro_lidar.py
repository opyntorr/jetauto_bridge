#!/usr/bin/env python3
import array
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class FiltroLidar(Node):
    def __init__(self):
        super().__init__('filtro_lidar')
        self.declare_parameter('self_clearance', 0.05)
        self.self_clearance = self.get_parameter('self_clearance').value
        # Alcance maximo util: lecturas mas lejanas se descartan (inf). Evita que el
        # lidar vea "afuera de los limites" (a traves de la jaula/reflejos) y que SLAM
        # marque mapa fuera del area real. 0 = sin limite.
        self.declare_parameter('max_range', 0.0)
        self.max_range = float(self.get_parameter('max_range').value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.get_logger().info(
            f'filtro_lidar listo: relay 360 con self_clearance={self.self_clearance} m')

    def scan_cb(self, msg):
        out = LaserScan()
        out.header          = msg.header
        out.angle_min       = msg.angle_min
        out.angle_increment = msg.angle_increment
        # FIX off-by-one: el driver del MS200 entrega un nº de puntos que NO cuadra
        # con (angle_max-angle_min)/increment+1, y slam_toolbox RECHAZA esos scans
        # ("contains N range readings, expected M") -> no construye /map. Recalculamos
        # angle_max desde el conteo real para que siempre sea consistente.
        out.angle_max       = msg.angle_min + (len(msg.ranges) - 1) * msg.angle_increment
        out.time_increment  = msg.time_increment
        out.scan_time       = msg.scan_time
        out.range_min       = msg.range_min
        out.range_max       = self.max_range if self.max_range > 0.0 else msg.range_max

        sc = self.self_clearance
        mx = self.max_range if self.max_range > 0.0 else float('inf')
        ranges = [float('inf') if (r < sc or r > mx) else float(r) for r in msg.ranges]
        out.ranges = array.array('f', ranges)
        if msg.intensities:
            out.intensities = array.array('f', [
                0.0 if (r < sc or r > mx) else float(i)
                for r, i in zip(msg.ranges, msg.intensities)
            ])
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FiltroLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
