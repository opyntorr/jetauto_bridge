#!/usr/bin/env python3
"""
Relay de /cmd_vel: laptop (hotspot) -> Nano (ethernet via Orin).
Anti-loop: ignora mensajes con contenido identico al ultimo publicado.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self._last_key = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self._cb, 10)
        self.get_logger().info('cmd_vel_relay listo')

    def _cb(self, msg):
        key = (round(msg.linear.x, 4), round(msg.linear.y, 4), round(msg.linear.z, 4),
               round(msg.angular.x, 4), round(msg.angular.y, 4), round(msg.angular.z, 4))
        if key == self._last_key:
            return
        self._last_key = key
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
