#!/usr/bin/env python3
"""Guarda un frame de /cam_1/image a /tmp/cam_frame.png (prueba de la camara)."""
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class FrameSaver(Node):
    def __init__(self):
        super().__init__('frame_saver')
        self.bridge = CvBridge()
        self.done = False
        self.create_subscription(Image, '/cam_1/image', self.cb, 10)

    def cb(self, msg):
        if self.done:
            return
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imwrite('/tmp/cam_frame.png', img)
        self.get_logger().info('saved %dx%d enc=%s' % (img.shape[1], img.shape[0], msg.encoding))
        self.done = True


def main():
    rclpy.init()
    n = FrameSaver()
    t0 = time.time()
    while rclpy.ok() and not n.done and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.5)
    print('RESULT_SAVED' if n.done else 'RESULT_TIMEOUT')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
