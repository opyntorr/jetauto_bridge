#!/usr/bin/env python3
# encoding: utf-8
"""
Color object tracking app. ROS2 (rclpy) port of
jetauto_app/scripts/object_tracking.py.

Picks a target color from a clicked image point, then drives the mecanum base
(via cmd_vel) to keep the colored object centered/at-distance in the camera.

Services (under /object_tracking):
  ~/enter (Trigger)               subscribe to the camera and reset state
  ~/exit  (Trigger)               stop and unsubscribe
  ~/set_running (SetBool)         enable/disable cmd_vel output
  ~/set_target_color (SetPoint)   pick color at normalized point (or x==y==-1 to clear)
  ~/get_target_color (Trigger)    returns "r,g,b" of the locked color in message
  ~/set_threshold (SetFloat64)    color match threshold
  ~/heartbeat (SetBool)           watchdog
Params: image_topic (default /camera/rgb/image_raw), cmd_vel_topic (default cmd_vel),
        machine_type (default JetAuto), lidar_type (default A1).
"""
import math
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, Trigger

from jetauto_app import pid
from jetauto_app import misc
from jetauto_app.common import ColorPicker, Heart
from jetauto_interfaces.srv import SetPoint, SetFloat64


class ObjectTracker:
    def __init__(self, color, node):
        self.node = node
        self.pid_yaw = pid.PID(0.006, 0.0, 0.0)
        self.pid_dist = pid.PID(0.002, 0.0, 0.00)
        self.last_color_circle = None
        self.lost_target_count = 0
        self.target_lab, self.target_rgb = color
        self.weight_sum = 1.0

    def __call__(self, image, result_image, threshold):
        twist = Twist()
        image = cv2.resize(image, (320, 240))
        image = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)  # RGB转LAB空间
        image = cv2.GaussianBlur(image, (5, 5), 5)

        target_color = [self.target_lab, ]
        min_color = [int(self.target_lab[0] - 50 * threshold * 2),
                     int(self.target_lab[1] - 50 * threshold),
                     int(self.target_lab[2] - 50 * threshold)]
        max_color = [int(self.target_lab[0] + 50 * threshold * 2),
                     int(self.target_lab[1] + 50 * threshold),
                     int(self.target_lab[2] + 50 * threshold)]
        target_color = self.target_lab, min_color, max_color
        mask = cv2.inRange(image, tuple(target_color[1]), tuple(target_color[2]))  # 二值化
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
        contour_area = map(lambda c: (c, math.fabs(cv2.contourArea(c))), contours)  # 计算各个轮廓的面积
        contour_area = list(filter(lambda c: c[1] > 40, contour_area))  # 剔除>面积过小的轮廓
        circle = None
        if len(contour_area) > 0:
            if self.last_color_circle is None:
                contour, area = max(contour_area, key=lambda c_a: c_a[1])
                circle = cv2.minEnclosingCircle(contour)
            else:
                (last_x, last_y), last_r = self.last_color_circle
                circles = map(lambda c: cv2.minEnclosingCircle(c[0]), contour_area)
                circle_dist = list(map(lambda c: (c, math.sqrt(((c[0][0] - last_x) ** 2) + ((c[0][1] - last_y) ** 2))),
                                       circles))
                circle, dist = min(circle_dist, key=lambda c: c[1])
                if dist < 100:
                    circle = circle
        if circle is not None:
            self.lost_target_count = 0
            (x, y), r = circle
            x = x / 320 * 640
            y = y / 240 * 480
            r = r / 320 * 640

            cv2.circle(result_image, (320, 340), 5, (255, 255, 0), -1)
            result_image = cv2.circle(result_image, (int(x), int(y)), int(r), (255 - self.target_rgb[0],
                                                                               255 - self.target_rgb[1],
                                                                               255 - self.target_rgb[2]), 2)
            vx = 0
            vw = 0
            if abs(y - 340) > 20:
                self.pid_dist.update(y - 340)
                twist.linear.x = misc.set_range(self.pid_dist.output, -0.35, 0.35)
            else:
                self.pid_dist.clear()
            if abs(x - 320) > 20:
                self.pid_yaw.update(x - 320)
                twist.angular.z = misc.set_range(self.pid_yaw.output, -2, 2)
            else:
                self.pid_yaw.clear()

        return result_image, twist


class OjbectTrackingNode(Node):
    def __init__(self):
        super().__init__('object_tracking')
        self.declare_parameter('image_topic', '/camera/rgb/image_raw')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('machine_type', 'JetAuto')
        self.declare_parameter('lidar_type', 'A1')
        self.image_topic = self.get_parameter('image_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.machine_type = self.get_parameter('machine_type').value
        self.lidar_type = self.get_parameter('lidar_type').value

        self.color_picker = None
        self.tracker = None
        self.is_running = False
        self.threshold = 0.2
        self.dist_threshold = 0.3
        self.lock = threading.RLock()
        self.image_sub = None
        self.bridge = CvBridge()

        self.mecanum_pub = self.create_publisher(Twist, cmd_vel_topic, 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)
        self.create_service(SetPoint, '~/set_target_color', self.set_target_color_srv_callback)
        self.create_service(Trigger, '~/get_target_color', self.get_target_color_srv_callback)
        self.create_service(SetFloat64, '~/set_threshold', self.set_threshold_srv_callback)
        self.heart = Heart(self, '~/heartbeat', 5, lambda _: self._stop())
        self.get_logger().info('object_tracking listo (machine_type=%s)' % self.machine_type)

    def _stop(self):
        with self.lock:
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None
            self.is_running = False
            self.color_picker = None
            self.tracker = None
            self.threshold = 0.2
            self.dist_threshold = 0.3
        self.mecanum_pub.publish(Twist())

    def enter_srv_callback(self, request, response):
        self.get_logger().info('object tracking enter')
        with self.lock:
            try:
                if self.image_sub is not None:
                    self.destroy_subscription(self.image_sub)
                    self.image_sub = None
            except Exception as e:
                self.get_logger().error(str(e))
            self.is_running = False
            self.threshold = 0.2
            self.tracker = None
            self.color_picker = None
            self.dist_threshold = 0.3
            # On the JetAuto base there is no arm/servo to position; just subscribe.
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self.image_callback, qos_profile_sensor_data)
            self.mecanum_pub.publish(Twist())
        response.success = True
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('object tracking exit')
        with self.lock:
            try:
                if self.image_sub is not None:
                    self.destroy_subscription(self.image_sub)
                    self.image_sub = None
            except Exception as e:
                self.get_logger().error(str(e))
            self.is_running = False
            self.color_picker = None
            self.tracker = None
            self.dist_threshold = 0.3
            self.mecanum_pub.publish(Twist())
        response.success = True
        return response

    def set_target_color_srv_callback(self, request, response):
        self.get_logger().info('set_target_color')
        with self.lock:
            x, y = request.data.x, request.data.y
            if x == -1 and y == -1:
                self.color_picker = None
                self.tracker = None
            else:
                self.tracker = None
                self.color_picker = ColorPicker(request.data, 20)
            self.mecanum_pub.publish(Twist())
        response.success = True
        return response

    def get_target_color_srv_callback(self, request, response):
        self.get_logger().info('get_target_color')
        response.success = False
        response.message = ''
        with self.lock:
            if self.tracker is not None:
                response.success = True
                rgb = self.tracker.target_rgb
                response.message = "{},{},{}".format(int(rgb[0]), int(rgb[1]), int(rgb[2]))
        return response

    def set_running_srv_callback(self, request, response):
        self.get_logger().info('set_running')
        with self.lock:
            self.is_running = request.data
            if not self.is_running:
                self.mecanum_pub.publish(Twist())
        response.success = request.data
        return response

    def set_threshold_srv_callback(self, request, response):
        self.get_logger().info('threshold')
        with self.lock:
            self.threshold = request.data
        response.success = True
        return response

    def image_callback(self, ros_image):
        rgb_image = self.bridge.imgmsg_to_cv2(ros_image, 'rgb8')  # 原始 RGB 画面
        result_image = np.copy(rgb_image)  # 显示结果用的画面
        with self.lock:
            # 颜色拾取器和识别追踪互斥, 如果拾取器存在就开始拾取
            if self.color_picker is not None:  # 拾取器存在
                target_color, result_image = self.color_picker(rgb_image, result_image)
                if target_color is not None:
                    self.color_picker = None
                    self.tracker = ObjectTracker(target_color, self)
            else:
                if self.tracker is not None:
                    try:
                        result_image, twist = self.tracker(rgb_image, result_image, self.threshold)
                        if self.is_running:
                            self.mecanum_pub.publish(twist)
                        else:
                            self.tracker.pid_dist.clear()
                            self.tracker.pid_yaw.clear()
                    except Exception as e:
                        self.get_logger().error(str(e))
        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, 'rgb8'))  # 发布图像


def main(args=None):
    rclpy.init(args=args)
    node = OjbectTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
