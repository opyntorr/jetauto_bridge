#!/usr/bin/env python3
# encoding: utf-8
"""
Lidar obstacle-avoidance / following / guarding app. ROS2 (rclpy) port of
jetauto_app/scripts/lidar.py. Camera-independent: uses only /scan and cmd_vel.

Services (under /lidar_app):
  ~/enter (Trigger)        subscribe to /scan and start
  ~/exit  (Trigger)        stop and unsubscribe
  ~/set_running (SetInt64) mode: 0 off, 1 avoid, 2 follow, 3 guard
  ~/set_parameters (SetFloat64List) [threshold, scan_angle_deg, speed]
  ~/heartbeat (SetBool)    watchdog
Params: lidar_type (A1|A2|G4, default A1), cmd_vel_topic (default cmd_vel).
"""
import math
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

from jetauto_app import pid
from jetauto_app import misc
from jetauto_app.common import Heart
from jetauto_interfaces.srv import SetInt64, SetFloat64List

MAX_SCAN_ANGLE = 240  # degrees (drop the always-occluded rear)


class LidarController(Node):
    def __init__(self):
        super().__init__('lidar_app')
        self.declare_parameter('lidar_type', 'A1')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.lidar_type = self.get_parameter('lidar_type').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.running_mode = 0
        self.threshold = 0.6
        self.scan_angle = math.radians(90)
        self.speed = 0.2
        self.last_act = 0
        self.timestamp = 0
        self.pid_yaw = pid.PID(1.6, 0, 0.16)
        self.pid_dist = pid.PID(1.7, 0, 0.16)
        self.lock = threading.RLock()
        self.lidar_sub = None

        self.mecanum_pub = self.create_publisher(Twist, cmd_vel_topic, 1)
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.create_service(SetInt64, '~/set_running', self.set_running_srv_callback)
        self.create_service(SetFloat64List, '~/set_parameters', self.set_parameters_srv_callback)
        self.heart = Heart(self, '~/heartbeat', 5, lambda _: self._stop())
        self.get_logger().info('lidar_app listo (lidar_type=%s)' % self.lidar_type)

    def reset_value(self):
        self.running_mode = 0
        self.threshold = 0.6
        self.scan_angle = math.radians(90)
        self.speed = 0.2
        self.last_act = 0
        self.timestamp = 0
        self.pid_yaw.clear()
        self.pid_dist.clear()
        if self.lidar_sub is not None:
            self.destroy_subscription(self.lidar_sub)
            self.lidar_sub = None

    def _stop(self):
        self.reset_value()
        self.mecanum_pub.publish(Twist())

    def enter_srv_callback(self, request, response):
        self.get_logger().info('lidar enter')
        self.reset_value()
        self.lidar_sub = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, qos_profile_sensor_data)
        response.success = True
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('lidar exit')
        self._stop()
        response.success = True
        return response

    def set_running_srv_callback(self, request, response):
        new_running_mode = request.data
        self.get_logger().info('set_running %d' % new_running_mode)
        if not 0 <= new_running_mode <= 3:
            response.success = False
            response.message = 'Invalid running mode {}'.format(new_running_mode)
        else:
            with self.lock:
                self.running_mode = new_running_mode
            response.success = True
        self.mecanum_pub.publish(Twist())
        return response

    def set_parameters_srv_callback(self, request, response):
        try:
            new_threshold, new_scan_angle, new_speed = request.data
        except ValueError:
            response.success = False
            response.message = 'data must be [threshold, scan_angle_deg, speed]'
            return response
        if not 0.3 <= new_threshold <= 1.5:
            response.success = False
            response.message = 'threshold {:.2f} out of range (0.3 ~ 1.5)'.format(new_threshold)
            return response
        if not 0 <= new_scan_angle <= 90:
            response.success = False
            response.message = 'scan angle {:.2f} out of range (0 ~ 90)'.format(new_scan_angle)
            return response
        if not new_speed > 0:
            response.success = False
            response.message = 'invalid speed'
            return response
        with self.lock:
            self.threshold = new_threshold
            self.scan_angle = math.radians(new_scan_angle)
            self.speed = new_speed
        response.success = True
        return response

    def lidar_callback(self, lidar_data):
        twist = Twist()
        if self.lidar_type in ['A1', 'A2']:
            max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[:max_index]
            right_ranges = lidar_data.ranges[::-1][:max_index]
        elif self.lidar_type == 'G4':
            min_index = int(math.radians((360 - MAX_SCAN_ANGLE) / 2.0) / lidar_data.angle_increment)
            max_index = int(math.radians(180) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[::-1][min_index:max_index][::-1]
            right_ranges = lidar_data.ranges[min_index:max_index][::-1]
        else:
            return

        with self.lock:
            angle = self.scan_angle / 2
            angle_index = int(angle / lidar_data.angle_increment + 0.50)
            left_range = np.array(left_ranges[:angle_index])
            right_range = np.array(right_ranges[:angle_index])

            if self.running_mode == 1 and self.timestamp <= time.time():
                left_nonzero = left_range.nonzero()
                right_nonzero = right_range.nonzero()
                if left_range[left_nonzero].size and right_range[right_nonzero].size:
                    min_dist_left = left_range[left_nonzero].min()
                    min_dist_right = right_range[right_nonzero].min()
                    if min_dist_left <= self.threshold and min_dist_right > self.threshold:
                        twist.linear.x = self.speed / 6
                        max_angle = math.radians(90)
                        w = self.speed * 6
                        twist.angular.z = -w
                        if self.last_act != 0 and self.last_act != 1:
                            twist.angular.z = w
                        self.last_act = 1
                        self.mecanum_pub.publish(twist)
                        self.timestamp = time.time() + (max_angle / w / 2)
                    elif min_dist_left <= self.threshold and min_dist_right <= self.threshold:
                        twist.linear.x = self.speed / 6
                        w = self.speed * 6
                        twist.angular.z = w
                        self.last_act = 3
                        self.mecanum_pub.publish(twist)
                        self.timestamp = time.time() + (math.radians(180) / w / 2)
                    elif min_dist_left > self.threshold and min_dist_right <= self.threshold:
                        twist.linear.x = self.speed / 6
                        max_angle = math.radians(90)
                        w = self.speed * 6
                        twist.angular.z = w
                        if self.last_act != 0 and self.last_act != 2:
                            twist.angular.z = -w
                        self.last_act = 2
                        self.mecanum_pub.publish(twist)
                        self.timestamp = time.time() + (max_angle / w / 2)
                    else:
                        self.last_act = 0
                        twist.linear.x = self.speed
                        self.mecanum_pub.publish(twist)
            elif self.running_mode == 2:
                ranges = np.append(right_range[::-1], left_range)
                nonzero = ranges.nonzero()
                if ranges[nonzero].size:
                    dist = ranges[nonzero].min()
                    min_index = list(ranges).index(dist)
                    angle = -angle + lidar_data.angle_increment * min_index
                    if dist < self.threshold and abs(math.degrees(angle)) > 5:
                        if self.lidar_type in ['A1', 'A2']:
                            self.pid_yaw.update(-angle)
                            twist.angular.z = misc.set_range(self.pid_yaw.output, -self.speed * 6, self.speed * 6)
                        else:
                            self.pid_yaw.update(angle)
                            twist.angular.z = -misc.set_range(self.pid_yaw.output, -self.speed * 6, self.speed * 6)
                    else:
                        self.pid_yaw.clear()
                    if dist < self.threshold and abs(0.2 - dist) > 0.02:
                        self.pid_dist.update(self.threshold / 2 - dist)
                        twist.linear.x = misc.set_range(self.pid_dist.output, -self.speed, self.speed)
                    else:
                        self.pid_dist.clear()
                    if abs(twist.angular.z) < 0.008:
                        twist.angular.z = 0.0
                    if abs(twist.linear.x) < 0.05:
                        twist.linear.x = 0.0
                self.mecanum_pub.publish(twist)
            elif self.running_mode == 3:
                ranges = np.append(right_range[::-1], left_range)
                nonzero = ranges.nonzero()
                if ranges[nonzero].size:
                    dist = ranges[nonzero].min()
                    min_index = list(ranges).index(dist)
                    angle = -angle + lidar_data.angle_increment * min_index
                    if dist < self.threshold and abs(math.degrees(angle)) > 5:
                        if self.lidar_type in ['A1', 'A2']:
                            self.pid_yaw.update(-angle)
                            twist.angular.z = misc.set_range(self.pid_yaw.output, -self.speed * 6, self.speed * 6)
                        else:
                            self.pid_yaw.update(angle)
                            twist.angular.z = -misc.set_range(self.pid_yaw.output, -self.speed * 6, self.speed * 6)
                    else:
                        self.pid_yaw.clear()
                    if abs(twist.angular.z) < 0.008:
                        twist.angular.z = 0.0
                    self.mecanum_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LidarController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
