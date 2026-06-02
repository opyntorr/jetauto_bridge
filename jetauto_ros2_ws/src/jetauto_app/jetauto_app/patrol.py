#!/usr/bin/env python3
# encoding: utf-8
"""
Intelligent patrol app. ROS2 (rclpy) port of jetauto_app/scripts/patrol.py.
Drives the mecanum base along fixed geometric paths (rectangle, triangle,
circle, parallelogram). Camera-independent and servo-free: only publishes
geometry_msgs/Twist to cmd_vel.

Services (under /patrol_app):
  ~/enter (Trigger)        reset state, ready to run
  ~/exit  (Trigger)        stop and reset
  ~/set_running (SetInt64) mode: 0 stop, 1 rectangle, 2 triangle, 3 circle,
                           4 parallelogram
  ~/heartbeat (SetBool)    watchdog
Params: cmd_vel_topic (default cmd_vel), machine_type (default JetAuto),
        lidar_type (default A1).
"""
import math
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from jetauto_app.common import Heart
from jetauto_interfaces.srv import SetInt64


class PatrolController(Node):
    def __init__(self):
        super().__init__('patrol_app')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('machine_type', 'JetAuto')
        self.declare_parameter('lidar_type', 'A1')
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.machine_type = self.get_parameter('machine_type').value
        self.lidar_type = self.get_parameter('lidar_type').value

        self.running_mode = 0
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.rectangle_h = 0.5
        self.rectangle_w = 0.5
        self.triangle_l = 0.5
        self.th = None
        self.last_mode = 0
        self.thread_running = True

        self.mecanum_pub = self.create_publisher(Twist, cmd_vel_topic, 1)  # chassis control
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)  # enter the game
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)  # exit the game
        self.create_service(SetInt64, '~/set_running', self.set_running_srv_callback)  # start the game
        self.heart = Heart(self, '~/heartbeat', 5, lambda _: self._stop())
        self.mecanum_pub.publish(Twist())
        self.get_logger().info('patrol_app listo (machine_type=%s)' % self.machine_type)

    def reset_value(self):
        '''reset parameter'''
        self.running_mode = 0
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.th = None
        self.last_mode = 0
        self.thread_running = False

    def _stop(self):
        self.reset_value()
        self.mecanum_pub.publish(Twist())

    def enter_srv_callback(self, request, response):
        self.get_logger().info('patrol enter')
        self.reset_value()
        response.success = True
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('patrol exit')
        self._stop()
        response.success = True
        return response

    def set_running_srv_callback(self, request, response):
        '''set the mode'''
        new_running_mode = request.data
        self.get_logger().info('set_running ' + str(new_running_mode))
        if not 0 <= new_running_mode <= 4:
            response.success = False
            response.message = 'Invalid running mode {}'.format(new_running_mode)
            self.mecanum_pub.publish(Twist())
            return response
        # run under child thread mode to allow pause
        if new_running_mode == 1:
            if self.th is None:
                self.th = threading.Thread(target=self.rectangle)
                self.th.start()
            else:
                if not self.th.is_alive():
                    self.th = threading.Thread(target=self.rectangle)
                    self.th.start()
                elif self.last_mode == new_running_mode:
                    pass
                else:
                    self.thread_running = False
                    time.sleep(0.1)
                    self.rectangle()
        elif new_running_mode == 2:
            if self.th is None:
                self.th = threading.Thread(target=self.triangle)
                self.th.start()
            else:
                if not self.th.is_alive():
                    self.th = threading.Thread(target=self.triangle)
                    self.th.start()
                elif self.last_mode == new_running_mode:
                    pass
                else:
                    self.thread_running = False
                    time.sleep(0.1)
                    self.triangle()
        elif new_running_mode == 3:
            if self.th is None:
                self.th = threading.Thread(target=self.circle)
                self.th.start()
            else:
                if not self.th.is_alive():
                    self.th = threading.Thread(target=self.circle)
                    self.th.start()
                elif self.last_mode == new_running_mode:
                    pass
                else:
                    self.thread_running = False
                    time.sleep(0.1)
                    self.circle()
        elif new_running_mode == 4:
            if self.th is None:
                self.th = threading.Thread(target=self.parallelogram)
                self.th.start()
            else:
                if not self.th.is_alive():
                    self.th = threading.Thread(target=self.parallelogram)
                    self.th.start()
                elif self.last_mode == new_running_mode:
                    pass
                else:
                    self.thread_running = False
                    time.sleep(0.1)
                    self.parallelogram()
        elif new_running_mode == 0:
            self.thread_running = False
            self.mecanum_pub.publish(Twist())
        self.running_mode = new_running_mode
        self.last_mode = self.running_mode
        response.success = True
        return response

    def rectangle(self):
        # patrol along rectangle
        status = 0
        t_start = time.time()
        self.thread_running = True
        while self.thread_running:
            current_time = time.time()
            if status == 0 and t_start < current_time:
                status = 1
                twist = Twist()
                twist.linear.x = self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_h / self.linear_speed
            elif status == 1 and t_start < current_time:
                status = 2
                twist = Twist()
                twist.linear.y = -self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_w / self.linear_speed
            elif status == 2 and t_start < current_time:
                status = 3
                twist = Twist()
                twist.linear.x = -self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_h / self.linear_speed
            elif status == 3 and t_start < current_time:
                status = 4
                twist = Twist()
                twist.linear.y = self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_w / self.linear_speed
            elif status == 4 and t_start < current_time:
                break

        self.mecanum_pub.publish(Twist())

    def parallelogram(self):
        # patrol along parallelogram
        status = 0
        t_start = time.time()
        self.thread_running = True
        while self.thread_running:
            current_time = time.time()
            if status == 0 and t_start < current_time:
                status = 1
                twist = Twist()
                twist.linear.x = self.linear_speed * math.cos(math.pi / 6)
                twist.linear.y = -self.linear_speed * math.sin(math.pi / 6)
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_w / self.linear_speed
            elif status == 1 and t_start < current_time:
                status = 2
                twist = Twist()
                twist.linear.y = -self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_h / self.linear_speed
            elif status == 2 and t_start < current_time:
                status = 3
                twist = Twist()
                twist.linear.x = -self.linear_speed * math.cos(math.pi / 6)
                twist.linear.y = self.linear_speed * math.sin(math.pi / 6)
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_w / self.linear_speed
            elif status == 3 and t_start < current_time:
                status = 4
                twist = Twist()
                twist.linear.y = self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_h / self.linear_speed
            elif status == 4 and t_start < current_time:
                break

        self.mecanum_pub.publish(Twist())

    def triangle(self):
        # patrol along triangle
        status = 0
        t_start = time.time()
        self.thread_running = True
        while self.thread_running:
            current_time = time.time()
            if status == 0 and t_start < current_time:
                status = 1
                twist = Twist()
                twist.linear.x = self.linear_speed * math.cos(math.pi / 6)
                twist.linear.y = -self.linear_speed * math.sin(math.pi / 6)
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.triangle_l / self.linear_speed
            elif status == 1 and t_start < current_time:
                status = 2
                twist = Twist()
                twist.linear.x = -self.linear_speed * math.cos(math.pi / 6)
                twist.linear.y = -self.linear_speed * math.sin(math.pi / 6)
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.triangle_l / self.linear_speed
            elif status == 2 and t_start < current_time:
                status = 3
                twist = Twist()
                twist.linear.y = self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.triangle_l / self.linear_speed
            elif status == 3 and t_start < current_time:
                break

        self.mecanum_pub.publish(Twist())

    def circle(self):
        # patrol along circle
        status = 0
        t_start = time.time()
        self.thread_running = True
        while self.thread_running:
            current_time = time.time()
            if status == 0 and t_start < current_time:
                status = 1
                twist = Twist()
                twist.linear.x = self.linear_speed
                twist.angular.z = -0.5
                self.mecanum_pub.publish(twist)
                t_start = current_time + 2 * math.pi / 0.5
            elif status == 1 and t_start < current_time:
                break

        self.mecanum_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PatrolController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
