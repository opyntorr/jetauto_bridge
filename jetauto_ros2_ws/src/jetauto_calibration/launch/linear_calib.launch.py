#!/usr/bin/env python3
"""Calibracion LINEAL de odometria del JetAuto.

Primero levanta la base:  ros2 launch jetauto_bringup robot.launch.py
Luego:                     ros2 launch jetauto_calibration linear_calib.launch.py
En rqt_reconfigure: ajusta test_distance/speed, pon start_test=true, y afina
odom_linear_scale_correction hasta que el carrito recorra la distancia real.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='jetauto_calibration', executable='calibrate_linear', output='screen'),
        Node(package='rqt_reconfigure', executable='rqt_reconfigure', name='calibrate_rqt_reconfigure'),
    ])
