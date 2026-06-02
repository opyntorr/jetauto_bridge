#!/usr/bin/env python3
"""Calibracion ANGULAR de odometria del JetAuto.

Primero levanta la base:  ros2 launch jetauto_bringup robot.launch.py
Luego:                     ros2 launch jetauto_calibration angular_calib.launch.py
En rqt_reconfigure: ajusta test_angle/speed, pon start_test=true, y afina
odom_angular_scale_correction hasta que el giro real iguale el comandado.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='jetauto_calibration', executable='calibrate_angular', output='screen'),
        Node(package='rqt_reconfigure', executable='rqt_reconfigure', name='calibrate_rqt_reconfigure'),
    ])
