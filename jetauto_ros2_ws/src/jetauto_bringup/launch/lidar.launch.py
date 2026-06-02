#!/usr/bin/env python3
"""LIDAR bring-up for the JetAuto.

Default: RPLIDAR A1/A2 via rplidar_ros (baud 115200). For a YDLIDAR G4 you need the
ydlidar_ros2_driver (build from source) — switch the node below accordingly.
Publishes /scan in the 'lidar_frame' frame (matches the URDF).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    frame_id = LaunchConfiguration('frame_id')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='LIDAR serial device (set a /dev/lidar udev symlink later)'),
        DeclareLaunchArgument('frame_id', default_value='lidar_frame'),

        Node(
            package='rplidar_ros', executable='rplidar_node', name='rplidar_node', output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': serial_port,
                'serial_baudrate': 115200,   # A1/A2
                'frame_id': frame_id,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
        ),
    ])
