#!/usr/bin/env python3
"""
Base driver bring-up for the JetAuto mecanum chassis (ROS2 Humble).

Starts:
  - jetauto_chassis   : cmd_vel -> I2C motors, publishes odom_raw (no TF)
  - imu_node          : MPU-6050 -> imu/data_raw
  - imu_filter        : madgwick, imu/data_raw -> imu/data (orientation)
  - ekf_filter_node   : robot_localization, fuses odom_raw + imu/data -> odom (+TF)

This is the equivalent of the ROS1 jetauto_controller.launch + odom_publish.launch.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('jetauto_controller')
    params = os.path.join(pkg, 'config', 'chassis_params.yaml')
    ekf_params = os.path.join(pkg, 'config', 'ekf.yaml')

    use_ekf = LaunchConfiguration('use_ekf')

    return LaunchDescription([
        DeclareLaunchArgument('use_ekf', default_value='true',
                              description='Fuse odom+imu with robot_localization EKF'),

        Node(package='jetauto_controller', executable='chassis_node', name='jetauto_chassis',
             output='screen', parameters=[params],
             remappings=[('odom', 'odom_raw')]),

        Node(package='jetauto_controller', executable='imu_node', name='imu_node',
             output='screen', parameters=[params]),

        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node', name='imu_filter',
             output='screen',
             parameters=[{'use_mag': False, 'publish_tf': False, 'world_frame': 'enu'}]),

        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             output='screen', parameters=[ekf_params],
             condition=IfCondition(use_ekf),
             remappings=[('odometry/filtered', 'odom')]),
    ])
