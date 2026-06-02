#!/usr/bin/env python3
"""
Master mobility bring-up for the JetAuto on ROS2 Humble.

Composes:
  - jetauto_description : robot_state_publisher (URDF -> static TF tree)
  - jetauto_controller  : chassis (cmd_vel -> I2C motors, odom_raw) + IMU + madgwick + EKF (-> /odom, TF)
  - rplidar             : /scan in lidar_frame

Run on the Orin (with the JetAuto I2C wired and the LIDAR on USB):
  ros2 launch jetauto_bringup robot.launch.py
Then teleop in another terminal:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc = get_package_share_directory('jetauto_description')
    ctrl = get_package_share_directory('jetauto_controller')
    bringup = get_package_share_directory('jetauto_bringup')

    lidar_type = LaunchConfiguration('lidar_type')
    lidar_port = LaunchConfiguration('lidar_port')
    use_lidar = LaunchConfiguration('use_lidar')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_type', default_value='A1', description='A1|A2|G4'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(desc, 'launch', 'description.launch.py')),
            launch_arguments={'lidar_type': lidar_type}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ctrl, 'launch', 'chassis.launch.py'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'lidar.launch.py')),
            launch_arguments={'serial_port': lidar_port, 'frame_id': 'lidar_frame'}.items(),
            condition=IfCondition(use_lidar)),

        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(bringup, 'config', 'mobility.rviz')],
             condition=IfCondition(use_rviz)),
    ])
