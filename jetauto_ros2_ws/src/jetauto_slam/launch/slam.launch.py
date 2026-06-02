#!/usr/bin/env python3
"""
2D SLAM bring-up for the JetAuto (ROS2 Humble, slam_toolbox online-async).

By default it also launches the robot (drivers + lidar + EKF) so a single command maps:
  ros2 launch jetauto_slam slam.launch.py use_rviz:=true
Save the map when done:
  ros2 run nav2_map_server map_saver_cli -f ~/map
  # (or: ros2 service call /slam_toolbox/save_map ...)
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
    slam_share = get_package_share_directory('jetauto_slam')
    bringup_share = get_package_share_directory('jetauto_bringup')
    params = os.path.join(slam_share, 'config', 'mapper_params_online_async.yaml')

    start_robot = LaunchConfiguration('start_robot')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('start_robot', default_value='true',
                              description='Also launch robot.launch.py (drivers + lidar + EKF)'),
        DeclareLaunchArgument('use_rviz', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'robot.launch.py')),
            condition=IfCondition(start_robot)),

        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen', parameters=[params]),

        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(bringup_share, 'config', 'mobility.rviz')],
             condition=IfCondition(use_rviz)),
    ])
