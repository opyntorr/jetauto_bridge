#!/usr/bin/env python3
"""
Autonomous navigation (Nav2) for the JetAuto on ROS2 Humble.

Make a map first with `ros2 launch jetauto_slam slam.launch.py` and save it:
    ros2 run nav2_map_server map_saver_cli -f ~/jetauto_map
Then navigate:
    ros2 launch jetauto_navigation navigation.launch.py map:=$HOME/jetauto_map.yaml use_rviz:=true
Send goals from RViz ("Nav2 Goal").
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
    nav_share = get_package_share_directory('jetauto_navigation')
    bringup_share = get_package_share_directory('jetauto_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(nav_share, 'config', 'nav2_params.yaml')

    map_yaml = LaunchConfiguration('map')
    start_robot = LaunchConfiguration('start_robot')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(nav_share, 'maps', 'map.yaml'),
                              description='Path to map .yaml created with SLAM'),
        DeclareLaunchArgument('start_robot', default_value='true',
                              description='Also launch robot.launch.py (drivers + lidar + EKF)'),
        DeclareLaunchArgument('use_rviz', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'robot.launch.py')),
            condition=IfCondition(start_robot)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'params_file': params_file,
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items()),

        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(bringup_share, 'config', 'mobility.rviz')],
             condition=IfCondition(use_rviz)),
    ])
