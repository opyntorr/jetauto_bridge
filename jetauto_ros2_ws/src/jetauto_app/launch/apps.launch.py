#!/usr/bin/env python3
"""
Start the JetAuto demo app nodes (ROS2). Each node idles until its `~/enter` service is
called, then `~/set_running`/`~/set_*` services drive it (same control model as the ROS1 app).

Prereqs running separately:
  - robot drivers + lidar:  ros2 launch jetauto_bringup robot.launch.py
  - RGB camera (for vision apps):  ros2 launch jetauto_bringup camera.launch.py

ar_app is gated behind use_ar:=true because it needs the `apriltag` python module.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_type = LaunchConfiguration('lidar_type')
    image_topic = LaunchConfiguration('image_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    use_ar = LaunchConfiguration('use_ar')

    common = [{
        'lidar_type': lidar_type,
        'image_topic': image_topic,
        'cmd_vel_topic': cmd_vel_topic,
    }]

    return LaunchDescription([
        DeclareLaunchArgument('lidar_type', default_value='A1'),
        DeclareLaunchArgument('image_topic', default_value='/camera/rgb/image_raw'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('use_ar', default_value='false'),

        Node(package='jetauto_app', executable='lidar_app', name='lidar_app',
             output='screen', parameters=common),
        Node(package='jetauto_app', executable='line_following', name='line_following',
             output='screen', parameters=common),
        Node(package='jetauto_app', executable='object_tracking', name='object_tracking',
             output='screen', parameters=common),
        Node(package='jetauto_app', executable='patrol', name='patrol',
             output='screen', parameters=common),
        Node(package='jetauto_app', executable='ar_app', name='ar_app',
             output='screen', parameters=[{'image_topic': image_topic}],
             condition=IfCondition(use_ar)),
    ])
