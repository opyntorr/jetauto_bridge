#!/usr/bin/env python3
"""robot_state_publisher for the JetAuto base URDF (ROS2)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('jetauto_description')
    xacro_file = os.path.join(pkg, 'urdf', 'jetauto.urdf.xacro')

    lidar_type = LaunchConfiguration('lidar_type')
    use_jsp = LaunchConfiguration('use_joint_state_publisher')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' lidar_type:=', lidar_type]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('lidar_type', default_value='A1', description='A1|A2|G4'),
        DeclareLaunchArgument('use_joint_state_publisher', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),

        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', condition=IfCondition(use_jsp)),

        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             condition=IfCondition(use_rviz)),
    ])
