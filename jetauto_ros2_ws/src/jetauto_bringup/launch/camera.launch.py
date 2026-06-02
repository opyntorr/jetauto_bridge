#!/usr/bin/env python3
"""
RGB camera bring-up for the JetAuto.

The Orbbec Astra Pro's RGB sensor is a standard UVC device, so we use v4l2_camera and
publish on /camera/rgb/image_raw — the topic the vision apps (jetauto_app) subscribe to.

NOTE: this provides RGB only. Full depth + registered point cloud needs the Orbbec ROS2
SDK driver (OrbbecSDK_ROS2), which must be built from source and validated with the camera
attached — that is a separate, later step.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device = LaunchConfiguration('video_device')
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0',
                              description='UVC RGB device of the Astra Pro'),
        Node(
            package='v4l2_camera', executable='v4l2_camera_node',
            name='v4l2_camera', namespace='camera', output='screen',
            parameters=[{
                'video_device': video_device,
                'image_size': [640, 480],
                'camera_frame_id': 'camera_link',
            }],
            remappings=[
                ('image_raw', 'rgb/image_raw'),
                ('camera_info', 'rgb/camera_info'),
            ],
        ),
    ])
