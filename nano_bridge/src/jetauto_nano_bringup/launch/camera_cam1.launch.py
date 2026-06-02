#!/usr/bin/env python3
"""
Camara Astra Pro Plus (Orbbec) en la Nano via OrbbecSDK (libuvc), SOLO color.
El kernel uvcvideo NO puede bindear esta camara (no crea /dev/video), por eso
usamos el driver Orbbec que abre el RGB por libusb/libuvc.

Perfil que SI soporta el device: 640x480 @ 30 fps, MJPG (640x480@10 RGB no existe).
Depth/IR deshabilitados (este proyecto solo usa RGB para ArUcos).
Remapea /camera/color/image_raw -> /cam_1/image (lo que consume el cerebro AGV).
"""
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    params = [{
        'camera_name': 'camera',
        'vendor_id': '0x2bc5',
        # color (UVC) — unico stream que necesitamos
        'enable_color': True,
        'color_width': 640,
        'color_height': 480,
        'color_fps': 30,
        'color_format': 'MJPG',
        'enable_color_auto_exposure': True,
        # depth / ir / nubes apagados (perfiles default no existen en este device)
        'enable_depth': False,
        'enable_ir': False,
        'enable_point_cloud': False,
        'enable_colored_point_cloud': False,
        'publish_tf': True,
        'tf_publish_rate': 10.0,
        'log_level': 'none',
    }]

    cam = ComposableNode(
        package='orbbec_camera',
        plugin='orbbec_camera::OBCameraNodeDriver',
        name='camera',
        namespace='',
        parameters=params,
        remappings=[
            ('/camera/color/image_raw', '/cam_1/image'),
            ('/camera/color/camera_info', '/cam_1/camera_info'),
        ],
    )

    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[cam],
        output='screen',
    )

    return LaunchDescription([
        GroupAction([PushRosNamespace('camera'), container]),
    ])
