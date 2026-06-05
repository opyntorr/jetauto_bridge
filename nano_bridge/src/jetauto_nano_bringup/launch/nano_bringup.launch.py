#!/usr/bin/env python3
"""
Bringup MÍNIMO de hardware del JetAuto en la Jetson Nano (dentro del Docker Humble).
Expone por DDS al Orin: /odom_raw, /imu/data_raw, /scan, /cam_1/image. Escucha /cmd_vel.
NO corre EKF/madgwick/RSP/SLAM/Nav2 — eso va en el Orin.
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
    pkg = get_package_share_directory('jetauto_nano_bringup')
    params = os.path.join(pkg, 'config', 'nano_params.yaml')
    camera_launch = os.path.join(pkg, 'launch', 'camera_cam1.launch.py')

    serial_port = LaunchConfiguration('serial_port')
    use_camera = LaunchConfiguration('use_camera')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/lidar',
                              description='LIDAR RPLIDAR A1 (symlink udev /dev/lidar -> ttyUSB del puerto 1-2.3)'),
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='Encender la camara Astra Pro Plus via OrbbecSDK (libuvc)'),

        # Base: cmd_vel -> motores I2C (bus 1), publica /odom_raw (sin TF)
        Node(package='jetauto_controller', executable='chassis_node', name='jetauto_chassis',
             output='screen', parameters=[params], remappings=[('odom', 'odom_raw')]),

        # IMU MPU-6050 -> /imu/data_raw
        Node(package='jetauto_controller', executable='imu_node', name='imu_node',
             output='screen', parameters=[params]),

        # Voltaje de bateria (placa 0x34 reg 0) -> /battery_voltage + /battery_state
        Node(package='jetauto_controller', executable='battery_node', name='battery_node',
             output='screen', parameters=[params]),

        # LIDAR MS200 (Oradar, ToF, 360deg, min 0.03 m) -> /scan (frame lidar_frame).
        # Reemplaza al RPLIDAR A1. Conexion serie CH343 en /dev/ttyACM0 @ 230400.
        # (El A1 sigue conectado en /dev/lidar pero su driver NO se lanza; backup .a1.bak.)
        Node(package='oradar_lidar', executable='oradar_scan', name='MS200', output='screen',
             parameters=[{
                 'device_model': 'MS200',
                 'frame_id': 'lidar_frame',
                 'scan_topic': 'scan',
                 'port_name': '/dev/ttyACM0',
                 'baudrate': 230400,
                 'angle_min': 0.0,
                 'angle_max': 360.0,
                 'range_min': 0.03,
                 'range_max': 12.0,
                 'clockwise': False,
                 'motor_speed': 15,
             }]),

        # Camara RGB Astra Pro Plus via OrbbecSDK/libuvc -> /cam_1/image
        # (el kernel uvcvideo NO puede bindear esta camara; va por libusb. uvcvideo
        #  esta blacklisteado en el host y el driver Orbbec se compilo en /nano_ws.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            condition=IfCondition(use_camera)),
    ])
