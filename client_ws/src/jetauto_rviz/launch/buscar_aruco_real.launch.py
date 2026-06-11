#!/usr/bin/env python3
"""
buscar_aruco_real.launch.py — UN comando para que el JetAuto real BUSQUE un ArUco
(default id 5) con su camara y se ACERQUE hasta quedar a 40 cm, viendolo desde la laptop.

Arranca, todo desde la laptop:
  1. El nodo buscar_aruco EN EL ORIN por SSH (lee /cam_1/image, publica /cmd_vel y
     una imagen anotada comprimida /buscar_aruco/debug/compressed). Se DETIENE al
     cerrar este launch (OnShutdown -> ssh pkill).
  2. republish (compressed -> raw) LOCAL para destapar la imagen de debug.
  3. rqt_image_view LOCAL mostrando /buscar_aruco/debug_view: lo que ve el carro +
     el marcador resaltado, el estado y la distancia.

⚠️ El nodo publica /cmd_vel directo -> CORRELO SOLO (sin navegacion/exploracion en
   paralelo) y con el robot en piso despejado, porque gira buscando y avanza al ver.

Correr desde una terminal del ESCRITORIO (rqt necesita DISPLAY):
    ros2 launch jetauto_rviz buscar_aruco_real.launch.py
    ros2 launch jetauto_rviz buscar_aruco_real.launch.py target_id:=7 stop_distance:=0.30

Args:
  target_id:=5            id del ArUco a buscar
  stop_distance:=0.40     metros a los que se detiene frente al marcador
  marker_size:=0.112      lado REAL del ArUco impreso (m) — define la distancia
  search_w:=0.45          velocidad de giro al buscar (rad/s)
  view:=true|false        abrir rqt_image_view (default true)
  orin_host:=jetson@10.42.1.1
  cyclonedds_uri:=~/cyclonedds-laptop.xml
  ros_domain_id:=0
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                            ExecuteProcess, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SSH_OPTS = ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', '-o', 'StrictHostKeyChecking=no']

DBG_COMPRESSED = '/buscar_aruco/debug/compressed'
DBG_VIEW = '/buscar_aruco/debug_view'


def generate_launch_description():
    default_dds = os.path.expanduser('~/cyclonedds-laptop.xml')
    orin = LaunchConfiguration('orin_host')

    # Entorno DDS del Orin para el SSH (dominio 0 + Cyclone + XML del Orin).
    remote_env = ('source /opt/ros/humble/setup.bash; '
                  'source /home/jetson/jetauto_ros2_ws/install/setup.bash; '
                  'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; '
                  'export CYCLONEDDS_URI=file:///home/jetson/cyclonedds-orin.xml; '
                  'export ROS_DOMAIN_ID=')

    # Nodo buscar_aruco en el Orin (SSH). exec -> el pkill mata el proceso real, no el bash.
    start_node = ExecuteProcess(
        cmd=['ssh', '-tt'] + SSH_OPTS
            + ['-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2', orin,
               [remote_env, LaunchConfiguration('ros_domain_id'), '; ',
                'exec ros2 run mi_proyecto_sim buscar_aruco.py --ros-args ',
                '-p target_id:=', LaunchConfiguration('target_id'), ' ',
                '-p stop_distance:=', LaunchConfiguration('stop_distance'), ' ',
                '-p marker_size:=', LaunchConfiguration('marker_size'), ' ',
                '-p search_w:=', LaunchConfiguration('search_w')]],
        name='orin_buscar_aruco', output='screen')

    # Limpieza determinista al cerrar: mata el nodo en el Orin (y suelta /cmd_vel).
    stop_node = ExecuteProcess(
        cmd=['ssh'] + SSH_OPTS
            + [orin, 'pkill -9 -f buscar_aruco; true'],
        name='orin_buscar_aruco_stop', output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('target_id', default_value='5'),
        DeclareLaunchArgument('stop_distance', default_value='0.40'),
        DeclareLaunchArgument('marker_size', default_value='0.112'),
        DeclareLaunchArgument('search_w', default_value='0.45'),
        DeclareLaunchArgument('view', default_value='true'),
        DeclareLaunchArgument('orin_host', default_value='jetson@10.42.1.1'),
        DeclareLaunchArgument('cyclonedds_uri', default_value=default_dds),
        DeclareLaunchArgument('ros_domain_id', default_value='0'),

        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', ['file://', LaunchConfiguration('cyclonedds_uri')]),

        # Destapa la imagen comprimida del Orin -> raw local para rqt.
        Node(package='image_transport', executable='republish',
             name='aruco_debug_republish', output='screen',
             arguments=['compressed', 'raw'],
             remappings=[('in/compressed', DBG_COMPRESSED), ('out', DBG_VIEW)],
             condition=IfCondition(LaunchConfiguration('view'))),

        # Visor de lo que ve el carro + deteccion (local).
        Node(package='rqt_image_view', executable='rqt_image_view',
             name='aruco_image_view', output='screen',
             arguments=[DBG_VIEW],
             condition=IfCondition(LaunchConfiguration('view'))),

        # Nodo en el Orin (SSH) + limpieza al cerrar.
        start_node,
        RegisterEventHandler(OnShutdown(on_shutdown=[stop_node])),
    ])
