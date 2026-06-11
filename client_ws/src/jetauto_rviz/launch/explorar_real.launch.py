#!/usr/bin/env python3
"""
explorar_real.launch.py — UN comando para EXPLORAR+MAPEAR autonomo el JetAuto real.

Igual que navegar_real, pero en vez de cargar un mapa y navegar a una meta, el robot
EXPLORA solo (fronteras), construye el mapa con SLAM y lo GUARDA al terminar.

Arranca, todo desde la laptop:
  1. RViz (view.rviz) para VER el mapa construirse en vivo.
  2. El cerebro de EXPLORACION en el ORIN por SSH (orin_brain_start.sh
     explorar_real.launch.py = filtro_lidar + slam_toolbox(mapeo) + nav_goal_bridge +
     planificador_rrt(A*-costo) + control_diferencial + explorador_frontera).
     Se DETIENE al cerrar este launch (OnShutdown).

Al completar la exploracion, el mapa queda en el ORIN:
  ~/jetauto_ros2_ws/src/mi_proyecto_sim/maps/mapa_AAAAMMDD_HHMMSS.{pgm,yaml}
y se reusa con:
  ros2 launch jetauto_rviz navegar_real.launch.py map:=<ese .yaml>

Correr desde una terminal del ESCRITORIO (RViz necesita DISPLAY):
    ros2 launch jetauto_rviz explorar_real.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                            ExecuteProcess, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SSH_OPTS = ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', '-o', 'StrictHostKeyChecking=no']


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory('jetauto_rviz'), 'rviz', 'view.rviz')
    teleop_cfg = os.path.join(get_package_share_directory('jetauto_teleop'), 'config', 'teleop.yaml')
    default_dds = os.path.expanduser('~/cyclonedds-laptop.xml')

    orin = LaunchConfiguration('orin_host')

    # Cerebro de EXPLORACION en el Orin (SSH). orin_brain_start.sh es idempotente
    # (mata el stack viejo) y hace exec ros2 launch mi_proyecto_sim "$@".
    start_brain = ExecuteProcess(
        cmd=['ssh', '-tt'] + SSH_OPTS
            + ['-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2',
               orin, ['bash ', LaunchConfiguration('orin_brain_script'),
                      ' explorar_real.launch.py']],
        name='orin_brain_explorar', output='screen')

    # Limpieza determinista al cerrar: mata todo el stack de exploracion en el Orin.
    stop_brain = ExecuteProcess(
        cmd=['ssh'] + SSH_OPTS
            + [orin, 'pkill -9 -f explorar_real; pkill -9 -f explorador_frontera; '
                     'pkill -9 -f async_slam_toolbox; pkill -9 -f slam_toolbox; '
                     'pkill -9 -f control_diferencial; pkill -9 -f planificador_rrt; '
                     'pkill -9 -f nav_goal_bridge; pkill -9 -f filtro_lidar; true'],
        name='orin_brain_stop', output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('orin_host', default_value='jetson@10.42.1.1'),
        DeclareLaunchArgument('orin_brain_script', default_value='/home/jetson/orin_brain_start.sh'),
        DeclareLaunchArgument('cyclonedds_uri', default_value=default_dds),
        DeclareLaunchArgument('ros_domain_id', default_value='0'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),

        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', ['file://', LaunchConfiguration('cyclonedds_uri')]),

        # Teleop de respaldo (opcional) — por si quieres ayudarlo manualmente
        Node(package='joy_linux', executable='joy_linux_node', name='joy_linux_node',
             parameters=[{'dev': LaunchConfiguration('joy_dev'),
                          'deadzone': 0.08, 'autorepeat_rate': 20.0}],
             output='screen',
             condition=IfCondition(LaunchConfiguration('teleop'))),
        Node(package='teleop_twist_joy', executable='teleop_node',
             name='teleop_twist_joy_node', parameters=[teleop_cfg], output='screen',
             condition=IfCondition(LaunchConfiguration('teleop'))),

        # RViz — local (para ver el mapa construirse)
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg], output='screen',
             condition=IfCondition(LaunchConfiguration('rviz'))),

        # Cerebro de exploracion en el Orin (SSH) + limpieza al cerrar
        start_brain,
        RegisterEventHandler(OnShutdown(on_shutdown=[stop_brain])),
    ])
