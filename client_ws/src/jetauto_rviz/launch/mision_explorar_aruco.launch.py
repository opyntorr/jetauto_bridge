#!/usr/bin/env python3
"""
mision_explorar_aruco.launch.py — UN comando para la MISION combinada del JetAuto real:
explora/mapea buscando el ArUco 5 y, al encontrarlo, se acerca a 40 cm y guarda el mapa.

Desde la laptop: abre RViz + arranca el cerebro de la mision en el Orin por SSH.
Se detiene/limpia al cerrar (OnShutdown).

    ros2 launch jetauto_rviz mision_explorar_aruco.launch.py
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
    default_dds = os.path.expanduser('~/cyclonedds-laptop.xml')
    orin = LaunchConfiguration('orin_host')

    # Cerebro DESPRENDIDO (setsid): el ssh devuelve enseguida y la mision sigue
    # corriendo en el Orin aunque se caiga el WiFi. orin_brain_start.sh es idempotente.
    start_brain = ExecuteProcess(
        cmd=['ssh'] + SSH_OPTS
            + [orin, ['setsid bash ', LaunchConfiguration('orin_brain_script'),
                      ' mision_explorar_aruco.launch.py > /tmp/mision.log 2>&1 < /dev/null &']],
        name='orin_brain_mision', output='screen')

    stop_brain = ExecuteProcess(
        cmd=['ssh'] + SSH_OPTS
            + [orin, 'pkill -9 -f mision_explorar_aruco; pkill -9 -f explorador_frontera; '
                     'pkill -9 -f async_slam_toolbox; pkill -9 -f slam_toolbox; '
                     'pkill -9 -f control_diferencial; pkill -9 -f planificador_rrt; '
                     'pkill -9 -f nav_goal_bridge; pkill -9 -f filtro_lidar; true'],
        name='orin_brain_stop', output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('orin_host', default_value='jetson@10.42.1.1'),
        DeclareLaunchArgument('orin_brain_script', default_value='/home/jetson/orin_brain_start.sh'),
        DeclareLaunchArgument('cyclonedds_uri', default_value=default_dds),
        DeclareLaunchArgument('ros_domain_id', default_value='0'),

        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', ['file://', LaunchConfiguration('cyclonedds_uri')]),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg], output='screen',
             condition=IfCondition(LaunchConfiguration('rviz'))),

        start_brain,
        RegisterEventHandler(OnShutdown(on_shutdown=[stop_brain])),
    ])
