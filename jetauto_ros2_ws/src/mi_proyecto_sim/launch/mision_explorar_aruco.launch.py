"""
mision_explorar_aruco.launch.py — Mision combinada del carro real:
explora/mapea buscando el ArUco 5 y, al encontrarlo, se acerca a 40 cm y guarda el mapa.

Nodos: filtro_lidar + slam_toolbox(mapeo) + nav_goal_bridge + planificador_rrt(A*/Theta*)
+ control_diferencial + mision_explorar_aruco (frontier + camara ArUco + aproximacion).

Precondiciones: contenedor del Nano (lidar + camara /cam_1) + jetauto-orin.service (RSP+EKF).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    mapper_params = os.path.join(pkg_sim, 'config', 'mapper_params_online_async.yaml')

    filtro_lidar_node = Node(
        package='mi_proyecto_sim', executable='filtro_lidar.py',
        name='filtro_lidar', output='screen',
        parameters=[{'use_sim_time': False, 'max_range': 5.0}])

    slam_toolbox_node = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[mapper_params, {'use_sim_time': False}])

    nav_goal_bridge = Node(
        package='mi_proyecto_sim', executable='nav_goal_bridge.py',
        name='nav_goal_bridge', output='screen',
        parameters=[{'use_sim_time': False}])

    planificador_rrt_node = Node(
        package='mi_proyecto_sim', executable='planificador_rrt',
        name='planificador_rrt', output='screen',
        parameters=[{'use_sim_time': False, 'robot_radius_m': 0.25,
                     'clearance_weight': 10.0, 'clearance_ref_m': 0.40,
                     'waypoint_spacing_m': 0.15}])

    control_diferencial_node = Node(
        package='mi_proyecto_sim', executable='control_diferencial.py',
        name='control_diferencial', output='screen',
        parameters=[{'use_sim_time': False}])

    mision_node = Node(
        package='mi_proyecto_sim', executable='mision_explorar_aruco.py',
        name='mision_explorar_aruco', output='screen',
        parameters=[{
            'use_sim_time': False,
            'target_ids': [5],   # SOLO el ArUco 5 (caras laterales del cubo)
            'stop_distance': 0.40,
            'marker_size': 0.12,
            'auto_save': True,
            # exploracion
            'reach_dist': 0.65, 'min_goal_dist': 0.55,
            'min_frontier_cells': 5,    # mas chico -> persigue rincones (antes 12, se rendia)
            'done_retries': 6,          # mas paciencia antes de concluir "mapeo completo"
            'goal_timeout': 35.0,
            'initial_spin': True, 'spin_on_arrival': True,
            'spin_seconds': 13.0, 'w_spin': 0.5,
            'explore_timeout': 90.0,   # safeguard: corta el mapeo a los 90 s
        }])

    delayed = TimerAction(period=8.0, actions=[
        nav_goal_bridge,
        planificador_rrt_node,
        control_diferencial_node,
        mision_node,
    ])

    return LaunchDescription([
        filtro_lidar_node,
        slam_toolbox_node,
        delayed,
    ])
