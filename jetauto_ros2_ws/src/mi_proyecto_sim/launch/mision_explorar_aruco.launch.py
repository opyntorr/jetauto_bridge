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
            'marker_size': 0.1125,        # lado real del ArUco del cubo (medido), antes 0.12
            'auto_save': True,
            'mission_timeout': 540.0,     # tope global: explore 240 + regreso + barrido + parqueo
            # exploracion
            'reach_dist': 0.50,         # set "rincones chicos": > stop del control (0.45)
            'min_goal_dist': 0.60,      # > reach_dist (0.50): si fuera menor, elige fronteras
                                        # ya "alcanzadas" -> se queda girando sin ir a ellas
            'min_frontier_cells': 5,    # mas chico -> persigue rincones (antes 12, se rendia)
            'done_retries': 6,          # mas paciencia antes de concluir "mapeo completo"
            'goal_timeout': 35.0,
            'initial_spin': True, 'spin_on_arrival': True,
            'spin_seconds': 7.0, 'w_spin': 0.5,   # media vuelta por frontera (antes 13s = vuelta entera)
            'explore_timeout': 240.0,  # safeguard: corta el mapeo a los 240 s (mas cobertura)
            # --- Parqueo mecanum final (servo visual + lidar A1 de abajo) ---
            'park_distance': 0.40,        # m camara->marcador (0.30 era muy cerca, perdia el marcador)
            'scan_low_topic': '/scan_low',
            'low_front_deg': 20.0,        # cono frontal del A1 del parqueo (pedido)
            'low_front_offset_deg': 0.0,  # A1 corregido fisicamente: su 0 apunta al frente
            'strafe_sign': 1.0,           # invertir si strafea al lado equivocado
            # --- Barrido LENTO escalonado al regresar (evita desenfoque del ArUco) ---
            'search_step_deg': 25.0,      # grados por pasito (mas chico = mas paradas)
            'search_w': 0.30,             # rad/s LENTO del pasito (bajar si aun sale borroso)
            'search_pause_s': 0.9,        # s parado mirando entre pasos (subir si tarda en enfocar)
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
