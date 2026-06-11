"""
explorar_real.launch.py — Exploracion autonoma + mapeo (SLAM) del JetAuto real.

El robot parte SIN mapa, explora solo por fronteras, va construyendo el mapa con
slam_toolbox y al terminar lo GUARDA (PGM+YAML). Luego se reusa con:
    ros2 launch jetauto_rviz navegar_real.launch.py map:=.../mapa_AAAAMMDD_HHMMSS.yaml

Nodos:
  - filtro_lidar         -> /scan_filtered (360, sin el cuerpo del robot)
  - slam_toolbox (async, mode=mapping) -> /map + TF map->odom
  - nav_goal_bridge      -> alineacion identidad map==map_dron_origin + /goal_pose->meta_aruco
  - planificador_rrt     -> A* con campo de costo, planea sobre el mapa vivo (/map_dron)
  - control_diferencial  -> /cmd_vel (3 conos + repulsion)
  - explorador_frontera  -> elige fronteras, manda /goal_pose, relaya /map->/map_dron,
                            y AUTO-GUARDA el mapa al completar.

Precondiciones (siempre arriba, como en navegar): contenedor del Nano (lidar) +
jetauto-orin.service (RSP + EKF -> TF odom->base_footprint).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    mapper_params = os.path.join(pkg_sim, 'config', 'mapper_params_online_async.yaml')

    # /scan (BEST_EFFORT) -> /scan_filtered (lo consumen SLAM y el control)
    filtro_lidar_node = Node(
        package='mi_proyecto_sim', executable='filtro_lidar.py',
        name='filtro_lidar', output='screen',
        parameters=[{'use_sim_time': False,
                     'max_range': 5.0}])  # descarta lecturas >5m -> no mapea fuera de los limites

    # SLAM en modo MAPEO: construye /map desde cero y publica TF map->odom
    slam_toolbox_node = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[mapper_params, {'use_sim_time': False}])

    # Puente de metas (alineacion identidad: sin 2D Pose Estimate, map==map_dron_origin)
    nav_goal_bridge = Node(
        package='mi_proyecto_sim', executable='nav_goal_bridge.py',
        name='nav_goal_bridge', output='screen',
        parameters=[{'use_sim_time': False}])

    # Planner A* con campo de costo (mismos params que en navegacion)
    planificador_rrt_node = Node(
        package='mi_proyecto_sim', executable='planificador_rrt',
        name='planificador_rrt', output='screen',
        parameters=[{'use_sim_time': False, 'robot_radius_m': 0.25,
                     'clearance_weight': 10.0, 'clearance_ref_m': 0.40,
                     'waypoint_spacing_m': 0.15}])  # waypoints mas largos al explorar (nav usa 0.05)

    control_diferencial_node = Node(
        package='mi_proyecto_sim', executable='control_diferencial.py',
        name='control_diferencial', output='screen',
        parameters=[{'use_sim_time': False}])

    # Cerebro de exploracion (frontier) + relay /map->/map_dron + auto-guardado
    explorador_node = Node(
        package='mi_proyecto_sim', executable='explorador_frontera.py',
        name='explorador_frontera', output='screen',
        parameters=[{
            'use_sim_time': False,
            'auto_save': True,
            'reach_dist': 0.65,        # coincide con la parada del control (<0.6)
            'min_goal_dist': 0.75,
            'goal_timeout': 35.0,
            'min_frontier_cells': 12,   # ignora "puntitos" de mapa: solo huecos de >=~60cm
            'done_retries': 3,
            # Escaneo en sitio (giro 360) para leer el entorno rapido
            'initial_spin': True,
            'spin_on_arrival': True,
            'spin_seconds': 13.0,
            'w_spin': 0.5,
        }])

    # Da margen a que SLAM publique /map y TF map->odom antes de planear/explorar
    delayed = TimerAction(period=8.0, actions=[
        nav_goal_bridge,
        planificador_rrt_node,
        control_diferencial_node,
        explorador_node,
    ])

    return LaunchDescription([
        filtro_lidar_node,
        slam_toolbox_node,
        delayed,
    ])
