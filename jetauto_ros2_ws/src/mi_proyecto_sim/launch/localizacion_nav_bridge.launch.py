"""
localizacion_nav_bridge.launch.py — Navegacion JetAuto real con AMCL (mapa fijo).

Reemplaza slam_toolbox por AMCL para localizacion estable sobre el mapa previo.
AMCL no modifica el mapa -> sin loop closures -> sin teleporting.

Flujo:
  1. map_server         -> /map       (frame map)             para AMCL
  2. map_server_planner -> /map_dron  (frame map_dron_origin) para el planner RRT
  3. amcl               -> TF map->odom (localizacion por particulas)
  4. En RViz: pulsa '2D Pose Estimate' para dar la pose inicial al AMCL.
  5. nav_goal_bridge    -> /goal_pose -> meta_aruco + /alignment_ready
  6. planificador_rrt   -> ruta en /map_dron
  7. control_diferencial -> /cmd_vel

Argumentos:
  map : path absoluto al .yaml del mapa (default: ~/maps/mapa_real.yaml)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim  = get_package_share_directory('mi_proyecto_sim')
    pkg_nav  = get_package_share_directory('jetauto_navigation')
    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    twist_mux_cfg = os.path.join(pkg_sim, 'config', 'twist_mux.yaml')
    default_map  = os.path.expanduser('~/maps/mapa_real.yaml')

    map_yaml = LaunchConfiguration('map')

    # /scan (BEST_EFFORT) -> /scan_filtered (evasion + RRT)
    filtro_lidar_node = Node(
        package='mi_proyecto_sim', executable='filtro_lidar.py',
        name='filtro_lidar', output='screen',
        parameters=[{'use_sim_time': False}])

    # Mapa para AMCL -> /map (frame map)
    map_server_amcl = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False, 'frame_id': 'map'}])

    # Mapa para el planner RRT -> /map_dron (frame map_dron_origin)
    map_server_planner = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server_planner', output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False, 'frame_id': 'map_dron_origin'}],
        remappings=[('/map', '/map_dron')])

    # AMCL: localiza sobre /map, publica TF map->odom
    amcl_node = Node(
        package='nav2_amcl', executable='amcl',
        name='amcl', output='screen',
        parameters=[nav2_params])

    # lifecycle_manager gestiona map_server + amcl
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'map_server_planner', 'amcl'],
        }])

    # Control + planeacion (con margen para que lifecycle active los servidores)
    nav_goal_bridge = Node(
        package='mi_proyecto_sim', executable='nav_goal_bridge.py',
        name='nav_goal_bridge', output='screen',
        parameters=[{'use_sim_time': False}])

    planificador_rrt_node = Node(
        package='mi_proyecto_sim', executable='planificador_rrt',
        name='planificador_rrt', output='screen',
        parameters=[{'use_sim_time': False, 'robot_radius_m': 0.25,
                     'clearance_weight': 10.0,   # mas penalizacion por cercania a paredes (antes 5.0)
                     'clearance_ref_m': 0.40}])  # penaliza hasta mas lejos (antes 0.35)

    # control_diferencial ahora publica a /cmd_vel_nav (entrada del mux), NO directo a /cmd_vel.
    control_diferencial_node = Node(
        package='mi_proyecto_sim', executable='control_diferencial.py',
        name='control_diferencial', output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[('/cmd_vel', '/cmd_vel_nav')])

    # twist_mux: prioriza teleop > nav y aplica el e-stop (lock /e_stop). Su salida
    # cmd_vel_out -> /cmd_vel (lo que lee el chasis; el chasis NO cambia).
    twist_mux_node = Node(
        package='twist_mux', executable='twist_mux',
        name='twist_mux', output='screen',
        parameters=[twist_mux_cfg, {'use_sim_time': False}],
        remappings=[('cmd_vel_out', '/cmd_vel')])

    delayed_nav_stack = TimerAction(period=6.0, actions=[
        lifecycle_manager,
        nav_goal_bridge,
        planificador_rrt_node,
        twist_mux_node,
        control_diferencial_node,
    ])

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Path al YAML del mapa (ruta en el Orin)'),
        filtro_lidar_node,
        map_server_amcl,
        map_server_planner,
        amcl_node,
        delayed_nav_stack,
    ])
