#!/usr/bin/env python3
"""
Exploracion autonoma por FRONTERAS para el JetAuto.

Idea: una frontera es el limite entre lo LIBRE conocido y lo DESCONOCIDO. Si el
robot va hacia esas fronteras, descubre territorio nuevo. Cuando ya no quedan
fronteras alcanzables, el mapa esta completo.

Flujo (se integra con el stack existente, sin tocarlo):
  - Lee /map (slam_toolbox, va creciendo) y la pose del robot (TF map->base_footprint).
  - Detecta fronteras, las agrupa y elige la mejor (grande + cercana, no en blacklist).
  - Publica /goal_pose (frame map)  -> nav_goal_bridge -> meta_aruco -> planificador A*
    -> control_diferencial -> el robot navega solo hacia la frontera.
  - Si llega: busca la siguiente. Si no llega en goal_timeout s: blacklist y siguiente.
  - Cuando no hay fronteras durante 'done_retries' rondas: termina y (auto_save) guarda
    el mapa a PGM+YAML (formato Nav2, igual que guardar_mapa_slam.py), listo para reusar.
"""
import math
import os
import time
import numpy as np
from scipy import ndimage

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None


def _default_maps_dir():
    try:
        share = get_package_share_directory('mi_proyecto_sim')
        ws = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
        return os.path.join(ws, 'src', 'mi_proyecto_sim', 'maps')
    except Exception:
        return os.getcwd()


class ExploradorFrontera(Node):
    def __init__(self):
        super().__init__('explorador_frontera')
        gp = lambda n, v: self.declare_parameter(n, v).value

        self.map_topic   = gp('map_topic', '/map')
        self.goal_topic  = gp('goal_topic', '/goal_pose')
        self.map_frame   = gp('map_frame', 'map')
        self.robot_frame = gp('robot_frame', 'base_footprint')

        self.min_frontier_cells = gp('min_frontier_cells', 6)    # celdas minimas por frontera
        # reach_dist ~0.65 para que coincida con la parada del control (dist_to_final<0.6):
        # asi "llego" se detecta de verdad (y dispara el escaneo) en vez de esperar timeout.
        self.reach_dist     = gp('reach_dist', 0.65)             # m: meta alcanzada
        self.min_goal_dist  = gp('min_goal_dist', 0.75)          # m: ignora fronteras pegadas al robot
        self.goal_timeout   = gp('goal_timeout', 35.0)           # s: si no llega -> blacklist
        self.blacklist_radius = gp('blacklist_radius', 0.40)     # m
        self.tick_period    = gp('tick_period', 2.0)             # s entre evaluaciones
        self.done_retries   = gp('done_retries', 3)              # rondas sin fronteras -> fin
        self.dist_weight    = gp('dist_weight', 2.5)             # cuanto pesa la cercania vs tamaño

        # Escaneo en sitio (giro 360) para "leer" el entorno rapido sin chocar.
        self.initial_spin   = gp('initial_spin', True)     # girar al arrancar
        self.spin_on_arrival = gp('spin_on_arrival', True) # girar al llegar a cada frontera
        self.spin_seconds   = gp('spin_seconds', 13.0)     # ~360 a 0.5 rad/s
        self.w_spin         = gp('w_spin', 0.5)            # rad/s del giro en sitio

        self.auto_save  = gp('auto_save', True)
        self.output_dir = gp('output_dir', _default_maps_dir())
        self.map_name   = gp('map_name', '')                     # '' -> timestamp
        self.occ_th     = gp('occupied_thresh', 0.65)
        self.free_th    = gp('free_thresh', 0.196)

        self.map_msg = None
        self.active_goal = None      # (x, y) en frame map
        self.goal_start_t = None
        self.blacklist = []          # [(x, y)] en frame map
        self.empty_rounds = 0
        self.finished = False
        # Maquina de escaneo en sitio
        self.phase = 'init'        # init -> (giro inicial) -> exploring
        self.spinning = False
        self.spin_end = 0.0

        latch = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, latch)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        # Aviso de giro al control_diferencial (latched) para que ceda /cmd_vel
        self.spin_pub = self.create_publisher(Bool, '/explorador_spin', latch)
        self.spin_pub.publish(Bool(data=False))
        # Relay /map -> /map_dron con QoS transient_local (el planner A* lo exige).
        # Durante exploracion no hay mapa-dron previo: el mapa de SLAM ES el del planner.
        self.relay_mapdron = gp('relay_mapdron', True)
        self.mapdron_pub = (self.create_publisher(OccupancyGrid, '/map_dron', latch)
                            if self.relay_mapdron else None)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(self.tick_period, self._tick)
        self.spin_timer = self.create_timer(0.1, self._spin_tick)
        self.get_logger().info(
            f'Explorador de fronteras listo. mapa={self.map_topic}, '
            f'auto_save={self.auto_save} -> {self.output_dir}')

    # ------------------------------------------------------------------
    def _map_cb(self, msg):
        self.map_msg = msg
        # Republicar a /map_dron (frame map_dron_origin == map por la TF identidad
        # de nav_goal_bridge) para que el planner A* planee sobre el mapa vivo.
        if self.mapdron_pub is not None:
            out = msg
            out.header.frame_id = 'map_dron_origin'
            self.mapdron_pub.publish(out)

    def _robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame,
                                                rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    # ------------------------------------------------------------------
    def _tick(self):
        if self.finished or self.map_msg is None:
            return
        if self.spinning:
            return  # el escaneo en sitio lo maneja _spin_tick

        # Arranque: escaneo 360 inicial para "ver" alrededor antes de explorar
        if self.phase == 'init':
            if self.initial_spin:
                self._start_spin('inicial')
                return
            self.phase = 'exploring'

        rxy = self._robot_xy()
        if rxy is None:
            self.get_logger().info('Esperando TF del robot (map->base_footprint)...', throttle_duration_sec=5.0)
            return

        # 1) ¿hay meta activa? revisar si se alcanzo o expiro
        if self.active_goal is not None:
            d = math.hypot(self.active_goal[0] - rxy[0], self.active_goal[1] - rxy[1])
            done = False
            if d < self.reach_dist:
                self.get_logger().info(f'Frontera alcanzada ({d:.2f} m).')
                done = True
            elif (time.time() - self.goal_start_t) > self.goal_timeout:
                self.get_logger().warn(f'No llegue en {self.goal_timeout:.0f}s -> blacklist.')
                self.blacklist.append(self.active_goal)
                done = True
            if not done:
                return  # seguir hacia la meta actual
            self.active_goal = None
            # escaneo rapido al llegar: lee el entorno nuevo antes de elegir la siguiente
            if self.spin_on_arrival:
                self._start_spin('llegada')
                return

        # 2) elegir la siguiente frontera
        target, n_front = self._pick_frontier(rxy)
        if target is None:
            self.empty_rounds += 1
            self.get_logger().info(
                f'Sin fronteras alcanzables ({self.empty_rounds}/{self.done_retries}).')
            if self.empty_rounds >= self.done_retries:
                self._finish()
            return

        self.empty_rounds = 0
        self.active_goal = target
        self.goal_start_t = time.time()
        self._publish_goal(target)
        self.get_logger().info(
            f'Meta de exploracion -> ({target[0]:+.2f}, {target[1]:+.2f})  '
            f'[{n_front} fronteras, {len(self.blacklist)} en blacklist]')

    # ------------------------------------------------------------------
    def _start_spin(self, motivo):
        """Arranca un giro 360 en sitio. Avisa al control para que ceda /cmd_vel."""
        self.spinning = True
        self.spin_end = time.time() + self.spin_seconds
        self.spin_pub.publish(Bool(data=True))
        self.get_logger().info(f'Escaneo 360 en sitio ({motivo})...')

    def _spin_tick(self):
        """Timer a 10 Hz: gira en sitio mientras dure el escaneo."""
        if not self.spinning:
            return
        if time.time() < self.spin_end:
            t = Twist()
            t.angular.z = float(self.w_spin)
            self.cmd_pub.publish(t)
        else:
            self.cmd_pub.publish(Twist())            # frena
            self.spinning = False
            self.spin_pub.publish(Bool(data=False))  # control retoma /cmd_vel
            if self.phase == 'init':
                self.phase = 'exploring'
            self.get_logger().info('Escaneo 360 completo.')

    # ------------------------------------------------------------------
    def _pick_frontier(self, rxy):
        """Devuelve (x,y) en frame map de la mejor frontera, o None."""
        m = self.map_msg
        w, h, res = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        data = np.asarray(m.data, dtype=np.int16).reshape((h, w))

        free    = (data >= 0) & (data <= int(self.free_th * 100))
        unknown = (data < 0)
        # frontera = celda libre con algun vecino desconocido
        unknown_dil = ndimage.binary_dilation(unknown, iterations=1)
        frontier = free & unknown_dil
        if not frontier.any():
            return None, 0

        labels, n = ndimage.label(frontier)
        if n == 0:
            return None, 0

        best, best_score = None, -1e18
        valid = 0
        for i in range(1, n + 1):
            ys, xs = np.where(labels == i)
            size = len(xs)
            if size < self.min_frontier_cells:
                continue
            # centroide -> mundo (OccupancyGrid: fila 0 = abajo, +y hacia arriba)
            wx = ox + (xs.mean() + 0.5) * res
            wy = oy + (ys.mean() + 0.5) * res
            dist = math.hypot(wx - rxy[0], wy - rxy[1])
            if dist < self.min_goal_dist:
                continue
            if any(math.hypot(wx - bx, wy - by) < self.blacklist_radius
                   for bx, by in self.blacklist):
                continue
            valid += 1
            # prefiere fronteras grandes y cercanas
            score = size - self.dist_weight * (dist / res)
            if score > best_score:
                best_score, best = score, (wx, wy)
        return best, valid

    # ------------------------------------------------------------------
    def _publish_goal(self, xy):
        g = PoseStamped()
        g.header.frame_id = self.map_frame
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = float(xy[0])
        g.pose.position.y = float(xy[1])
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)

    # ------------------------------------------------------------------
    def _finish(self):
        self.finished = True
        self.cmd_pub.publish(Twist())  # frena el robot
        self.get_logger().info('===== EXPLORACION COMPLETA: no quedan fronteras. =====')
        if self.auto_save and self.map_msg is not None:
            self._save_map(self.map_msg)
        self.get_logger().info('Explorador terminado. Puedes lanzar navegacion con el mapa guardado.')

    def _save_map(self, msg):
        w, h, res = msg.info.width, msg.info.height, msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        oq = msg.info.origin.orientation
        yaw = math.atan2(2.0 * (oq.w * oq.z + oq.x * oq.y),
                         1.0 - 2.0 * (oq.y * oq.y + oq.z * oq.z))
        data = np.asarray(msg.data, dtype=np.int16).reshape((h, w))
        img = np.full((h, w), 205, dtype=np.uint8)
        img[(data >= 0) & (data <= int(self.free_th * 100))] = 254
        img[data >= int(self.occ_th * 100)] = 0
        img = np.flipud(img)

        os.makedirs(self.output_dir, exist_ok=True)
        name = self.map_name or ('mapa_' + time.strftime('%Y%m%d_%H%M%S'))
        pgm = os.path.join(self.output_dir, name + '.pgm')
        ym  = os.path.join(self.output_dir, name + '.yaml')
        with open(pgm, 'wb') as f:
            f.write(f'P5\n{w} {h}\n255\n'.encode('ascii'))
            f.write(img.tobytes())
        import yaml
        with open(ym, 'w') as f:
            yaml.safe_dump({
                'image': name + '.pgm',
                'mode': 'trinary',
                'resolution': float(res),
                'origin': [float(ox), float(oy), float(yaw)],
                'negate': 0,
                'occupied_thresh': float(self.occ_th),
                'free_thresh': float(self.free_th),
            }, f, default_flow_style=None, sort_keys=False)
        self.get_logger().info(f'MAPA GUARDADO: {pgm} ({w}x{h} @ {res:.3f} m/px)')
        self.get_logger().info(f'YAML:          {ym}')


def main(args=None):
    rclpy.init(args=args)
    node = ExploradorFrontera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
