#!/usr/bin/env python3
"""
mision_explorar_aruco.py — Mision combinada del carro:

  1. EXPLORANDO: explora por fronteras (mapea con SLAM). En cada frame de camara
     busca el ArUco objetivo (id 5); al verlo, MARCA su posicion en el mapa como
     checkpoint (NO se detiene). Guarda la mejor lectura (la mas cercana).
  2. (Mapeo listo = no quedan fronteras) -> VOLVIENDO: manda una meta ~0.5 m del
     checkpoint y navega de regreso con el planner.
  3. APROX_FINAL: al llegar y volver a ver el ArUco, toma /cmd_vel y se centra +
     acerca a stop_distance (40 cm) exactos.
  4. LLEGUE: se detiene y guarda el mapa.

Si termina el mapeo sin haber visto el ArUco -> guarda el mapa y avisa.
Combina explorador_frontera + buscar_aruco. Camara /cam_1/image, DICT_4X4_50.
"""
import math
import os
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, LaserScan
from std_msgs.msg import Bool, Empty
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
from scipy import ndimage

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


def _build_detector():
    try:
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        p = cv2.aruco.DetectorParameters()
        return ('new', cv2.aruco.ArucoDetector(d, p), d, p)
    except Exception:
        d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        p = cv2.aruco.DetectorParameters_create()
        return ('old', None, d, p)


def _yaw(q):
    return math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))


class MisionExplorarAruco(Node):
    def __init__(self):
        super().__init__('mision_explorar_aruco')
        gp = lambda n, v: self.declare_parameter(n, v).value

        # ---- Exploracion ----
        self.map_topic   = gp('map_topic', '/map')
        self.goal_topic  = gp('goal_topic', '/goal_pose')
        self.map_frame   = gp('map_frame', 'map')
        self.robot_frame = gp('robot_frame', 'base_footprint')
        self.min_frontier_cells = gp('min_frontier_cells', 12)
        self.reach_dist     = gp('reach_dist', 0.65)
        self.min_goal_dist  = gp('min_goal_dist', 0.75)
        self.goal_timeout   = gp('goal_timeout', 35.0)
        self.blacklist_radius = gp('blacklist_radius', 0.40)
        self.tick_period    = gp('tick_period', 2.0)
        self.done_retries   = gp('done_retries', 3)
        self.dist_weight    = gp('dist_weight', 2.5)
        self.initial_spin   = gp('initial_spin', True)
        self.spin_on_arrival = gp('spin_on_arrival', True)
        self.spin_seconds   = gp('spin_seconds', 13.0)
        self.w_spin         = gp('w_spin', 0.5)
        self.explore_timeout = gp('explore_timeout', 90.0)    # s: safeguard de terminacion del mapeo
        self.mission_timeout = gp('mission_timeout', 240.0)   # s: tope GLOBAL de seguridad -> detener todo
        self.auto_save  = gp('auto_save', True)
        self.output_dir = gp('output_dir', _default_maps_dir())
        self.map_name   = gp('map_name', '')
        self.occ_th     = gp('occupied_thresh', 0.65)
        self.free_th    = gp('free_thresh', 0.196)

        # ---- ArUco / regreso ----
        # Un cubo de ArUco trae 6 caras (ids 0-5): aceptamos CUALQUIERA = el cubo.
        self.target_ids    = [int(x) for x in gp('target_ids', [0, 1, 2, 3, 4, 5])]
        self.seen_id       = None
        self.stop_distance = float(gp('stop_distance', 0.40))
        self.marker_size   = float(gp('marker_size', 0.12))
        self.cam_topic     = gp('camera_topic', '/cam_1/image')
        self.info_topic    = gp('camera_info_topic', '/cam_1/camera_info')
        self.return_offset = float(gp('return_offset', 0.55))  # m del checkpoint a la meta de regreso
        # > 0.6 (control_diferencial se detiene a ~0.6 m de la meta): si fuera menor,
        # el robot para mas lejos que return_reach y NUNCA dispara el parqueo.
        self.return_reach  = float(gp('return_reach', 0.75))   # m para considerar "llegue al checkpoint"
        self.k_ang   = float(gp('k_ang', 1.3))
        # Signo del giro de centrado. Si gira AL REVES del cubo (imagen espejeada o
        # convencion de giro del chasis), invertir: +1.0 o -1.0.
        self.steer_sign = float(gp('steer_sign', 1.0))
        self.k_lin   = float(gp('k_lin', 0.5))
        self.v_max   = float(gp('v_max', 0.15))
        self.w_max   = float(gp('w_max', 0.6))
        self.app_dist_tol   = float(gp('dist_tol', 0.03))
        self.app_center_tol = float(gp('center_tol', 0.06))
        self.aruco_lost_timeout = float(gp('aruco_lost_timeout', 1.0))
        self.confirm_frames = int(gp('confirm_frames', 3))

        # --- Parqueo MECANUM (servo visual + lidar de abajo A1 /scan_low) ---
        # Avance por distancia del lidar bajo (ve el cubo); strafe por marker_asymmetry
        # (perpendicularidad a la cara); giro por centrado del marcador en la imagen.
        self.park_distance  = float(gp('park_distance', 0.30))      # m al cubo (lidar A1)
        self.scan_low_topic = gp('scan_low_topic', '/scan_low')
        self.low_front_half = math.radians(float(gp('low_front_deg', 20.0)))       # cono frontal
        self.low_front_off  = math.radians(float(gp('low_front_offset_deg', 0.0))) # si el A1 esta rotado
        self.kp_w = float(gp('kp_w', 0.010));  self.ki_w = float(gp('ki_w', 0.0015))  # ki bajo: menos windup
        self.kp_strafe = float(gp('kp_strafe', 0.35)); self.ki_strafe = float(gp('ki_strafe', 0.03))
        self.kp_v = float(gp('kp_v', 0.4));    self.ki_v = float(gp('ki_v', 0.10))
        self.vy_max = float(gp('vy_max', 0.10))
        self.strafe_sign = float(gp('strafe_sign', 1.0))   # invertir si strafea al lado equivocado
        # --- Anti-oscilacion del parqueo ---
        self.park_w_max     = float(gp('park_w_max', 0.35))    # cap de giro suave (vs w_max 0.6)
        self.park_dead_px   = float(gp('park_dead_px', 12.0))  # zona muerta del centrado (px)
        self.park_dead_asym = float(gp('park_dead_asym', 0.04))# zona muerta de la asimetria
        self.park_strafe_gate_px = float(gp('park_strafe_gate_px', 45.0))  # solo strafe si ~centrado
        self.park_stop_px   = float(gp('park_stop_px', 20.0))  # tolerancia de paro: centrado
        self.park_stop_asym = float(gp('park_stop_asym', 0.08))# tolerancia de paro: perpendicular
        # Fuente de la distancia de avance: True = camara (aruco_dist, siempre ve el cubo),
        # False = lidar A1 (/scan_low). El A1 no ve el cubo de forma fiable en este robot.
        self.advance_use_camera = bool(gp('advance_use_camera', True))
        # Acercamiento MECANUM al punto exacto (seen_from) sin restriccion de lookahead:
        # al "llegar" aprox con el Kelly diferencial, el mecanum lleva el CENTRO al punto.
        self.mec_tol = float(gp('mec_tol', 0.10))   # m: "llegue al punto exacto"
        self.mec_v   = float(gp('mec_v', 0.12))     # m/s max del acercamiento
        self.mec_k   = float(gp('mec_k', 0.8))      # ganancia
        # Repulsion DURANTE el acercamiento mecanum (esquiva paredes con el MS200 de arriba,
        # que NO ve el cubo bajito, asi que no estorba el acercamiento al cubo).
        self.scan_main_topic = gp('scan_main_topic', '/scan')
        self.mec_rep_thr = float(gp('mec_rep_thr', 0.40))   # m: a partir de aqui repele
        self.mec_k_rep   = float(gp('mec_k_rep', 0.12))     # ganancia de la repulsion
        self.mec_rep_max = float(gp('mec_rep_max', 0.10))   # m/s: tope de la repulsion
        self.rep_x = 0.0; self.rep_y = 0.0
        self.img_w = 640
        self.marker_cx = None
        self.marker_asymmetry = None
        self.low_front_dist = None
        self.vs_w_i = 0.0; self.vs_strafe_i = 0.0; self.vs_v_i = 0.0

        # ---- estado ----
        self.phase = 'EXPLORANDO'   # EXPLORANDO -> VOLVIENDO -> APROX_FINAL -> LLEGUE
        self.t0 = time.time()
        self.map_msg = None
        self.active_goal = None
        self.goal_start_t = None
        self.goal_resend_t = 0.0
        self.replan_period = float(gp('replan_period', 3.0))  # s: re-mandar meta + replan en exploracion
        self.blacklist = []
        self.empty_rounds = 0
        self.exp_started = False
        self.spinning = False
        self.spin_end = 0.0
        # checkpoint del aruco (mejor lectura)
        self.checkpoint = None       # (mx, my) en map
        self.seen_from = None        # (rx, ry) del robot al verlo
        self.checkpoint_best_z = 1e9
        self.return_goal = None
        self.return_sent_t = 0.0
        self.search_at_checkpoint = False
        self.search_attempts = 0
        self.max_search = int(gp('max_search', 4))   # reintentos de barrido para reencontrar el cubo
        # --- Barrido LENTO escalonado para reencontrar el cubo (evita desenfoque) ---
        # En vez de girar continuo (borroso), gira un pasito -> se DETIENE -> mira -> repite.
        self.search_step_deg = float(gp('search_step_deg', 25.0))   # grados por pasito
        self.search_w        = float(gp('search_w', 0.30))          # rad/s LENTO del pasito
        self.search_pause_s  = float(gp('search_pause_s', 0.9))     # s parado mirando (camara nitida)
        self.searching_stepped = False
        self.search_substate = 'pause'   # 'pause' (mirando) | 'rotate' (girando un paso)
        self.search_substate_end = 0.0
        self.search_steps_done = 0
        self.search_total_steps = max(1, int(round(360.0 / max(1.0, self.search_step_deg))))
        # deteccion aruco (para aproximacion final). Calibracion de fabrica del Orbbec
        # Astra Pro Plus RGB 640x480 (la sobreescribe /cam_1/camera_info si llega).
        self.K = np.array([[539.1278076171875, 0, 320.458251953125],
                           [0, 539.1278076171875, 240.0159454345703],
                           [0, 0, 1]], dtype=np.float64)
        self.D = np.array([0.13392946124076843, -0.2676388621330261,
                           0.0025758773554116488, 0.00267409672960639,
                           0.14543378353118896], dtype=np.float64)  # k1 k2 p1 p2 k3
        self.aruco_u = None
        self.aruco_dist = None
        self.aruco_last_t = None
        self.aruco_count = 0
        self.bridge = CvBridge()
        self.api, self.detector, self.adict, self.aparams = _build_detector()

        latch = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, latch)
        self.create_subscription(CameraInfo, self.info_topic, self._info_cb, 10)
        self.create_subscription(Image, self.cam_topic, self._img_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_low_topic, self._scan_low_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_main_topic, self._scan_main_cb, qos_profile_sensor_data)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.spin_pub = self.create_publisher(Bool, '/explorador_spin', latch)
        self.replan_pub = self.create_publisher(Empty, '/replan_request', 10)
        self.spin_pub.publish(Bool(data=False))
        self.mapdron_pub = self.create_publisher(OccupancyGrid, '/map_dron', latch)
        self.marker_pub = self.create_publisher(MarkerArray, '/mision_markers', latch)
        # imagen de debug anotada (lo que ve el robot + deteccion ArUco). Cruda pero
        # reducida a 480x360 y a ~5 Hz -> ligera para el WiFi y se ve directo en rqt.
        self.dbg_pub = self.create_publisher(Image, '/mision_aruco_debug', 5)
        self.dbg_last_t = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(self.tick_period, self._mission_tick)
        self.fast = self.create_timer(0.066, self._fast_tick)

        self.get_logger().info(
            f'Mision lista: explora+mapea, marca el cubo (ArUco {self.target_ids}), al terminar el '
            f'mapeo vuelve y se acerca a {self.stop_distance:.2f} m.')

    # ================= ArUco =================
    def _info_cb(self, msg):
        if len(msg.k) >= 9 and msg.k[0] > 0.0:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            if len(msg.d) >= 4:
                self.D = np.array(msg.d, dtype=np.float64)

    def _img_cb(self, msg):
        if self.phase == 'LLEGUE':
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        self.img_w = gray.shape[1]
        if self.api == 'new':
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.adict, parameters=self.aparams)
        found = False
        if ids is not None:
            ids = ids.flatten().tolist()
            # Puede haber 2 caras del cubo (dos id 5) en vista de esquina. Antes se tomaba
            # ids.index(tid) = el PRIMERO, cuyo orden NO es estable -> marker_cx saltaba entre
            # las dos caras y el robot oscilaba "entre los 2 arucos". Ahora elegimos la cara
            # MAS GRANDE (la mas de frente/cercana): objetivo unico y estable para el parqueo.
            cand = [j for j, mid in enumerate(ids) if mid in self.target_ids]
            if cand:
                i = max(cand, key=lambda j: cv2.contourArea(
                    corners[j].reshape(4, 2).astype(np.float32)))
                self.seen_id = ids[i]
                c = corners[i].reshape(4, 2)
                self.aruco_u = float(c[:, 0].mean())
                self.marker_cx = self.aruco_u
                # asimetria de la cara (perpendicularidad): lado der vs lado izq
                ld = float(np.linalg.norm(c[1] - c[2]))
                li = float(np.linalg.norm(c[3] - c[0]))
                avg = (ld + li) / 2.0
                self.marker_asymmetry = (ld - li) / avg if avg > 0 else 0.0
                try:
                    _, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[i]], self.marker_size, self.K, self.D)
                    self.aruco_dist = float(tvec[0][0][2])
                except Exception:
                    side = np.linalg.norm(c[0] - c[1])
                    self.aruco_dist = (self.K[0, 0] * self.marker_size) / max(side, 1.0)
                self.aruco_last_t = self.get_clock().now()
                found = True
        if not found:
            self.marker_cx = None
            self.marker_asymmetry = None
        self.aruco_count = min(self.confirm_frames, self.aruco_count + 1) if found else 0

        # Imagen de debug anotada (~6 Hz, suave para el WiFi)
        now_s = time.time()
        if now_s - self.dbg_last_t >= 0.15:
            self.dbg_last_t = now_s
            self._publish_debug(img, corners, ids)

        # Marcar checkpoint solo durante exploracion (con deteccion confirmada)
        if found and self.aruco_count >= self.confirm_frames and self.phase == 'EXPLORANDO':
            self._mark_checkpoint()

    def _publish_debug(self, img, corners, ids):
        """Publica la imagen del robot con la deteccion ArUco dibujada + datos del
        parqueo (fase, visible, dist camara/A1, err_x, asimetria, modo de giro).
        Para VER en vivo por que intercala parqueo<->giro."""
        try:
            dbg = img.copy()
            h, w = dbg.shape[:2]
            cxi = w // 2
            cv2.line(dbg, (cxi, 0), (cxi, h), (0, 255, 255), 1)          # centro de imagen
            if ids is not None and len(corners) > 0:
                cv2.aruco.drawDetectedMarkers(dbg, corners, np.array(ids).reshape(-1, 1))
            vis = self._aruco_visible()
            err_x = (w / 2.0 - self.marker_cx) if self.marker_cx is not None else None
            if self.marker_cx is not None:
                mcx = int(self.marker_cx)
                col = (0, 255, 0) if vis else (0, 165, 255)             # verde=visible, naranja=sin confirmar
                cv2.circle(dbg, (mcx, h // 2), 6, col, -1)
                cv2.line(dbg, (cxi, h // 2), (mcx, h // 2), col, 2)
            asym = self.marker_asymmetry
            dcam = self.aruco_dist
            dlow = self.low_front_dist
            modo = 'BARRIDO' if self.searching_stepped else ('GIRO' if self.spinning else '')
            lines = [
                f'fase={self.phase}  visible={vis} ({self.aruco_count}/{self.confirm_frames}) {modo}',
                f'cara={self.seen_id}  dist_cam=' + (f'{dcam:.2f}m' if dcam else '--'),
                'A1=' + (f'{dlow:.2f}m' if dlow is not None else 'None') +
                    '  err_x=' + (f'{err_x:+.0f}px' if err_x is not None else '--') +
                    '  asim=' + (f'{asym:+.2f}' if asym is not None else '--'),
            ]
            y = 18
            for t in lines:
                cv2.putText(dbg, t, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(dbg, t, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                y += 20
            if dbg.shape[1] != 480:
                dbg = cv2.resize(dbg, (480, 360))
            self.dbg_pub.publish(self.bridge.cv2_to_imgmsg(dbg, 'bgr8'))
        except Exception as e:
            self.get_logger().warn(f'debug img: {e}', throttle_duration_sec=5.0)

    def _mark_checkpoint(self):
        pose = self._robot_pose()
        if pose is None:
            return
        rx, ry, rth = pose
        z = self.aruco_dist
        if z is None or z <= 0.05 or z > 6.0:
            return
        # lateral del marcador (m) desde el pixel: x_cam ~= z*(u-cx)/fx
        x_cam = z * (self.aruco_u - self.K[0, 2]) / self.K[0, 0]
        # a frame robot: adelante=z, izquierda=-x_cam ; a map con la pose del robot
        fxr, fyr = z, -x_cam
        mx = rx + fxr*math.cos(rth) - fyr*math.sin(rth)
        my = ry + fxr*math.sin(rth) + fyr*math.cos(rth)
        if z < self.checkpoint_best_z:   # quedarse con la lectura mas cercana (mas precisa)
            self.checkpoint_best_z = z
            self.checkpoint = (mx, my)
            self.seen_from = (rx, ry)
            self.get_logger().info(
                f'Cubo (cara {self.seen_id}) MARCADO en mapa ({mx:+.2f},{my:+.2f}) [a {z:.2f} m]. '
                f'Sigo mapeando.')
            self._publish_markers()

    def _scan_low_cb(self, msg):
        # Distancia FRONTAL del lidar de abajo (A1): min en un cono al frente.
        # Ve el cubo (objeto bajito) que el MS200 de arriba no alcanza.
        best = float('inf')
        a = msg.angle_min
        for r in msg.ranges:
            if 0.05 < r < float('inf') and not math.isnan(r):
                d = math.atan2(math.sin(a - self.low_front_off), math.cos(a - self.low_front_off))
                if abs(d) <= self.low_front_half and r < best:
                    best = r
            a += msg.angle_increment
        self.low_front_dist = best if best < float('inf') else None

    def _scan_main_cb(self, msg):
        # Vector de repulsion en frame ROBOT (angulo 0 = frente) desde el MS200 de arriba.
        # Solo se usa en el acercamiento mecanum para esquivar paredes.
        rx = ry = 0.0
        thr = self.mec_rep_thr
        a = msg.angle_min
        for r in msg.ranges:
            if 0.05 < r < thr and not math.isnan(r):
                w = (thr - r) / thr
                rx += -math.cos(a) * w
                ry += -math.sin(a) * w
            a += msg.angle_increment
        self.rep_x = rx; self.rep_y = ry

    def _aruco_seen_now(self):
        # Mas laxo que _aruco_visible (no exige confirm_frames): vio el cubo recien.
        # Para comprometerse a parquear sobre la cara que ve AHORA, no la guardada.
        if self.aruco_last_t is None or self.marker_cx is None or self.aruco_count < 2:
            return False
        return (self.get_clock().now() - self.aruco_last_t).nanoseconds/1e9 < self.aruco_lost_timeout

    def _aruco_visible(self):
        if self.aruco_last_t is None or self.aruco_count < self.confirm_frames:
            return False
        return (self.get_clock().now() - self.aruco_last_t).nanoseconds/1e9 < self.aruco_lost_timeout

    # ================= mision (2 Hz) =================
    def _mission_tick(self):
        if self.phase in ('MEC_APPROACH', 'APROX_FINAL', 'LLEGUE'):
            return
        if self.spinning:
            return

        if self.phase == 'VOLVIENDO':
            self._tick_volviendo()
            return

        # ---- EXPLORANDO ----
        if self.map_msg is None:
            return
        if not self.exp_started:
            self.exp_started = True
            if self.initial_spin:
                self._start_spin('inicial'); return

        rxy = self._robot_xy()
        if rxy is None:
            self.get_logger().info('Esperando TF del robot...', throttle_duration_sec=5.0)
            return

        if self.active_goal is not None:
            d = math.hypot(self.active_goal[0]-rxy[0], self.active_goal[1]-rxy[1])
            done = False
            if d < self.reach_dist:
                done = True
            elif (time.time()-self.goal_start_t) > self.goal_timeout:
                self.blacklist.append(self.active_goal); done = True
            if not done:
                # Replaneo PERIODICO: el planner no replanea solo tras el primer plan
                # (su timer se auto-cancela). Re-mandamos la meta + /replan_request cada
                # replan_period s para no quedarnos pegados a un waypoint inalcanzable.
                if time.time() - self.goal_resend_t > self.replan_period:
                    self._publish_goal(self.active_goal)
                    self.replan_pub.publish(Empty())
                    self.goal_resend_t = time.time()
                return
            self.active_goal = None
            if self.spin_on_arrival:
                self._start_spin('llegada'); return

        target, n = self._pick_frontier(rxy)
        timed_out = (time.time() - self.t0) > self.explore_timeout
        if target is None:
            self.empty_rounds += 1
        else:
            self.empty_rounds = 0

        # Condicion de TERMINACION del mapeo: sin fronteras N rondas (o timeout)
        if (target is None and self.empty_rounds >= self.done_retries) or timed_out:
            self._on_mapping_done(timed_out)
            return

        if target is not None:
            self.active_goal = target
            self.goal_start_t = time.time()
            self.goal_resend_t = time.time()
            self._publish_goal(target)
            self.get_logger().info(f'Frontera -> ({target[0]:+.2f},{target[1]:+.2f}) [{n}]')

    def _on_mapping_done(self, timed_out):
        why = 'timeout' if timed_out else 'sin fronteras'
        if self.checkpoint is None:
            self.get_logger().warn(f'Mapeo COMPLETO ({why}) pero NO se vio el cubo de ArUco. Guardo y termino.')
            self._finish(found=False)
            return
        # Meta de regreso = DONDE el robot vio mejor el cubo (su propia pose, confiable).
        # Asi no dependemos de la posicion calculada del cubo (que puede salir mal por
        # tamaño de marcador/inclinacion de camara). Ahi el cubo vuelve a estar a la vista.
        self.return_goal = self.seen_from if self.seen_from else self.checkpoint
        gx, gy = self.return_goal
        self.phase = 'VOLVIENDO'
        self.get_logger().info(
            f'Mapeo COMPLETO ({why}). VOLVIENDO a donde vi el cubo: ({gx:+.2f},{gy:+.2f}).')
        self._publish_goal(self.return_goal)
        self.return_sent_t = time.time()
        self._publish_markers()

    def _tick_volviendo(self):
        rxy = self._robot_xy()
        if rxy is None:
            return
        # re-mandar la meta cada 4 s (mantiene viva la replanificacion)
        if time.time() - self.return_sent_t > 4.0:
            self._publish_goal(self.return_goal)
            self.return_sent_t = time.time()
        d = math.hypot(self.return_goal[0]-rxy[0], self.return_goal[1]-rxy[1])
        # Si ya veo el cubo en el camino, directo al parqueo (sobre la cara que veo)
        if self._aruco_seen_now():
            self.get_logger().info('Veo el cubo de regreso -> aproximacion final.')
            self.vs_w_i = self.vs_strafe_i = self.vs_v_i = 0.0
            self.phase = 'APROX_FINAL'
            self.spin_pub.publish(Bool(data=True))
            return
        # Llegada APROX del Kelly diferencial (se detiene ~0.6 m por el lookahead):
        # tomar control MECANUM para llevar el CENTRO al punto EXACTO y ahi girar.
        if d < self.return_reach and not self.spinning:
            self.get_logger().info('Llegada aprox -> control MECANUM al punto exacto + buscar.')
            self.phase = 'MEC_APPROACH'
            self.spin_pub.publish(Bool(data=True))   # control diferencial cede /cmd_vel

    # ================= fast (15 Hz): giros + aproximacion final =================
    def _fast_tick(self):
        # Tope GLOBAL de seguridad: pase lo que pase, detener y guardar al expirar.
        if self.phase != 'LLEGUE' and (time.time() - self.t0) > self.mission_timeout:
            self.get_logger().warn('TIMEOUT GLOBAL de mision -> deteniendo y guardando.')
            self._finish(found=(self.phase == 'APROX_FINAL'))
            return

        # ---- BARRIDO LENTO escalonado: reencontrar el cubo SIN desenfoque ----
        if self.searching_stepped:
            now = time.time()
            # mira en CADA tick (sobre todo durante la pausa, ya parado y nitido)
            if self._aruco_seen_now():
                self.cmd_pub.publish(Twist())
                self.searching_stepped = False
                self.vs_w_i = self.vs_strafe_i = self.vs_v_i = 0.0
                self.phase = 'APROX_FINAL'
                self.get_logger().info('Cubo encontrado en el barrido lento -> parqueo.')
                return
            if self.search_substate == 'pause':
                self.cmd_pub.publish(Twist())   # PARADO mirando (camara nitida)
                if now >= self.search_substate_end:
                    if self.search_steps_done >= self.search_total_steps:
                        # vuelta completa sin verlo -> fin del barrido (MEC_APPROACH reintenta/rinde)
                        self.searching_stepped = False
                        self.search_at_checkpoint = False
                    else:
                        self.search_substate = 'rotate'
                        self.search_substate_end = now + (math.radians(self.search_step_deg) / self.search_w)
            else:  # rotate: un pasito LENTO
                if now < self.search_substate_end:
                    t = Twist(); t.angular.z = float(self.search_w); self.cmd_pub.publish(t)
                else:
                    self.cmd_pub.publish(Twist())   # detener ANTES de mirar
                    self.search_steps_done += 1
                    self.search_substate = 'pause'
                    self.search_substate_end = now + self.search_pause_s
            return

        if self.spinning:
            searching = self.search_at_checkpoint and self.phase == 'MEC_APPROACH'
            # DURANTE el giro de busqueda: si aparece el cubo, CORTAR el giro y parquear
            if searching and self._aruco_seen_now():
                self.cmd_pub.publish(Twist())
                self.spinning = False
                self.vs_w_i = self.vs_strafe_i = self.vs_v_i = 0.0
                self.phase = 'APROX_FINAL'
                self.get_logger().info('Cubo encontrado durante el giro -> parqueo.')
                return
            if time.time() < self.spin_end:
                t = Twist(); t.angular.z = float(self.w_spin); self.cmd_pub.publish(t)
            else:
                self.cmd_pub.publish(Twist())
                self.spinning = False
                if searching:
                    self.search_at_checkpoint = False   # MEC_APPROACH reintenta/rinde (mantiene el gate)
                else:
                    self.spin_pub.publish(Bool(data=False))   # giro de exploracion -> devolver control
            return

        # ---- ACERCAMIENTO MECANUM al punto exacto (sin lookahead) + buscar ----
        if self.phase == 'MEC_APPROACH':
            # Si ve el cubo AHORA (cualquier cara), se compromete a parquear sobre esa,
            # no sigue manejando al punto guardado.
            if self._aruco_seen_now():
                self.vs_w_i = self.vs_strafe_i = self.vs_v_i = 0.0
                self.phase = 'APROX_FINAL'
                self.get_logger().info('Veo una cara del cubo -> parqueo sobre esa.')
                return
            pose = self._robot_pose()
            if pose is None:
                return
            rx, ry, rth = pose
            ex = self.return_goal[0] - rx
            ey = self.return_goal[1] - ry
            dist = math.hypot(ex, ey)
            if dist >= self.mec_tol:
                # mecanum: llevar el CENTRO al punto (vx adelante, vy strafe), sin lookahead
                fx =  ex * math.cos(rth) + ey * math.sin(rth)   # adelante (robot)
                fy = -ex * math.sin(rth) + ey * math.cos(rth)   # izquierda (robot)
                # repulsion (frame robot, MS200) capada -> esquiva paredes en el mecanum
                repx = self.mec_k_rep * self.rep_x
                repy = self.mec_k_rep * self.rep_y
                rm = math.hypot(repx, repy)
                if rm > self.mec_rep_max:
                    repx = repx / rm * self.mec_rep_max
                    repy = repy / rm * self.mec_rep_max
                cmd = Twist()
                cmd.linear.x = max(-self.mec_v, min(self.mec_v, self.mec_k * fx + repx))
                cmd.linear.y = max(-self.mec_v, min(self.mec_v, self.mec_k * fy + repy))
                self.cmd_pub.publish(cmd)
                self.get_logger().info(
                    f'Mecanum -> punto exacto: dist={dist:.2f} m rep=({repx:+.2f},{repy:+.2f})',
                    throttle_duration_sec=1.0)
            else:
                # en el punto -> girar a buscar el cubo (con reintentos)
                self.cmd_pub.publish(Twist())
                if self.search_attempts < self.max_search:
                    self.search_attempts += 1
                    self.search_at_checkpoint = True
                    self._start_stepped_search(f'buscar-en-punto {self.search_attempts}/{self.max_search}')
                else:
                    self.get_logger().warn('No reencontre el cubo tras varios giros. Guardo y termino.')
                    self._finish(found=False)
            return

        if self.phase != 'APROX_FINAL':
            return

        # ---- PARQUEO MECANUM: 3 ejes simultaneos (servo visual + lidar A1) ----
        if not self._aruco_visible() or self.marker_cx is None:
            t = Twist(); t.angular.z = self.steer_sign * self.search_w   # re-buscar girando LENTO
            self.cmd_pub.publish(t)
            return
        dt = 0.066
        cmd = Twist()
        err_x = (self.img_w / 2.0) - self.marker_cx
        asym = self.marker_asymmetry if self.marker_asymmetry is not None else 0.0

        # 1) GIRO: centrar el marcador (P+I) con ZONA MUERTA y cap suave (no jitterea)
        if abs(err_x) < self.park_dead_px:
            self.vs_w_i = 0.0
            cmd.angular.z = 0.0
        else:
            self.vs_w_i = max(-100.0, min(100.0, self.vs_w_i + err_x * dt))
            w = self.steer_sign * (self.kp_w * err_x + self.ki_w * self.vs_w_i)
            cmd.angular.z = max(-self.park_w_max, min(self.park_w_max, w))

        # 2) STRAFE: perpendicularidad, SOLO cuando ya esta ~centrado (desacopla giro<->strafe)
        if abs(err_x) < self.park_strafe_gate_px and abs(asym) > self.park_dead_asym:
            self.vs_strafe_i = max(-1.0, min(1.0, self.vs_strafe_i + asym * dt))
            strafe = self.kp_strafe * asym + self.ki_strafe * self.vs_strafe_i
            cmd.linear.y = self.strafe_sign * max(-self.vy_max, min(self.vy_max, strafe))
        else:
            self.vs_strafe_i = 0.0
            cmd.linear.y = 0.0

        # 3) AVANCE: distancia del A1, SOLO bien alineado (asi el cono del A1 cae sobre el cubo,
        # no barre paredes al girar). Gate de avance mas laxo que el de paro para que progrese.
        adv_dist = self.aruco_dist if self.advance_use_camera else self.low_front_dist
        if adv_dist is None:
            cmd.linear.x = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info('Esperando distancia para el avance...', throttle_duration_sec=3.0)
            return
        dist_err = adv_dist - self.park_distance
        advance_ok = abs(err_x) < 30.0 and abs(asym) < 0.12
        stop_ok    = abs(err_x) < self.park_stop_px and abs(asym) < self.park_stop_asym
        if abs(dist_err) > 0.04 and advance_ok:
            self.vs_v_i = max(-0.5, min(0.5, self.vs_v_i + dist_err * dt))
            v = self.kp_v * dist_err + self.ki_v * self.vs_v_i
            cmd.linear.x = max(-0.10, min(0.10, v))
        else:
            self.vs_v_i = 0.0
            cmd.linear.x = 0.0
        # PARO: distancia ok + centrado + perpendicular
        if abs(dist_err) < 0.04 and stop_ok:
            self.cmd_pub.publish(Twist())
            self._finish(found=True)
            return
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'Parqueo: dist={adv_dist:.2f}m centro={err_x:+.0f}px asim={asym:+.2f} '
            f'[v={cmd.linear.x:+.2f} vy={cmd.linear.y:+.2f} w={cmd.angular.z:+.2f}]',
            throttle_duration_sec=1.0)

    def _finish(self, found):
        if self.phase == 'LLEGUE':
            return
        self.phase = 'LLEGUE'
        self.cmd_pub.publish(Twist())
        self.spin_pub.publish(Bool(data=False))
        if found:
            self.get_logger().info(
                f'===== MISION COMPLETA: cubo (cara {self.seen_id}) a {self.aruco_dist:.2f} m. =====')
        if self.auto_save and self.map_msg is not None:
            self._save_map(self.map_msg)

    # ================= helpers =================
    def _map_cb(self, msg):
        self.map_msg = msg
        out = msg
        out.header.frame_id = 'map_dron_origin'
        self.mapdron_pub.publish(out)

    def _robot_xy(self):
        p = self._robot_pose()
        return None if p is None else (p[0], p[1])

    def _robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y,
                    _yaw(t.transform.rotation))
        except Exception:
            return None

    def _start_spin(self, motivo):
        self.spinning = True
        self.spin_end = time.time() + self.spin_seconds
        self.spin_pub.publish(Bool(data=True))
        self.get_logger().info(f'Escaneo 360 ({motivo})...')

    def _start_stepped_search(self, motivo):
        """Barrido LENTO escalonado: empieza PARADO mirando, luego pasito a pasito."""
        self.searching_stepped = True
        self.search_substate = 'pause'
        self.search_substate_end = time.time() + self.search_pause_s
        self.search_steps_done = 0
        self.spin_pub.publish(Bool(data=True))   # control diferencial cede /cmd_vel
        self.get_logger().info(
            f'Barrido lento ({motivo}): {self.search_total_steps} pasos de '
            f'{self.search_step_deg:.0f} grados a {self.search_w:.2f} rad/s.')

    def _pick_frontier(self, rxy):
        m = self.map_msg
        w, h, res = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        data = np.asarray(m.data, dtype=np.int16).reshape((h, w))
        free = (data >= 0) & (data <= int(self.free_th*100))
        unknown = (data < 0)
        frontier = free & ndimage.binary_dilation(unknown, iterations=1)
        if not frontier.any():
            return None, 0
        labels, n = ndimage.label(frontier)
        best, best_score, valid = None, -1e18, 0
        for i in range(1, n+1):
            ys, xs = np.where(labels == i)
            if len(xs) < self.min_frontier_cells:
                continue
            wx = ox + (xs.mean()+0.5)*res
            wy = oy + (ys.mean()+0.5)*res
            dist = math.hypot(wx-rxy[0], wy-rxy[1])
            if dist < self.min_goal_dist:
                continue
            if any(math.hypot(wx-bx, wy-by) < self.blacklist_radius for bx, by in self.blacklist):
                continue
            valid += 1
            score = len(xs) - self.dist_weight*(dist/res)
            if score > best_score:
                best_score, best = score, (wx, wy)
        return best, valid

    def _publish_goal(self, xy):
        g = PoseStamped()
        g.header.frame_id = self.map_frame
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = float(xy[0]); g.pose.position.y = float(xy[1])
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)

    def _publish_markers(self):
        """Marcadores en RViz (/mision_markers): checkpoint del cubo, donde lo vio,
        y la meta de regreso. Para VER si el calculo de la posicion del cubo es correcto."""
        arr = MarkerArray()
        def mk(mid, xy, rgb, scale, kind=Marker.SPHERE):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'mision'; m.id = mid; m.type = kind; m.action = Marker.ADD
            m.pose.position.x = float(xy[0]); m.pose.position.y = float(xy[1])
            m.pose.position.z = 0.1; m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 0.9
            return m
        if self.checkpoint:
            arr.markers.append(mk(0, self.checkpoint, (1.0, 0.1, 0.1), 0.18, Marker.CUBE))  # cubo calc: ROJO
        if self.seen_from:
            arr.markers.append(mk(1, self.seen_from, (0.1, 1.0, 0.1), 0.14))                # lo vio aqui: VERDE
        if self.return_goal:
            arr.markers.append(mk(2, self.return_goal, (0.1, 0.4, 1.0), 0.14))              # meta regreso: AZUL
        self.marker_pub.publish(arr)

    def _save_map(self, msg):
        w, h, res = msg.info.width, msg.info.height, msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        oq = msg.info.origin.orientation
        yaw = math.atan2(2.0*(oq.w*oq.z+oq.x*oq.y), 1.0-2.0*(oq.y*oq.y+oq.z*oq.z))
        data = np.asarray(msg.data, dtype=np.int16).reshape((h, w))
        img = np.full((h, w), 205, dtype=np.uint8)
        img[(data >= 0) & (data <= int(self.free_th*100))] = 254
        img[data >= int(self.occ_th*100)] = 0
        img = np.flipud(img)
        os.makedirs(self.output_dir, exist_ok=True)
        name = self.map_name or ('mapa_' + time.strftime('%Y%m%d_%H%M%S'))
        pgm = os.path.join(self.output_dir, name+'.pgm')
        ym  = os.path.join(self.output_dir, name+'.yaml')
        with open(pgm, 'wb') as f:
            f.write(f'P5\n{w} {h}\n255\n'.encode('ascii')); f.write(img.tobytes())
        import yaml
        with open(ym, 'w') as f:
            yaml.safe_dump({'image': name+'.pgm', 'mode': 'trinary', 'resolution': float(res),
                            'origin': [float(ox), float(oy), float(yaw)], 'negate': 0,
                            'occupied_thresh': float(self.occ_th), 'free_thresh': float(self.free_th)},
                           f, default_flow_style=None, sort_keys=False)
        self.get_logger().info(f'MAPA GUARDADO: {pgm}')


def main(args=None):
    rclpy.init(args=args)
    node = MisionExplorarAruco()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
