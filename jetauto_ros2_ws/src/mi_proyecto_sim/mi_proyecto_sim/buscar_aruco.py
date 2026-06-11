#!/usr/bin/env python3
"""
buscar_aruco.py — El carro BUSCA un ArUco (default id 5) con su camara y, al
encontrarlo, se centra y se ACERCA hasta quedar a una distancia objetivo (40 cm).

Estados:
  BUSCANDO  -> gira en sitio hasta detectar el marcador objetivo (varios frames).
  ACERCANDO -> centra el marcador (giro) y avanza/retrocede hasta stop_distance.
  LLEGUE    -> se detiene. Si lo pierde un rato, vuelve a BUSCAR.

Publica /cmd_vel directo, asi que CORRELO SOLO (sin navegacion/exploracion a la vez).
Camara: /cam_1/image (+ /cam_1/camera_info para intrinsecos). DICT_4X4_50.
"""
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


def _build_detector():
    """Compatibilidad cv2 viejo (4.5, API funcional) y nuevo (4.7+, ArucoDetector)."""
    try:  # API nueva
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        p = cv2.aruco.DetectorParameters()
        det = cv2.aruco.ArucoDetector(d, p)
        return ('new', det, d, p)
    except Exception:
        d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        p = cv2.aruco.DetectorParameters_create()
        return ('old', None, d, p)


class BuscarAruco(Node):
    def __init__(self):
        super().__init__('buscar_aruco')
        gp = lambda n, v: self.declare_parameter(n, v).value
        self.target_id     = int(gp('target_id', 5))
        self.stop_distance = float(gp('stop_distance', 0.40))   # m
        self.marker_size   = float(gp('marker_size', 0.112))    # m (lado del ArUco)
        self.cam_topic     = gp('camera_topic', '/cam_1/image')
        self.info_topic    = gp('camera_info_topic', '/cam_1/camera_info')

        self.search_w   = float(gp('search_w', 0.45))   # rad/s al buscar
        self.k_ang      = float(gp('k_ang', 1.3))       # ganancia de centrado
        self.k_lin      = float(gp('k_lin', 0.5))       # ganancia de avance
        self.v_max      = float(gp('v_max', 0.15))
        self.w_max      = float(gp('w_max', 0.6))
        self.dist_tol   = float(gp('dist_tol', 0.03))   # m: tolerancia de llegada
        self.center_tol = float(gp('center_tol', 0.06)) # error normalizado de centrado
        self.lost_timeout = float(gp('lost_timeout', 1.0))   # s sin ver -> "no visible"
        self.confirm_frames = int(gp('confirm_frames', 2))   # frames para confirmar
        # Recuperacion: si lo pierde ACERCANDO, se DETIENE y espera re-deteccion (no gira)
        self.hold_on_lost = float(gp('hold_on_lost', 4.0))   # s parado esperando antes de girar a buscar
        self.slow_dist    = float(gp('slow_dist', 0.60))     # m: por debajo baja velocidad (anti motion-blur)
        self.slow_v       = float(gp('slow_v', 0.08))        # m/s tope en zona lenta
        self.publish_debug  = bool(gp('publish_debug', True)) # imagen anotada para la laptop
        self.debug_every    = int(gp('debug_every', 3))      # publica 1 de cada N frames (~10 Hz)

        # Intrinsecos: fallback a los conocidos de cam_1; se actualizan con camera_info
        self.K = np.array([[539.13, 0.0, 320.46],
                           [0.0, 539.13, 240.02],
                           [0.0, 0.0, 1.0]], dtype=np.float64)
        self.D = np.zeros((5,), dtype=np.float64)
        self.have_info = False

        self.bridge = CvBridge()
        self.api, self.detector, self.aruco_dict, self.params = _build_detector()

        self.state = 'BUSCANDO'
        self.seen_count = 0
        self.last_seen_t = None
        self.last_u = None       # pixel x del centro del marcador
        self.last_dist = None    # m
        self.img_cx = 320.0
        self.dbg_count = 0       # contador para diezmar la imagen de debug
        self.search_dir = 1.0    # sentido de giro al buscar (hacia el ultimo lado visto)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dbg_pub = (self.create_publisher(CompressedImage, '/buscar_aruco/debug/compressed', 5)
                        if self.publish_debug else None)
        self.create_subscription(CameraInfo, self.info_topic, self._info_cb, 10)
        self.create_subscription(Image, self.cam_topic, self._img_cb, 10)
        self.timer = self.create_timer(0.066, self._control)   # ~15 Hz

        self.get_logger().info(
            f'buscar_aruco listo: busca ArUco {self.target_id}, para a {self.stop_distance:.2f} m. '
            f'Camara {self.cam_topic}, marcador {self.marker_size} m. '
            f'Debug: {"/buscar_aruco/debug/compressed" if self.publish_debug else "off"}.')

    # ------------------------------------------------------------------
    def _info_cb(self, msg):
        if len(msg.k) >= 9 and msg.k[0] > 0.0:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            if len(msg.d) >= 4:
                self.D = np.array(msg.d, dtype=np.float64)
            self.img_cx = self.K[0, 2]
            if not self.have_info:
                self.have_info = True
                self.get_logger().info('Intrinsecos de camera_info recibidos.')

    def _detect(self, gray):
        if self.api == 'new':
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)
        return corners, ids

    def _img_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            try:
                img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            except Exception as e:
                self.get_logger().warn(f'cv_bridge: {e}', throttle_duration_sec=5.0)
                return
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        corners, ids = self._detect(gray)

        found = False
        if ids is not None:
            ids = ids.flatten().tolist()
            if self.target_id in ids:
                i = ids.index(self.target_id)
                c = corners[i].reshape(4, 2)
                u = float(c[:, 0].mean())   # centro x en pixeles
                # distancia: pose del marcador (z = profundidad optica)
                try:
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[i]], self.marker_size, self.K, self.D)
                    dist = float(tvec[0][0][2])
                except Exception:
                    # fallback: distancia por tamaño aparente (fx * tamaño_real / lado_px)
                    side_px = np.linalg.norm(c[0] - c[1])
                    dist = (self.K[0, 0] * self.marker_size) / max(side_px, 1.0)
                self.last_u = u
                self.last_dist = dist
                self.last_seen_t = self.get_clock().now()
                found = True

        if found:
            self.seen_count = min(self.confirm_frames, self.seen_count + 1)
        else:
            self.seen_count = 0

        self._publish_debug(img, corners, ids, found)

    # ------------------------------------------------------------------
    def _publish_debug(self, img, corners, ids, found):
        """Imagen anotada (JPEG comprimido) para verla desde la laptop con rqt_image_view."""
        if self.dbg_pub is None:
            return
        self.dbg_count += 1
        if self.debug_every > 1 and (self.dbg_count % self.debug_every) != 0:
            return
        try:
            vis = img.copy() if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            h, w = vis.shape[:2]
            # todos los marcadores detectados (gris) y el objetivo resaltado (verde)
            if ids:
                cv2.aruco.drawDetectedMarkers(vis, corners)
                if found and self.target_id in ids:
                    i = ids.index(self.target_id)
                    c = corners[i].reshape(4, 2).astype(int)
                    cv2.polylines(vis, [c], True, (0, 255, 0), 3)
                    cx, cy = int(c[:, 0].mean()), int(c[:, 1].mean())
                    cv2.circle(vis, (cx, cy), 6, (0, 255, 0), -1)
            # mira central (centro de imagen = donde debe quedar el marcador)
            cv2.line(vis, (w // 2, 0), (w // 2, h), (255, 180, 0), 1)
            # banner de estado
            color = (0, 200, 0) if self.state == 'LLEGUE' else (
                     (0, 255, 0) if found else (0, 165, 255))
            dist_txt = f'{self.last_dist:.2f}m' if (found and self.last_dist is not None) else '--'
            txt = f'[{self.state}] ArUco {self.target_id}  dist={dist_txt}  meta={self.stop_distance:.2f}m'
            cv2.rectangle(vis, (0, 0), (w, 26), (0, 0, 0), -1)
            cv2.putText(vis, txt, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

            ok, buf = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if not ok:
                return
            out = CompressedImage()
            out.header.stamp = self.get_clock().now().to_msg()
            out.format = 'jpeg'
            out.data = buf.tobytes()
            self.dbg_pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f'debug img: {e}', throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    def _control(self):
        if self.state == 'LLEGUE':
            self.cmd_pub.publish(Twist())
            return

        now = self.get_clock().now()
        visible = (self.last_seen_t is not None and
                   (now - self.last_seen_t).nanoseconds / 1e9 < self.lost_timeout and
                   self.seen_count >= self.confirm_frames)

        dt_lost = (float('inf') if self.last_seen_t is None
                   else (now - self.last_seen_t).nanoseconds / 1e9)

        cmd = Twist()
        if not visible:
            # Si lo perdimos ACERCANDO hace poco: NOS DETENEMOS y esperamos a
            # re-detectarlo (parado la imagen se ve nitida -> lo recupera y sigue).
            # NO giramos, para no alejarnos apuntando.
            if self.state == 'ACERCANDO' and dt_lost < self.hold_on_lost:
                self.cmd_pub.publish(Twist())   # stop, mantener posicion
                self.get_logger().info(
                    f'Marcador perdido {dt_lost:.1f}s -> me detengo y espero re-deteccion...',
                    throttle_duration_sec=1.0)
                return
            # Ya paso la gracia (o nunca estuvo cerca): buscar girando hacia el
            # ultimo lado donde se vio.
            if self.state != 'BUSCANDO':
                self.get_logger().info('Marcador no recuperado -> BUSCANDO (giro en sitio).')
            self.state = 'BUSCANDO'
            cmd.angular.z = self.search_w * self.search_dir
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f'Buscando ArUco {self.target_id}...', throttle_duration_sec=3.0)
            return

        # ACERCANDO
        if self.state != 'ACERCANDO':
            self.get_logger().info(f'ArUco {self.target_id} encontrado. Acercando...')
        self.state = 'ACERCANDO'

        ang_err = (self.last_u - self.img_cx) / self.img_cx   # -1..1 (der = +)
        dist_err = self.last_dist - self.stop_distance        # >0 = aun lejos
        # recordar hacia donde girar si lo perdemos (hacia el lado donde estaba)
        if abs(ang_err) > 0.03:
            self.search_dir = 1.0 if ang_err > 0 else -1.0

        if abs(dist_err) < self.dist_tol and abs(ang_err) < self.center_tol:
            self.state = 'LLEGUE'
            self.cmd_pub.publish(Twist())
            self.get_logger().info(
                f'LLEGUE: ArUco {self.target_id} a {self.last_dist:.2f} m, centrado. Detenido.')
            return

        w = max(-self.w_max, min(self.w_max, -self.k_ang * ang_err))
        v = max(-self.v_max, min(self.v_max, self.k_lin * dist_err))
        # si esta muy descentrado, primero gira (avanza poco)
        if abs(ang_err) > 0.20:
            v *= 0.25
        # zona lenta cerca del objetivo: menos motion-blur -> no pierde el marcador
        if self.last_dist < self.slow_dist:
            v = max(-self.slow_v, min(self.slow_v, v))
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'Acercando: dist={self.last_dist:.2f}m err_centro={ang_err:+.2f} v={v:+.2f} w={w:+.2f}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = BuscarAruco()
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
