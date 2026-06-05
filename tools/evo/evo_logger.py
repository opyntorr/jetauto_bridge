#!/usr/bin/env python3
"""
evo_logger.py — captura trayectorias en formato TUM para evaluar SLAM/AMCL en el JetAuto REAL.

Con OptiTrack disponible HAY ground-truth continua -> APE/RPE de verdad. Este logger captura, en una
sola pasada y con UN MISMO reloj (el de la laptop, para que EVO asocie limpio entre maquinas):
  - est.tum         pose ESTIMADA  = TF est-frame->robot-frame (sirve para SLAM mapping y para AMCL)
  - gt.tum          GROUND TRUTH   = /optitrack/rigid_body (OptiTrack/Motive)
  - odom.tum        /odom (EKF)    (para comparar)
  - amcl_pose.tum   /amcl_pose     (solo en modo AMCL)
  - checkpoints.csv label,t,x,...  (Enter marca la pose estimada actual)
  - summary.txt     resumen + drift de cierre de lazo + comandos evo_ape/evo_rpe listos

Corre en la LAPTOP con el entorno del bridge cargado (ve /tf, /odom, /amcl_pose y /optitrack por DDS):
    source ~/jetauto_rviz_env.sh
    python3 evo_logger.py --out-dir ~/evo_runs/run1
    # opciones: --est-frame map --robot-frame base_footprint --rate 20
    #           --gt-topic /optitrack/rigid_body --gt-body <NOMBRE_RIGID_BODY_EN_MOTIVE>

Si no pasas --gt-body, el logger se "engancha" al PRIMER rigid body que llegue y te dice su nombre;
si tienes varios en Motive, pasa --gt-body con el del robot.

Con GT (OptiTrack) el analisis fuerte es APE/RPE alineados (evo_ape/evo_rpe -a). El drift de cierre de
lazo sigue saliendo como verificacion independiente.
Formato TUM: timestamp tx ty tz qx qy qz qw     |     Terminar: Ctrl-C.
"""

import argparse
import math
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data

import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


def _yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _tum_line(t, px, py, pz, qx, qy, qz, qw):
    return f"{t:.6f} {px:.6f} {py:.6f} {pz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n"


class EvoLogger(Node):
    def __init__(self, args):
        super().__init__('evo_logger')
        self.est_frame = args.est_frame
        self.robot_frame = args.robot_frame
        self.gt_topic = args.gt_topic
        self.gt_body = args.gt_body
        self.out_dir = os.path.expanduser(args.out_dir)
        os.makedirs(self.out_dir, exist_ok=True)

        self._lock = threading.Lock()
        self._latest_est = None          # (t, x, y, z, qx, qy, qz, qw)
        self._first_est = None
        self._last_est = None
        self._path_len = 0.0
        self._prev_xy = None
        self._n_est = 0
        self._n_gt = 0
        self._n_odom = 0
        self._n_amcl = 0
        self._n_cp = 0
        self._tf_warned = 0
        self._gt_locked_name = args.gt_body or None
        self._gt_names_seen = set()

        # Archivos de salida
        self.f_est = open(os.path.join(self.out_dir, 'est.tum'), 'w')
        self.f_gt = open(os.path.join(self.out_dir, 'gt.tum'), 'w')
        self.f_odom = open(os.path.join(self.out_dir, 'odom.tum'), 'w')
        self.f_amcl = open(os.path.join(self.out_dir, 'amcl_pose.tum'), 'w')
        self.f_cp = open(os.path.join(self.out_dir, 'checkpoints.csv'), 'w')
        self.f_cp.write('label,t,x,y,z,qx,qy,qz,qw,yaw_deg\n')
        self.f_cp.flush()

        # TF (pose estimada)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscripciones
        self.create_subscription(Odometry, '/odom', self._odom_cb, 20)
        amcl_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, amcl_qos)
        # OptiTrack publica con SensorDataQoS (best effort)
        self.create_subscription(PoseStamped, self.gt_topic, self._gt_cb, qos_profile_sensor_data)

        self.period = 1.0 / float(args.rate)
        self.create_timer(self.period, self._sample_est)

        # Hilo lector de stdin para marcar checkpoints
        self._stdin_thread = threading.Thread(target=self._stdin_loop, daemon=True)
        self._stdin_thread.start()

        gt_desc = f"'{self.gt_body}'" if self.gt_body else "(primer rigid body que llegue)"
        self.get_logger().info(
            f"evo_logger: est={self.est_frame}->{self.robot_frame} @ {args.rate}Hz -> {self.out_dir}\n"
            f"  GT (OptiTrack): topic={self.gt_topic} body={gt_desc}\n"
            f"  Enter = marca checkpoint | Ctrl-C = termina y escribe summary.txt + comandos evo.")

    def _t(self):
        # Reloj del logger (laptop). Sella TODO con el mismo reloj -> EVO asocia gt/est sin depender
        # de la sincronia entre laptop y Orin.
        return self.get_clock().now().nanoseconds * 1e-9

    # --- pose estimada por TF ---
    def _sample_est(self):
        try:
            tr = self.tf_buffer.lookup_transform(
                self.est_frame, self.robot_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self._tf_warned += 1
            if self._tf_warned in (20, 100) or self._tf_warned % 400 == 0:
                self.get_logger().warn(
                    f"sin TF {self.est_frame}->{self.robot_frame} aun ({e}). "
                    f"Arranca SLAM/AMCL y (en AMCL) da 2D Pose Estimate.")
            return
        t = self._t()
        tx = tr.transform.translation.x
        ty = tr.transform.translation.y
        tz = tr.transform.translation.z
        q = tr.transform.rotation
        self.f_est.write(_tum_line(t, tx, ty, tz, q.x, q.y, q.z, q.w))
        self._n_est += 1
        if self._n_est % 20 == 0:
            self.f_est.flush()
        pose = (t, tx, ty, tz, q.x, q.y, q.z, q.w)
        with self._lock:
            self._latest_est = pose
            if self._first_est is None:
                self._first_est = pose
            self._last_est = pose
            if self._prev_xy is not None:
                self._path_len += math.hypot(tx - self._prev_xy[0], ty - self._prev_xy[1])
            self._prev_xy = (tx, ty)

    # --- ground truth OptiTrack ---
    def _gt_cb(self, msg: PoseStamped):
        name = msg.header.frame_id
        self._gt_names_seen.add(name)
        if self.gt_body:
            if name != self.gt_body:
                return
        else:
            if self._gt_locked_name is None:
                self._gt_locked_name = name
                self.get_logger().info(
                    f"GT enganchado al rigid body '{name}'. Si NO es el robot, relanza con "
                    f"--gt-body NOMBRE.")
            if name != self._gt_locked_name:
                return
        p = msg.pose.position
        o = msg.pose.orientation
        self.f_gt.write(_tum_line(self._t(), p.x, p.y, p.z, o.x, o.y, o.z, o.w))
        self._n_gt += 1
        if self._n_gt % 20 == 0:
            self.f_gt.flush()

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.f_odom.write(_tum_line(self._t(), p.x, p.y, p.z, o.x, o.y, o.z, o.w))
        self._n_odom += 1
        if self._n_odom % 20 == 0:
            self.f_odom.flush()

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.f_amcl.write(_tum_line(self._t(), p.x, p.y, p.z, o.x, o.y, o.z, o.w))
        self._n_amcl += 1
        self.f_amcl.flush()

    # --- checkpoints por stdin ---
    def _stdin_loop(self):
        try:
            for line in sys.stdin:
                label = line.strip() or f"cp{self._n_cp + 1}"
                with self._lock:
                    pose = self._latest_est
                if pose is None:
                    self.get_logger().warn("checkpoint ignorado: aun no hay pose estimada.")
                    continue
                t, x, y, z, qx, qy, qz, qw = pose
                yaw_deg = math.degrees(_yaw_from_quat(qx, qy, qz, qw))
                self.f_cp.write(f"{label},{t:.6f},{x:.6f},{y:.6f},{z:.6f},"
                                f"{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f},{yaw_deg:.3f}\n")
                self.f_cp.flush()
                self._n_cp += 1
                self.get_logger().info(f"checkpoint '{label}': x={x:.3f} y={y:.3f} yaw={yaw_deg:.1f}deg")
        except Exception:
            pass

    # --- cierre / resumen ---
    def finish(self):
        L = []
        L.append(f"out_dir: {self.out_dir}")
        L.append(f"est poses (TF {self.est_frame}->{self.robot_frame}): {self._n_est}")
        L.append(f"gt  poses (OptiTrack {self.gt_topic}, body={self._gt_locked_name}): {self._n_gt}")
        L.append(f"odom poses (/odom): {self._n_odom}")
        L.append(f"amcl_pose poses (/amcl_pose): {self._n_amcl}")
        L.append(f"checkpoints: {self._n_cp}")
        L.append(f"rigid bodies vistos en OptiTrack: {sorted(self._gt_names_seen) or '(ninguno)'}")
        L.append(f"path length (est): {self._path_len:.3f} m")
        if self._first_est and self._last_est and self._n_est >= 2:
            x0, y0 = self._first_est[1], self._first_est[2]
            x1, y1 = self._last_est[1], self._last_est[2]
            yaw0 = _yaw_from_quat(*self._first_est[4:8])
            yaw1 = _yaw_from_quat(*self._last_est[4:8])
            d = math.hypot(x1 - x0, y1 - y0)
            dyaw = math.degrees(abs(_wrap(yaw1 - yaw0)))
            L.append("")
            L.append("=== CIERRE DE LAZO (verificacion, asume vuelta al START fisico) ===")
            L.append(f"  DRIFT posicion = {d:.3f} m   |   DRIFT yaw = {dyaw:.2f} deg")
            if self._path_len > 0:
                L.append(f"  drift / path = {100.0 * d / self._path_len:.2f} %")
        L.append("")
        if self._n_gt > 0 and self._n_est > 0:
            est = os.path.join(self.out_dir, 'est.tum')
            gt = os.path.join(self.out_dir, 'gt.tum')
            L.append("=== METRICAS CON GROUND TRUTH (OptiTrack) — corre en el venv (evo) ===")
            L.append("  source ~/evo_venv/bin/activate")
            L.append(f"  evo_ape tum {gt} {est} -a -p --plot_mode xy")
            L.append(f"  evo_rpe tum {gt} {est} -a -p --plot_mode xy")
            L.append("  (anade --save_results rondaN.zip y compara con: evo_res ronda*.zip -p)")
        elif self._n_gt > 0 and self._n_est == 0:
            L.append("GT capturada, pero NO hubo pose estimada (est=0).")
            L.append("  Corre SLAM/AMCL y, en AMCL, da el 2D Pose Estimate para que exista map->base.")
        else:
            L.append("AVISO: no se capturo GT de OptiTrack (gt poses = 0).")
            L.append("  Revisa que el optitrack_client corra, que Motive este streameando y el --gt-body.")
        txt = "\n".join(L) + "\n"
        with open(os.path.join(self.out_dir, 'summary.txt'), 'w') as f:
            f.write(txt)
        for fh in (self.f_est, self.f_gt, self.f_odom, self.f_amcl, self.f_cp):
            try:
                fh.flush(); fh.close()
            except Exception:
                pass
        print("\n" + txt)


def main():
    ap = argparse.ArgumentParser(description="Logger de trayectorias TUM para EVO (JetAuto real, GT OptiTrack).")
    ap.add_argument('--out-dir', required=True, help="carpeta de salida (p.ej. ~/evo_runs/run1)")
    ap.add_argument('--est-frame', default='map', help="frame global de la pose estimada (default: map)")
    ap.add_argument('--robot-frame', default='base_footprint', help="frame del robot (default: base_footprint)")
    ap.add_argument('--rate', type=float, default=20.0, help="Hz de muestreo de la TF (default: 20)")
    ap.add_argument('--gt-topic', default='/optitrack/rigid_body',
                    help="topic del OptiTrack (default: /optitrack/rigid_body)")
    ap.add_argument('--gt-body', default='',
                    help="nombre del rigid body del robot en Motive (default: el primero que llegue)")
    args = ap.parse_args()

    rclpy.init()
    node = EvoLogger(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
