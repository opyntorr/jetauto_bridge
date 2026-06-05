#!/usr/bin/env python3
"""
evo_calib_orientation.py — calibra el offset de orientacion entre el rigid body de OptiTrack y el
base_footprint del robot, descarta los "flips" de OptiTrack, y reescribe gt -> gt_corr.tum (limpio).

Por que: el rigid body de Motive tiene su marco local definido por como pusiste los marcadores, que NO
coincide con base_footprint (aqui: un yaw de ~177 deg). Eso NO afecta la traslacion, pero corrompe la
rotacion APE y el RPE. Ademas, en perdidas momentaneas de tracking Motive a veces INVIERTE el rigid
body 180 deg ("flips") -> ground truth de orientacion corrupto en esas muestras.

Este script: (1) sincroniza y alinea est->gt en posicion; (2) estima el offset CONSTANTE de orientacion
de forma robusta (2 pasadas, ignorando flips); (3) escribe gt_corr.tum con la orientacion corregida y
SIN las muestras flip. Asi gt_corr representa la orientacion real del robot y las metricas de rotacion
quedan validas.

Uso (en el venv):
    source ~/evo_venv/bin/activate
    python3 evo_calib_orientation.py --gt run/gt.tum --est run/est.tum --out run/gt_corr.tum \
        [--save-offset ~/evo_gt_offset.txt] [--flip-thresh 90]
Luego:
    evo_ape tum run/gt_corr.tum run/est.tum -a -r angle_deg     # heading (rotacion) ya valido
    evo_rpe tum run/gt_corr.tum run/est.tum -a --delta 1 --delta_unit m
    evo_ape tum run/gt_corr.tum run/est.tum -a                  # traslacion (igual que con gt original)

NOTA: para eliminar los flips de raiz, define el rigid body en Motive con marcadores ASIMETRICOS y buena
cobertura de camaras (asi Motive nunca confunde frente/atras).
"""

import argparse
import numpy as np
from evo.tools import file_interface
from evo.core import sync
from evo.core.trajectory import PoseTrajectory3D
from scipy.spatial.transform import Rotation as Rot


def wxyz_to_scipy(q):
    return Rot.from_quat(np.column_stack([q[:, 1], q[:, 2], q[:, 3], q[:, 0]]))


def scipy_to_wxyz(r):
    q = r.as_quat()  # (x,y,z,w)
    return np.column_stack([q[:, 3], q[:, 0], q[:, 1], q[:, 2]])


def main():
    ap = argparse.ArgumentParser(description="Calibra orientacion gt (OptiTrack) -> base_footprint y descarta flips.")
    ap.add_argument('--gt', required=True)
    ap.add_argument('--est', required=True)
    ap.add_argument('--out', required=True, help="salida gt_corr.tum (sincronizado, corregido, sin flips)")
    ap.add_argument('--save-offset', default='', help="opcional: guarda el quaternion del offset (xyzw)")
    ap.add_argument('--flip-thresh', type=float, default=90.0, help="deg sobre el offset = flip a descartar")
    ap.add_argument('--max-diff', type=float, default=0.05)
    a = ap.parse_args()

    gt = file_interface.read_tum_trajectory_file(a.gt)
    est = file_interface.read_tum_trajectory_file(a.est)
    gt, est = sync.associate_trajectories(gt, est, max_diff=a.max_diff)
    est.align(gt)  # alinea est->gt en posicion (Umeyama) para medir la rotacion residual

    Rg = wxyz_to_scipy(gt.orientations_quat_wxyz)
    Re = wxyz_to_scipy(est.orientations_quat_wxyz)
    Q = Rg.inv() * Re  # ~ offset constante (mas flips)

    # offset robusto: media, descarta outliers >thresh, recalcula
    Qavg = Q.mean()
    dev = np.degrees((Qavg.inv() * Q).magnitude())
    inlier = dev <= a.flip_thresh
    Qavg = Q[inlier].mean()
    dev = np.degrees((Qavg.inv() * Q).magnitude())
    inlier = dev <= a.flip_thresh

    rpy = Qavg.as_euler('xyz', degrees=True)
    nflip = int((~inlier).sum())
    di = dev[inlier]
    print(f"asociados: {len(Q)} | offset R_base_optibody rpy(deg)= roll {rpy[0]:.1f} pitch {rpy[1]:.1f} yaw {rpy[2]:.1f}")
    print(f"FLIPS descartados (>{a.flip_thresh:.0f} deg): {nflip} = {100*nflip/len(Q):.1f}%")
    print(f"heading error inliers (= rotacion APE esperada): mediana {np.median(di):.2f} | mean {di.mean():.2f} | p95 {np.percentile(di,95):.2f} deg")

    # escribe gt_corr: muestras INLIER, posicion del gt, orientacion = R_gt * Qavg
    Rg_corr = (Rg * Qavg)[inlier]
    gt_corr = PoseTrajectory3D(
        positions_xyz=gt.positions_xyz[inlier],
        orientations_quat_wxyz=scipy_to_wxyz(Rg_corr),
        timestamps=gt.timestamps[inlier])
    file_interface.write_tum_trajectory_file(a.out, gt_corr, confirm_overwrite=False)
    print(f"escrito: {a.out}  ({inlier.sum()} poses limpias)")

    if a.save_offset:
        q = Qavg.as_quat()  # xyzw
        np.savetxt(a.save_offset, q.reshape(1, 4), header='qx qy qz qw  (R_base_optibody; gt_corr = gt * Q)')
        print(f"offset guardado: {a.save_offset}")


if __name__ == '__main__':
    main()
