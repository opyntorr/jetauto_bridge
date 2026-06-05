# Métricas EVO para SLAM / AMCL — JetAuto REAL (ground truth OptiTrack)

Evaluar la localización del robot real con números objetivos: **EVO** (APE/RPE) usando **OptiTrack**
como **ground truth** continuo.

## Arquitectura (por la topología de red)
Hay DOS islas DDS y el **Orin es el único en ambas**, así que la **captura corre en el Orin**:

```
laptop2 (ASUS_38, 192.168.0.152)         tu laptop (MiJetson_AP, 10.42.1.78)
  optitrack_client                          ros2 launch ... (localizacion + RViz)
  -> /optitrack/rigid_body (120 Hz)         ~/evo_run.sh  --ssh-->  ORIN
            \                                                         |
             \---- ASUS_38 (unicast) ----> ORIN (192.168.0.151) <----/
                                             evo_capture.sh -> evo_logger.py
                                             ve /optitrack (peer .152) + /tf,/odom (loopback)
                                             escribe gt.tum + est.tum + odom.tum  (un solo reloj)
                                             scp de vuelta a tu laptop -> evo (venv)
```

- **est.tum** = TF `map → base_footprint` (SLAM mapping o AMCL).
- **gt.tum**  = OptiTrack (rigid body, p.ej. `Robot`).
- **odom.tum**= `/odom` (EKF). **amcl_pose.tum** = `/amcl_pose` (modo AMCL).

Por qué en el Orin: tu laptop (MiJetson_AP) NO está en la red del OptiTrack (ASUS_38); el Orin sí, y
además tiene el robot. Una sola config DDS en el Orin (`~/cyclone_evo.xml`) ve OptiTrack por **peer
unicast** a `192.168.0.152` (el multicast entre clientes WiFi de ASUS_38 está bloqueado) y el `/tf`+
`/odom` locales por **loopback**. Todo se sella con el reloj del Orin → APE asocia sin problemas.

## Requisitos
- **evo** en venv aislado en tu laptop: `source ~/evo_venv/bin/activate` (recrear: `python3 -m venv
  ~/evo_venv && ~/evo_venv/bin/pip install evo`).
- **laptop2**: el `optitrack_client` corriendo con DDS del bridge y la interfaz ASUS fijada. Lanzarlo:
  ```bash
  # config (una vez): copia cyclone_laptop2_optitrack.xml a ~/cyclone_optitrack.xml en laptop2
  export ROS_DOMAIN_ID=0
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file://$HOME/cyclone_optitrack.xml
  ros2 run optitrack_client optitrack_client
  ```
  Verifica en laptop2: `ros2 topic hz /optitrack/rigid_body` (~120 Hz). El rigid body del robot se
  llama **Robot** (header.frame_id).
- **Orin** (ya desplegado): `~/evo_logger.py`, `~/cyclone_evo.xml`, `~/evo_capture.sh`.

## Medición — un comando (en tu laptop)
1. **Terminal 1** — localización (lo de siempre):
   ```bash
   ros2 launch jetauto_rviz mapear_real.launch.py                  # SLAM
   #  o:  ros2 launch jetauto_rviz navegar_real_amcl.launch.py map:=/home/jetson/maps/mi_mapa.yaml teleop:=true   # AMCL (+2D Pose Estimate)
   ```
2. **Terminal 2** — captura (SSH al Orin + trae los .tum):
   ```bash
   ~/evo_run.sh slam_ronda1 Robot
   #            ^nombre      ^rigid body (default Robot)
   ```
   Maneja el recorrido (cierra lazo si quieres el drift). **Ctrl-C** termina, y los `.tum` se copian a
   `~/evo_runs/slam_ronda1/` en tu laptop. (Enter durante la captura marca un checkpoint.)

## Calibración de orientación (automática)
El rigid body de OptiTrack tiene su marco local definido por los marcadores, que NO coincide con
`base_footprint` (aquí: yaw ~177°). Eso NO afecta la traslación, pero **corrompe rotación APE y RPE**;
además OptiTrack a veces invierte el rigid body 180° ("flips") en pérdidas de tracking. `evo_run.sh`
corre solo `evo_calib_orientation.py` tras traer los datos: estima el offset constante, lo aplica,
**descarta los flips** y escribe **`gt_corr.tum`**. → **Analiza siempre con `gt_corr.tum`.**
> Raíz del problema (flips): usa marcadores ASIMÉTRICOS + buena cobertura de cámaras en Motive.

## Análisis (en tu laptop, venv) — usa gt_corr.tum
```bash
source ~/evo_venv/bin/activate
R=~/evo_runs/slam_ronda1
evo_ape tum $R/gt_corr.tum $R/est.tum -a --save_results $R.zip        # traslacion (m) = precision
evo_ape tum $R/gt_corr.tum $R/est.tum -a -r angle_deg                 # heading (grados)
evo_rpe tum $R/gt_corr.tum $R/est.tum -a --delta 1 --delta_unit m     # drift local (m por m)
evo_ape tum $R/gt_corr.tum $R/est.tum -a --plot_mode xy --save_plot $R.png   # grafica
```
- `-a` = alineación Umeyama: alinea gt y est aunque estén en frames distintos. APE traslación en
  **metros**, rotación en **grados**. Menos = mejor (mira `rmse`; usa `median` si hay outliers).
- ⚠️ Para **guardar** usa `--save_results`/`--save_plot`, NO `-p` (la ventana interactiva interfiere).
- Útil: `evo_ape tum $R/gt_corr.tum $R/odom.tum -a` → error de la odom cruda (cuánto corrige SLAM/AMCL).

## Comparar configuraciones (método de tuning)
Una ronda = UN grupo de parámetros. Mismo recorrido. `.zip` por ronda y:
```bash
evo_res ~/evo_runs/slam_ronda0.zip ~/evo_runs/slam_ronda1.zip ... -p
```
Rankea por menor APE (RMSE) y menor drift de cierre de lazo (en `summary.txt`).

## SLAM vs AMCL (overlay contra GT)
```bash
evo_traj tum ~/evo_runs/slam/est.tum ~/evo_runs/amcl/est.tum ~/evo_runs/slam/odom.tum \
    --ref ~/evo_runs/slam/gt.tum -p --plot_mode xy
```

## Diagnóstico
- "**Degenerate covariance ... Umeyama**" en `evo_ape -a`: el robot casi no se movió (trayectoria
  ~estática). Maneja un recorrido con extensión y reintenta. (Sin `-a` corre pero el número no sirve.)
- `gt poses = 0` en `summary.txt`: el Orin no recibió OptiTrack. Revisa que laptop2 publique con
  cyclone+dom0+interfaz ASUS fijada, y que su IP siga siendo `192.168.0.152` (si cambió, actualiza el
  peer en `~/cyclone_evo.xml` del Orin y el de laptop2). Test desde el Orin:
  `CYCLONEDDS_URI=file://~/cyclone_evo.xml ros2 topic hz /optitrack/rigid_body`.
- `est poses = 0`: no hay TF `map→base` → corre SLAM/AMCL y (en AMCL) da el 2D Pose Estimate.

## Archivos
- En tu laptop: `~/evo_run.sh` (= `tools/evo/evo_run.sh`), `tools/evo/evo_logger.py`,
  `evo_checkpoints.py`, las configs de referencia.
- En el Orin: `~/evo_logger.py`, `~/cyclone_evo.xml`, `~/evo_capture.sh`.
- En laptop2: `~/cyclone_optitrack.xml` (= `tools/evo/cyclone_laptop2_optitrack.xml`).

## Parámetros a tunear (ver el plan)
- **SLAM** `jetauto_slam/config/mapper_params_online_async.yaml`; **AMCL**
  `localizacion_nav_amcl_bridge.launch.py`. Un grupo por ronda, despliega al Orin, mismo recorrido.
