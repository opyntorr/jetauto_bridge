#!/usr/bin/env bash
# evo_run.sh — UN comando (desde tu laptop) para medir la localizacion con EVO + GT de OptiTrack.
#
# Arquitectura (por la topologia de red): la CAPTURA corre EN EL ORIN, que es el unico que ve a la vez
# el OptiTrack (publicado por laptop2 en ASUS_38) y el TF/odom del robot. Este script hace SSH al Orin,
# corre el logger alli, y al terminar TRAE los .tum a tu laptop para analizarlos con evo.
#
# Uso (en tu laptop):
#     ~/evo_run.sh [nombre] [rigid_body]      (default rigid_body = Robot)
#
# Precondiciones:
#   1) Localizacion corriendo (desde tu laptop, en OTRA terminal):
#        SLAM:  ros2 launch jetauto_rviz mapear_real.launch.py
#        AMCL:  ros2 launch jetauto_rviz navegar_real_amcl.launch.py map:=/home/jetson/maps/mi_mapa.yaml teleop:=true  (+ 2D Pose Estimate)
#   2) optitrack_client corriendo en laptop2 (cyclone+dom0, interfaz ASUS fijada -> ver cyclone_laptop2_optitrack.xml).
#
# Flujo: corre esto -> maneja el recorrido (cierra lazo si quieres drift) -> Ctrl-C aqui.
set -u

NAME="${1:-run_$(date +%Y%m%d_%H%M%S)}"
BODY="${2:-Robot}"
ORIN="${ORIN_HOST:-jetson@10.42.1.1}"

echo "============================================================"
echo " EVO run: $NAME   (gt-body=$BODY)   captura EN EL ORIN"
echo " ANTES: localizacion (SLAM/AMCL) corriendo + optitrack en laptop2."
echo " AHORA: maneja el recorrido. Ctrl-C aqui para terminar."
echo " (Enter marca un checkpoint.)"
echo "============================================================"

# -tt = TTY: Ctrl-C y Enter (checkpoints) se propagan al logger remoto.
ssh -tt -o ConnectTimeout=10 -o StrictHostKeyChecking=no "$ORIN" "bash ~/evo_capture.sh '$NAME' '$BODY'"

echo
echo "Trayendo los .tum del Orin a la laptop..."
mkdir -p "$HOME/evo_runs"
scp -r -o ConnectTimeout=20 "$ORIN:evo_runs/$NAME" "$HOME/evo_runs/" 2>&1 | tail -1

RUN="$HOME/evo_runs/$NAME"
# Calibra orientacion (offset constante rigid-body->base_footprint) y descarta flips de OptiTrack
# -> gt_corr.tum. Usa el venv (evo+scipy). Necesita gt.tum y est.tum con datos.
GT_FOR_EVO="$RUN/gt.tum"
if [ -s "$RUN/gt.tum" ] && [ -s "$RUN/est.tum" ]; then
  echo
  echo "Calibrando orientacion (gt_corr.tum)..."
  "$HOME/evo_venv/bin/python" "$HOME/jetauto_migration/tools/evo/evo_calib_orientation.py" \
      --gt "$RUN/gt.tum" --est "$RUN/est.tum" --out "$RUN/gt_corr.tum" 2>&1 | sed 's/^/  /'
  [ -s "$RUN/gt_corr.tum" ] && GT_FOR_EVO="$RUN/gt_corr.tum"
fi

echo
echo "Listo: $RUN"
echo "Analiza (en el venv) — usa gt_corr.tum (orientacion corregida, sin flips):"
echo "  source ~/evo_venv/bin/activate"
echo "  evo_ape tum $GT_FOR_EVO $RUN/est.tum -a --save_results $RUN.zip            # traslacion (m)"
echo "  evo_ape tum $GT_FOR_EVO $RUN/est.tum -a -r angle_deg                       # heading (deg)"
echo "  evo_rpe tum $GT_FOR_EVO $RUN/est.tum -a --delta 1 --delta_unit m           # drift local (m/m)"
echo "  evo_ape tum $GT_FOR_EVO $RUN/est.tum -a --plot_mode xy --save_plot $RUN.png  # grafica"
echo "  # comparar rondas:  evo_res ~/evo_runs/*.zip -p"
