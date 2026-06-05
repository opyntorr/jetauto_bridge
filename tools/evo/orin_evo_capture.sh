#!/usr/bin/env bash
# orin_evo_capture.sh — COPIA versionada de ~/evo_capture.sh EN EL ORIN.
# Captura EVO en el Orin: ve OptiTrack (laptop2 .152, unicast) + TF/odom del bridge (loopback),
# todo con el reloj del Orin (un solo reloj -> APE asocia limpio). Lo invoca evo_run.sh por SSH.
# Uso: bash ~/evo_capture.sh [nombre] [rigid_body]   (default rigid_body=Robot)
# NOTA: sin 'set -u' a proposito -> los setup.bash de ROS usan vars no definidas y abortarian.
NAME="${1:-run_$(date +%Y%m%d_%H%M%S)}"
BODY="${2:-Robot}"
OUT="$HOME/evo_runs/$NAME"
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclone_evo.xml
echo "[evo] capturando en $OUT (gt-body=$BODY). Maneja el robot; Ctrl-C para terminar."
python3 "$HOME/evo_logger.py" --out-dir "$OUT" --gt-body "$BODY"
