#!/usr/bin/env bash
# serializar_mapa.sh (vive en el Orin como ~/serializar_mapa.sh) ----------------------------------
# Serializa el mapa SLAM ACTUAL como pose-graph (.posegraph + .data) Y guarda el .pgm/.yaml fresco,
# ambos con el MISMO basename y origen, para usarlos en navegar_real (slam_toolbox modo
# localization). Correr MIENTRAS mapear_real esta mapeando (slam_toolbox vivo) y cuando el mapa
# ya se vea bien; NO sigas manejando entre los dos pasos (para que .pgm y .posegraph coincidan).
#
# Uso (desde la laptop):
#   ssh jetson@10.42.1.1 'bash ~/serializar_mapa.sh'                # -> mapa_frente208
#   ssh jetson@10.42.1.1 'bash ~/serializar_mapa.sh mapa_laberinto' # -> otro nombre
set -e
NAME="${1:-mapa_frente208}"
DIR=/home/jetson/maps

export HOME=/home/jetson
source /opt/ros/humble/setup.bash
source /home/jetson/jetauto_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds-orin.xml

echo "[1/2] serializando pose-graph -> $DIR/$NAME.posegraph (+ .data)"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$DIR/$NAME'}"

echo "[2/2] guardando occupancy grid -> $DIR/$NAME.pgm (+ .yaml)"
ros2 run mi_proyecto_sim guardar_mapa_slam.py --ros-args -p output_dir:="$DIR" -p map_name:="$NAME"

echo "OK. Artefactos de $NAME:"
ls -la "$DIR/$NAME".* 2>/dev/null
