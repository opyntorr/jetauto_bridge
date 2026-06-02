#!/usr/bin/env bash
# Despliega y compila en la Orin: nuestro stack (P1) + el cerebro AGV mi_proyecto_sim
# SIN Gazebo (P2). El driver del Tello (P3) es opcional y se corre con: deploy_agv_orin.sh tello
#
# Pensado para correr ANTES de cablear el robot (no necesita el hardware del JetAuto).
# El apt de la Orin usa su uplink Tec (no compite con el respaldo Nano->laptop).
set +e
ORIN=jetson@10.42.1.1
SRC=/home/opyntorr/agv_uav_project/src
DST=jetauto_ros2_ws/src        # overlay sobre nuestro port en la Orin

step() { echo; echo "########## $* ##########"; }

# ---------------------------------------------------------------- P1
step "P1: rsync jetauto_calibration + colcon build de NUESTRO stack"
rsync -az /home/opyntorr/jetauto_migration/jetauto_ros2_ws/src/jetauto_calibration/ \
  "$ORIN:jetauto_ros2_ws/src/jetauto_calibration/" && echo "rsync calibration OK"
ssh "$ORIN" 'source /opt/ros/humble/setup.bash && cd ~/jetauto_ros2_ws && colcon build 2>&1 | tail -20; echo "P1_BUILD_RC=${PIPESTATUS[0]}"'

# ---------------------------------------------------------------- P2
step "P2: desplegar cerebro mi_proyecto_sim (sin media pesada)"
rsync -az --exclude __pycache__ --exclude build --exclude install --exclude log \
  --exclude mision_output --exclude 'maps/occupancy_map.pgm' --exclude 'maps/mapa_mision.pgm' \
  --exclude 'maps/*.png' --exclude 'maps/*.pbstream' \
  "$SRC/mi_proyecto_sim" "$ORIN:$DST/" && echo "rsync brain OK"

echo "--- fix sin Gazebo: quitar dep ros_gz_bridge del package.xml ---"
ssh "$ORIN" "sed -i '/ros_gz_bridge/d' $DST/mi_proyecto_sim/package.xml && echo 'restantes ros_gz:' \$(grep -c ros_gz $DST/mi_proyecto_sim/package.xml)"

echo "--- deps apt (Orin) ---"
ssh "$ORIN" "echo jetson | sudo -S -p '' apt-get install -y \
  python3-opencv python3-transforms3d python3-scipy python3-yaml libopencv-dev \
  ros-humble-cv-bridge ros-humble-tf-transformations ros-humble-tf2-geometry-msgs \
  ros-humble-joy ros-humble-teleop-twist-joy ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager 2>&1 | tail -6; echo APT_RC=\$?"
echo "    (si APT_RC!=0 por indices viejos: ssh $ORIN 'echo jetson|sudo -S apt-get update' y reintentar)"
# --no-deps: NO jalar numpy/opencv-python por pip (rompen ROS Humble con numpy 2.x). Usa los del sistema (numpy 1.21.5 + cv2 4.5.4).
ssh "$ORIN" "pip3 install --no-deps djitellopy2 2>&1 | tail -3; echo PIP_RC=\$?"
ssh "$ORIN" "python3 -c 'import cv2,cv2.aruco,transforms3d,yaml,scipy; print(\"py deps OK, cv2\",cv2.__version__)' 2>&1 | tail -2"

echo "--- build del cerebro (incluye RRT C++) ---"
ssh "$ORIN" 'source /opt/ros/humble/setup.bash && cd ~/jetauto_ros2_ws && colcon build --packages-select mi_proyecto_sim 2>&1 | tail -35; echo "P2_BUILD_RC=${PIPESTATUS[0]}"'

echo "--- smoke test: ejecutables del cerebro registrados ---"
ssh "$ORIN" 'source /opt/ros/humble/setup.bash && source ~/jetauto_ros2_ws/install/setup.bash && ros2 pkg executables mi_proyecto_sim 2>&1 | head'

# ---------------------------------------------------------------- P3 (opcional: tello)
if [ "$1" = "tello" ]; then
  step "P3: desplegar + compilar driver Tello (iterativo; puede requerir ajustes)"
  # driver "main" (real, djitellopy2) + msgs + control, y tello_control_pos top-level
  rsync -az --exclude __pycache__ --exclude build --exclude install --exclude log \
    "$SRC/demo_tello_sim/src/tello-ros2-main/workspace/src/tello" \
    "$SRC/demo_tello_sim/src/tello-ros2-main/workspace/src/tello_msg" \
    "$SRC/demo_tello_sim/src/tello-ros2-main/workspace/src/tello_control" \
    "$SRC/tello_control_pos" \
    "$ORIN:$DST/" && echo "rsync tello OK"
  ssh "$ORIN" 'source /opt/ros/humble/setup.bash && cd ~/jetauto_ros2_ws && colcon build --packages-select tello tello_msg tello_control tello_control_pos 2>&1 | tail -40; echo "P3_BUILD_RC=${PIPESTATUS[0]}"'
fi

step "FIN"
