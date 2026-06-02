# Imagen MÍNIMA para la Jetson Nano: capa de hardware del JetAuto en ROS2 Humble.
# Solo expone motores/IMU (I2C bus 1) + lidar + cámara, y los publica por DDS al Orin.
# NADA de EKF/SLAM/Nav2/cerebro (eso corre en el Orin).
#
# PRERREQUISITOS DEL HOST (Nano) — NO van en la imagen, son del kernel/udev del host:
#   1. Blacklist uvcvideo: /etc/modprobe.d/blacklist-orbbec-uvc.conf con "blacklist uvcvideo".
#      (uvcvideo NO puede bindear la Astra Pro Plus y deja la interfaz UVC ocupada;
#       el driver Orbbec la abre por libusb/libuvc, asi que uvcvideo debe NO cargar.)
#   2. udev rules de Orbbec: cp src/orbbec_camera/scripts/99-obsensor-libusb.rules
#      a /etc/udev/rules.d/ + udevadm control --reload + udevadm trigger.
#   El contenedor corre --privileged -v /dev:/dev, asi ve el USB de la camara.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-smbus2 \
      ros-humble-rplidar-ros \
      ros-humble-rmw-cyclonedds-cpp \
      python3-colcon-common-extensions \
      ros-humble-image-transport \
      ros-humble-image-transport-plugins \
      ros-humble-compressed-image-transport \
      ros-humble-image-publisher \
      ros-humble-camera-info-manager \
      ros-humble-diagnostic-updater \
      ros-humble-tf2-sensor-msgs \
      ros-humble-cv-bridge \
      libusb-1.0-0-dev \
      libgflags-dev \
      libgoogle-glog-dev \
      libeigen3-dev \
      nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# Config del bridge DDS (fija al ethernet eth0 + peer unicast al Orin)
COPY cyclonedds-nano.xml /cfg/cyclonedds.xml

# Workspace: jetauto_controller (i2c_bus=1) + jetauto_nano_bringup + driver Orbbec
# (orbbec_camera, orbbec_camera_msgs, orbbec_description = OrbbecSDK_ROS2 v1.9.7 legacy).
WORKDIR /nano_ws
COPY src/ /nano_ws/src/
# orbbec_camera se compila aparte: enlaza el libOrbbecSDK.so precompilado y necesita
# que el RPATH se hornee al linkear (-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON), porque el
# paso de RPATH_CHANGE post-install falla sobre el .so del SDK.
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --packages-skip orbbec_camera && \
    colcon build --packages-select orbbec_camera \
      --cmake-args -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
                   -DCMAKE_INSTALL_RPATH=/nano_ws/src/orbbec_camera/SDK/lib/arm64

ENV ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///cfg/cyclonedds.xml

CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && source /nano_ws/install/setup.bash && exec ros2 launch jetauto_nano_bringup nano_bringup.launch.py"]
