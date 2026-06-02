# jetauto_bridge — stack del robot real JetAuto (Orin + Nano)

Firmware/plataforma del robot **JetAuto** en su arquitectura **bridge**: el Jetson Nano
mantiene el hardware (motores I2C, IMU, LiDAR, cámara) en un contenedor ROS2 Humble, y el
Jetson Orin hace el cómputo (RSP, EKF, SLAM, Nav2). Ambos hablan por DDS sobre el ethernet
Orin↔Nano.

> El **cliente** (teleop, RViz, lanzadores desde la laptop) y el **cerebro AGV**
> (`mi_proyecto_sim`) viven en el repo `port_bot_sim_ws` (rama `jetauto`). Este repo es solo
> la plataforma del robot.

## Estructura

| Carpeta | Qué es |
|---|---|
| `jetauto_ros2_ws/src/` | Workspace ROS2 del **Orin** (8 paquetes): `jetauto_bringup`, `jetauto_controller` (driver I2C 0x34 + IMU + EKF), `jetauto_description` (URDF), `jetauto_slam`, `jetauto_navigation`, `jetauto_calibration`, `jetauto_interfaces`, `jetauto_app`. |
| `nano_bridge/` | Build-context Docker del **Nano**: `Dockerfile`, `jetauto_nano_bringup`, configs CycloneDDS, `jetauto-nano.service`, scripts de caracterización (motor/imu/lidar). |
| `orin_systemd/` | Units systemd del Orin. |
| `nano/jetauto_ws_src/` | Código ROS1 Melodic **original de fábrica** (Hiwonder), extraído del Nano — referencia del port. |
| `nano/cfg`, `nano/sys` | Configs y reglas udev/systemd originales del Nano. |
| `deploy_agv_orin.sh` | Script de despliegue al Orin. |

## NO versionado (ver `.gitignore`)

- `nano_backup/` — imagen de disco completa del Nano (~12 GB).
- `OrbbecSDK_ROS2*/` — SDK de cámara de terceros (repo propio).
- `nano/home/` — volcado del home del Nano (artefactos de build, no código).
- `build/`, `install/`, `log/`, mapas, bags, imágenes.

## Hardware (resumen)

- **Motores:** driver Hiwonder 4-ch a **I2C 0x34** (no encoders en uso; odom = dead-reckoning
  del cmd_vel con `go_factor`/`turn_factor`).
- **IMU:** MPU6050 a **I2C 0x68**.
- **LiDAR:** RPLIDAR A1 (`/dev/lidar`, 12 m).
- **Cámara:** Orbbec Astra Pro (RGB, `/cam_1/image`).
- Filtro de seguridad de motores en el driver (clamp + rampa + watchdog).

Detalle del bridge, DDS, relojes y arranque: ver los comentarios en `nano_bridge/` y
`jetauto_ros2_ws/src/jetauto_bringup`.
