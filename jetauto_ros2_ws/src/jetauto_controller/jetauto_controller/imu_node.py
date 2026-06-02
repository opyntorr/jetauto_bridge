#!/usr/bin/env python3
# encoding: utf-8
"""
ROS2 IMU node: reads the MPU-6050 over I2C and publishes sensor_msgs/Imu (raw).
Orientation is left empty (covariance[0] = -1); fuse with imu_filter_madgwick to get
orientation on /imu/data. Port of mpu_6050_driver/imu_node.py + jetauto_sdk/imu.py.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from .mpu6050 import MPU6050


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('freq', 50.0)
        self.declare_parameter('calibrate_gyro', True)
        self.declare_parameter('gyro_calib_samples', 200)

        i2c_bus = self.get_parameter('i2c_bus').value
        self.frame_id = self.get_parameter('frame_id').value
        freq = float(self.get_parameter('freq').value)

        try:
            self.imu = MPU6050(i2c_bus=i2c_bus)
        except Exception as e:
            self.get_logger().error(
                f'No pude abrir el IMU (MPU6050 0x68) en I2C bus {i2c_bus}: {e}')
            raise

        # Auto-calibracion de bias de gyro al arranque (robot QUIETO). Resta el drift
        # constante del MPU6050 (medido ~0.023 rad/s en z = 1.3 deg/s sin corregir).
        import time as _t
        self.gbx = self.gby = self.gbz = 0.0
        if self.get_parameter('calibrate_gyro').value:
            n = int(self.get_parameter('gyro_calib_samples').value)
            sx = sy = sz = 0.0
            got = 0
            for _ in range(n):
                try:
                    _, _, _, gx, gy, gz = self.imu.read()
                    sx += gx; sy += gy; sz += gz; got += 1
                except OSError:
                    pass
                _t.sleep(1.0 / freq)
            if got > 0:
                self.gbx, self.gby, self.gbz = sx / got, sy / got, sz / got
            self.get_logger().info(
                'gyro bias (rad/s): x=%.5f y=%.5f z=%.5f (n=%d, el robot debia estar quieto)'
                % (self.gbx, self.gby, self.gbz, got))

        self.pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.create_timer(1.0 / freq, self.tick)
        self.get_logger().info(f'imu_node listo (I2C bus {i2c_bus}, {freq:.0f} Hz)')

    def tick(self):
        try:
            ax, ay, az, gx, gy, gz = self.imu.read()
        except OSError as e:
            self.get_logger().warn(f'I2C IMU read fallo: {e}')
            return
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = gx - self.gbx
        msg.angular_velocity.y = gy - self.gby
        msg.angular_velocity.z = gz - self.gbz
        msg.orientation_covariance[0] = -1.0  # orientation unknown (sensor_msgs/Imu convention)
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.imu.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
