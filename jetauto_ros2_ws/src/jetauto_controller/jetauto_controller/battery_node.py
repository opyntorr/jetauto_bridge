#!/usr/bin/env python3
# encoding: utf-8
"""
Publica el voltaje de la bateria del JetAuto leido de la placa de motores Hiwonder
(I2C 0x34, registro 0 = uint16 little-endian en mV). Corre en el Nano (acceso local al bus).

Publica:
  /battery_voltage  (std_msgs/Float32)        -> voltios
  /battery_state    (sensor_msgs/BatteryState) -> voltage + percentage (para RViz/herramientas)

La bateria del JetAuto es 3S LiPo (12.6V llena / 9.9V vacia).
"""
import struct
import smbus2

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState


class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('addr', 0x34)
        self.declare_parameter('reg', 0)        # registro de voltaje (uint16 mV)
        self.declare_parameter('rate', 1.0)     # Hz
        self.declare_parameter('cells', 3)      # 3S LiPo
        bus = self.get_parameter('i2c_bus').value
        self.addr = int(self.get_parameter('addr').value)
        self.reg = int(self.get_parameter('reg').value)
        cells = int(self.get_parameter('cells').value)
        rate = float(self.get_parameter('rate').value)
        self.v_full = 4.2 * cells
        self.v_empty = 3.3 * cells

        try:
            self.bus = smbus2.SMBus(bus)
        except Exception as e:
            self.get_logger().error(f'No pude abrir I2C bus {bus}: {e}')
            raise

        self.pub_v = self.create_publisher(Float32, 'battery_voltage', 10)
        self.pub_bs = self.create_publisher(BatteryState, 'battery_state', 10)
        self._warned_low = False
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'battery_node listo (I2C 0x{self.addr:02X} reg {self.reg}, {cells}S). '
            f'Publica /battery_voltage y /battery_state.')

    def tick(self):
        try:
            data = self.bus.read_i2c_block_data(self.addr, self.reg, 2)
            mv = struct.unpack('<H', bytes(data))[0]
        except OSError as e:
            self.get_logger().warn(f'lectura de bateria fallo: {e}', throttle_duration_sec=5.0)
            return
        v = mv / 1000.0
        # filtro de plausibilidad: ignorar lecturas absurdas (ruido I2C)
        if not (5.0 < v < 14.0):
            return

        self.pub_v.publish(Float32(data=v))

        bs = BatteryState()
        bs.header.stamp = self.get_clock().now().to_msg()
        bs.voltage = v
        bs.percentage = max(0.0, min(1.0, (v - self.v_empty) / (self.v_full - self.v_empty)))
        bs.present = True
        self.pub_bs.publish(bs)

        if v <= 10.8 and not self._warned_low:
            self.get_logger().warn(f'BATERIA BAJA: {v:.2f} V (recargar; giros erraticos por debajo de ~11V)')
            self._warned_low = True
        elif v > 11.2:
            self._warned_low = False


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
