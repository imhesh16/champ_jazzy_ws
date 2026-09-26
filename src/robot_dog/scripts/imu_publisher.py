#!/usr/bin/env python3
"""Publica sensor_msgs/Imu leyendo la IMU Hiwonder IM10A por serie.

Protocolo estandar Hiwonder/WitMotion (verificado en vivo, ver
CLAUDE.md): paquetes de 11 bytes, byte0=0x55, byte1=tipo, bytes2-9 = 4
int16 little endian, byte10 = checksum (suma de bytes0-9 & 0xFF).

  0x51 = Aceleracion (ax, ay, az, temp) -- raw/32768*16 (g)
  0x52 = Giroscopio   (wx, wy, wz, temp) -- raw/32768*2000 (deg/s)
  0x53 = Angulo       (roll, pitch, yaw, temp) -- raw/32768*180 (deg)

sensor_msgs/Imu requiere unidades SI: aceleracion en m/s^2, velocidad
angular en rad/s, orientacion como cuaternion (a partir de roll/pitch/
yaw, convencion intrinseca XYZ: roll sobre X, pitch sobre Y, yaw sobre
Z, la misma que usa tf_transformations.quaternion_from_euler).

No se publica una matriz de covarianza medida realmente (no
calibrada); se usa una covarianza fija razonable, marcada como tal.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

PORT = "/dev/imu"  # nombre fijo creado por hardware/udev/99-robot-dog.rules
BAUD = 9600
FRAME_ID = "imu_link"
PUBLISH_ON_ANGLE_ONLY = False  # si True, solo publica cuando llega un paquete 0x53

G_TO_MS2 = 9.80665
DEG_TO_RAD = math.pi / 180.0


def quaternion_from_euler(roll, pitch, yaw):
    """Cuaternion (x,y,z,w) a partir de roll/pitch/yaw en radianes,
    convencion intrinseca XYZ (misma que tf_transformations)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


class ImuPublisher(Node):

    def __init__(self):
        super().__init__("imu_publisher")

        self.declare_parameter("port", PORT)
        self.declare_parameter("baud", BAUD)
        self.declare_parameter("frame_id", FRAME_ID)

        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value
        self.frame_id = self.get_parameter("frame_id").value

        self.publisher = self.create_publisher(Imu, "/imu/data", 10)

        try:
            self.serial = serial.Serial(port, baud, timeout=0.05)
        except Exception as exc:
            self.get_logger().error(f"No se pudo abrir {port}: {exc}")
            raise

        self._buf = bytearray()
        self._accel = None  # (ax, ay, az) en g
        self._gyro = None   # (wx, wy, wz) en deg/s
        self._angle = None  # (roll, pitch, yaw) en deg

        # 200Hz de polling del puerto; el propio sensor publica a la
        # tasa que tenga configurada internamente (tipicamente ~10-100Hz).
        self.timer = self.create_timer(1.0 / 200.0, self._poll_serial)

        self.get_logger().info(
            f"IMU publisher activo: {port} @ {baud} baud -> /imu/data (frame_id={self.frame_id})"
        )

    def _poll_serial(self):
        chunk = self.serial.read(256)
        if chunk:
            self._buf.extend(chunk)

        while len(self._buf) >= 11:
            if self._buf[0] != 0x55:
                self._buf.pop(0)
                continue
            pkt = bytes(self._buf[:11])
            checksum = sum(pkt[0:10]) & 0xFF
            if checksum != pkt[10]:
                self._buf.pop(0)
                continue
            self._parse_packet(pkt)
            del self._buf[:11]

    def _parse_packet(self, pkt):
        ptype = pkt[1]
        raw = [
            int.from_bytes(pkt[2:4], "little", signed=True),
            int.from_bytes(pkt[4:6], "little", signed=True),
            int.from_bytes(pkt[6:8], "little", signed=True),
        ]

        if ptype == 0x51:
            self._accel = tuple(v / 32768.0 * 16.0 for v in raw)
        elif ptype == 0x52:
            self._gyro = tuple(v / 32768.0 * 2000.0 for v in raw)
        elif ptype == 0x53:
            self._angle = tuple(v / 32768.0 * 180.0 for v in raw)
            if PUBLISH_ON_ANGLE_ONLY or (self._accel and self._gyro):
                self._publish_imu()

    def _publish_imu(self):
        if self._angle is None:
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        roll, pitch, yaw = (a * DEG_TO_RAD for a in self._angle)
        x, y, z, w = quaternion_from_euler(roll, pitch, yaw)
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation.w = w
        # Covarianza no calibrada: valores fijos razonables (diagonal).
        msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        if self._gyro is not None:
            gx, gy, gz = (g * DEG_TO_RAD for g in self._gyro)
            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
            msg.angular_velocity_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02]
        else:
            msg.angular_velocity_covariance[0] = -1.0  # desconocida

        if self._accel is not None:
            ax, ay, az = (a * G_TO_MS2 for a in self._accel)
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            msg.linear_acceleration_covariance = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]
        else:
            msg.linear_acceleration_covariance[0] = -1.0  # desconocida

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
