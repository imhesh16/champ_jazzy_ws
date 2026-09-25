#!/usr/bin/env python3
"""Puente entre CHAMP y el firmware del ESP32.

Se suscribe a las trayectorias articulares reales que ya usa CHAMP
(trajectory_msgs/JointTrajectory, publicadas por
all_legs_trajectory_controller a ~200 Hz) y las republica como un
std_msgs/Float32MultiArray de 12 posiciones en radianes, en el mismo
orden fijo de las 12 articulaciones. Este formato es mucho mas simple
de manejar en un microcontrolador vía micro-ROS (sin arrays anidados
ni strings) que el mensaje original.

IMPORTANTE: la republicacion se hace a una tasa fija (PUBLISH_RATE_HZ,
20 Hz por defecto) en vez de reenviar cada uno de los ~200 mensajes/seg
que publica CHAMP. El transporte serie del micro-ROS del ESP32
(115200 baudios) no tiene ancho de banda para 200 Hz de forma
confiable; a esa tasa los mensajes llegan atrasados/mezclados y el
servo termina mostrando una posicion que no corresponde al instante
actual. Un timer separado siempre publica la ULTIMA posicion recibida,
desacoplando la tasa de entrada (CHAMP) de la de salida (ESP32).

Orden fijo de las 12 posiciones publicadas (indices 0-11):
  0 front_left_shoulder   4 front_right_shoulder   8  rear_left_shoulder
  1 front_left_leg        5 front_right_leg         9  rear_left_leg
  2 front_left_foot       6 front_right_foot        10 rear_left_foot
  3 (repite patron)       7 (repite patron)          -
Ver joints.yaml para el orden exacto declarado.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32MultiArray

INPUT_TOPIC = "/all_legs_trajectory_controller/joint_trajectory"
OUTPUT_TOPIC = "/esp32/joint_targets"
EXPECTED_JOINT_COUNT = 12
PUBLISH_RATE_HZ = 20.0


class JointTrajectoryBridge(Node):

    def __init__(self):
        super().__init__("joint_trajectory_bridge")

        self._joint_names = None
        self._latest_positions = None

        self.subscription = self.create_subscription(
            JointTrajectory,
            INPUT_TOPIC,
            self._on_trajectory,
            10,
        )

        self.publisher = self.create_publisher(
            Float32MultiArray,
            OUTPUT_TOPIC,
            10,
        )

        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ,
            self._publish_latest,
        )

        self.get_logger().info(
            f"Puente activo: {INPUT_TOPIC} (~200 Hz) -> "
            f"{OUTPUT_TOPIC} ({PUBLISH_RATE_HZ:.0f} Hz)"
        )

    def _on_trajectory(self, msg: JointTrajectory):
        if not msg.points:
            return

        positions = msg.points[0].positions

        if len(positions) != EXPECTED_JOINT_COUNT:
            self.get_logger().warn(
                f"Se esperaban {EXPECTED_JOINT_COUNT} articulaciones, "
                f"llegaron {len(positions)}. Se descarta el mensaje.",
                throttle_duration_sec=5.0,
            )
            return

        if self._joint_names != list(msg.joint_names):
            self._joint_names = list(msg.joint_names)
            self.get_logger().info(
                f"Orden de articulaciones detectado: {self._joint_names}"
            )

        # Solo guarda la mas reciente; el timer se encarga de publicarla
        # a una tasa fija, independiente de cuan seguido llega CHAMP.
        self._latest_positions = [float(p) for p in positions]

    def _publish_latest(self):
        if self._latest_positions is None:
            return
        out = Float32MultiArray()
        out.data = self._latest_positions
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
