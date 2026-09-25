#!/usr/bin/env python3
"""Prueba de apoyo sobre 2 patas opuestas (sin caminar).

Manda posturas directo a /esp32/joint_targets (12 angulos en radianes,
mismo orden que el puente): de pie -> levanta delantera derecha +
trasera izquierda -> baja -> levanta delantera izquierda + trasera
derecha -> baja. Sirve para ver si el robot se sostiene sobre 2 patas.

IMPORTANTE: el puente (joint_trajectory_bridge.py) debe estar APAGADO,
si no publica en el mismo topico y se pisan las ordenes.
Ctrl+C: vuelve a la postura de pie.
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Postura de pie de CHAMP (nominal_height 0.165) y pie levantado ~3 cm.
STAND = (0.0, 0.9061, -1.6149)
LIFT = (0.0, 1.1105, -1.9469)

HZ = 20.0
RAMP_S = 4.0     # duracion de subir o bajar una pareja (mas lento que antes: 1.5 -> 4.0)
HOLD_S = 8.0     # tiempo sosteniendo sobre 2 patas
STAND_S = 4.0    # tiempo de pie entre pasos

# indices: 0 FL, 1 FR, 2 RL, 3 RR (cada una con shoulder, leg, foot)
def pose(lifted):
    out = []
    for leg in range(4):
        out.extend(LIFT if leg in lifted else STAND)
    return out


class Prueba(Node):
    def __init__(self):
        super().__init__("prueba_dos_patas")
        self.pub = self.create_publisher(Float32MultiArray, "/esp32/joint_targets", 10)
        self.current = pose(())
        time.sleep(1.0)

    def send(self, p):
        m = Float32MultiArray()
        m.data = [float(v) for v in p]
        self.pub.publish(m)
        self.current = list(p)

    def hold(self, p, secs, msg):
        self.get_logger().info(msg)
        end = time.time() + secs
        while time.time() < end:
            self.send(p)
            time.sleep(1.0 / HZ)

    def ramp(self, target, secs, msg):
        self.get_logger().info(msg)
        start = list(self.current)
        n = max(1, int(secs * HZ))
        for k in range(1, n + 1):
            f = k / n
            self.send([a + (b - a) * f for a, b in zip(start, target)])
            time.sleep(1.0 / HZ)

    def run(self):
        stand = pose(())
        self.hold(stand, STAND_S, "De pie (4 patas)")
        for lifted, name in (((1, 2), "delantera DERECHA + trasera IZQUIERDA"),):
            self.ramp(pose(lifted), RAMP_S, f"Levantando {name}")
            self.hold(pose(lifted), HOLD_S,
                      f"Sosteniendo sobre las otras 2 patas ({HOLD_S:.0f}s): OBSERVA")
            self.ramp(stand, RAMP_S, "Bajando")
            self.hold(stand, STAND_S, "De pie (4 patas)")
        self.get_logger().info("Fin de la prueba.")


def main():
    rclpy.init()
    n = Prueba()
    try:
        n.run()
    except KeyboardInterrupt:
        n.get_logger().info("Interrumpido: vuelvo a la postura de pie")
        n.ramp(pose(()), 1.0, "Volviendo de pie")
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
