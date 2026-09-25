#!/usr/bin/env python3
"""Caminata "creep" (una pata a la vez) independiente de CHAMP.

Prueba experimental: no usa CHAMP para nada, manda posturas directo a
/esp32/joint_targets (el mismo topico que siempre, la calibracion del
ESP32 no se toca). Sirve para probar si evitar por completo la ventana
de 2 patas en el aire (como hace el trote de CHAMP) resuelve la
inestabilidad.

Modo "en el sitio" (por defecto): cada pata sube y baja sin avanzar,
solo para probar estabilidad pura.

Modo "avance" (--avanzar): mientras una pata esta en el aire, su femur
barre hacia adelante (de -SWING_HALF a +SWING_HALF) y, al mismo
tiempo, las otras 3 patas (apoyadas, sin levantarse) barren su femur
un poco hacia atras (-SWING_HALF/3 cada una) para empujar el cuerpo
hacia adelante -- un gait "creep" clasico de una pata a la vez, con
avance real, sin ningun instante con menos de 3 patas apoyadas.

IMPORTANTE: el puente (joint_trajectory_bridge.py) debe estar APAGADO
mientras corre esto, si no se pisan las ordenes.
Ctrl+C: vuelve a la postura de pie.
"""
import time
import sys
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Postura de pie (mismos angulos que usa CHAMP a la altura actual,
# nominal_height=0.13 en gait.yaml). Si cambias nominal_height,
# recalcula estos dos valores (femur, tibia).
STAND_LEG = 1.1445
STAND_FOOT = -1.9970
LIFT_FOOT = -2.25   # rodilla mas doblada: levanta el pie unos cm

SWING_HALF = 0.15   # amplitud del paso (rad de femur), moderado para empezar

HZ = 20.0
LIFT_S = 1.0       # duracion de levantar/avanzar/bajar cada pata
HOLD_S = 0.5        # pausa breve con la pata en el aire, antes de bajar
STAND_S = 0.8       # pausa de pie entre patas
SHIFT_S = 0.8        # duracion del desplazamiento de peso antes de levantar

LEG_NAMES = ["front_left", "front_right", "rear_left", "rear_right"]

# Posicion nominal de cada pata respecto al centro del cuerpo (del xacro:
# shiftx=0.093, shifty=0.039). Con las 4 patas apoyadas, el centro de masa
# del cuerpo cae justo en el centro de este rectangulo. Al levantar una
# pata, las otras 3 forman un triangulo cuyo centroide NO coincide con
# ese centro -- cae en el borde del triangulo (margen de estabilidad
# cero). Por eso antes de levantar cualquier pata hay que desplazar el
# "centro de apoyo" hacia el centroide de las 3 patas que van a quedar,
# usando el mismo truco de offset de femur/hombro que ya usa el modo
# avance (las patas apoyadas no se mueven del piso, pero su angulo
# cambia como si el cuerpo se hubiera desplazado).
SHIFTX = 0.093
SHIFTY = 0.039
LEG_XY = [
    (+SHIFTX, +SHIFTY),  # front_left
    (+SHIFTX, -SHIFTY),  # front_right
    (-SHIFTX, +SHIFTY),  # rear_left
    (-SHIFTX, -SHIFTY),  # rear_right
]

# Ganancia angulo/metro aproximada (pequenos angulos) para convertir el
# desplazamiento XY deseado del cuerpo en offsets de femur (adelante-atras,
# eje X) y hombro (lateral, eje Y). L2+L3 = brazo efectivo de la pata.
LEG_REACH = 0.1075 + 0.130

def shift_offsets_for(lifted_leg):
    """Offsets (femur, shoulder) para las 3 patas de apoyo al levantar
    'lifted_leg', de modo que el centro de apoyo se desplace hacia el
    centroide de esas 3 patas antes de levantarla."""
    others = [i for i in range(4) if i != lifted_leg]
    cx = sum(LEG_XY[i][0] for i in others) / 3.0
    cy = sum(LEG_XY[i][1] for i in others) / 3.0
    # El cuerpo debe moverse (cx, cy). En el marco del cuerpo, las patas
    # apoyadas deben verse desplazadas en la direccion opuesta.
    dx = -cx / LEG_REACH
    dy = cy / LEG_REACH
    femur_offset = [0.0, 0.0, 0.0, 0.0]
    shoulder_offset = [0.0, 0.0, 0.0, 0.0]
    for i in others:
        femur_offset[i] = dx
        shoulder_offset[i] = dy
    return femur_offset, shoulder_offset


class CreepUnaPata(Node):
    def __init__(self, avanzar):
        super().__init__("creep_una_pata")
        self.avanzar = avanzar
        self.pub = self.create_publisher(Float32MultiArray, "/esp32/joint_targets", 10)
        # offset de femur por pata (0 = angulo de pie normal), solo se
        # usa en modo avance.
        self.offset = [0.0, 0.0, 0.0, 0.0]
        self.current = self.pose_en_sitio(None)
        time.sleep(1.0)

    def pose_en_sitio(self, lifted_leg, femur_off=None, shoulder_off=None):
        """Modo sin avance: sube/baja en el sitio. femur_off/shoulder_off
        (si se pasan) desplazan el peso del cuerpo hacia las patas de
        apoyo antes/mientras se levanta 'lifted_leg'."""
        if femur_off is None:
            femur_off = [0.0, 0.0, 0.0, 0.0]
        if shoulder_off is None:
            shoulder_off = [0.0, 0.0, 0.0, 0.0]
        out = []
        for leg in range(4):
            foot = LIFT_FOOT if leg == lifted_leg else STAND_FOOT
            out.extend([shoulder_off[leg], STAND_LEG + femur_off[leg], foot])
        return out

    def pose_con_offsets(self, offsets, lifted_leg=None, lift_foot=False, shoulder_off=None):
        """Modo avance: cada pata usa su propio offset de femur (avance +
        desplazamiento de peso combinados) y opcionalmente de hombro."""
        if shoulder_off is None:
            shoulder_off = [0.0, 0.0, 0.0, 0.0]
        out = []
        for leg in range(4):
            foot = LIFT_FOOT if (lift_foot and leg == lifted_leg) else STAND_FOOT
            out.extend([shoulder_off[leg], STAND_LEG + offsets[leg], foot])
        return out

    def send(self, p):
        m = Float32MultiArray()
        m.data = [float(v) for v in p]
        self.pub.publish(m)
        self.current = list(p)

    def hold(self, p, secs, msg=""):
        if msg:
            self.get_logger().info(msg)
        end = time.time() + secs
        while time.time() < end:
            self.send(p)
            time.sleep(1.0 / HZ)

    def ramp(self, target, secs, msg=""):
        if msg:
            self.get_logger().info(msg)
        start = list(self.current)
        n = max(1, int(secs * HZ))
        for k in range(1, n + 1):
            f = k / n
            self.send([a + (b - a) * f for a, b in zip(start, target)])
            time.sleep(1.0 / HZ)

    def paso_en_sitio(self, leg, ciclo):
        name = LEG_NAMES[leg]
        femur_off, shoulder_off = shift_offsets_for(leg)

        # 1) desplaza el peso hacia las 3 patas que quedaran de apoyo,
        # con las 4 patas TODAVIA en el suelo.
        p_shift = self.pose_en_sitio(None, femur_off, shoulder_off)
        self.ramp(p_shift, SHIFT_S, f"Vuelta {ciclo+1}: desplazando peso antes de levantar {name}")

        # 2) con el peso ya sobre las otras 3, levanta esta pata.
        p_lift = self.pose_en_sitio(leg, femur_off, shoulder_off)
        self.ramp(p_lift, LIFT_S, f"Levantando {name}")
        self.hold(p_lift, HOLD_S, f"{name} en el aire, sosteniendo sobre las otras 3")

        # 3) baja de nuevo (peso sigue desplazado, pata recien bajada
        # todavia sin retomar su parte del peso de golpe).
        p_down = self.pose_en_sitio(None, femur_off, shoulder_off)
        self.ramp(p_down, LIFT_S, f"Bajando {name}")
        self.hold(p_down, STAND_S * 0.3)

        # 4) recentra el peso entre las 4 antes de pasar a la siguiente.
        neutral = self.pose_en_sitio(None)
        self.ramp(neutral, SHIFT_S, "Centrando peso")
        self.hold(neutral, STAND_S * 0.5)

    def paso_con_avance(self, leg, ciclo):
        name = LEG_NAMES[leg]
        femur_shift, shoulder_shift = shift_offsets_for(leg)

        def combined(offsets):
            return [offsets[i] + femur_shift[i] for i in range(4)]

        # 0) desplaza el peso hacia las 3 patas que quedaran de apoyo,
        # con las 4 patas todavia en el suelo.
        p_shift = self.pose_con_offsets(combined(self.offset), lifted_leg=None, shoulder_off=shoulder_shift)
        self.ramp(p_shift, SHIFT_S, f"Vuelta {ciclo+1}: desplazando peso antes de levantar {name}")

        # 1) levanta el pie de la pata que va a avanzar (las otras 3 siguen apoyadas).
        p = self.pose_con_offsets(combined(self.offset), lifted_leg=leg, lift_foot=True, shoulder_off=shoulder_shift)
        self.ramp(p, LIFT_S * 0.4, f"Levantando {name}")

        # 2) con esa pata en el aire, barre su femur hacia adelante y,
        # al mismo tiempo, las otras 3 (apoyadas) barren un poco hacia
        # atras -- eso empuja el cuerpo hacia adelante.
        new_offset = list(self.offset)
        new_offset[leg] = SWING_HALF
        for i in range(4):
            if i != leg:
                new_offset[i] -= SWING_HALF / 3.0
        target = self.pose_con_offsets(combined(new_offset), lifted_leg=leg, lift_foot=True, shoulder_off=shoulder_shift)
        self.ramp(target, LIFT_S, f"{name} avanzando en el aire, cuerpo empujado por las otras 3")
        self.offset = new_offset

        # 3) apoya de nuevo esa pata en su nueva posicion adelantada (peso
        # todavia desplazado hacia las otras 3, que son las que sostenian).
        p2 = self.pose_con_offsets(combined(self.offset), lifted_leg=None, shoulder_off=shoulder_shift)
        self.ramp(p2, LIFT_S * 0.4, f"Apoyando {name}")
        self.hold(p2, STAND_S * 0.3)

        # 4) recentra el peso entre las 4 antes de pasar a la siguiente.
        neutral = self.pose_con_offsets(self.offset, lifted_leg=None)
        self.ramp(neutral, SHIFT_S, "Centrando peso")
        self.hold(neutral, STAND_S * 0.5)

    def run(self, cycles):
        stand = self.pose_en_sitio(None)
        self.hold(stand, 2.0, "De pie (4 patas)")

        for c in range(cycles):
            for leg in range(4):
                if self.avanzar:
                    self.paso_con_avance(leg, c)
                else:
                    self.paso_en_sitio(leg, c)

        self.get_logger().info("Fin de la prueba.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cycles", nargs="?", type=int, default=3, help="vueltas completas (4 patas cada una)")
    ap.add_argument("--avanzar", action="store_true", help="avanza de verdad en vez de subir/bajar en el sitio")
    args = ap.parse_args()

    rclpy.init()
    n = CreepUnaPata(avanzar=args.avanzar)
    try:
        n.run(args.cycles)
    except KeyboardInterrupt:
        n.get_logger().info("Interrumpido: vuelvo a la postura de pie")
        n.ramp(n.pose_en_sitio(None), 1.0, "Volviendo de pie")
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
