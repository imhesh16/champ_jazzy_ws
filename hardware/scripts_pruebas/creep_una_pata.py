#!/usr/bin/env python3
"""Caminata "creep" (una pata a la vez) independiente de CHAMP.

Manda posturas directo a /esp32/joint_targets (12 angulos en rad, orden
FL, FR, RL, RR x shoulder, leg, foot). No usa CHAMP, pero SI su cinematica:
los pies se definen como posiciones en metros (adelante = +X, izquierda = +Y)
y los angulos salen de la cinematica inversa de CHAMP (kinematics.h), asi
los signos son los reales del robot.

Modo "en el sitio" (por defecto): antes de levantar cada pata, el cuerpo se
desplaza hacia el centroide de las otras 3 (con las 4 patas apoyadas); luego
se levanta la pata, se baja y se recentra.

Modo "avance" (--avanzar): igual, pero la pata en el aire se adelanta STEP
y las otras 3 (apoyadas) retroceden STEP/3 respecto al cuerpo, que asi avanza.

IMPORTANTE: el puente (joint_trajectory_bridge.py) debe estar APAGADO
mientras corre esto. Ctrl+C: vuelve a la postura de pie.
"""
import math
import time
import argparse

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray
except ImportError:            # permite probar la logica sin ROS
    rclpy = None
    Node = object

# --- geometria del robot (xacro) y postura de pie (gait_real.yaml) ---
L2, L3 = 0.1075, 0.130            # femur, tibia
SHIFT = 0.043                     # desplazamiento lateral hombro -> plano de la pata
SHIFTX, SHIFTY = 0.093, 0.039     # posicion del hombro respecto al centro del cuerpo
NOMINAL_H = 0.13                  # nominal_height

LEG_NAMES = ["front_left", "front_right", "rear_left", "rear_right"]
SIDE = [+1, -1, +1, -1]           # +1 izquierda, -1 derecha
# pie en reposo, marco del cuerpo (x adelante, y izquierda)
FOOT_BODY = [(+SHIFTX, +(SHIFTY + SHIFT)), (+SHIFTX, -(SHIFTY + SHIFT)),
             (-SHIFTX, +(SHIFTY + SHIFT)), (-SHIFTX, -(SHIFTY + SHIFT))]
LIMITS = [(-0.548, 0.548), (-2.666, 1.548), (-2.6, 0.1)]
LIMIT_MARGIN = 0.05   # rad: nunca se pide un angulo a menos de esto de su limite

LIFT_H = 0.025     # cuanto se levanta el pie (m)
STEP = 0.012       # avance de la pata en el aire (m), solo con --avanzar (a 0.13 m de altura el femur tiene poco recorrido)

HZ = 20.0
LIFT_S = 1.0       # duracion de levantar / avanzar / bajar
HOLD_S = 0.5       # pausa con la pata en el aire
STAND_S = 0.8      # pausa de pie entre patas
SHIFT_S = 0.8      # duracion del desplazamiento de peso


def champ_ik(x, y, z, l0):
    """Port de champ::Kinematics::inverse (knee_direction = -1, orientacion '>>').
    (x, y, z): posicion del pie respecto al hombro. l0 = SIDE * SHIFT."""
    hip = -(math.atan(y / z) - (math.pi / 2 - math.acos(-l0 / math.hypot(y, z))))
    c, s = math.cos(-hip), math.sin(-hip)
    z2 = s * y + c * z
    if math.hypot(x, z2) >= L2 + L3:
        raise ValueError(f"pie fuera de alcance: x={x:.3f} z={z2:.3f}")
    low = -math.acos((z2 * z2 + x * x - L2 * L2 - L3 * L3) / (2 * L2 * L3))
    up = math.atan(x / z2) - math.atan((-L3 * math.sin(low)) / (-L2 - L3 * math.cos(low)))
    if up < 0:
        up += math.pi
    return hip, up, low


def leg_angles(leg, dx=0.0, dy=0.0, dz=0.0):
    """Angulos de una pata para un pie desplazado (dx, dy, dz) desde su reposo."""
    y0 = SIDE[leg] * SHIFT
    q = champ_ik(dx, y0 + dy, -NOMINAL_H + dz, y0)
    for v, (lo, hi) in zip(q, LIMITS):
        if not lo + LIMIT_MARGIN <= v <= hi - LIMIT_MARGIN:
            raise ValueError(f"{LEG_NAMES[leg]}: angulo {v:.3f} fuera de limites {lo}..{hi}")
    return q


def make_pose(feet):
    """feet: lista de 4 tuplas (dx, dy, dz) -> lista de 12 angulos."""
    out = []
    for leg, (dx, dy, dz) in enumerate(feet):
        out.extend(leg_angles(leg, dx, dy, dz))
    return out


def support_shift(lifted, xoff):
    """Desplazamiento (dx, dy) que hay que aplicar a los 4 pies (apoyados) para
    que el centro del cuerpo quede sobre el centroide de las 3 patas que
    quedaran de apoyo. Si el cuerpo se mueve (cx, cy), los pies apoyados se
    ven, desde el cuerpo, desplazados (-cx, -cy)."""
    others = [i for i in range(4) if i != lifted]
    cx = sum(FOOT_BODY[i][0] + xoff[i] for i in others) / 3.0
    cy = sum(FOOT_BODY[i][1] for i in others) / 3.0
    return -cx, -cy


def feasible(poses_feet):
    try:
        for f in poses_feet:
            make_pose(f)
        return True
    except ValueError:
        return False


def stability_margin(lifted, feet):
    """Distancia (m) del centro del cuerpo a los lados del triangulo de apoyo
    (positivo = dentro). 'feet' incluye el desplazamiento; el cuerpo esta en (0, 0)."""
    pts = [(FOOT_BODY[i][0] + feet[i][0], FOOT_BODY[i][1] + feet[i][1]) for i in range(4) if i != lifted]
    cross = (pts[1][0] - pts[0][0]) * (pts[2][1] - pts[0][1]) - (pts[1][1] - pts[0][1]) * (pts[2][0] - pts[0][0])
    m = []
    for i in range(3):
        a, b = pts[i], pts[(i + 1) % 3]
        ex, ey = b[0] - a[0], b[1] - a[1]
        d = (-a[0] * (-ey) + -a[1] * ex) / math.hypot(ex, ey)     # (0 - a) . n, n = (-ey, ex)/|e|
        m.append(d if cross > 0 else -d)
    return min(m)


class CreepUnaPata(Node):
    def __init__(self, avanzar):
        super().__init__("creep_una_pata")
        self.avanzar = avanzar
        self.pub = self.create_publisher(Float32MultiArray, "/esp32/joint_targets", 10)
        self.xoff = [0.0] * 4                 # avance acumulado de cada pie respecto al cuerpo (m)
        self.current = self.stand()
        time.sleep(1.0)

    def stand(self):
        return make_pose([(0.0, 0.0, 0.0)] * 4)

    def feet(self, xoff, shift=(0.0, 0.0), lifted=None):
        return [(xoff[i] + shift[0], shift[1], LIFT_H if i == lifted else 0.0) for i in range(4)]

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

    def shift_factible(self, leg):
        """Desplazamiento de peso completo hacia el centroide, reducido (si hace falta)
        a la mayor fraccion que mantiene todos los angulos dentro de sus limites."""
        full = support_shift(leg, self.xoff)
        new = list(self.xoff)
        new[leg] += STEP
        for i in range(4):
            if i != leg:
                new[i] -= STEP / 3.0

        def ok(k):
            s = (full[0] * k, full[1] * k)
            fs = [self.feet(self.xoff, s), self.feet(self.xoff, s, leg)]
            if self.avanzar:
                fs += [self.feet(new, s, leg), self.feet(new, s)]
            return feasible(fs)

        lo, hi = 0.0, 1.0
        if ok(1.0):
            lo = 1.0
        else:
            for _ in range(14):
                mid = (lo + hi) / 2
                lo, hi = (mid, hi) if ok(mid) else (lo, mid)
        s = (full[0] * lo, full[1] * lo)
        marg = stability_margin(leg, self.feet(self.xoff, s, leg))
        self.get_logger().info(f"{LEG_NAMES[leg]}: desplazamiento de peso al {100*lo:.0f}% (margen de estabilidad {1000*marg:.0f} mm)")
        return s

    def paso(self, leg, ciclo):
        name = LEG_NAMES[leg]
        shift = self.shift_factible(leg)

        # 1) desplaza el peso hacia las 3 patas de apoyo (las 4 siguen en el suelo)
        self.ramp(make_pose(self.feet(self.xoff, shift)), SHIFT_S, f"Vuelta {ciclo+1}: desplazando peso antes de levantar {name}")
        # 2) levanta la pata
        self.ramp(make_pose(self.feet(self.xoff, shift, leg)), LIFT_S, f"Levantando {name}")

        if self.avanzar:
            # 3) adelanta la pata en el aire; las otras 3 retroceden STEP/3 respecto al cuerpo
            new = list(self.xoff)
            new[leg] += STEP
            for i in range(4):
                if i != leg:
                    new[i] -= STEP / 3.0
            self.ramp(make_pose(self.feet(new, shift, leg)), LIFT_S, f"{name} avanzando en el aire")
            self.xoff = new
        else:
            self.hold(make_pose(self.feet(self.xoff, shift, leg)), HOLD_S, f"{name} en el aire, sosteniendo sobre las otras 3")

        # 4) baja la pata (el peso sigue desplazado)
        self.ramp(make_pose(self.feet(self.xoff, shift)), LIFT_S, f"Bajando {name}")
        # 5) recentra el peso entre las 4
        self.ramp(make_pose(self.feet(self.xoff)), SHIFT_S, "Centrando peso")
        self.hold(make_pose(self.feet(self.xoff)), STAND_S * 0.5)

    def run(self, cycles):
        self.hold(self.stand(), 2.0, "De pie (4 patas)")
        for c in range(cycles):
            for leg in range(4):
                self.paso(leg, c)
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
        n.ramp(n.stand(), 1.0, "Volviendo de pie")
    except ValueError as e:
        n.get_logger().error(f"Movimiento no realizable ({e}); vuelvo a la postura de pie")
        n.ramp(n.stand(), 1.0, "Volviendo de pie")
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
