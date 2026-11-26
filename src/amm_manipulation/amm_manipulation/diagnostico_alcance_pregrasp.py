#!/usr/bin/env python3
"""
DIAGNÓSTICO DE ALCANCE DEL PRE-GRASP — SOLO LECTURA.

Comprueba si la petición de IK funciona para una pose conocida (la pose
ACTUAL de la muñeca) y explora pequeñas variaciones del objetivo sin
mover el brazo ni modificar la Planning Scene.

Ejecutar desde ~/autonomous_mobile_manipulator tras copiar este archivo
junto a vertical_grasp.py:

python3 src/amm_manipulation/amm_manipulation/diagnostico_alcance_pregrasp.py \
  --ros-args -p use_sim_time:=true

Todas las posiciones se calculan en base_link; la distancia desde la
base del UR5e es orientativa y NO sustituye la cinemática inversa.
"""

import math
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from tf2_ros import TransformException

from vertical_grasp import (
    VerticalGrasp, FRAME_BASE, FRAME_MUNECA, GRUPO_MOVEIT,
    vector_transformacion, cuaternion_transformacion,
)

FRAME_BASE_BRAZO = "ur5e_base_link"


def esperar_tf(nodo, destino, origen, timeout=10.0):
    """Espera TF2 procesando callbacks para no consultar un búfer vacío."""
    limite = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < limite:
        try:
            return nodo.tf_buffer.lookup_transform(destino, origen, Time())
        except TransformException:
            rclpy.spin_once(nodo, timeout_sec=0.15)
    raise RuntimeError(f"No se ha recibido TF2: {destino} <- {origen}")


def estado_articular(nodo):
    """Semilla IK a partir de las posiciones articulares actuales."""
    art = nodo.posiciones_articulares
    if sum(nombre.startswith("ur5e_") and "joint" in nombre for nombre in art) < 6:
        raise RuntimeError("Faltan las seis articulaciones del UR5e en /joint_states")
    estado = RobotState()
    estado.is_diff = True
    estado.joint_state = JointState()
    estado.joint_state.name = list(art)
    estado.joint_state.position = [art[nombre] for nombre in art]
    return estado


def llamada_ik(nodo, cliente, estado, posicion, cuaternion, timeout=2.0):
    """Pide una solución de IK SIN excluir colisiones: prueba alcance puro."""
    peticion = GetPositionIK.Request()
    ik = peticion.ik_request
    ik.group_name = GRUPO_MOVEIT
    ik.ik_link_name = FRAME_MUNECA
    ik.robot_state = estado
    ik.avoid_collisions = False
    ik.timeout = Duration(seconds=timeout).to_msg()
    ik.pose_stamped.header.frame_id = FRAME_BASE
    ik.pose_stamped.header.stamp = nodo.get_clock().now().to_msg()
    ik.pose_stamped.pose.position.x = float(posicion[0])
    ik.pose_stamped.pose.position.y = float(posicion[1])
    ik.pose_stamped.pose.position.z = float(posicion[2])
    (ik.pose_stamped.pose.orientation.x,
     ik.pose_stamped.pose.orientation.y,
     ik.pose_stamped.pose.orientation.z,
     ik.pose_stamped.pose.orientation.w) = cuaternion

    futuro = cliente.call_async(peticion)
    rclpy.spin_until_future_complete(nodo, futuro, timeout_sec=timeout + 4.0)
    if not futuro.done() or futuro.result() is None:
        raise RuntimeError("/compute_ik no respondió")
    return futuro.result().error_code.val


def distancia(a, b):
    return math.dist(a, b)


def resultado(nodo, cliente, estado, etiqueta, p, q, timeout=2.0):
    codigo = llamada_ik(nodo, cliente, estado, p, q, timeout)
    marca = "SÍ" if codigo == MoveItErrorCodes.SUCCESS else "NO"
    print(f"  {etiqueta:<41} IK={marca:<3} (código {codigo})", flush=True)
    return codigo == MoveItErrorCodes.SUCCESS


def main():
    rclpy.init()
    nodo = None
    try:
        nodo = VerticalGrasp()
        print("\nEsperando detección, /joint_states y transformaciones TF2...", flush=True)
        datos = nodo.esperar_datos(15.0)
        nodo.congelar_scene()

        tf_brazo = esperar_tf(nodo, FRAME_BASE, FRAME_BASE_BRAZO)
        tf_muneca = esperar_tf(nodo, FRAME_BASE, FRAME_MUNECA)
        base_brazo = vector_transformacion(tf_brazo)
        muneca_actual = vector_transformacion(tf_muneca)
        q_actual = cuaternion_transformacion(tf_muneca)
        estado = estado_articular(nodo)

        cliente = nodo.create_client(GetPositionIK, "/compute_ik")
        if not cliente.wait_for_service(timeout_sec=8.0):
            raise RuntimeError("No existe /compute_ik; comprueba move_group")

        objetivo, q_obj = nodo.calcular_pregrasp(datos, 0.075, 0.15, 0.0)
        radio = lambda p: math.hypot(p[0] - base_brazo[0], p[1] - base_brazo[1])

        print("\n=== REFERENCIAS GEOMÉTRICAS (base_link) ===")
        print(f"  Base UR5e:       ({base_brazo[0]:+.4f}, {base_brazo[1]:+.4f}, {base_brazo[2]:+.4f}) m")
        print(f"  Muñeca actual:   ({muneca_actual[0]:+.4f}, {muneca_actual[1]:+.4f}, {muneca_actual[2]:+.4f}) m")
        print(f"  Muñeca objetivo: ({objetivo[0]:+.4f}, {objetivo[1]:+.4f}, {objetivo[2]:+.4f}) m")
        print(f"  Radio horizontal actual:   {radio(muneca_actual):.3f} m")
        print(f"  Radio horizontal objetivo: {radio(objetivo):.3f} m")
        print(f"  Distancia 3D base->objetivo: {distancia(base_brazo, objetivo):.3f} m")
        print("  Referencia UR5e: alcance nominal 0.850 m (no es un límite esférico exacto para la muñeca).")

        print("\n=== PRUEBA DE CONTROL POSITIVO ===")
        ok_actual = resultado(nodo, cliente, estado, "IK de la postura REAL ACTUAL", muneca_actual, q_actual, 3.0)
        if not ok_actual:
            print("\nATENCIÓN: el servicio de IK falla incluso para la pose actual de TF2.")
            print("Revisar primero plugin de IK, grupo, enlace y semilla; no asumir límite de alcance.")
            return 0

        print("\n=== COMPARACIÓN CON EL OBJETIVO DE AYER ===")
        posicion_ayer = (-0.0926, -0.7644, objetivo[2])
        resultado(nodo, cliente, estado, "Punto de ayer, yaw 0°", posicion_ayer, q_obj, 2.0)

        print("\n=== ALTERNATIVAS PARA EL OBJETIVO DE HOY (SIN MOVER) ===")
        # Los cambios de yaw son giros sobre el eje vertical. Mantienen +Z del gripper hacia abajo.
        for yaw in (180.0, 270.0):
            _, q_alternativa = nodo.calcular_pregrasp(datos, 0.075, 0.15, yaw)
            resultado(nodo, cliente, estado, f"Posición actual, yaw {yaw:.0f}°", objetivo, q_alternativa)

        # Comprobar si pequeños cambios de altura permiten llegar a una posición vertical.
        for dz in (-0.05, +0.05):
            p = (objetivo[0], objetivo[1], objetivo[2] + dz)
            resultado(nodo, cliente, estado, f"Posición actual, dz={dz:+.2f} m, yaw 0°", p, q_obj)

        # Acercar el objetivo hacia la base real del UR5e, no hacia (0,0) por suposición.
        dx = objetivo[0] - base_brazo[0]
        dy = objetivo[1] - base_brazo[1]
        r = math.hypot(dx, dy)
        if r > 0.01:
            for desplazamiento in (0.04, 0.08, 0.12):
                p = (objetivo[0] - desplazamiento*dx/r,
                     objetivo[1] - desplazamiento*dy/r,
                     objetivo[2])
                resultado(nodo, cliente, estado,
                          f"Objetivo {int(desplazamiento*1000)} mm más cerca de la base", p, q_obj)

        print("\nDIAGNÓSTICO TERMINADO. No se ha ordenado ningún movimiento.")
        print("IMPORTANTE: el éxito IK sin colisiones NO autoriza un movimiento.")
    except (RuntimeError, ValueError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    finally:
        if nodo is not None:
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
