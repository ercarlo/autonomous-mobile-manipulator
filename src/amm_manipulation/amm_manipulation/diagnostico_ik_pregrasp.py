#!/usr/bin/env python3
"""
Diagnóstico de cinemática inversa y colisiones del pre-grasp.

No ordena movimientos, no modifica objetos de la Planning Scene y no
acciona la pinza. Reutiliza el cálculo exacto de vertical_grasp.py.

Ejecutar desde el workspace, después de copiar este archivo al mismo
Directorio que vertical_grasp.py:

  python3 src/amm_manipulation/amm_manipulation/diagnostico_ik_pregrasp.py \
      --ros-args -p use_sim_time:=true
"""

import argparse
import math
import sys

import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from moveit_msgs.msg import MoveItErrorCodes, RobotState

# Importamos el cálculo existente para que diagnóstico y ejecución usen
# exactamente las mismas poses objetivo, geometría y transformaciones.
from vertical_grasp import VerticalGrasp, FRAME_BASE, FRAME_MUNECA, GRUPO_MOVEIT


def llamada(nodo, cliente, peticion, espera=10.0):
    """Llama a un servicio y procesa las respuestas ROS 2."""
    futuro = cliente.call_async(peticion)
    rclpy.spin_until_future_complete(nodo, futuro, timeout_sec=espera)
    if not futuro.done() or futuro.result() is None:
        raise RuntimeError(f"Servicio sin respuesta tras {espera:.1f} s: {cliente.srv_name}")
    return futuro.result()


def estado_actual(nodo):
    """Forma una semilla IK con los valores recientes de /joint_states."""
    joints = nodo.posiciones_articulares
    articulaciones_ur5e = [nombre for nombre in joints if nombre.startswith("ur5e_") and "joint" in nombre]
    if len(articulaciones_ur5e) < 6:
        raise RuntimeError(
            "No se han recibido las seis articulaciones del UR5e en /joint_states. "
            f"Disponibles: {sorted(joints)}"
        )
    estado = RobotState()
    # Las demás articulaciones las completa MoveIt a partir de su estado actual.
    estado.is_diff = True
    estado.joint_state = JointState()
    estado.joint_state.name = list(joints.keys())
    estado.joint_state.position = [joints[nombre] for nombre in estado.joint_state.name]
    return estado


def resumen_colisiones(respuesta, limite=12):
    """Imprime los pares en colisión sin duplicarlos."""
    print(f"    Estado válido: {'SÍ' if respuesta.valid else 'NO'}")
    pares = sorted({tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in respuesta.contacts})
    if pares:
        print(f"    Contactos detectados ({len(respuesta.contacts)} entradas):")
        for primero, segundo in pares[:limite]:
            print(f"       - {primero}  <->  {segundo}")
        if len(pares) > limite:
            print(f"       ... y {len(pares) - limite} pares más")
    elif not respuesta.valid:
        print("    No se devolvieron contactos; comprobar límites articulares o restricciones.")
    else:
        print("    Sin contactos de colisión.")


def comprobar_estado(nodo, cliente, estado):
    solicitud = GetStateValidity.Request()
    solicitud.group_name = GRUPO_MOVEIT
    solicitud.robot_state = estado
    return llamada(nodo, cliente, solicitud, espera=8.0)


def comprobar_ik(nodo, cliente, estado, posicion, orientacion, evitar_colisiones):
    solicitud = GetPositionIK.Request()
    ik = solicitud.ik_request
    ik.group_name = GRUPO_MOVEIT
    ik.ik_link_name = FRAME_MUNECA
    ik.robot_state = estado
    ik.avoid_collisions = evitar_colisiones
    ik.timeout = Duration(seconds=4.0).to_msg()
    ik.pose_stamped.header.frame_id = FRAME_BASE
    ik.pose_stamped.header.stamp = nodo.get_clock().now().to_msg()
    ik.pose_stamped.pose.position.x = posicion[0]
    ik.pose_stamped.pose.position.y = posicion[1]
    ik.pose_stamped.pose.position.z = posicion[2]
    (ik.pose_stamped.pose.orientation.x,
     ik.pose_stamped.pose.orientation.y,
     ik.pose_stamped.pose.orientation.z,
     ik.pose_stamped.pose.orientation.w) = orientacion
    return llamada(nodo, cliente, solicitud, espera=7.0)


def main():
    parser = argparse.ArgumentParser(description="Diagnóstico de IK y colisiones (sin movimiento).")
    parser.add_argument("--diameter", type=float, default=0.075)
    parser.add_argument("--clearance", type=float, default=0.15)
    parser.add_argument("--timeout", type=float, default=15.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    nodo = None
    try:
        nodo = VerticalGrasp()
        print("\nEsperando percepción, pinza cerrada, articulaciones y TF2...", flush=True)
        datos = nodo.esperar_datos(args.timeout)
        # Congelación del puente perceptivo: no se eliminan ni cambian obstáculos.
        nodo.congelar_scene()

        cliente_ik = nodo.create_client(GetPositionIK, "/compute_ik")
        cliente_validez = nodo.create_client(GetStateValidity, "/check_state_validity")
        if not cliente_ik.wait_for_service(timeout_sec=6.0):
            raise RuntimeError("No está disponible /compute_ik.")
        if not cliente_validez.wait_for_service(timeout_sec=6.0):
            raise RuntimeError("No está disponible /check_state_validity.")

        estado = estado_actual(nodo)
        print("\n=== ESTADO ACTUAL DEL ROBOT ===")
        resumen_colisiones(comprobar_estado(nodo, cliente_validez, estado))

        for yaw in (0.0, 90.0):
            print(f"\n=== OBJETIVO PRE-GRASP: YAW {yaw:.0f}° ===")
            posicion, orientacion = nodo.calcular_pregrasp(
                datos, args.diameter, args.clearance, yaw
            )
            print("  IK sin colisiones: ", end="", flush=True)
            libre = comprobar_ik(nodo, cliente_ik, estado, posicion, orientacion, False)
            print(f"código MoveIt={libre.error_code.val} "
                  f"({'SOLUCIÓN' if libre.error_code.val == MoveItErrorCodes.SUCCESS else 'SIN SOLUCIÓN'})")
            if libre.error_code.val == MoveItErrorCodes.SUCCESS:
                print("  Validez de esa configuración con la escena actual:")
                resumen_colisiones(comprobar_estado(nodo, cliente_validez, libre.solution))

            print("  IK con colisiones: ", end="", flush=True)
            protegida = comprobar_ik(nodo, cliente_ik, estado, posicion, orientacion, True)
            print(f"código MoveIt={protegida.error_code.val} "
                  f"({'SOLUCIÓN' if protegida.error_code.val == MoveItErrorCodes.SUCCESS else 'SIN SOLUCIÓN'})")
            if protegida.error_code.val == MoveItErrorCodes.SUCCESS:
                print("  Validez de la solución que evita colisiones:")
                resumen_colisiones(comprobar_estado(nodo, cliente_validez, protegida.solution))

        print("\nDIAGNÓSTICO TERMINADO. NO SE HA MOVIDO EL ROBOT.")
        print("NOTA: una única petición IK no descarta todas las soluciones posibles.")
        return 0
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"ERROR DEL DIAGNÓSTICO: {exc}", file=sys.stderr)
        return 1
    finally:
        if nodo is not None:
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
