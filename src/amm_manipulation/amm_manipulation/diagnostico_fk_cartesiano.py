#!/usr/bin/env python3
"""
Diagnóstico SIN MOVIMIENTO: comprobar que TF2 y la cinemática directa (FK)
que utiliza MoveIt coinciden, y revisar la trayectoria cartesiana solicitada.

ROS 2 Jazzy / MoveIt 2. No modifica la Planning Scene, no mueve el robot,
no abre ni cierra la pinza. Todos los cálculos se expresan en base_link.

Ejemplo:
  python3 diagnostico_fk_cartesiano.py --distance 0.01 --ros-args -p use_sim_time:=true
"""

import argparse
import copy
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

BASE = 'base_link'
MUNECA = 'ur5e_wrist_3_link'
BRAZO = 'ur5e_arm'


def pos_de_pose(p):
    return (float(p.position.x), float(p.position.y), float(p.position.z))


def dist(a, b):
    return math.sqrt(sum((u - v) ** 2 for u, v in zip(a, b)))


def mostrar(nombre, xyz):
    print(f'  {nombre:28s} [{xyz[0]:+.5f}, {xyz[1]:+.5f}, {xyz[2]:+.5f}] m')


class DiagnosticoFK(Node):
    def __init__(self):
        super().__init__('amm_diagnostico_fk_cartesiano')
        self.articulaciones = {}
        self.ultima_lectura = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sub_joints = self.create_subscription(
            JointState, '/joint_states', self.recibir_joints, qos_profile_sensor_data
        )
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        self.cartesiano = self.create_client(GetCartesianPath, '/compute_cartesian_path')

    def recibir_joints(self, msg):
        for i, nombre in enumerate(msg.name):
            if i < len(msg.position) and math.isfinite(float(msg.position[i])):
                self.articulaciones[nombre] = float(msg.position[i])
        self.ultima_lectura = time.monotonic()

    def esperar_datos(self, max_segundos=12.0):
        limite = time.monotonic() + max_segundos
        while rclpy.ok() and time.monotonic() < limite:
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.articulaciones) < 6:
                continue
            try:
                transformacion = self.tf_buffer.lookup_transform(BASE, MUNECA, Time())
                return transformacion
            except TransformException:
                continue
        raise RuntimeError('No están disponibles /joint_states o TF2 (base_link -> muñeca).')

    def estado_actual(self):
        """Usar las mismas articulaciones que utiliza cartesian_descent.py."""
        estado = RobotState()
        estado.is_diff = True
        estado.joint_state = JointState()
        estado.joint_state.header.stamp = self.get_clock().now().to_msg()
        estado.joint_state.name = list(self.articulaciones)
        estado.joint_state.position = [self.articulaciones[n] for n in estado.joint_state.name]
        return estado

    def llamar(self, cliente, req, descripcion, timeout=12.0):
        if not cliente.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'No disponible el servicio {descripcion}.')
        futuro = cliente.call_async(req)
        rclpy.spin_until_future_complete(self, futuro, timeout_sec=timeout)
        if not futuro.done() or futuro.result() is None:
            raise RuntimeError(f'No respondió {descripcion}.')
        return futuro.result()

    def obtener_fk(self, estado):
        req = GetPositionFK.Request()
        req.header.frame_id = BASE
        req.header.stamp = self.get_clock().now().to_msg()
        req.fk_link_names = [MUNECA]
        req.robot_state = estado
        res = self.llamar(self.fk, req, '/compute_fk')
        if res.error_code.val != MoveItErrorCodes.SUCCESS or len(res.pose_stamped) != 1:
            raise RuntimeError(f'/compute_fk falló con código {res.error_code.val}')
        return res.pose_stamped[0].pose

    def calcular(self, origen, estado, distancia):
        req = GetCartesianPath.Request()
        req.header.frame_id = BASE
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = BRAZO
        req.link_name = MUNECA
        objetivo = Pose()
        objetivo.position.x = float(origen.transform.translation.x)
        objetivo.position.y = float(origen.transform.translation.y)
        objetivo.position.z = float(origen.transform.translation.z - distancia)
        objetivo.orientation = copy.deepcopy(origen.transform.rotation)
        req.waypoints = [objetivo]
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = 0.05
        req.max_acceleration_scaling_factor = 0.05
        req.start_state = estado
        res = self.llamar(self.cartesiano, req, '/compute_cartesian_path', timeout=25.0)
        return res, objetivo

    def ejecutar_diagnostico(self, distancia):
        print('\n=== DIAGNÓSTICO SIN MOVIMIENTO ===', flush=True)
        tf_actual = self.esperar_datos()
        estado = self.estado_actual()
        pose_fk = self.obtener_fk(estado)
        p_tf = (
            tf_actual.transform.translation.x,
            tf_actual.transform.translation.y,
            tf_actual.transform.translation.z,
        )
        p_fk = pos_de_pose(pose_fk)
        mostrar('Muñeca actual según TF2:', p_tf)
        mostrar('Muñeca actual según FK:', p_fk)
        diferencia_fk_tf = dist(p_tf, p_fk)
        print(f'  Diferencia FK ↔ TF2: {diferencia_fk_tf * 1000:.1f} mm')
        if diferencia_fk_tf > 0.01:
            print('  ALERTA: MoveIt y TF2 no coinciden en la pose inicial.')
        else:
            print('  FK y TF2 concuerdan aproximadamente en la pose inicial.')

        print('\n=== PLAN CARTESIANO DE DIAGNÓSTICO (NO SE EJECUTA) ===')
        respuesta, objetivo = self.calcular(tf_actual, estado, distancia)
        mostrar('Objetivo solicitado:', pos_de_pose(objetivo))
        puntos = respuesta.solution.joint_trajectory.points
        nombres = respuesta.solution.joint_trajectory.joint_names
        print(f'  Fracción: {respuesta.fraction * 100:.1f} %')
        print(f'  Código MoveIt: {respuesta.error_code.val}')
        print(f'  Número de puntos: {len(puntos)}')
        if len(puntos) < 2:
            print('  No hay suficientes puntos para analizar. NO se ha movido el robot.')
            return

        # Cada muestra se evalúa CON LA MISMA FK que usa MoveIt.
        # Conservamos todos los joints/multi-DOF del estado inicial devuelto.
        base_estado = respuesta.start_state
        indices = sorted({0, len(puntos) // 4, len(puntos) // 2,
                          (3 * len(puntos)) // 4, len(puntos) - 1})
        valores_z = []
        valores_xy = []
        p_primero = None
        p_ultimo = None
        for i in indices:
            estado_muestra = copy.deepcopy(base_estado)
            mapa = {n: float(v) for n, v in zip(
                estado_muestra.joint_state.name, estado_muestra.joint_state.position)}
            mapa.update({n: float(v) for n, v in zip(nombres, puntos[i].positions)})
            estado_muestra.joint_state.name = list(mapa)
            estado_muestra.joint_state.position = [mapa[n] for n in estado_muestra.joint_state.name]
            estado_muestra.is_diff = False
            pose = self.obtener_fk(estado_muestra)
            p = pos_de_pose(pose)
            if p_primero is None:
                p_primero = p
            p_ultimo = p
            valores_z.append(p[2])
            valores_xy.append((p[0], p[1]))
            mostrar(f'FK punto {i:3d}/{len(puntos) - 1}:', p)

        print(f'  Distancia FK inicial frente TF2: {dist(p_primero, p_tf)*1000:.1f} mm')
        print(f'  Distancia FK final al objetivo: {dist(p_ultimo, pos_de_pose(objetivo))*1000:.1f} mm')
        print(f'  Diferencia Z de primer a último punto: {(p_ultimo[2] - p_primero[2])*1000:+.1f} mm')
        subida_intermedia = any(z2 > z1 + 0.001 for z1, z2 in zip(valores_z, valores_z[1:]))
        deriva = max(math.hypot(x - valores_xy[0][0], y - valores_xy[0][1])
                     for x, y in valores_xy)
        print(f'  Alguna subida entre muestras: {"SÍ" if subida_intermedia else "NO"}')
        print(f'  Máxima deriva XY muestreada: {deriva * 1000:.1f} mm')
        if diferencia_fk_tf > 0.01 or subida_intermedia or deriva > 0.01 or \
           dist(p_ultimo, pos_de_pose(objetivo)) > 0.01:
            print('\n  ALERTA: NO ES SEGURO EJECUTAR ESTE PLAN. Revisar configuración FK / frames.')
        else:
            print('\n  La trayectoria diagnosticada es geométricamente coherente;')
            print('  esto NO autoriza ningún movimiento ni verifica el controlador.')
        print('  EL ROBOT NO SE HA MOVIDO.')


def main():
    parser = argparse.ArgumentParser(description='Diagnóstico FK y cartesiano sin movimiento')
    parser.add_argument('--distance', type=float, default=0.01,
                        help='Distancia solo para el cálculo de prueba; por defecto 0.01 m')
    args, ros_args = parser.parse_known_args()
    if not 0.005 <= args.distance <= 0.04:
        parser.error('--distance debe estar entre 0.005 y 0.04 m')
    rclpy.init(args=ros_args)
    nodo = DiagnosticoFK()
    try:
        nodo.ejecutar_diagnostico(args.distance)
    except (RuntimeError, ValueError) as e:
        nodo.get_logger().error(str(e))
        return 1
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
