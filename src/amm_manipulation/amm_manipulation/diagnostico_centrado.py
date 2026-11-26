#!/usr/bin/env python3
"""
Diagnóstico del centrado de la pinza sobre amm_detected_apple.

Consulta los extremos de los dedos mediante TF2 y la caja de colisión
CONGELADA desde /get_planning_scene. Compara las posiciones X/Y en base_link.
No modifica la escena ni mueve el robot.

Ejemplo:
  python3 diagnostico_centrado.py --ros-args -p use_sim_time:=true
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from tf2_ros import Buffer, TransformException, TransformListener

BASE = 'base_link'
IZQUIERDO = 'gripper_robotiq_85_left_finger_tip_link'
DERECHO = 'gripper_robotiq_85_right_finger_tip_link'
OBJETO = 'amm_detected_apple'


def suma(a, b):
    return tuple(x + y for x, y in zip(a, b))


def quat_rot(q, v):
    """Gira un vector v por un cuaternión q (xyzw) normalizado."""
    x, y, z, w = q
    norma = math.sqrt(x*x+y*y+z*z+w*w)
    if norma < 1e-12:
        return v  # La orientación vacía de algunos mensajes equivale a identidad.
    x, y, z, w = (a/norma for a in (x,y,z,w))
    vx, vy, vz = v
    tx, ty, tz = (2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx))
    return (vx+w*tx+(y*tz-z*ty), vy+w*ty+(z*tx-x*tz), vz+w*tz+(x*ty-y*tx))


def campos_punto(p):
    return (float(p.x),float(p.y),float(p.z))


def campos_quat(q):
    return (float(q.x),float(q.y),float(q.z),float(q.w))


class Diagnostico(Node):
    def __init__(self):
        super().__init__('amm_diagnostico_centrado')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.client = self.create_client(GetPlanningScene, '/get_planning_scene')

    def esperar_tf(self, origen, plazo=12.0):
        hasta = time.monotonic()+plazo
        while rclpy.ok() and time.monotonic() < hasta:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.buffer.lookup_transform(BASE, origen, Time())
                return campos_punto(tf.transform.translation)
            except TransformException:
                pass
        raise RuntimeError(f'No se pudo consultar TF {BASE} <- {origen}')

    def consultar_cubo(self):
        if not self.client.wait_for_service(timeout_sec=8.0):
            raise RuntimeError('No existe /get_planning_scene. ¿Está MoveIt activo?')
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        futuro = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, futuro, timeout_sec=8.0)
        if not futuro.done() or futuro.result() is None:
            raise RuntimeError('No se obtuvo respuesta de /get_planning_scene.')
        objetos = futuro.result().scene.world.collision_objects
        objeto = next((o for o in objetos if o.id == OBJETO), None)
        if objeto is None:
            presentes = ', '.join(o.id for o in objetos)
            raise RuntimeError(f'{OBJETO} no está en Planning Scene. Objetos: {presentes}')
        if not objeto.primitive_poses:
            raise RuntimeError(f'{OBJETO} no tiene primitive_poses.')
        # Pose de la primitiva dentro del objeto, seguida de pose general del objeto.
        p_rel = campos_punto(objeto.primitive_poses[0].position)
        p_obj = campos_punto(objeto.pose.position)
        p_rel = suma(quat_rot(campos_quat(objeto.pose.orientation), p_rel), p_obj)
        frame = objeto.header.frame_id or BASE
        if frame != BASE:
            # Esperamos TF incluso si el origen está publicado con retardo.
            hasta = time.monotonic()+12.0
            while rclpy.ok() and time.monotonic() < hasta:
                rclpy.spin_once(self, timeout_sec=0.1)
                try:
                    tf = self.buffer.lookup_transform(BASE, frame, Time())
                    q_tf = campos_quat(tf.transform.rotation)
                    p_rel = suma(quat_rot(q_tf, p_rel), campos_punto(tf.transform.translation))
                    break
                except TransformException:
                    pass
            else:
                raise RuntimeError(f'No existe TF {BASE} <- {frame}')
        return p_rel, frame

    def imprimir(self):
        izq = self.esperar_tf(IZQUIERDO)
        der = self.esperar_tf(DERECHO)
        medio = tuple((a+b)/2.0 for a,b in zip(izq,der))
        cubo, frame = self.consultar_cubo()
        dx, dy = medio[0]-cubo[0], medio[1]-cubo[1]
        print('\n=== CENTRADO PRE-GRASP (sin mover robot) ===')
        print(f'Centro de dedos real, pinza actual [{BASE}]: {medio}')
        print(f'Centro caja {OBJETO}, convertido a {BASE}: {cubo}')
        print(f'Desfase X: {dx*1000:+.1f} mm')
        print(f'Desfase Y: {dy*1000:+.1f} mm')
        print(f'Desfase XY total: {math.hypot(dx,dy)*1000:.1f} mm')
        print(f'La caja está almacenada en el frame: {frame}')
        print('NOTA: el centro del cubo es una APROXIMACIÓN de la manzana,')
        print('basada en una detección RGB-D anterior; no es ground truth de Gazebo.')


def main():
    rclpy.init()
    nodo = Diagnostico()
    try:
        nodo.imprimir()
    except RuntimeError as error:
        nodo.get_logger().error(str(error))
        return 1
    finally:
        nodo.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
