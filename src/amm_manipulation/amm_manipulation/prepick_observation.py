#!/usr/bin/env python3
"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - OBSERVACIÓN PRE-PICK
============================================================

Paquete: amm_manipulation
Archivo: prepick_observation.py

Objetivo:
    Calcular una postura de cámara cercana a un objeto
    localizado por YOLO + RGB-D y convertirla en una postura
    de la muñeca UR5e para MoveIt 2.

En esta etapa:
    - La cámara conserva su orientación actual.
    - El objeto puede colocarse fuera del centro de la imagen
      para evitar que la propia pinza lo oculte.
    - Los desplazamientos se expresan en ejes ópticos de cámara.
    - Por defecto solo calcula y muestra las poses.
    - Con --plan true solicita una trayectoria a MoveIt.
    - Por defecto NO ejecuta ningún movimiento.
    - Con --plan true --execute true, MoveIt planifica y
      ejecuta el movimiento con velocidad reducida.

Los puntos de objetos provienen de percepción, no de Gazebo.
============================================================
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

from geometry_msgs.msg import PointStamped, PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from action_msgs.msg import GoalStatus
from std_srvs.srv import SetBool


# ============================================================
# CONFIGURACIÓN DEL ROBOT
# ============================================================

FRAME_BASE = "base_link"
FRAME_CAMARA = "wrist_camera_depth_optical_frame"
FRAME_MUNECA = "ur5e_wrist_3_link"
GRUPO_MOVEIT = "ur5e_arm"
SERVICIO_CONGELAR = "/amm_manipulation/freeze_scene"


# ============================================================
# OPERACIONES CON CUATERNIONES Y VECTORES
# ============================================================

def normalizar_cuaternion(q):
    """Devuelve (x, y, z, w) con norma unidad."""
    valores = (q.x, q.y, q.z, q.w)
    norma = math.sqrt(sum(v * v for v in valores))
    if norma < 1e-10:
        raise ValueError("Cuaternión de TF2 inválido.")
    return tuple(v / norma for v in valores)


def producto_cuaterniones(a, b):
    """Compone rotaciones: primero b y después a."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def conjugado(q):
    """Inversa de un cuaternión unitario."""
    return (-q[0], -q[1], -q[2], q[3])


def rotar_vector(q, v):
    """Rota un vector tridimensional con el cuaternión unitario q."""
    vx, vy, vz = v
    x, y, z, w = q
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def restar(a, b):
    return tuple(x - y for x, y in zip(a, b))


def sumar(a, b):
    return tuple(x + y for x, y in zip(a, b))


def escalar(k, v):
    return tuple(k * x for x in v)


def norma(v):
    return math.sqrt(sum(x * x for x in v))


def vector_traslacion(transformacion):
    t = transformacion.transform.translation
    return (t.x, t.y, t.z)


def mostrar_vector(nombre, v):
    print(f"  {nombre}: [{v[0]:.4f}, {v[1]:.4f}, {v[2]:.4f}] m")


def crear_pose_stamped(nodo, posicion, orientacion):
    """Crea un objetivo cartesiano referido a base_link."""
    msg = PoseStamped()
    msg.header.frame_id = FRAME_BASE
    msg.header.stamp = nodo.get_clock().now().to_msg()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = posicion
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = orientacion
    return msg


# ============================================================
# NODO DE OBSERVACIÓN PRE-PICK
# ============================================================

class PrepickObservation(Node):

    def __init__(self, clase):
        super().__init__("amm_prepick_observation")
        self.punto_objeto = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.suscripcion = self.create_subscription(
            PointStamped,
            f"/perception/{clase}/point_base",
            self.recibir_punto,
            qos_profile_sensor_data,
        )
        self.cliente_moveit = ActionClient(self, MoveGroup, "/move_action")
        self.cliente_congelar = self.create_client(SetBool, SERVICIO_CONGELAR)
        self.get_logger().info(
            f"Esperando detección de '{clase}' y transformaciones TF2."
        )

    def recibir_punto(self, mensaje):
        """Conserva la medición más reciente de percepción."""
        valores = (mensaje.point.x, mensaje.point.y, mensaje.point.z)
        if not all(math.isfinite(v) for v in valores):
            return
        if mensaje.header.frame_id != FRAME_BASE:
            self.get_logger().warning(
                f"Se esperaba {FRAME_BASE}; recibido {mensaje.header.frame_id}."
            )
            return
        self.punto_objeto = mensaje

    def esperar_datos(self, timeout):
        """Espera una detección y las dos transformaciones necesarias."""
        inicio = time.monotonic()
        while rclpy.ok() and time.monotonic() - inicio < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.punto_objeto is None:
                continue
            try:
                # La cámara respecto a la base: orientación ACTUAL.
                base_camara = self.tf_buffer.lookup_transform(
                    FRAME_BASE, FRAME_CAMARA, Time()
                )
                # Montaje fijo de la cámara en la muñeca.
                muneca_camara = self.tf_buffer.lookup_transform(
                    FRAME_MUNECA, FRAME_CAMARA, Time()
                )
                return self.punto_objeto, base_camara, muneca_camara
            except TransformException:
                continue
        raise RuntimeError(
            "No se ha recibido la detección y/o TF2 dentro del tiempo límite."
        )

    def calcular_posturas(
        self, punto_msg, base_camara, muneca_camara,
        distancia, offset_x, offset_y,
    ):
        """
        Calcula una observación de cámara descentrada.

        El punto del objeto, en coordenadas de la cámara
        objetivo, queda aproximadamente en:
            X = offset_x  (positivo: derecha de la imagen)
            Y = offset_y  (positivo: abajo en imagen óptica)
            Z = distancia (profundidad sobre el eje óptico)

        Esto permite dejar el objeto a un lado de la pinza
        visible en la imagen en lugar de situarlo detrás de ella.
        Se conserva la orientación actual de la cámara.

        T_base_muñeca = T_base_cámara_objetivo * inv(T_muñeca_cámara)
        """
        objeto = (
            punto_msg.point.x,
            punto_msg.point.y,
            punto_msg.point.z,
        )
        camara_actual = vector_traslacion(base_camara)
        q_base_camara = normalizar_cuaternion(base_camara.transform.rotation)
        q_muneca_camara = normalizar_cuaternion(muneca_camara.transform.rotation)
        t_muneca_camara = vector_traslacion(muneca_camara)

        # El desplazamiento objeto-cámara deseado se expresa
        # en el sistema óptico de la RealSense:
        #   +X = derecha, +Y = abajo, +Z = hacia la escena.
        # Un offset_x positivo deja el objeto a la derecha
        # del centro de la imagen y lejos de la pinza central.
        desplazamiento_optico = (offset_x, offset_y, distancia)
        desplazamiento_base = rotar_vector(
            q_base_camara, desplazamiento_optico
        )

        # T_base_objeto = T_base_cámara_objetivo + R * desplazamiento_optico.
        # Por tanto, despejamos la posición de la cámara.
        camara_objetivo = restar(objeto, desplazamiento_base)

        # Orientación objetivo de la muñeca.
        q_base_muneca = producto_cuaterniones(
            q_base_camara, conjugado(q_muneca_camara)
        )
        q_base_muneca = tuple(
            v / math.sqrt(sum(z * z for z in q_base_muneca))
            for v in q_base_muneca
        )

        # Corregir el desplazamiento del montaje de la cámara.
        desplazamiento_montaje = rotar_vector(
            q_base_muneca, t_muneca_camara
        )
        muneca_objetivo = restar(camara_objetivo, desplazamiento_montaje)

        print("\n=== PROPUESTA DE OBSERVACIÓN PRE-PICK ===")
        mostrar_vector("Objeto (YOLO + RGB-D)", objeto)
        mostrar_vector("Cámara actual", camara_actual)
        mostrar_vector("Cámara objetivo", camara_objetivo)
        mostrar_vector("Muñeca objetivo", muneca_objetivo)
        print(f"  Distancia actual cámara-objeto: {norma(restar(objeto, camara_actual)):.3f} m")
        print(f"  Profundidad objetivo (eje óptico Z): {distancia:.3f} m")
        print(f"  Offset en imagen (X derecha, Y abajo): [{offset_x:+.3f}, {offset_y:+.3f}] m")
        print(f"  Distancia espacial objetivo: {norma(desplazamiento_optico):.3f} m")
        print("  Se conserva la orientación de la cámara.")
        print("  El objeto NO queda necesariamente centrado en imagen.")
        print("  La postura NO es una pose de agarre.\n")

        if camara_objetivo[2] < 0.80:
            raise RuntimeError(
                "La cámara objetivo queda demasiado baja para esta prueba "
                "(z < 0.80 m); no se solicita planificación."
            )

        return (
            crear_pose_stamped(self, camara_objetivo, q_base_camara),
            crear_pose_stamped(self, muneca_objetivo, q_base_muneca),
        )


    def congelar_scene(self, timeout=10.0):
        """
        Congela las actualizaciones de obstáculos procedentes de YOLO.

        La planificación y ejecución solo comienzan cuando el servicio
        confirma que no quedan modificaciones pendientes en MoveIt.
        No detiene la cámara ni el nodo de percepción.
        """
        if not self.cliente_congelar.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                "No está disponible /amm_manipulation/freeze_scene. "
                "Inicia detected_objects_scene.py antes de planificar."
            )

        inicio = time.monotonic()
        while rclpy.ok() and time.monotonic() - inicio < timeout:
            solicitud = SetBool.Request()
            solicitud.data = True
            futuro = self.cliente_congelar.call_async(solicitud)
            rclpy.spin_until_future_complete(self, futuro, timeout_sec=3.0)

            if not futuro.done() or futuro.result() is None:
                raise RuntimeError(
                    "El gestor de escena no confirmó la congelación."
                )

            respuesta = futuro.result()
            if respuesta.success:
                self.get_logger().info(
                    "Planning Scene CONGELADA: "
                    "no se modificarán los obstáculos durante el movimiento."
                )
                return

            # La respuesta puede indicar que hay una petición anterior
            # a MoveIt todavía en curso; concedemos tiempo para terminar.
            time.sleep(0.3)

        raise RuntimeError(
            "No se ha confirmado la congelación de la escena. "
            "No se autoriza el movimiento."
        )

    def planificar_y_mover(self, postura_muneca, ejecutar=False):
        """
        Envía un objetivo cartesiano a MoveIt 2.

        - ejecutar=False: solo planifica; no mueve el UR5e.
        - ejecutar=True: planifica una trayectoria NUEVA y,
          si es válida, solicita su ejecución en Gazebo.

        Conserva las restricciones que ya validamos en la
        Etapa 7.4.2. Comprueba tanto el estado de la acción ROS 2
        como el código de error devuelto por MoveIt.
        """
        # Congelar antes de que MoveIt planifique y, opcionalmente, ejecute.
        self.congelar_scene()

        if not self.cliente_moveit.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("No está disponible /move_action.")

        pose = postura_muneca.pose
        goal = MoveGroup.Goal()
        goal.request.group_name = GRUPO_MOVEIT
        goal.request.pipeline_id = "ompl"
        goal.request.num_planning_attempts = 8
        goal.request.allowed_planning_time = 10.0

        # Velocidades reducidas en esta primera prueba física.
        goal.request.max_velocity_scaling_factor = 0.10
        goal.request.max_acceleration_scaling_factor = 0.10

        # El punto de partida debe ser el estado REAL actual.
        goal.request.start_state.is_diff = True

        # Restricción de posición: región esférica de tolerancia.
        posicion = PositionConstraint()
        posicion.header.frame_id = FRAME_BASE
        posicion.link_name = FRAME_MUNECA
        posicion.weight = 1.0
        esfera = SolidPrimitive()
        esfera.type = SolidPrimitive.SPHERE
        esfera.dimensions = [0.03]
        region = Pose()
        region.position = pose.position
        region.orientation.w = 1.0
        posicion.constraint_region.primitives.append(esfera)
        posicion.constraint_region.primitive_poses.append(region)

        # La orientación de muñeca mantiene la dirección óptica
        # actual de la cámara durante la observación pre-pick.
        orientacion = OrientationConstraint()
        orientacion.header.frame_id = FRAME_BASE
        orientacion.link_name = FRAME_MUNECA
        orientacion.orientation = pose.orientation
        orientacion.absolute_x_axis_tolerance = 0.12
        orientacion.absolute_y_axis_tolerance = 0.12
        orientacion.absolute_z_axis_tolerance = 0.12
        orientacion.weight = 1.0

        restricciones = Constraints()
        restricciones.position_constraints.append(posicion)
        restricciones.orientation_constraints.append(orientacion)
        goal.request.goal_constraints.append(restricciones)

        # false: planificación y ejecución; true: plan únicamente.
        # La decisión de ejecutar requiere --execute true explícito.
        goal.planning_options.plan_only = not ejecutar
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        if ejecutar:
            self.get_logger().warning(
                "PLAN + EXECUTE activado: el UR5e se moverá en Gazebo."
            )
        else:
            self.get_logger().info(
                "Solicitando Plan ONLY: NO se moverá el UR5e."
            )

        envio = self.cliente_moveit.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, envio, timeout_sec=20.0)
        if not envio.done() or envio.result() is None:
            raise RuntimeError("MoveIt no ha respondido a la solicitud.")

        manejador = envio.result()
        if not manejador.accepted:
            raise RuntimeError("MoveIt ha rechazado el objetivo.")

        resultado_futuro = manejador.get_result_async()
        # La ejecución puede tardar más que la planificación.
        espera = 120.0 if ejecutar else 35.0
        rclpy.spin_until_future_complete(
            self, resultado_futuro, timeout_sec=espera
        )
        if not resultado_futuro.done() or resultado_futuro.result() is None:
            raise RuntimeError(
                "No se ha recibido el resultado de MoveIt. "
                "Si habías solicitado ejecución, comprueba Gazebo y "
                "los registros de move_group antes de volver a ordenar el movimiento."
            )

        resultado = resultado_futuro.result()
        if (
            resultado.status != GoalStatus.STATUS_SUCCEEDED
            or resultado.result.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError(
                f"Movimiento fallido. Estado acción={resultado.status}; "
                f"código MoveIt={resultado.result.error_code.val}. "
                "Revisa los registros de move_group."
            )

        puntos = len(
            resultado.result.planned_trajectory.joint_trajectory.points
        )
        if ejecutar:
            self.get_logger().info(
                f"PLAN + EXECUTE CORRECTO ({puntos} puntos de plan). "
                "MoveIt informa ejecución completada."
            )
        else:
            self.get_logger().info(
                f"PLAN CORRECTO ({puntos} puntos). Ejecución: DESACTIVADA."
            )

    def comprobar_camara(self, postura_camara):
        """
        Verifica mediante TF2 dónde quedó la cámara tras mover el brazo.

        Esta comprobación no vuelve a ejecutar YOLO: esa segunda
        medición del objeto corresponde a la Etapa 7.4.4.
        """
        objetivo = (
            postura_camara.pose.position.x,
            postura_camara.pose.position.y,
            postura_camara.pose.position.z,
        )
        inicio = time.monotonic()
        while time.monotonic() - inicio < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = self.tf_buffer.lookup_transform(
                    FRAME_BASE, FRAME_CAMARA, Time()
                )
                posicion_real = vector_traslacion(t)
                error = norma(restar(posicion_real, objetivo))
                if error <= 0.06:
                    mostrar_vector("Cámara final medida por TF2", posicion_real)
                    print(f"  Error respecto a pre-pick: {error:.3f} m")
                    self.get_logger().info(
                        "Postura pre-pick confirmada mediante TF2."
                    )
                    return
            except TransformException:
                pass

        raise RuntimeError(
            "MoveIt informó ejecución correcta, pero TF2 no confirmó "
            "la posición de cámara con una tolerancia de 6 cm. "
            "Comprueba Gazebo y TF2 antes de continuar."
        )


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Calcular y planificar una observación pre-pick."
    )
    parser.add_argument(
        "--object", choices=["apple", "bottle", "cup", "bowl"], default="apple"
    )
    parser.add_argument("--distance", type=float, default=0.40)
    parser.add_argument(
        "--offset-x", type=float, default=0.0,
        help="Offset horizontal del objeto en frame óptico (m). +X = derecha de la imagen.",
    )
    parser.add_argument(
        "--offset-y", type=float, default=0.0,
        help="Offset vertical del objeto en frame óptico (m). -Y = parte superior.",
    )
    parser.add_argument("--plan", choices=["true", "false"], default="false")
    parser.add_argument("--execute", choices=["true", "false"], default="false")
    parser.add_argument("--timeout", type=float, default=12.0)
    argumentos, _ = parser.parse_known_args(args)
    if not (0.30 <= argumentos.distance <= 0.65):
        parser.error("Para esta prueba, --distance debe estar entre 0.30 y 0.65 m.")
    if not (-0.15 <= argumentos.offset_x <= 0.15):
        parser.error("--offset-x debe estar entre -0.15 y 0.15 m.")
    if not (-0.08 <= argumentos.offset_y <= 0.08):
        parser.error("--offset-y debe estar entre -0.08 y 0.08 m.")
    if argumentos.execute == "true" and argumentos.plan != "true":
        parser.error("Para mover el robot, usa --plan true --execute true.")

    rclpy.init(args=args)
    nodo = None
    try:
        nodo = PrepickObservation(argumentos.object)
        datos = nodo.esperar_datos(argumentos.timeout)
        camara, muneca = nodo.calcular_posturas(
            *datos, argumentos.distance,
            argumentos.offset_x, argumentos.offset_y,
        )
        if argumentos.plan == "true":
            nodo.planificar_y_mover(
                muneca, ejecutar=(argumentos.execute == "true")
            )
            if argumentos.execute == "true":
                nodo.comprobar_camara(camara)
        else:
            nodo.get_logger().info(
                "Cálculo finalizado. No se ha solicitado planificación ni ejecución."
            )
        return 0
    except (RuntimeError, ValueError) as error:
        if nodo is not None:
            nodo.get_logger().error(str(error))
        else:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    finally:
        if nodo is not None:
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())