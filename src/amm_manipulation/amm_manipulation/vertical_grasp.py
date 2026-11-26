#!/usr/bin/env python3
"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - PRE-GRASP VERTICAL
============================================================

Paquete: amm_manipulation
Archivo: vertical_grasp.py
Etapa: 7.5.3 - refinamiento horizontal

Objetivo:
    Calcular una postura de pre-grasp vertical para la manzana
    a partir de la segunda observación YOLO + RGB-D.

Proceso:
    1. Recibir el punto 3D de la manzana en base_link.
    2. Obtener la altura del tablero desde workstations.yaml.
    3. Estimar el centro de la manzana con su diámetro conocido.
    4. Consultar TF2 para hallar el punto medio real de los
       extremos de los dedos con la Robotiq CERRADA.
    5. Orientar el eje +Z de la pinza hacia abajo.
    6. Calcular la pose objetivo de ur5e_wrist_3_link.
    7. Planificar una única vez mediante MoveIt 2.
    8. Revisar el plan en RViz y, si se autoriza, ejecutarlo sin
       volver a planificar mediante /execute_trajectory.
    9. Verificar la llegada y la verticalidad real por TF2.
    10. Solo entonces ABRIR la pinza.

Modo opcional --refine-xy true:
    - Se usa SOLO cuando el robot YA ESTÁ en pre-grasp vertical,
      con la Robotiq ABIERTA y el puente YOLO congelado.
    - Se calcula un desplazamiento horizontal pequeño desde las TF2
      actuales hacia la caja congelada amm_detected_apple.
    - Se conservan Z, orientación y apertura de la pinza.
    - Se revisa el plan en RViz antes de ejecutar la misma trayectoria.

SEGURIDAD DE ESTA ETAPA:
    - Por defecto, NO se ejecuta ningún movimiento.
    - La ejecución requiere --plan true --execute true.
    - La Robotiq debe estar CERRADA antes de iniciar.
    - La apertura se ordena SOLAMENTE tras verificar el pre-grasp.
    - No desciende hasta el objeto ni intenta agarrarlo.
    - No modifica la Planning Scene ni elimina obstáculos.

Las posiciones X/Y de los objetos provienen de percepción,
no de las poses internas de Gazebo.
============================================================
"""

import argparse
import math
import sys
import time
from pathlib import Path
import os
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PointStamped
from control_msgs.action import ParallelGripperCommand
from std_srvs.srv import SetBool
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import (
    Constraints,
    DisplayTrajectory,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    PlanningSceneComponents,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener


# ============================================================
# CONSTANTES DEL ROBOT
# ============================================================

FRAME_BASE = "base_link"
FRAME_MUNECA = "ur5e_wrist_3_link"
FRAME_PINZA = "gripper_robotiq_85_base_link"
FRAME_DEDO_IZQUIERDO = "gripper_robotiq_85_left_finger_tip_link"
FRAME_DEDO_DERECHO = "gripper_robotiq_85_right_finger_tip_link"
JOINT_PINZA = "gripper_robotiq_85_left_knuckle_joint"
GRUPO_MOVEIT = "ur5e_arm"
TOPICO_MANZANA = "/perception/apple/point_base"
ACCION_PINZA = "/gripper_controller/gripper_cmd"
SERVICIO_CONGELAR = "/amm_manipulation/freeze_scene"
POSICION_ABIERTA = 0.05
# En este modelo de Gazebo, una posición cercana a 0.7929 indica pinza cerrada.
MINIMO_CERRADA = 0.70


# ============================================================
# OPERACIONES GEOMÉTRICAS
# ============================================================

def vector_transformacion(mensaje):
    """Devuelve la traslación de una transformación TF2."""
    t = mensaje.transform.translation
    return (float(t.x), float(t.y), float(t.z))


def normalizar_cuaternion(q):
    """Normaliza el cuaternión (x, y, z, w)."""
    if not all(math.isfinite(v) for v in q):
        raise ValueError("Se recibió un cuaternión no finito.")
    norma = math.sqrt(sum(v * v for v in q))
    if norma < 1e-10:
        raise ValueError("Se recibió un cuaternión de norma nula.")
    return tuple(v / norma for v in q)


def cuaternion_transformacion(mensaje):
    q = mensaje.transform.rotation
    return normalizar_cuaternion((q.x, q.y, q.z, q.w))


def producto_cuaterniones(a, b):
    """Compone rotaciones: aplica b y, a continuación, a."""
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
    """Rota un vector mediante un cuaternión unitario."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def sumar(a, b):
    return tuple(x + y for x, y in zip(a, b))


def restar(a, b):
    return tuple(x - y for x, y in zip(a, b))


def promedio(a, b):
    return tuple((x + y) / 2.0 for x, y in zip(a, b))


def distancia(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def imprimir_punto(titulo, p):
    print(f"  {titulo}: [{p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f}] m")


# ============================================================
# CONFIGURACIÓN DE LA MESA
# ============================================================

def cargar_tablero():
    """
    Lee la geometría conocida de la mesa en workstations.yaml.

    Conocer la posición de la ESTACIÓN es válido. En cambio,
    no se leen las posiciones de los objetos desde Gazebo.
    """
    ruta = (
        Path(get_package_share_directory("amm_manipulation"))
        / "config"
        / "workstations.yaml"
    )
    with ruta.open("r", encoding="utf-8") as fichero:
        configuracion = yaml.safe_load(fichero)

    if not isinstance(configuracion, dict):
        raise ValueError("Configuración de estaciones inválida.")

    frame = configuracion.get("frame_id")
    obstaculos = configuracion.get("obstacles", [])
    mesa = next(
        (obj for obj in obstaculos if obj.get("id") == "amm_workbench_top"),
        None,
    )
    if not frame or mesa is None:
        raise ValueError("No se encontró amm_workbench_top y su frame_id.")
    if mesa.get("geometry") != "box":
        raise ValueError("El tablero debe tener geometría de caja.")

    pos = [float(v) for v in mesa["position"]]
    tam = [float(v) for v in mesa["dimensions"]]
    if len(pos) != 3 or len(tam) != 3 or any(t <= 0 for t in tam):
        raise ValueError("Geometría del tablero incorrecta.")

    # El YAML registra el CENTRO de la caja del tablero.
    # Su superficie superior está a media altura de ese centro.
    centro_superficie = (pos[0], pos[1], pos[2] + tam[2] / 2.0)
    return frame, centro_superficie


# ============================================================
# NODO ROS 2
# ============================================================

class VerticalGrasp(Node):

    def __init__(self):
        super().__init__("amm_vertical_grasp")

        self.punto = None
        self.apertura_actual = None
        # Posiciones articulares reales para verificar que el brazo
        # no se ha movido mientras se revisa el plan en RViz.
        self.posiciones_articulares = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.suscripcion_objeto = self.create_subscription(
            PointStamped,
            TOPICO_MANZANA,
            self.recibir_manzana,
            qos_profile_sensor_data,
        )
        self.suscripcion_joints = self.create_subscription(
            JointState,
            "/joint_states",
            self.recibir_joints,
            qos_profile_sensor_data,
        )
        # /move_action se utiliza SOLO para planificar.
        # /execute_trajectory ejecuta la MISMA trayectoria ya aprobada.
        self.cliente_moveit = ActionClient(self, MoveGroup, "/move_action")
        self.cliente_ejecutar = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )
        self.cliente_congelar = self.create_client(SetBool, SERVICIO_CONGELAR)
        self.cliente_escena = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.cliente_pinza = ActionClient(
            self, ParallelGripperCommand, ACCION_PINZA
        )

        # Facilita visualizar la trayectoria solicitada desde
        # nuestro nodo, sin utilizar el botón Plan de RViz.
        qos_visualizacion = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publicador_trayectoria = self.create_publisher(
            DisplayTrajectory,
            "/display_planned_path",
            qos_visualizacion,
        )

        self.get_logger().info(
            "Esperando posición de apple, estado de Robotiq y TF2."
        )

    def recibir_manzana(self, mensaje):
        if mensaje.header.frame_id != FRAME_BASE:
            return
        p = (mensaje.point.x, mensaje.point.y, mensaje.point.z)
        if all(math.isfinite(valor) for valor in p):
            self.punto = mensaje

    def recibir_joints(self, mensaje):
        # Guardar el estado más reciente de todas las articulaciones.
        for indice, nombre in enumerate(mensaje.name):
            if indice < len(mensaje.position):
                posicion = float(mensaje.position[indice])
                if math.isfinite(posicion):
                    self.posiciones_articulares[nombre] = posicion
        if JOINT_PINZA in self.posiciones_articulares:
            self.apertura_actual = self.posiciones_articulares[JOINT_PINZA]

    def esperar_datos(self, timeout):
        """Espera una detección, la apertura real y TF2 completo."""
        inicio = time.monotonic()
        while rclpy.ok() and time.monotonic() - inicio < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.punto is None or self.apertura_actual is None:
                continue
            try:
                tf_izquierda = self.tf_buffer.lookup_transform(
                    FRAME_MUNECA, FRAME_DEDO_IZQUIERDO, Time()
                )
                tf_derecha = self.tf_buffer.lookup_transform(
                    FRAME_MUNECA, FRAME_DEDO_DERECHO, Time()
                )
                tf_pinza = self.tf_buffer.lookup_transform(
                    FRAME_MUNECA, FRAME_PINZA, Time()
                )
                frame_mesa, superficie = cargar_tablero()
                if frame_mesa == FRAME_BASE:
                    superficie_base = superficie
                else:
                    tf_mesa = self.tf_buffer.lookup_transform(
                        FRAME_BASE, frame_mesa, Time()
                    )
                    superficie_base = sumar(
                        rotar_vector(
                            cuaternion_transformacion(tf_mesa), superficie
                        ),
                        vector_transformacion(tf_mesa),
                    )
                return (
                    self.punto,
                    tf_izquierda,
                    tf_derecha,
                    tf_pinza,
                    superficie_base,
                )
            except TransformException:
                continue
        raise RuntimeError(
            "No se dispone de detección, /joint_states, TF2 o mesa. "
            "Comprueba los nodos de percepción y robot_state_publisher."
        )

    def calcular_pregrasp(
        self, datos, diametro, distancia_vertical, yaw_grados
    ):
        """
        Calcula la postura de MUÑECA para un agarre desde arriba.

        Se sitúa el punto medio de los orígenes de los dedos
        encima del centro estimado del objeto. El eje +Z de la
        pinza apunta verticalmente hacia abajo.

        La orientación del eje X de la pinza depende de yaw.
        """
        punto_msg, tf_izquierda, tf_derecha, tf_pinza, superficie = datos
        punto_detectado = (
            punto_msg.point.x,
            punto_msg.point.y,
            punto_msg.point.z,
        )

        # El usuario ha elegido observación y movimiento de pre-grasp
        # con la pinza CERRADA. La abriremos al llegar al destino.
        if not (MINIMO_CERRADA <= self.apertura_actual <= 0.81):
            raise RuntimeError(
                f"La Robotiq debe estar CERRADA antes de salir de "
                f"pre-pick (lectura: {self.apertura_actual:.3f} rad). "
                "Ciérrala con gripper_controller a 0.7929 rad."
            )

        p_izquierdo = vector_transformacion(tf_izquierda)
        p_derecho = vector_transformacion(tf_derecha)
        p_centro_dedos = promedio(p_izquierdo, p_derecho)
        separacion_origenes = distancia(p_izquierdo, p_derecho)
        if not (0.00 <= separacion_origenes <= 0.20):
            raise RuntimeError(
                "Geometría inesperada entre extremos de dedos: "
                f"{separacion_origenes:.3f} m. Revisa TF2."
            )

        # Centro aproximado para una manzana apoyada en la mesa.
        # X e Y proceden de la detección RGB-D, no de Gazebo.
        centro_objeto = (
            punto_detectado[0],
            punto_detectado[1],
            superficie[2] + diametro / 2.0,
        )

        # Punto medio de los dedos 15 cm por ENCIMA del centro
        # estimado. Es pre-grasp, NO es la postura de contacto.
        objetivo_centro_dedos = (
            centro_objeto[0],
            centro_objeto[1],
            centro_objeto[2] + distancia_vertical,
        )

        # Rotación de 180° sobre X: +Z del gripper mira hacia abajo.
        # El giro adicional de yaw orienta la apertura de los dedos
        # alrededor del eje vertical, sin cambiar la verticalidad.
        angulo = math.radians(yaw_grados)
        medio = angulo / 2.0
        q_z = (0.0, 0.0, math.sin(medio), math.cos(medio))
        q_x_180 = (1.0, 0.0, 0.0, 0.0)
        q_base_pinza_deseada = normalizar_cuaternion(
            producto_cuaterniones(q_z, q_x_180)
        )

        # Transformar orientación deseada de la pinza a muñeca:
        # R_base_muñeca = R_base_pinza * R_muñeca_pinza^(-1).
        q_muneca_pinza = cuaternion_transformacion(tf_pinza)
        q_base_muneca = normalizar_cuaternion(
            producto_cuaterniones(
                q_base_pinza_deseada, conjugado(q_muneca_pinza)
            )
        )

        # T_base_centro_dedos = T_base_muñeca +
        #                        R_base_muñeca * p_muñeca_centro_dedos.
        # Se despeja la posición objetivo de la muñeca.
        p_base_muñeca = restar(
            objetivo_centro_dedos,
            rotar_vector(q_base_muneca, p_centro_dedos),
        )

        print("\n=== PRE-GRASP VERTICAL: MANZANA ===")
        imprimir_punto("Punto detectado (RGB-D)", punto_detectado)
        imprimir_punto("Superficie del tablero", superficie)
        imprimir_punto("Centro estimado de la manzana", centro_objeto)
        imprimir_punto("Centro entre los dedos (en muñeca)", p_centro_dedos)
        imprimir_punto("Centro de dedos objetivo", objetivo_centro_dedos)
        imprimir_punto("Muñeca objetivo", p_base_muñeca)
        print(f"  Separación de orígenes de los dedos: {separacion_origenes:.3f} m")
        print(f"  Joint de la pinza (CERRADA): {self.apertura_actual:.3f} rad")
        print(f"  Separación vertical sobre centro: {distancia_vertical:.3f} m")
        print(f"  Yaw de agarre: {yaw_grados:.1f} grados")
        print("  +Z de la pinza apunta verticalmente HACIA ABAJO.")
        print("  No se solicita descenso ni agarre del objeto.\n")

        # Barrera conservadora contra una pose calculada por
        # debajo de la superficie conocida de trabajo.
        if objetivo_centro_dedos[2] <= superficie[2] + diametro / 2.0 + 0.09:
            raise RuntimeError("Pre-grasp demasiado cerca de la manzana.")

        return p_base_muñeca, q_base_muneca


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


    # ========================================================
    # AJUSTE FINO DESDE PRE-GRASP: ROBOT YA VERTICAL Y PINZA ABIERTA
    # ========================================================

    def obtener_centro_cubo_congelado(self):
        """Obtiene amm_detected_apple de MoveIt, NO de la cámara.

        Se hace después de congelar el puente de detecciones. Esto
        evita recalcular el objetivo con posibles oclusiones cuando
        la muñeca está encima de la manzana.
        """
        if not self.cliente_escena.wait_for_service(timeout_sec=8.0):
            raise RuntimeError("No está disponible /get_planning_scene.")

        solicitud = GetPlanningScene.Request()
        solicitud.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        futuro = self.cliente_escena.call_async(solicitud)
        rclpy.spin_until_future_complete(self, futuro, timeout_sec=8.0)
        if not futuro.done() or futuro.result() is None:
            raise RuntimeError("Sin respuesta del servicio /get_planning_scene.")

        objetos = futuro.result().scene.world.collision_objects
        objeto = next((o for o in objetos if o.id == "amm_detected_apple"), None)
        if objeto is None or not objeto.primitive_poses:
            raise RuntimeError(
                "No está amm_detected_apple en la Planning Scene. "
                "No se permite el refinamiento sin una referencia congelada."
            )

        # Combinar pose de la primitiva con pose general del objeto.
        pose_relativa = objeto.primitive_poses[0]
        p_relativo = (
            pose_relativa.position.x,
            pose_relativa.position.y,
            pose_relativa.position.z,
        )
        p_objeto = (
            objeto.pose.position.x,
            objeto.pose.position.y,
            objeto.pose.position.z,
        )
        q_objeto = (
            objeto.pose.orientation.x,
            objeto.pose.orientation.y,
            objeto.pose.orientation.z,
            objeto.pose.orientation.w,
        )
        if sum(v * v for v in q_objeto) < 1e-12:
            q_objeto = (0.0, 0.0, 0.0, 1.0)
        else:
            q_objeto = normalizar_cuaternion(q_objeto)
        centro_frame = sumar(rotar_vector(q_objeto, p_relativo), p_objeto)

        # Convertir al mismo frame que empleamos para la muñeca.
        frame = objeto.header.frame_id or FRAME_BASE
        if frame == FRAME_BASE:
            return centro_frame

        limite = time.monotonic() + 12.0
        while rclpy.ok() and time.monotonic() < limite:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.tf_buffer.lookup_transform(FRAME_BASE, frame, Time())
                return sumar(
                    rotar_vector(cuaternion_transformacion(tf), centro_frame),
                    vector_transformacion(tf),
                )
            except TransformException:
                continue
        raise RuntimeError(f"TF2 no puede transformar {frame} a {FRAME_BASE}.")

    def centro_dedos_actual(self):
        """Centro de los dos finger tip links EN LA APERTURA ACTUAL."""
        limite = time.monotonic() + 12.0
        while rclpy.ok() and time.monotonic() < limite:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                izq = self.tf_buffer.lookup_transform(
                    FRAME_BASE, FRAME_DEDO_IZQUIERDO, Time()
                )
                der = self.tf_buffer.lookup_transform(
                    FRAME_BASE, FRAME_DEDO_DERECHO, Time()
                )
                muneca = self.tf_buffer.lookup_transform(
                    FRAME_BASE, FRAME_MUNECA, Time()
                )
                return (
                    promedio(vector_transformacion(izq), vector_transformacion(der)),
                    vector_transformacion(muneca),
                    cuaternion_transformacion(muneca),
                )
            except TransformException:
                continue
        raise RuntimeError("No hay transformaciones TF2 actuales de dedos y muñeca.")

    def calcular_refinamiento_xy(self, centro_cubo):
        """Corrige solo X/Y, conservando Z, rotación y apertura actuales."""
        if self.apertura_actual is None:
            # Esperar joint_states recientes antes de validar la apertura.
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.apertura_actual is not None:
                    break

        if self.apertura_actual is None or abs(self.apertura_actual - POSICION_ABIERTA) > 0.06:
            raise RuntimeError(
                "Este ajuste está previsto para la Robotiq ABIERTA en pre-grasp "
                "(0.05 rad). No se moverá el robot."
            )

        # Antes de mover una pinza abierta, confirmar que apunta hacia abajo.
        self.comprobar_verticalidad(max_inclinacion_grados=2.5)
        centro_dedos, muneca, orientacion = self.centro_dedos_actual()
        dx = centro_cubo[0] - centro_dedos[0]
        dy = centro_cubo[1] - centro_dedos[1]
        distancia_xy = math.hypot(dx, dy)

        print("\n=== AJUSTE FINO XY DESDE PRE-GRASP ===")
        imprimir_punto("Centro dedos actual", centro_dedos)
        imprimir_punto("Centro cubo congelado", centro_cubo)
        print(f"  Desplazamiento solicitado X: {dx * 1000:+.1f} mm")
        print(f"  Desplazamiento solicitado Y: {dy * 1000:+.1f} mm")
        print(f"  Desplazamiento horizontal: {distancia_xy * 1000:.1f} mm")
        print("  Z y orientación de muñeca: NO CAMBIAN")
        print("  Robotiq: permanece ABIERTA")
        print("  No se solicita descenso ni contacto con manzana.\n")

        if centro_dedos[2] - centro_cubo[2] < 0.10:
            raise RuntimeError(
                "La pinza no tiene suficiente altura respecto a la caja "
                "para autorizar este ajuste lateral."
            )
        if distancia_xy > 0.035:
            raise RuntimeError(
                "Desfase mayor de 35 mm: no parece un ajuste fino. "
                "Revisar detección y estado antes de mover el robot."
            )
        if distancia_xy < 0.002:
            self.get_logger().info("Centrado ya dentro de 2 mm: no hace falta corregir.")
            return None, None

        objetivo_muneca = (muneca[0] + dx, muneca[1] + dy, muneca[2])
        return objetivo_muneca, orientacion

    def imprimir_centrado_posterior(self):
        """Mide el error residual sin modificar la escena."""
        centro_objeto = self.obtener_centro_cubo_congelado()
        centro_dedos, _, _ = self.centro_dedos_actual()
        dx = centro_dedos[0] - centro_objeto[0]
        dy = centro_dedos[1] - centro_objeto[1]
        error = math.hypot(dx, dy)
        self.get_logger().info(
            f"CENTRADO FINAL frente al cubo congelado: "
            f"dx={dx * 1000:+.1f} mm; dy={dy * 1000:+.1f} mm; "
            f"error XY={error * 1000:.1f} mm."
        )
        if error > 0.007:
            self.get_logger().warning(
                "Error residual superior a 7 mm. "
                "No comenzar todavía el descenso."
            )

    def planificar(self, posicion_muneca, orientacion_muneca, ejecutar=False, abrir_al_llegar=True):
        """
        1. Congelar la escena.
        2. Calcular UNA trayectoria con MoveIt (Plan Only).
        3. Publicar y permitir su revisión en RViz.
        4. Si --execute true, ejecutar ESA MISMA trayectoria mediante
           /execute_trajectory después de la confirmación explícita.

        Esto evita que Plan + Execute vuelva a muestrear otra ruta.
        """
        self.congelar_scene()

        if not self.cliente_moveit.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveIt /move_action no está disponible.")

        goal = MoveGroup.Goal()
        goal.request.group_name = GRUPO_MOVEIT
        goal.request.pipeline_id = "ompl"
        goal.request.num_planning_attempts = 8
        # Permitir más tiempo al planificador estocástico RRTConnect.
        goal.request.allowed_planning_time = 20.0
        goal.request.max_velocity_scaling_factor = 0.10
        goal.request.max_acceleration_scaling_factor = 0.10
        goal.request.start_state.is_diff = True

        # Objetivo de posición: región esférica de RADIO 5 mm.
        # SolidPrimitive.SPHERE usa radio, no diámetro; 20 mm antes
        # permitían descentrados importantes frente a objetos pequeños.
        c_posicion = PositionConstraint()
        c_posicion.header.frame_id = FRAME_BASE
        c_posicion.link_name = FRAME_MUNECA
        c_posicion.weight = 1.0
        esfera = SolidPrimitive()
        esfera.type = SolidPrimitive.SPHERE
        esfera.dimensions = [0.005]
        centro = Pose()
        centro.position.x, centro.position.y, centro.position.z = posicion_muneca
        centro.orientation.w = 1.0
        c_posicion.constraint_region.primitives.append(esfera)
        c_posicion.constraint_region.primitive_poses.append(centro)

        # Objetivo de orientación: pinza dirigida verticalmente abajo.
        # Tolerancias ajustadas a 0.02 rad (~1.15 grados por eje).
        # La versión anterior admitía 0.12 rad y podía dejarla inclinada.
        c_orientacion = OrientationConstraint()
        c_orientacion.header.frame_id = FRAME_BASE
        c_orientacion.link_name = FRAME_MUNECA
        (
            c_orientacion.orientation.x,
            c_orientacion.orientation.y,
            c_orientacion.orientation.z,
            c_orientacion.orientation.w,
        ) = orientacion_muneca
        c_orientacion.absolute_x_axis_tolerance = 0.02
        c_orientacion.absolute_y_axis_tolerance = 0.02
        c_orientacion.absolute_z_axis_tolerance = 0.02
        c_orientacion.weight = 1.0

        restricciones = Constraints()
        restricciones.position_constraints.append(c_posicion)
        restricciones.orientation_constraints.append(c_orientacion)
        goal.request.goal_constraints.append(restricciones)

        # CLAVE: siempre solicitamos Plan Only y conservamos su resultado.
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.get_logger().info(
            "Solicitando PLAN ONLY con escena congelada. "
            "El robot NO se moverá durante la planificación."
        )
        envio = self.cliente_moveit.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, envio, timeout_sec=15.0)
        if not envio.done() or envio.result() is None:
            raise RuntimeError("MoveIt no respondió a la solicitud de planificación.")

        manejador = envio.result()
        if not manejador.accepted:
            raise RuntimeError("MoveIt rechazó el objetivo de planificación.")

        respuesta_futura = manejador.get_result_async()
        rclpy.spin_until_future_complete(self, respuesta_futura, timeout_sec=35.0)
        if not respuesta_futura.done() or respuesta_futura.result() is None:
            raise RuntimeError("No se recibió a tiempo el resultado de planificación.")

        respuesta = respuesta_futura.result()
        codigo = respuesta.result.error_code.val
        if (
            respuesta.status != GoalStatus.STATUS_SUCCEEDED
            or codigo != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError(
                "Planificación fallida. "
                f"Estado={respuesta.status}; código MoveIt={codigo}. "
                "No se ha ordenado ningún movimiento."
            )

        trayectoria = respuesta.result.planned_trajectory
        serie = trayectoria.joint_trajectory
        puntos = len(serie.points)
        if puntos < 2 or not serie.joint_names:
            raise RuntimeError("MoveIt devolvió una trayectoria vacía o incompleta.")

        # Métrica informativa: longitud en el espacio articular.
        # No equivale a distancia cartesiana ni garantiza optimalidad.
        recorrido = sum(
            math.sqrt(sum((b - a) ** 2 for a, b in zip(anterior.positions, siguiente.positions)))
            for anterior, siguiente in zip(serie.points[:-1], serie.points[1:])
        )

        self.get_logger().info(
            f"PLAN VERTICAL CORRECTO ({puntos} puntos). "
            f"Recorrido articular aprox.: {recorrido:.2f} rad."
        )

        # Publicación transitoria para visualizar exactamente la trayectoria
        # que devolverá MoveIt, antes de autorizar la ejecución.
        visualizacion = DisplayTrajectory()
        visualizacion.trajectory_start = respuesta.result.trajectory_start
        visualizacion.trajectory.append(trayectoria)
        self.publicador_trayectoria.publish(visualizacion)
        rclpy.spin_once(self, timeout_sec=1.0)

        if not ejecutar:
            self.get_logger().info(
                "Modo --execute false: no se ha movido el robot. "
                "El siguiente lanzamiento calculará otro plan diferente."
            )
            return False

        # IMPORTANTE: conservar el mismo plan y revisarlo en RViz.
        # Repetir el comando --execute true NO reutilizaría una ruta anterior.
        print("\nREVISA EN RVIZ LA TRAYECTORIA PUBLICADA.", flush=True)
        print(
            "Si hace movimientos excesivos o pasa cerca de un obstáculo, "
            "NO la autorices.", flush=True
        )
        try:
            autorizacion = input(
                "Escribe EJECUTAR para mover el brazo con ESTE plan "
                "(cualquier otra entrada cancela): "
            ).strip()
        except (EOFError, KeyboardInterrupt):
            autorizacion = ""

        if autorizacion != "EJECUTAR":
            self.get_logger().warning(
                "Ejecución cancelada por el usuario. "
                "La apertura de la Robotiq no se modifica."
            )
            return False

        # Comprobar que la postura de inicio sigue siendo la actual.
        # Si se ha movido durante la revisión, es inseguro reutilizar el plan.
        self.verificar_inicio_trayectoria(serie)
        self.ejecutar_trayectoria( trayectoria )

        # Mantener pinza abierta en ajuste fino; abrir solo en pre-grasp normal.
        self.comprobar_llegada(posicion_muneca, orientacion_muneca)
        self.comprobar_verticalidad(max_inclinacion_grados=2.5)
        if abrir_al_llegar:
            self.abrir_pinza()
        else:
            self.get_logger().info(
                "Ajuste fino finalizado. La Robotiq continúa ABIERTA; "
                "no se ha ordenado un movimiento de pinza."
            )
        return True

    def verificar_inicio_trayectoria(self, serie):
        """
        Bloquea la ejecución si el robot ha cambiado de postura
        desde la planificación. La verificación de MoveIt también
        comprobará el estado inicial antes de ejecutar.
        """
        # Recoger mensajes recientes después de la confirmación.
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        primer_punto = serie.points[0]
        diferencias = []
        for nombre, objetivo in zip(serie.joint_names, primer_punto.positions):
            actual = self.posiciones_articulares.get(nombre)
            if actual is None:
                raise RuntimeError(
                    f"No hay estado articular actual para {nombre}. "
                    "No se autoriza la ejecución."
                )
            diferencias.append((nombre, abs(actual - objetivo)))

        articulacion, error = max(diferencias, key=lambda dato: dato[1])
        if error > 0.08:
            raise RuntimeError(
                f"El brazo se ha movido respecto al inicio del plan: "
                f"{articulacion} difiere {error:.3f} rad. "
                "Vuelve a planificar desde su posición actual."
            )
        self.get_logger().info(
            f"Estado inicial verificado; máxima diferencia "
            f"articular {error:.3f} rad."
        )

    def ejecutar_trayectoria(self, trayectoria):
        """
        Envía exactamente el RobotTrajectory obtenido en Plan Only.

        MoveIt delega la ejecución al controlador y mantiene sus
        comprobaciones de comienzo de trayectoria y supervisión.
        No se calcula una trayectoria nueva.
        """
        if not self.cliente_ejecutar.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                "La acción /execute_trajectory no está disponible."
            )

        solicitud = ExecuteTrajectory.Goal()
        solicitud.trajectory = trayectoria
        # Lista vacía: MoveIt elige los controladores configurados.
        solicitud.controller_names = []

        self.get_logger().info(
            "Ejecutando EXACTAMENTE el plan previamente revisado en RViz. "
            "No se modifica la apertura de la Robotiq durante el movimiento."
        )
        envio = self.cliente_ejecutar.send_goal_async(solicitud)
        rclpy.spin_until_future_complete(self, envio, timeout_sec=15.0)
        if not envio.done() or envio.result() is None:
            raise RuntimeError("No hubo respuesta a la ejecución solicitada.")
        manejador = envio.result()
        if not manejador.accepted:
            raise RuntimeError("MoveIt rechazó la ejecución de la trayectoria.")

        respuesta_futura = manejador.get_result_async()
        rclpy.spin_until_future_complete(
            self, respuesta_futura, timeout_sec=120.0
        )
        if not respuesta_futura.done() or respuesta_futura.result() is None:
            raise RuntimeError(
                "No se recibió el resultado final de ejecución. "
                "Comprueba la posición real del UR5e antes de repetir."
            )
        respuesta = respuesta_futura.result()
        codigo = respuesta.result.error_code.val
        if (
            respuesta.status != GoalStatus.STATUS_SUCCEEDED
            or codigo != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError(
                f"Ejecución abortada o fallida. Estado de acción="
                f"{respuesta.status}, código MoveIt={codigo}. "
                "La pinza NO se abrirá."
            )
        self.get_logger().info(
            "TRAYECTORIA EJECUTADA CORRECTAMENTE. Comprobando TF2."
        )


    # ========================================================
    # VALIDACIÓN DE LLEGADA Y APERTURA DE PINZA
    # ========================================================

    def comprobar_llegada(self, objetivo_posicion, objetivo_orientacion):
        """
        Comprueba la pose REAL de la muñeca mediante TF2.

        Se evita abrir la pinza si el brazo no ha alcanzado
        el pre-grasp solicitado con un error de posicion maximo de 10 mm.
        """
        inicio = time.monotonic()
        objetivo_q = normalizar_cuaternion(objetivo_orientacion)
        ultimo_error = None

        while rclpy.ok() and (time.monotonic() - inicio) < 6.0:
            rclpy.spin_once(self, timeout_sec=0.10)
            try:
                tf_muneca = self.tf_buffer.lookup_transform(
                    FRAME_BASE, FRAME_MUNECA, Time()
                )
                pos_real = vector_transformacion(tf_muneca)
                q_real = cuaternion_transformacion(tf_muneca)
                error_pos = distancia(pos_real, objetivo_posicion)
                producto_escalar = abs(sum(a * b for a, b in zip(q_real, objetivo_q)))
                error_angulo = 2.0 * math.acos(min(1.0, max(0.0, producto_escalar)))
                ultimo_error = (error_pos, error_angulo)
                if error_pos <= 0.010 and error_angulo <= 0.05:
                    self.get_logger().info(
                        f"PRE-GRASP ALCANZADO: error de posición "
                        f"{error_pos * 1000:.1f} mm; "
                        f"error angular {math.degrees(error_angulo):.1f}°."
                    )
                    return
            except TransformException:
                pass

        detalle = (
            "TF2 no ofreció una transformación válida."
            if ultimo_error is None else
            f"Últimos errores: {ultimo_error[0]:.3f} m y "
            f"{math.degrees(ultimo_error[1]):.1f}°."
        )
        raise RuntimeError(
            "No se ha podido verificar la llegada a pre-grasp. "
            "Por seguridad, NO se abre la pinza. " + detalle
        )

    def comprobar_verticalidad(self, max_inclinacion_grados=2.5):
        """
        Verifica geométricamente la verticalidad REAL de la Robotiq.

        En el modelo URDF de este proyecto, el eje local +Z de la base
        de la pinza debe mirar hacia el -Z de base_link al hacer un
        agarre superior. Esta comprobación es independiente de que
        MoveIt haya devuelto SUCCESS.

        Solo se permite abrir la pinza si el error angular respecto
        a la vertical descendente es inferior al umbral elegido.
        """
        # TF2 no garantiza disponer de TODA la cadena al iniciar un nodo.
        # Durante los primeros instantes puede haber varios árboles parciales
        # aunque robot_state_publisher y el robot funcionen correctamente.
        # Esperamos a que la transformación esté disponible, procesando
        # callbacks de /tf y /tf_static antes de cada nuevo intento.
        limite = time.monotonic() + 12.0
        ultimo_error = None
        tf_pinza_actual = None
        while rclpy.ok() and time.monotonic() < limite:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf_pinza_actual = self.tf_buffer.lookup_transform(
                    FRAME_BASE, FRAME_PINZA, Time()
                )
                break
            except TransformException as error:
                ultimo_error = error

        if tf_pinza_actual is None:
            raise RuntimeError(
                "No se pudo verificar la verticalidad tras 12 s esperando "
                f"TF2 ({FRAME_BASE} -> {FRAME_PINZA}). "
                f"Último error: {ultimo_error}. "
                "Comprueba /tf, /tf_static y robot_state_publisher."
            )

        q_pinza = cuaternion_transformacion(tf_pinza_actual)
        eje_z_real = rotar_vector(q_pinza, (0.0, 0.0, 1.0))

        # Producto escalar con la dirección vertical descendente.
        coseno = max(-1.0, min(1.0, -eje_z_real[2]))
        inclinacion_grados = math.degrees(math.acos(coseno))

        self.get_logger().info(
            f"Eje +Z real de la Robotiq en {FRAME_BASE}: "
            f"[{eje_z_real[0]:+.4f}, {eje_z_real[1]:+.4f}, "
            f"{eje_z_real[2]:+.4f}] | "
            f"Inclinación respecto a la vertical descendente: "
            f"{inclinacion_grados:.2f}°"
        )
        if inclinacion_grados > max_inclinacion_grados:
            raise RuntimeError(
                f"Pre-grasp demasiado inclinado: {inclinacion_grados:.2f}° "
                f"(máximo permitido: {max_inclinacion_grados:.2f}°). "
                "La pinza NO se abrirá. Revisa el objetivo y el plan."
            )

    def abrir_pinza(self):
        """
        Abre la Robotiq a 0.05 rad mediante el controlador
        ParallelGripperCommand que ya se validó en Gazebo.

        Se llama solo DESPUÉS de comprobar la ejecución y TF2.
        """
        if not self.cliente_pinza.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                "No está disponible la acción de apertura de la Robotiq: "
                f"{ACCION_PINZA}"
            )

        comando = ParallelGripperCommand.Goal()
        comando.command.name = [JOINT_PINZA]
        comando.command.position = [POSICION_ABIERTA]

        self.get_logger().info(
            "Brazo en pre-grasp. Abriendo AHORA la Robotiq a 0.05 rad."
        )
        futuro_envio = self.cliente_pinza.send_goal_async(comando)
        rclpy.spin_until_future_complete(
            self, futuro_envio, timeout_sec=15.0
        )
        if not futuro_envio.done() or futuro_envio.result() is None:
            raise RuntimeError("No hubo respuesta a la orden de apertura.")

        manejador = futuro_envio.result()
        if not manejador.accepted:
            raise RuntimeError("El controlador rechazó la apertura.")

        futuro_resultado = manejador.get_result_async()
        rclpy.spin_until_future_complete(
            self, futuro_resultado, timeout_sec=20.0
        )
        if not futuro_resultado.done() or futuro_resultado.result() is None:
            raise RuntimeError(
                "No se confirmó la apertura; revisa el estado de la pinza."
            )

        resultado_accion = futuro_resultado.result()
        resultado = resultado_accion.result
        posicion_real = (
            resultado.state.position[0]
            if len(resultado.state.position) else float("nan")
        )
        if (
            resultado_accion.status != GoalStatus.STATUS_SUCCEEDED
            or not resultado.reached_goal
            or resultado.stalled
            or not math.isfinite(posicion_real)
            or abs(posicion_real - POSICION_ABIERTA) > 0.05
        ):
            raise RuntimeError(
                "La apertura no terminó correctamente. "
                f"Estado={resultado_accion.status}, "
                f"reached_goal={resultado.reached_goal}, "
                f"stalled={resultado.stalled}, "
                f"posición={posicion_real:.4f} rad."
            )

        self.get_logger().info(
            f"ROBOTIQ ABIERTA EN PRE-GRASP: {posicion_real:.4f} rad. "
            "No se ha iniciado ningún descenso ni agarre."
        )


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Planificar/ejecutar pre-grasp vertical y abrir la pinza al llegar."
    )
    parser.add_argument(
        "--object", choices=["apple"], default="apple",
        help="Objeto para esta primera validación.",
    )
    parser.add_argument(
        "--diameter", type=float, default=0.075,
        help="Diámetro supuesto de la manzana en metros.",
    )
    parser.add_argument(
        "--clearance", type=float, default=0.15,
        help="Altura del centro de los dedos sobre el centro de la manzana.",
    )
    parser.add_argument(
        "--yaw-deg", type=float, default=0.0,
        help="Rotación alrededor de la vertical en grados.",
    )
    parser.add_argument(
        "--plan", choices=["true", "false"], default="false",
        help="true: solicitar planificación. false: solo mostrar la postura.",
    )
    parser.add_argument(
        "--execute", choices=["true", "false"], default="false",
        help=(
            "true: mostrar plan, pedir autorización, ejecutar el MISMO plan "
            "y abrir al llegar; false: solo planificar."
        ),
    )
    parser.add_argument(
        "--refine-xy", choices=["true", "false"], default="false",
        help=(
            "true: desde pre-grasp con pinza ABIERTA, corrige SOLO X/Y "
            "respecto al cubo congelado amm_detected_apple. "
            "No depende de una nueva detección YOLO y NO baja ni cierra la pinza."
        ),
    )
    parser.add_argument("--timeout", type=float, default=15.0)
    argumentos, _ = parser.parse_known_args(args)

    if not (0.05 <= argumentos.diameter <= 0.11):
        parser.error("--diameter debe estar entre 0.05 y 0.11 m.")
    if not (0.10 <= argumentos.clearance <= 0.25):
        parser.error("--clearance debe estar entre 0.10 y 0.25 m.")
    if not math.isfinite(argumentos.yaw_deg):
        parser.error("--yaw-deg debe ser finito.")
    if argumentos.timeout <= 0:
        parser.error("--timeout debe ser positivo.")
    if argumentos.execute == "true" and argumentos.plan != "true":
        parser.error("Para mover el robot, usa --plan true --execute true.")

    rclpy.init(args=args)
    nodo = None
    try:
        nodo = VerticalGrasp()
        if argumentos.refine_xy == "true":
            # Posicionamiento fino desde el pre-grasp ya alcanzado:
            # no cerrar pinza ni utilizar una imagen posiblemente ocluida.
            nodo.congelar_scene()
            centro_cubo = nodo.obtener_centro_cubo_congelado()
            posicion, orientacion = nodo.calcular_refinamiento_xy(centro_cubo)
            if posicion is not None and argumentos.plan == "true":
                ejecutar = argumentos.execute == "true"
                realizado = nodo.planificar(
                    posicion, orientacion,
                    ejecutar=ejecutar, abrir_al_llegar=False,
                )
                if realizado:
                    nodo.imprimir_centrado_posterior()
            elif argumentos.plan != "true":
                nodo.get_logger().info("Solo cálculo del ajuste, sin movimiento.")
        else:
            datos = nodo.esperar_datos(argumentos.timeout)
            posicion, orientacion = nodo.calcular_pregrasp(
                datos,
                argumentos.diameter,
                argumentos.clearance,
                argumentos.yaw_deg,
            )
            if argumentos.plan == "true":
                # Planificar o ejecutar el movimiento de pre-grasp.
                ejecutar = argumentos.execute == "true"

                realizado = nodo.planificar(
                    posicion,
                    orientacion,
                    ejecutar=ejecutar,
                )

                # Por ahora, comprobamos el valor de retorno únicamente
                # cuando solicitamos ejecución. En modo PLAN ONLY, la
                # función actual no devuelve necesariamente True aunque
                # MoveIt haya planificado correctamente.
                if ejecutar and not realizado:
                    raise RuntimeError(
                        "No se ha completado correctamente la ejecución "
                        "del pre-grasp vertical."
                    )
            else:
                nodo.get_logger().info(
                    "Postura calculada. Sin planificación ni ejecución."
                )
        return 0
    except (RuntimeError, ValueError, OSError, KeyError) as error:
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