#!/usr/bin/env python3
"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - OBJETOS EN PLANNING SCENE
============================================================

Paquete: amm_manipulation
Archivo: detected_objects_scene.py

Objetivo:
    Añadir a MoveIt 2 los obstáculos detectados por YOLO + RGB-D.
    Permitir congelar las actualizaciones durante los movimientos.

Entradas:
    /perception/<clase>/point_base  (geometry_msgs/PointStamped)

Servicios:
    /apply_planning_scene                       (MoveIt 2)
    /amm_manipulation/freeze_scene               (std_srvs/SetBool)

Contrato del servicio freeze_scene:
    data: true  -> congelar las actualizaciones.
    data: false -> permitir nuevas actualizaciones.

La escena empieza CONGELADA. Se habilita la actualización
únicamente cuando el brazo está detenido observando la mesa.

Notas:
    - No se consultan posiciones reales de Gazebo.
    - Cada clase se representa mediante una caja aproximada.
    - Esta versión admite una instancia por clase.
    - No elimina automáticamente las detecciones perdidas.
    - No modifica los objetos de colisión cuando está congelada.
============================================================
"""

import math
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from std_msgs.msg import UInt64

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)


CONFIG_FILE = "detected_objects.yaml"
SERVICIO_SCENE = "/apply_planning_scene"
SERVICIO_CONGELACION = "/amm_manipulation/freeze_scene"
PREFIJO_OBJETOS = "amm_detected_"


def rotar_vector(vector, quaternion):
    """Rota un vector tridimensional utilizando un cuaternión unitario."""
    x, y, z = vector
    qx, qy, qz, qw = (
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    )

    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)

    cx = qy * tz - qz * ty
    cy = qz * tx - qx * tz
    cz = qx * ty - qy * tx

    return (x + qw * tx + cx, y + qw * ty + cy, z + qw * tz + cz)


def distancia(punto_a, punto_b):
    """Distancia euclídea entre dos puntos XYZ."""
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(punto_a, punto_b)))


class DetectedObjectsScene(Node):
    """Puente de detecciones a MoveIt, con congelación controlada."""

    def __init__(self):
        super().__init__("amm_detected_objects_scene")

        ruta = (
            Path(get_package_share_directory("amm_manipulation"))
            / "config"
            / CONFIG_FILE
        )
        with ruta.open("r", encoding="utf-8") as archivo:
            configuracion = yaml.safe_load(archivo)

        self.planning_frame = configuracion["planning_frame"]
        self.periodo = float(configuracion["update_period"])
        self.desplazamiento_minimo = float(
            configuracion["minimum_displacement"]
        )
        self.objetos = configuracion["objects"]

        if not isinstance(self.planning_frame, str) or not self.planning_frame:
            raise ValueError("planning_frame debe ser una cadena no vacía.")
        if not math.isfinite(self.periodo) or self.periodo <= 0:
            raise ValueError("update_period debe ser positivo y finito.")
        if (
            not math.isfinite(self.desplazamiento_minimo)
            or self.desplazamiento_minimo < 0
        ):
            raise ValueError("minimum_displacement debe ser finito y no negativo.")
        if not isinstance(self.objetos, dict) or not self.objetos:
            raise ValueError("objects debe contener las clases configuradas.")

        for clase, datos in self.objetos.items():
            dimensiones = datos["dimensions"]
            if (
                len(dimensiones) != 3
                or any(
                    not math.isfinite(float(valor)) or float(valor) <= 0
                    for valor in dimensiones
                )
            ):
                raise ValueError(f"Dimensiones incorrectas: {clase}")

        # TF2 permite fijar cada detección en la referencia de planificación.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Cliente para enviar cambios diferenciales a MoveIt 2.
        self.scene_client = self.create_client(
            ApplyPlanningScene, SERVICIO_SCENE
        )

        # Estado y datos de la observación.
        # IMPORTANTE: comenzar congelado evita obstáculos inesperados
        # antes de que el usuario habilite expresamente una observación.
        self.congelada = True
        self.ultimas_detecciones = {}
        self.posiciones_registradas = {}
        self.solicitud_pendiente = None

        # ============================================================
        # CONFIRMACIÓN DE ACTUALIZACIÓN DE LA MANZANA
        # ============================================================

        # Contador de actualizaciones aceptadas por MoveIt.
        self.revision_manzana = 0

        # TRANSIENT_LOCAL permite que un coordinador que se
        # conecte posteriormente recupere la última revisión.
        qos_revision = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publicador_revision_manzana = self.create_publisher(
            UInt64,
            "/amm_manipulation/apple_scene_revision",
            qos_revision,
        )

        # Publicamos el valor inicial.
        self.publicador_revision_manzana.publish(UInt64(data=0))

        # Servicio de control: data=true congela, data=false reanuda.
        self.servicio_congelacion = self.create_service(
            SetBool,
            SERVICIO_CONGELACION,
            self.congelacion_callback,
        )

        # Mantener referencias de las suscripciones durante toda la ejecución.
        self.suscripciones = []
        for clase, datos in self.objetos.items():
            topico = datos["topic"]
            suscripcion = self.create_subscription(
                PointStamped,
                topico,
                lambda mensaje, nombre=clase: self.deteccion_callback(
                    mensaje, nombre
                ),
                qos_profile_sensor_data,
            )
            self.suscripciones.append(suscripcion)
            self.get_logger().info(f"Escuchando {clase}: {topico}")

        self.timer = self.create_timer(self.periodo, self.actualizar_scene)
        self.get_logger().info(
            "Puente de percepción iniciado. "
            "Actualizaciones CONGELADAS por defecto."
        )

    def congelacion_callback(self, solicitud, respuesta):
        """Permite o bloquea actualizaciones de objetos en MoveIt 2."""

        if solicitud.data:
            # Bloquear de inmediato la creación de nuevas solicitudes.
            self.congelada = True
            self.ultimas_detecciones.clear()

            # Una petición ya enviada no se puede deshacer. El servicio
            # avisa para que no se inicie el movimiento todavía.
            if self.solicitud_pendiente is not None:
                respuesta.success = False
                respuesta.message = (
                    "Actualizaciones detenidas, pero hay una petición "
                    "a MoveIt pendiente. Repite la congelación en 1 s "
                    "y espera success=true antes de planificar."
                )
            else:
                respuesta.success = True
                respuesta.message = (
                    "Planning Scene congelada: los obstáculos permanecen "
                    "registrados, sin nuevas actualizaciones de YOLO."
                )
            self.get_logger().info(respuesta.message)
            return respuesta

        # Habilitar una observación NUEVA, sin reutilizar mensajes
        # recibidos mientras el robot se estaba moviendo.
        if self.solicitud_pendiente is not None:
            respuesta.success = False
            respuesta.message = (
                "Hay una modificación de escena pendiente; "
                "reintenta cuando termine."
            )
            return respuesta

        self.ultimas_detecciones.clear()
        self.posiciones_registradas.clear()
        self.congelada = False
        respuesta.success = True
        respuesta.message = (
            "Actualizaciones habilitadas. Se utilizarán únicamente "
            "detecciones recibidas desde ahora."
        )
        self.get_logger().info(respuesta.message)
        return respuesta

    def deteccion_callback(self, mensaje, clase):
        """Conservar una medición nueva solo durante la observación."""
        if self.congelada:
            return

        valores = (mensaje.point.x, mensaje.point.y, mensaje.point.z)
        if not all(math.isfinite(valor) for valor in valores):
            self.get_logger().warning(f"Detección no válida: {clase}")
            return
        if not mensaje.header.frame_id:
            self.get_logger().warning(f"Detección sin frame_id: {clase}")
            return

        self.ultimas_detecciones[clase] = mensaje

    def transformar_punto(self, mensaje):
        """Transformar el punto observado al marco de planificación."""
        origen = mensaje.header.frame_id
        punto = (mensaje.point.x, mensaje.point.y, mensaje.point.z)
        if origen == self.planning_frame:
            return punto

        transformacion = self.tf_buffer.lookup_transform(
            self.planning_frame,
            origen,
            Time.from_msg(mensaje.header.stamp),
            timeout=Duration(seconds=0.0),
        )
        rotacion = transformacion.transform.rotation
        traslacion = transformacion.transform.translation
        punto_rotado = rotar_vector(punto, rotacion)
        return (
            punto_rotado[0] + traslacion.x,
            punto_rotado[1] + traslacion.y,
            punto_rotado[2] + traslacion.z,
        )

    def crear_objeto_colision(self, clase, posicion):
        """Crear la caja de colisión aproximada de una clase detectada."""
        objeto = CollisionObject()
        objeto.id = f"{PREFIJO_OBJETOS}{clase}"
        objeto.header.frame_id = self.planning_frame
        objeto.header.stamp = self.get_clock().now().to_msg()
        objeto.operation = CollisionObject.ADD

        geometria = SolidPrimitive()
        geometria.type = SolidPrimitive.BOX
        geometria.dimensions = [
            float(valor) for valor in self.objetos[clase]["dimensions"]
        ]

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = posicion
        pose.orientation.w = 1.0
        objeto.primitives.append(geometria)
        objeto.primitive_poses.append(pose)
        return objeto

    def actualizar_scene(self):
        """Enviar una sola modificación asíncrona por ciclo, si está permitido."""
        if self.congelada or self.solicitud_pendiente is not None:
            return
        if not self.scene_client.service_is_ready():
            return

        for clase in self.objetos:
            mensaje = self.ultimas_detecciones.get(clase)
            if mensaje is None:
                continue

            try:
                posicion = self.transformar_punto(mensaje)
            except TransformException:
                continue

            anterior = self.posiciones_registradas.get(clase)
            if anterior is not None and distancia(
                posicion, anterior
            ) < self.desplazamiento_minimo:
                continue

            objeto = self.crear_objeto_colision(clase, posicion)
            escena = PlanningScene()
            escena.is_diff = True
            escena.robot_state.is_diff = True
            escena.world.collision_objects.append(objeto)
            solicitud = ApplyPlanningScene.Request()
            solicitud.scene = escena

            # Almacenar los datos antes del callback de la respuesta.
            self.solicitud_pendiente = (clase, posicion)
            try:
                futuro = self.scene_client.call_async(solicitud)
                futuro.add_done_callback(self.respuesta_scene_callback)
            except Exception as error:
                self.solicitud_pendiente = None
                self.get_logger().error(
                    f"No se pudo solicitar actualización de {clase}: {error}"
                )
            return

    def respuesta_scene_callback(self, futuro):
        """Confirmar solo los obstáculos efectivamente aceptados por MoveIt."""
        pendiente = self.solicitud_pendiente
        if pendiente is None:
            return

        clase, posicion = pendiente
        try:
            respuesta = futuro.result()
            if respuesta is None or not respuesta.success:
                self.get_logger().error(
                    f"MoveIt rechazó el objeto: {clase}"
                )
                return

            self.posiciones_registradas[clase] = posicion

            # Notificar únicamente actualizaciones de la manzana
            # que MoveIt haya aceptado correctamente.
            if clase == "apple":

                self.revision_manzana += 1

                mensaje = UInt64()
                mensaje.data = self.revision_manzana

                self.publicador_revision_manzana.publish(mensaje)

                self.get_logger().info(
                    "Nueva revisión de manzana confirmada: "
                    f"{self.revision_manzana}"
                )
            self.get_logger().info(
                f"Objeto actualizado: {clase} | "
                f"Frame: {self.planning_frame} | "
                f"XYZ: [{posicion[0]:.3f}, "
                f"{posicion[1]:.3f}, {posicion[2]:.3f}]"
            )
        except Exception as error:
            self.get_logger().error(
                f"Error actualizando {clase}: {error}"
            )
        finally:
            self.solicitud_pendiente = None


def main(args=None):
    rclpy.init(args=args)
    nodo = None
    try:
        nodo = DetectedObjectsScene()
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if nodo is not None:
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()