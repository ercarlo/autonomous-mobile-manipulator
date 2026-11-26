#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - PLANNING SCENE MANAGER
============================================================

Paquete: amm_manipulation
Archivo: planning_scene.py

Objetivo:
    Gestionar los objetos de colisión del entorno
    de planificación de MoveIt 2.

Funcionalidades:
    - Añadir obstáculos geométricos.
    - Actualizar obstáculos existentes.
    - Eliminar obstáculos.
    - Consultar los objetos registrados.

Interfaces ROS 2:
    /apply_planning_scene
    /get_planning_scene

Esta primera versión utiliza cajas geométricas.

Las posiciones se reciben mediante argumentos y NO
se obtienen directamente de Gazebo.

En etapas posteriores se integrarán las detecciones
de YOLO11s-seg, la profundidad y TF2.

============================================================
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.msg import (
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)

from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetPlanningScene,
)


# ============================================================
# CONFIGURACIÓN
# ============================================================

SERVICIO_APLICAR = "/apply_planning_scene"
SERVICIO_CONSULTAR = "/get_planning_scene"

TIMEOUT_SERVICIO = 5.0
TIMEOUT_RESPUESTA = 10.0

# Identificador reservado para los objetos creados
# por nuestra aplicación de manipulación.

PREFIJO_OBJETOS = "amm_"


# ============================================================
# GESTOR DE PLANNING SCENE
# ============================================================

class PlanningSceneManager(Node):

    def __init__(self):

        super().__init__("amm_planning_scene_manager")

        # Cliente para aplicar modificaciones a la escena.
        self.cliente_aplicar = self.create_client(
            ApplyPlanningScene,
            SERVICIO_APLICAR,
        )

        # Cliente para consultar la escena actual.
        self.cliente_consultar = self.create_client(
            GetPlanningScene,
            SERVICIO_CONSULTAR,
        )

        self.get_logger().info(
            "Gestor de Planning Scene inicializado."
        )

    # ========================================================
    # LLAMADAS A SERVICIOS ROS 2
    # ========================================================

    def llamar_servicio(self, cliente, solicitud):

        """
        Realiza una llamada a un servicio ROS 2.

        Comprueba:
            - Disponibilidad del servicio.
            - Tiempo máximo de respuesta.
            - Errores durante la comunicación.

        Devuelve:
            Respuesta del servicio o None.
        """

        if not cliente.wait_for_service(
            timeout_sec=TIMEOUT_SERVICIO
        ):

            self.get_logger().error(
                f"Servicio no disponible: {cliente.srv_name}"
            )

            return None

        futuro = cliente.call_async(solicitud)

        rclpy.spin_until_future_complete(
            self,
            futuro,
            timeout_sec=TIMEOUT_RESPUESTA,
        )

        if not futuro.done():

            self.get_logger().error(
                "Tiempo de espera agotado."
            )

            return None

        if futuro.exception() is not None:

            self.get_logger().error(
                f"Error en el servicio: {futuro.exception()}"
            )

            return None

        return futuro.result()

    # ========================================================
    # CREACIÓN DE OBJETOS DE COLISIÓN
    # ========================================================

    def crear_caja(
        self,
        identificador,
        frame,
        posicion,
        dimensiones,
    ):

        """
        Crea un CollisionObject con geometría de caja.

        Parámetros:
            identificador:
                Nombre único del obstáculo.

            frame:
                Sistema de referencia de la posición.

            posicion:
                Coordenadas XYZ del centro de la caja.

            dimensiones:
                Longitud X, anchura Y y altura Z.

        Todas las magnitudes espaciales están en metros.
        """

        objeto = CollisionObject()

        # Identificación y sistema de referencia.

        objeto.id = identificador

        objeto.header.frame_id = frame

        objeto.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # Operación de incorporación a la escena.

        objeto.operation = CollisionObject.ADD

        # Geometría de colisión simplificada.

        geometria = SolidPrimitive()

        geometria.type = SolidPrimitive.BOX

        geometria.dimensions = [
            float(dimensiones[0]),
            float(dimensiones[1]),
            float(dimensiones[2]),
        ]

        # Pose del centro geométrico del obstáculo.

        pose = Pose()

        pose.position.x = float(posicion[0])
        pose.position.y = float(posicion[1])
        pose.position.z = float(posicion[2])

        # Orientación identidad: caja sin rotación.

        pose.orientation.w = 1.0

        objeto.primitives.append(geometria)
        objeto.primitive_poses.append(pose)

        return objeto

    # ========================================================
    # APLICAR MODIFICACIONES
    # ========================================================

    def aplicar_objeto(self, objeto):

        """
        Envía una modificación a MoveIt 2.

        Se utiliza una Planning Scene diferencial para
        conservar los objetos y configuraciones existentes.
        """

        escena = PlanningScene()

        # Indicar que únicamente enviamos diferencias.
        # No queremos reemplazar toda la escena existente.

        escena.is_diff = True
        escena.robot_state.is_diff = True

        escena.world.collision_objects.append(objeto)

        solicitud = ApplyPlanningScene.Request()

        solicitud.scene = escena

        respuesta = self.llamar_servicio(
            self.cliente_aplicar,
            solicitud,
        )

        if respuesta is None:
            return False

        if not respuesta.success:

            self.get_logger().error(
                "MoveIt ha rechazado la modificación."
            )

            return False

        self.get_logger().info(
            f"Modificación aplicada: {objeto.id}"
        )

        return True

    # ========================================================
    # AÑADIR O ACTUALIZAR OBSTÁCULO
    # ========================================================

    def agregar_caja(
        self,
        identificador,
        frame,
        posicion,
        dimensiones,
    ):

        """
        Añade una caja a la Planning Scene.

        Si existe un objeto con el mismo identificador,
        MoveIt actualizará su representación.
        """

        objeto = self.crear_caja(
            identificador,
            frame,
            posicion,
            dimensiones,
        )

        self.get_logger().info(
            f"Añadiendo obstáculo: {identificador}"
        )

        return self.aplicar_objeto(objeto)

    # ========================================================
    # ELIMINAR OBSTÁCULO
    # ========================================================

    def eliminar_objeto(self, identificador):

        """
        Elimina un objeto de colisión mediante su ID.

        No modifica los demás elementos de la escena.
        """

        objeto = CollisionObject()

        objeto.id = identificador
        objeto.operation = CollisionObject.REMOVE

        self.get_logger().info(
            f"Eliminando obstáculo: {identificador}"
        )

        return self.aplicar_objeto(objeto)

    # ========================================================
    # CONSULTAR OBSTÁCULOS
    # ========================================================

    def consultar_objetos(self):

        """
        Obtiene los objetos de colisión presentes
        en el mundo de planificación de MoveIt 2.
        """

        solicitud = GetPlanningScene.Request()

        # Solicitar únicamente la geometría de los
        # obstáculos del mundo, evitando recuperar
        # información innecesaria de toda la escena.

        solicitud.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )

        respuesta = self.llamar_servicio(
            self.cliente_consultar,
            solicitud,
        )

        if respuesta is None:
            return None

        return respuesta.scene.world.collision_objects

    def listar_objetos(self):

        """
        Muestra los identificadores y las geometrías
        de los obstáculos conocidos por MoveIt.
        """

        objetos = self.consultar_objetos()

        if objetos is None:
            return False

        if not objetos:

            self.get_logger().info(
                "La Planning Scene no contiene "
                "obstáculos del mundo."
            )

            return True

        self.get_logger().info(
            f"Objetos registrados: {len(objetos)}"
        )

        for objeto in objetos:

            self.get_logger().info(
                f"ID: {objeto.id} | "
                f"Frame: {objeto.header.frame_id} | "
                f"Primitivas: {len(objeto.primitives)}"
            )

        return True


# ============================================================
# FUNCIÓN PRINCIPAL
# ============================================================

def main(args=None):

    # --------------------------------------------------------
    # ARGUMENTOS
    # --------------------------------------------------------

    parser = argparse.ArgumentParser(
        description="Gestor de Planning Scene del AMM."
    )

    parser.add_argument(
        "--op",
        choices=["add", "remove", "list"],
        required=True,
        help="Operación que se desea realizar.",
    )

    parser.add_argument(
        "--id",
        type=str,
        default=None,
        help="Identificador del obstáculo.",
    )

    parser.add_argument(
        "--frame",
        type=str,
        default="base_link",
        help="Sistema de referencia del obstáculo.",
    )

    parser.add_argument(
        "--position",
        type=float,
        nargs=3,
        default=None,
        metavar=("X", "Y", "Z"),
        help="Posición XYZ del centro en metros.",
    )

    parser.add_argument(
        "--size",
        type=float,
        nargs=3,
        default=None,
        metavar=("SX", "SY", "SZ"),
        help="Dimensiones de la caja en metros.",
    )

    argumentos, _ = parser.parse_known_args(args)

    # --------------------------------------------------------
    # VALIDAR ARGUMENTOS
    # --------------------------------------------------------

    if argumentos.op in ("add", "remove"):

        if not argumentos.id:

            parser.error(
                "Las operaciones add y remove "
                "requieren --id."
            )

        # Evitar modificar accidentalmente objetos
        # que no pertenecen a nuestra aplicación.

        if not argumentos.id.startswith(PREFIJO_OBJETOS):

            parser.error(
                f"El ID debe comenzar por "
                f"'{PREFIJO_OBJETOS}'."
            )

    if argumentos.op == "add":

        if argumentos.position is None:

            parser.error(
                "La operación add requiere --position."
            )

        if argumentos.size is None:

            parser.error(
                "La operación add requiere --size."
            )

        if any(
            not math.isfinite(valor)
            for valor in argumentos.position + argumentos.size
        ):

            parser.error(
                "Las posiciones y dimensiones "
                "deben ser valores finitos."
            )

        if any(
            dimension <= 0
            for dimension in argumentos.size
        ):

            parser.error(
                "Las dimensiones deben ser positivas."
            )

    # --------------------------------------------------------
    # INICIALIZAR ROS 2
    # --------------------------------------------------------

    rclpy.init(args=args)

    gestor = None

    correcto = False

    try:

        gestor = PlanningSceneManager()

        if argumentos.op == "add":

            correcto = gestor.agregar_caja(
                identificador=argumentos.id,
                frame=argumentos.frame,
                posicion=argumentos.position,
                dimensiones=argumentos.size,
            )

        elif argumentos.op == "remove":

            correcto = gestor.eliminar_objeto(
                argumentos.id
            )

        elif argumentos.op == "list":

            correcto = gestor.listar_objetos()

    except Exception as error:

        if gestor is not None:

            gestor.get_logger().error(
                f"Error en Planning Scene: {error}"
            )

        else:

            print(
                f"Error inicializando gestor: {error}",
                file=sys.stderr,
            )

    finally:

        if gestor is not None:
            gestor.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0 if correcto else 1


if __name__ == "__main__":
    sys.exit(main())