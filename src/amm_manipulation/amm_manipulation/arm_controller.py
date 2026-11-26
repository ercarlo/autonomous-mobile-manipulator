#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - ARM CONTROLLER
============================================================

Paquete: amm_manipulation
Archivo: arm_controller.py

Objetivo:
    Controlar el UR5e mediante las acciones oficiales
    de MoveIt 2.

Funcionalidades:
    - Conectarse al servidor /move_action.
    - Leer configuraciones articulares desde el SRDF.
    - Establecer objetivos home y observation.
    - Utilizar el estado actual como punto de partida.
    - Solicitar planificación mediante MoveIt 2.
    - Ejecutar opcionalmente la trayectoria.
    - Comprobar los resultados de la acción.

Arquitectura:
    Este nodo utiliza move_group como servidor de
    planificación y ejecución.

    No crea una instancia propia de MoveItCpp.

============================================================
"""

import argparse
import os
import xml.etree.ElementTree as ET

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
)

from ament_index_python.packages import (
    get_package_share_directory,
)


# ============================================================
# CONFIGURACIÓN
# ============================================================

GRUPO_PLANIFICACION = "ur5e_arm"

ACCION_MOVEIT = "/move_action"

ESTADOS_DISPONIBLES = [
    "home",
    "observation",
    "transport",
    "place_approach_mesa1",
    "place_drop_mesa1",

]

TIEMPO_PLANIFICACION = 5.0

ESCALA_VELOCIDAD = 0.2
ESCALA_ACELERACION = 0.2

TOLERANCIA_ARTICULAR = 0.01


# ============================================================
# CONTROLADOR DEL UR5E
# ============================================================

class ArmController(Node):

    def __init__(self):

        super().__init__("amm_arm_controller")

        # Cliente de la acción oficial de MoveIt 2.
        #
        # move_group se encarga de planificar y ejecutar
        # las trayectorias solicitadas.

        self.cliente_moveit = ActionClient(
            self,
            MoveGroup,
            ACCION_MOVEIT,
        )

        # Cargar las configuraciones articulares definidas
        # en el SRDF del robot.

        self.configuraciones = self.cargar_configuraciones()

        self.get_logger().info(
            "Controlador del UR5e inicializado."
        )

        self.get_logger().info(
            f"Grupo de planificación: {GRUPO_PLANIFICACION}"
        )

    # ========================================================
    # LECTURA DEL SRDF
    # ========================================================

    def cargar_configuraciones(self):

        """
        Obtiene las configuraciones articulares definidas
        en el archivo SRDF del robot.

        Esto evita duplicar las posiciones articulares
        de home y observation en nuestro código.
        """

        ruta_paquete = get_package_share_directory(
            "amm_moveit_config"
        )

        ruta_srdf = os.path.join(
            ruta_paquete,
            "config",
            "autonomous_mobile_manipulator.srdf",
        )

        robot = ET.parse(ruta_srdf).getroot()

        configuraciones = {}

        for estado in robot.findall("group_state"):

            # Utilizar únicamente las configuraciones
            # correspondientes al brazo UR5e.

            if estado.get("group") != GRUPO_PLANIFICACION:
                continue

            nombre_estado = estado.get("name")

            articulaciones = {}

            for joint in estado.findall("joint"):

                nombre_joint = joint.get("name")

                posicion = float(
                    joint.get("value")
                )

                articulaciones[nombre_joint] = posicion

            configuraciones[nombre_estado] = articulaciones

        return configuraciones

    # ========================================================
    # GENERACIÓN DE RESTRICCIONES ARTICULARES
    # ========================================================

    def crear_restricciones(self, objetivo):

        """
        Convierte una configuración del SRDF en
        restricciones articulares de MoveIt 2.

        Cada articulación tendrá:
            - Posición objetivo.
            - Tolerancia superior.
            - Tolerancia inferior.
            - Peso de la restricción.
        """

        articulaciones = self.configuraciones[objetivo]

        restricciones = Constraints()

        restricciones.name = objetivo

        for nombre, posicion in articulaciones.items():

            restriccion = JointConstraint()

            restriccion.joint_name = nombre

            restriccion.position = posicion

            restriccion.tolerance_above = (
                TOLERANCIA_ARTICULAR
            )

            restriccion.tolerance_below = (
                TOLERANCIA_ARTICULAR
            )

            restriccion.weight = 1.0

            restricciones.joint_constraints.append(
                restriccion
            )

        return restricciones

    # ========================================================
    # GENERACIÓN DEL OBJETIVO MOVEIT
    # ========================================================

    def crear_objetivo(self, objetivo, ejecutar):

        """
        Prepara una solicitud de planificación.

        Utiliza:
            - Grupo ur5e_arm.
            - Estado articular actual.
            - Restricciones del estado objetivo.
            - Planificador OMPL.

        Si ejecutar=False:
            Solo calcula la trayectoria.

        Si ejecutar=True:
            Solicita planificación y ejecución.
        """

        goal = MoveGroup.Goal()

        # Grupo de articulaciones que queremos mover.

        goal.request.group_name = GRUPO_PLANIFICACION

        # Utilizar el estado actual del robot.
        #
        # is_diff=True indica que el estado inicial
        # debe completarse a partir del estado actual.

        goal.request.start_state.is_diff = True

        # Establecer el objetivo articular.

        goal.request.goal_constraints = [
            self.crear_restricciones(objetivo)
        ]

        # Configuración del planificador.

        goal.request.pipeline_id = "ompl"

        goal.request.num_planning_attempts = 1

        goal.request.allowed_planning_time = (
            TIEMPO_PLANIFICACION
        )

        # Reducir velocidades y aceleraciones durante
        # nuestras primeras pruebas de manipulación.

        goal.request.max_velocity_scaling_factor = (
            ESCALA_VELOCIDAD
        )

        goal.request.max_acceleration_scaling_factor = (
            ESCALA_ACELERACION
        )

        # Configuración de la ejecución.
        #
        # plan_only=True:
        #     Planificar sin ejecutar.
        #
        # plan_only=False:
        #     Planificar y ejecutar.

        goal.planning_options.plan_only = not ejecutar

        goal.planning_options.look_around = False

        goal.planning_options.replan = False

        # Mantener la escena actual de MoveIt 2.

        goal.planning_options.planning_scene_diff.is_diff = True

        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        return goal

    # ========================================================
    # PLANIFICACIÓN Y EJECUCIÓN
    # ========================================================

    def mover_a(self, objetivo, ejecutar=False):

        """
        Envía una solicitud al servidor move_group.

        Devuelve:
            True si la operación ha finalizado correctamente.
            False si se produce algún error.
        """

        # Comprobar que existe la configuración solicitada.

        if objetivo not in ESTADOS_DISPONIBLES:

            self.get_logger().error(
                f"Objetivo no permitido: {objetivo}"
            )

            return False

        if objetivo not in self.configuraciones:

            self.get_logger().error(
                f"El estado {objetivo} no existe en el SRDF."
            )

            return False

        # Esperar al servidor MoveIt 2.

        self.get_logger().info(
            "Esperando al servidor /move_action..."
        )

        if not self.cliente_moveit.wait_for_server(
            timeout_sec=10.0
        ):

            self.get_logger().error(
                "No se encuentra el servidor MoveIt 2."
            )

            return False

        # Construir la solicitud.

        goal = self.crear_objetivo(
            objetivo,
            ejecutar,
        )

        self.get_logger().info(
            f"Objetivo solicitado: {objetivo}"
        )

        self.get_logger().info(
            f"Ejecución habilitada: {ejecutar}"
        )

        # Enviar la solicitud de forma asíncrona.

        futuro_envio = self.cliente_moveit.send_goal_async(
            goal
        )

        # Esperar a que MoveIt acepte o rechace la solicitud.

        rclpy.spin_until_future_complete(
            self,
            futuro_envio,
        )

        if futuro_envio.exception() is not None:

            self.get_logger().error(
                f"Error enviando objetivo: {futuro_envio.exception()}"
            )

            return False

        goal_handle = futuro_envio.result()

        if goal_handle is None or not goal_handle.accepted:

            self.get_logger().error(
                "MoveIt ha rechazado el objetivo."
            )

            return False

        self.get_logger().info(
            "Objetivo aceptado por MoveIt 2."
        )

        # Esperar al resultado de planificación/ejecución.

        futuro_resultado = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            futuro_resultado,
        )

        if futuro_resultado.exception() is not None:

            self.get_logger().error(
                f"Error esperando resultado: {futuro_resultado.exception()}"
            )

            return False

        respuesta = futuro_resultado.result()

        if respuesta is None:

            self.get_logger().error(
                "No se ha recibido resultado de MoveIt."
            )

            return False

        resultado = respuesta.result

        # Comprobar tanto el estado de la acción ROS 2
        # como el código de resultado de MoveIt.

        if (
            respuesta.status == GoalStatus.STATUS_SUCCEEDED
            and
            resultado.error_code.val == MoveItErrorCodes.SUCCESS
        ):

            self.get_logger().info(
                "Operación completada correctamente."
            )

            self.get_logger().info(
                f"Tiempo de planificación: "
                f"{resultado.planning_time:.3f} s"
            )

            return True

        self.get_logger().error(
            f"Estado de la acción: {respuesta.status}. "
            f"Código MoveIt: {resultado.error_code.val}"
        )

        return False


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--target",
        choices=ESTADOS_DISPONIBLES,
        default="observation",
    )

    parser.add_argument(
        "--execute",
        choices=["true", "false"],
        default="false",
    )

    argumentos, _ = parser.parse_known_args(args)

    rclpy.init(args=args)

    controlador = None

    try:

        controlador = ArmController()

        ejecutar = argumentos.execute == "true"

        correcto = controlador.mover_a(
            objetivo=argumentos.target,
            ejecutar=ejecutar,
        )

        if not correcto:

            controlador.get_logger().error(
                "La operación ha fallado."
            )

            raise SystemExit(1)

    finally:

        if controlador is not None:
            controlador.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()