#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - GRIPPER CONTROLLER
============================================================

Paquete: amm_manipulation
Archivo: gripper_controller.py

Objetivo:
    Controlar la pinza Robotiq 2F-85 mediante ROS 2 Actions.

Funcionalidades:
    - Conectar con el controlador de la pinza.
    - Enviar posiciones articulares objetivo.
    - Comprobar si el objetivo fue aceptado.
    - Esperar el resultado de la acción.
    - Distinguir entre posición alcanzada y bloqueo.
    - Permitir pruebas sin ejecutar movimientos.

Esta primera versión utiliza directamente la posición
articular del mecanismo de la pinza, expresada en radianes.

============================================================
"""

import argparse

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from control_msgs.action import ParallelGripperCommand


# ============================================================
# CONFIGURACIÓN
# ============================================================

ACCION_PINZA = "/gripper_controller/gripper_cmd"

ARTICULACION_PINZA = (
    "gripper_robotiq_85_left_knuckle_joint"
)

TIMEOUT_SERVIDOR = 10.0
TIMEOUT_RESULTADO = 20.0


# ============================================================
# CONTROLADOR DE LA PINZA
# ============================================================

class GripperController(Node):

    def __init__(self):

        super().__init__("amm_gripper_controller")

        # Cliente ROS 2 Action para el controlador
        # de la pinza Robotiq.

        self.cliente = ActionClient(
            self,
            ParallelGripperCommand,
            ACCION_PINZA,
        )

        self.get_logger().info(
            "Controlador Robotiq inicializado."
        )

    # ========================================================
    # MOVIMIENTO
    # ========================================================

    def mover_a(self, posicion, ejecutar=False):

        """
        Solicita una posición articular de la pinza.

        Parámetros:
            posicion:
                Posición objetivo en radianes.

            ejecutar:
                False -> mostrar la orden.
                True  -> enviar la orden al controlador.

        Devuelve:
            True si la operación se completa correctamente.
            False en caso contrario.
        """

        self.get_logger().info(
            f"Posición solicitada: {posicion:.4f} rad"
        )

        # En modo de prueba no se envían movimientos.

        if not ejecutar:

            self.get_logger().info(
                "Modo de prueba: movimiento deshabilitado."
            )

            return True

        # Esperar a que el controlador esté disponible.

        self.get_logger().info(
            "Esperando al controlador de la pinza..."
        )

        if not self.cliente.wait_for_server(
            timeout_sec=TIMEOUT_SERVIDOR
        ):

            self.get_logger().error(
                "No se encuentra el servidor de la pinza."
            )

            return False

        # Construir el objetivo de la acción.
        #
        # ParallelGripperCommand utiliza un JointState
        # para indicar la articulación y su posición.

        objetivo = ParallelGripperCommand.Goal()

        objetivo.command.name = [
            ARTICULACION_PINZA
        ]

        objetivo.command.position = [
            float(posicion)
        ]

        # Enviamos únicamente la posición.
        # Los límites de velocidad y esfuerzo se gestionan
        # mediante la configuración del controlador.

        self.get_logger().info(
            "Enviando orden a la pinza Robotiq."
        )

        futuro_envio = self.cliente.send_goal_async(
            objetivo
        )

        rclpy.spin_until_future_complete(
            self,
            futuro_envio,
            timeout_sec=TIMEOUT_SERVIDOR,
        )

        if not futuro_envio.done():

            self.get_logger().error(
                "Tiempo agotado al enviar el objetivo."
            )

            return False

        if futuro_envio.exception() is not None:

            self.get_logger().error(
                f"Error al enviar el objetivo: "
                f"{futuro_envio.exception()}"
            )

            return False

        goal_handle = futuro_envio.result()

        if goal_handle is None or not goal_handle.accepted:

            self.get_logger().error(
                "El controlador ha rechazado el movimiento."
            )

            return False

        self.get_logger().info(
            "Movimiento aceptado por el controlador."
        )

        # Esperar la finalización de la acción.

        futuro_resultado = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            futuro_resultado,
            timeout_sec=TIMEOUT_RESULTADO,
        )

        if not futuro_resultado.done():

            self.get_logger().error(
                "Tiempo de ejecución agotado."
            )

            # Solicitar la cancelación del movimiento.
            futuro_cancelacion = goal_handle.cancel_goal_async()

            rclpy.spin_until_future_complete(
                self,
                futuro_cancelacion,
                timeout_sec=3.0,
            )

            return False

        if futuro_resultado.exception() is not None:

            self.get_logger().error(
                f"Error durante la ejecución: "
                f"{futuro_resultado.exception()}"
            )

            return False

        respuesta = futuro_resultado.result()

        if respuesta is None:

            self.get_logger().error(
                "No se ha recibido resultado."
            )

            return False

        resultado = respuesta.result

        # Mostrar la posición articular comunicada
        # por el controlador al finalizar.

        if resultado.state.position:

            posicion_final = resultado.state.position[0]

            self.get_logger().info(
                f"Posición final: {posicion_final:.4f} rad"
            )

        # Informar de las condiciones de finalización.

        self.get_logger().info(
            f"Posición objetivo alcanzada: "
            f"{resultado.reached_goal}"
        )

        self.get_logger().info(
            f"Movimiento bloqueado: {resultado.stalled}"
        )

        # Para esta primera etapa, una operación correcta
        # significa que se ha alcanzado la posición.
        #
        # Durante el agarre físico estudiaremos cómo
        # interpretar el bloqueo contra un objeto.

        correcto = (
            respuesta.status == GoalStatus.STATUS_SUCCEEDED
            and resultado.reached_goal
        )

        if correcto:

            self.get_logger().info(
                "Movimiento de la pinza completado."
            )

            return True

        self.get_logger().error(
            "La pinza no ha alcanzado su objetivo."
        )

        return False


# ============================================================
# FUNCIÓN PRINCIPAL
# ============================================================

def main(args=None):

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--position",
        type=float,
        required=True,
        help="Posición articular objetivo en radianes.",
    )

    parser.add_argument(
        "--execute",
        choices=["true", "false"],
        default="false",
        help="Habilitar ejecución del movimiento.",
    )

    argumentos, _ = parser.parse_known_args(args)

    rclpy.init(args=args)

    controlador = None

    try:

        controlador = GripperController()

        ejecutar = argumentos.execute == "true"

        correcto = controlador.mover_a(
            posicion=argumentos.position,
            ejecutar=ejecutar,
        )

        if not correcto:
            raise SystemExit(1)

    finally:

        if controlador is not None:
            controlador.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()