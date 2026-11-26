
#!/usr/bin/env python3

"""
Cliente de seguimiento de trayectorias del MiR100.

Recibe la ruta calculada por planning_client y solicita
su ejecución al Controller Server mediante FollowPath.

La ejecución requiere una orden explícita:
    /amm/execute_path

También permite cancelar el movimiento:
    /amm/cancel_path
"""

import math
import time

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformListener, TransformException


class PathFollowerClient(Node):

    def __init__(self):
        super().__init__("path_follower_client")

        # ----------------------------------------------------
        # 1. Cliente de acción del Controller Server
        # ----------------------------------------------------

        self.action_client = ActionClient(
            self,
            FollowPath,
            "/follow_path"
        )

        # ----------------------------------------------------
        # 2. Suscripción a la ruta calculada
        # ----------------------------------------------------
        # Utilizamos QoS VOLATILE (por defecto) para no
        # recuperar rutas antiguas al arrancar el nodo.

        self.path_sub = self.create_subscription(
            Path,
            "/amm/planned_path",
            self.path_callback,
            10
        )

        # ----------------------------------------------------
        # 3. Servicios de ejecución y cancelación
        # ----------------------------------------------------

        self.execute_service = self.create_service(
            Trigger,
            "/amm/execute_path",
            self.execute_callback
        )

        self.cancel_service = self.create_service(
            Trigger,
            "/amm/cancel_path",
            self.cancel_callback
        )

        # ----------------------------------------------------
        # 4. TF2 y estado interno
        # ----------------------------------------------------

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.path = None
        self.path_time = None
        self.goal_handle = None
        self.executing = False

        self.last_feedback_time = 0.0

        self.get_logger().info(
            "Cliente de seguimiento preparado."
        )

    def path_callback(self, msg):

        if not msg.poses:
            self.get_logger().warn("Ruta vacía recibida.")
            return

        self.path = msg
        self.path_time = time.monotonic()

        self.get_logger().info(
            f"Ruta recibida: {len(msg.poses)} poses."
        )

    def execute_callback(self, request, response):

        # Evitamos ejecutar dos trayectorias simultáneas.
        if self.executing:
            response.success = False
            response.message = "Ya existe una ejecución activa."
            return response

        if self.path is None:
            response.success = False
            response.message = "No hay ninguna ruta disponible."
            return response

        # No ejecutamos trayectorias demasiado antiguas.
        if time.monotonic() - self.path_time > 60.0:
            response.success = False
            response.message = "Ruta antigua. Calcula otra."
            return response

        if not self.action_client.server_is_ready():
            response.success = False
            response.message = "Controller Server no disponible."
            return response

        # ----------------------------------------------------
        # 5. Comprobar la posición inicial de la trayectoria
        # ----------------------------------------------------
        # Evita iniciar una ruta calculada desde una posición
        # demasiado diferente de la posición actual.

        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                Time()
            )
        except TransformException as exc:
            response.success = False
            response.message = f"Error TF2: {exc}"
            return response

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        start = self.path.poses[0].pose.position

        distance = math.hypot(
            robot_x - start.x,
            robot_y - start.y
        )

        if distance > 0.35:
            response.success = False
            response.message = (
                "El robot está demasiado lejos del inicio "
                f"de la ruta: {distance:.2f} m."
            )
            return response

        # ----------------------------------------------------
        # 6. Solicitar el seguimiento de la trayectoria
        # ----------------------------------------------------

        goal = FollowPath.Goal()

        goal.path = self.path

        # Identificadores definidos en nav2_params.yaml.
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "goal_checker"
        goal.progress_checker_id = "progress_checker"

        self.executing = True
        self.goal_handle = None

        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(
            self.goal_response_callback
        )

        response.success = True
        response.message = (
            "Solicitud enviada. Esperando aceptación de Nav2."
        )

        return response

    def goal_response_callback(self, future):

        try:
            handle = future.result()
        except Exception as exc:
            self.executing = False
            self.get_logger().error(str(exc))
            return

        if not handle.accepted:
            self.executing = False
            self.get_logger().error(
                "Nav2 ha rechazado la trayectoria."
            )
            return

        self.goal_handle = handle

        self.get_logger().info(
            "Trayectoria aceptada. Iniciando movimiento."
        )

        result_future = handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )

    def feedback_callback(self, msg):

        # Limitamos la frecuencia de los mensajes del log.
        now = time.monotonic()

        if now - self.last_feedback_time < 2.0:
            return

        self.last_feedback_time = now

        feedback = msg.feedback

        self.get_logger().info(
            f"Distancia restante: "
            f"{feedback.distance_to_goal:.2f} m"
        )

    def result_callback(self, future):

        self.executing = False
        self.goal_handle = None

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"Error al obtener resultado: {exc}"
            )
            return

        if (
            response.status == GoalStatus.STATUS_SUCCEEDED
            and response.result.error_code == FollowPath.Result.NONE
        ):
            self.get_logger().info(
                "Trayectoria completada correctamente."
            )
        else:
            self.get_logger().error(
                f"Seguimiento finalizado sin éxito. "
                f"Estado: {response.status}, "
                f"error: {response.result.error_code}"
            )

    def cancel_callback(self, request, response):

        if self.goal_handle is None:
            response.success = False
            response.message = (
                "No hay un objetivo aceptado para cancelar."
            )
            return response

        # La cancelación es asíncrona; Nav2 debe confirmarla.
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_response_callback)

        response.success = True
        response.message = "Solicitud de cancelación enviada."

        return response

    def cancel_response_callback(self, future):

        try:
            result = future.result()

            self.get_logger().info(
                f"Cancelación: "
                f"{len(result.goals_canceling)} objetivo(s) "
                "aceptados para cancelar."
            )
        except Exception as exc:
            self.get_logger().error(
                f"Error al cancelar: {exc}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = PathFollowerClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
