
#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - NAVIGATION CLIENT
============================================================

Paquete: amm_navigation
Archivo: amm_navigation/navigation_client.py

Objetivo:
    Enviar destinos al BT Navigator mediante NavigateToPose.

Entradas:
    /goal_pose           -> Objetivos manuales desde RViz.
    /amm/navigation_goal -> Objetivos generados por software.

Servicios:
    /amm/cancel_navigation -> Solicitar cancelación.

Funcionamiento:
    1. Recibe una pose objetivo.
    2. Comprueba que Nav2 está disponible.
    3. Envía una acción NavigateToPose.
    4. Supervisa el progreso mediante feedback.
    5. Comprueba el resultado final.

No implementamos los algoritmos internos de navegación:
utilizamos los servidores profesionales de Nav2.
============================================================
"""

import math
import time

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger


class NavigationClient(Node):

    def __init__(self):

        super().__init__("navigation_client")

        # ====================================================
        # 1. CLIENTE DE ACCIÓN NAV2
        # ====================================================

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose"
        )

        # ====================================================
        # 2. SUSCRIPCIONES A OBJETIVOS
        # ====================================================

        # Objetivos manuales publicados desde RViz.
        self.rviz_goal_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_callback,
            10
        )

        # Objetivos generados por nuestro propio software.
        self.automatic_goal_sub = self.create_subscription(
            PoseStamped,
            "/amm/navigation_goal",
            self.goal_callback,
            10
        )

        # ====================================================
        # 3. SERVICIO DE CANCELACIÓN
        # ====================================================

        self.cancel_service = self.create_service(
            Trigger,
            "/amm/cancel_navigation",
            self.cancel_callback
        )

        # ====================================================
        # 4. VARIABLES DE ESTADO
        # ====================================================

        # Indica si tenemos una navegación pendiente o activa.
        self.navigating = False

        # Identificador del objetivo aceptado por Nav2.
        self.goal_handle = None

        # Controla la frecuencia de mensajes de feedback.
        self.last_feedback_time = 0.0

        self.get_logger().info(
            "Cliente NavigateToPose preparado."
        )

        self.get_logger().info(
            "Esperando objetivos en /goal_pose "
            "o /amm/navigation_goal."
        )

    # ========================================================
    # 5. RECEPCIÓN DE OBJETIVOS
    # ========================================================

    def goal_callback(self, pose):

        # Evitamos iniciar navegaciones simultáneas.
        # Más adelante incorporaremos gestión de preempción.
        if self.navigating:
            self.get_logger().warn(
                "Ya hay una navegación en curso. "
                "Objetivo descartado."
            )
            return

        # Inicialmente solo aceptamos destinos en el mapa.
        if pose.header.frame_id != "map":
            self.get_logger().error(
                "El objetivo debe estar en el frame 'map'."
            )
            return

        # Validamos que el objetivo contiene valores finitos
        # y una orientación aproximadamente normalizada.
        position = pose.pose.position
        orientation = pose.pose.orientation

        values = [
            position.x, position.y, position.z,
            orientation.x, orientation.y,
            orientation.z, orientation.w
        ]

        quaternion_norm = math.sqrt(
            orientation.x ** 2 +
            orientation.y ** 2 +
            orientation.z ** 2 +
            orientation.w ** 2
        )

        if (
            not all(math.isfinite(v) for v in values)
            or abs(quaternion_norm - 1.0) > 0.01
        ):
            self.get_logger().error(
                "Objetivo inválido: coordenadas u "
                "orientación incorrectas."
            )
            return

        # Comprobamos que BT Navigator está disponible.
        if not self.action_client.server_is_ready():
            self.get_logger().error(
                "NavigateToPose no está disponible."
            )
            return

        # ====================================================
        # 6. PREPARAR LA ACCIÓN
        # ====================================================

        goal = NavigateToPose.Goal()

        # Construimos una pose con marca temporal actual.
        # Así evitamos reutilizar timestamps antiguos.
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = pose.pose

        # Cadena vacía: utilizar el Behavior Tree predeterminado.
        goal.behavior_tree = ""

        self.navigating = True
        self.goal_handle = None

        self.get_logger().info(
            f"Solicitando navegación hasta "
            f"x={position.x:.2f}, y={position.y:.2f}"
        )

        # Enviamos el objetivo de forma asíncrona.
        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    # ========================================================
    # 7. RESPUESTA DE ACEPTACIÓN
    # ========================================================

    def goal_response_callback(self, future):

        try:
            handle = future.result()

        except Exception as exc:
            self.navigating = False
            self.get_logger().error(
                f"Error al enviar el objetivo: {exc}"
            )
            return

        if not handle.accepted:
            self.navigating = False
            self.get_logger().error(
                "BT Navigator ha rechazado el objetivo."
            )
            return

        self.goal_handle = handle

        self.get_logger().info(
            "Objetivo aceptado. Navegación iniciada."
        )

        # Esperamos el resultado final sin bloquear.
        result_future = handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )

    # ========================================================
    # 8. FEEDBACK DE NAVEGACIÓN
    # ========================================================

    def feedback_callback(self, feedback_msg):

        now = time.monotonic()

        # Mostramos información cada dos segundos.
        if now - self.last_feedback_time < 2.0:
            return

        self.last_feedback_time = now

        feedback = feedback_msg.feedback

        self.get_logger().info(
            f"Distancia restante: "
            f"{feedback.distance_remaining:.2f} m | "
            f"Recuperaciones: "
            f"{feedback.number_of_recoveries}"
        )

    # ========================================================
    # 9. RESULTADO FINAL
    # ========================================================

    def result_callback(self, future):

        self.navigating = False
        self.goal_handle = None

        try:
            response = future.result()

        except Exception as exc:
            self.get_logger().error(
                f"Error al obtener el resultado: {exc}"
            )
            return

        result = response.result

        if (
            response.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == NavigateToPose.Result.NONE
        ):

            self.get_logger().info(
                "Navegación completada correctamente."
            )

        elif response.status == GoalStatus.STATUS_CANCELED:

            self.get_logger().warn(
                "Navegación cancelada."
            )

        else:

            self.get_logger().error(
                f"Navegación fallida. "
                f"Estado={response.status}, "
                f"error={result.error_code}, "
                f"mensaje={result.error_msg}"
            )

    # ========================================================
    # 10. CANCELACIÓN DE NAVEGACIÓN
    # ========================================================

    def cancel_callback(self, request, response):

        if not self.navigating:
            response.success = False
            response.message = "No hay navegación activa."
            return response

        if self.goal_handle is None:
            response.success = False
            response.message = (
                "El objetivo todavía no ha sido aceptado."
            )
            return response

        # La cancelación es asíncrona.
        future = self.goal_handle.cancel_goal_async()

        future.add_done_callback(
            self.cancel_response_callback
        )

        response.success = True
        response.message = "Cancelación solicitada."

        return response

    def cancel_response_callback(self, future):

        try:
            response = future.result()

            self.get_logger().info(
                f"Objetivos aceptados para cancelar: "
                f"{len(response.goals_canceling)}"
            )

        except Exception as exc:
            self.get_logger().error(
                f"Error solicitando cancelación: {exc}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = NavigationClient()

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
