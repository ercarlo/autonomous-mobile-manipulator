
#!/usr/bin/env python3

"""
Cliente ROS 2 de planificación global para el MiR100.

Recibe objetivos de navegación mediante /goal_pose,
solicita una ruta al Planner Server de Nav2 y publica
la trayectoria calculada para visualizarla en RViz.

En esta etapa no se envían comandos de movimiento.
"""

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose

from action_msgs.msg import GoalStatus


class PlanningClient(Node):

    def __init__(self):
        super().__init__("planning_client")

        # Cliente de acción conectado al servidor de Nav2.
        self._action_client = ActionClient(
            self,
            ComputePathToPose,
            "compute_path_to_pose"
        )

        # Suscripción a los objetivos seleccionados en RViz.
        self._goal_subscriber = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_callback,
            10
        )

        # Publicación de la última ruta calculada.
        # Transient Local permite que RViz reciba el último
        # resultado aunque se conecte después.
        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        path_qos.reliability = ReliabilityPolicy.RELIABLE

        self._path_publisher = self.create_publisher(
            Path,
            "/amm/planned_path",
            path_qos
        )

        # Evita enviar varias solicitudes simultáneas.
        self._planning = False

        self.get_logger().info(
            "Cliente de planificación preparado. "
            "Esperando objetivos en /goal_pose..."
        )

    def goal_callback(self, pose):

        if self._planning:
            self.get_logger().warn(
                "Ya hay una planificación en curso."
            )
            return

        if pose.header.frame_id != "map":
            self.get_logger().warn(
                "El objetivo debe estar expresado en el frame map."
            )
            return

        if not self._action_client.server_is_ready():
            self.get_logger().error(
                "El Planner Server no está disponible."
            )
            return

        # Construimos la solicitud de planificación.
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = pose
        goal_msg.planner_id = "GridBased"

        # Nav2 obtiene la posición inicial actual mediante TF2.
        goal_msg.use_start = False

        self._planning = True

        self.get_logger().info(
            f"Calculando ruta hasta "
            f"x={pose.pose.position.x:.2f}, "
            f"y={pose.pose.position.y:.2f}"
        )

        # Enviamos la solicitud sin bloquear el nodo.
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._planning = False
            self.get_logger().error(
                f"Error al enviar el objetivo: {exc}"
            )
            return

        if not goal_handle.accepted:
            self._planning = False
            self.get_logger().warn(
                "El Planner Server ha rechazado el objetivo."
            )
            return

        self.get_logger().info(
            "Objetivo aceptado. Esperando trayectoria..."
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):

        self._planning = False

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"Error al recibir la planificación: {exc}"
            )
            return

        result = response.result

        if (
            response.status != GoalStatus.STATUS_SUCCEEDED
            or result.error_code != ComputePathToPose.Result.NONE
        ):
            self.get_logger().error(
                f"Planificación fallida. "
                f"Estado={response.status}, "
                f"código={result.error_code}"
            )
            return

        path = result.path

        if not path.poses:
            self.get_logger().warn(
                "El planificador ha devuelto una ruta vacía."
            )
            return

        # Publicamos la ruta para visualizarla en RViz.
        self._path_publisher.publish(path)

        self.get_logger().info(
            f"Ruta calculada correctamente: "
            f"{len(path.poses)} poses."
        )


def main(args=None):

    rclpy.init(args=args)

    node = PlanningClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
