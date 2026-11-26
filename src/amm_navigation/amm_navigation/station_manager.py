
#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - STATION MANAGER
============================================================

Paquete: amm_navigation
Archivo: amm_navigation/station_manager.py

Objetivo:
    Gestionar estaciones predefinidas del almacén y
    publicar objetivos de navegación mediante PoseStamped.

Entradas:
    /amm/station_command -> Nombre de la estación.

Salidas:
    /amm/navigation_goal -> Pose objetivo en el mapa.

Configuración:
    config/stations.yaml

Este nodo no ejecuta la navegación directamente.
Delega esa responsabilidad en navigation_client.
============================================================
"""

import math
import os

import rclpy
import yaml

from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class StationManager(Node):

    def __init__(self):

        super().__init__("station_manager")

        # ====================================================
        # 1. LOCALIZAR EL ARCHIVO DE ESTACIONES
        # ====================================================

        package_dir = get_package_share_directory(
            "amm_navigation"
        )

        default_config = os.path.join(
            package_dir,
            "config",
            "stations.yaml"
        )

        # Permite utilizar otro archivo de estaciones
        # sin modificar el código del nodo.
        self.declare_parameter(
            "stations_file",
            default_config
        )

        config_path = self.get_parameter(
            "stations_file"
        ).value

        # ====================================================
        # 2. CARGAR LAS ESTACIONES
        # ====================================================

        try:

            with open(config_path, "r", encoding="utf-8") as file:
                config = yaml.safe_load(file)

            self.stations = config["stations"]

            if not isinstance(self.stations, dict):
                raise ValueError(
                    "La sección 'stations' debe ser un diccionario."
                )

        except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError) as exc:

            self.get_logger().error(
                f"No se pudieron cargar las estaciones: {exc}"
            )
            raise RuntimeError(
                "Error al cargar stations.yaml"
            ) from exc

        # ====================================================
        # 3. PUBLICADOR DE OBJETIVOS
        # ====================================================

        # navigation_client ya está suscrito a este topic.
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            "/amm/navigation_goal",
            10
        )

        # ====================================================
        # 4. SUSCRIPTOR A ÓRDENES DE ESTACIÓN
        # ====================================================

        self.station_subscriber = self.create_subscription(
            String,
            "/amm/station_command",
            self.station_callback,
            10
        )

        self.get_logger().info(
            "Gestor de estaciones iniciado."
        )

        self.get_logger().info(
            f"Estaciones disponibles: "
            f"{', '.join(self.stations.keys())}"
        )

    # ========================================================
    # 5. PROCESAR UNA ORDEN DE ESTACIÓN
    # ========================================================

    def station_callback(self, msg):

        # Normalizamos el nombre para evitar problemas
        # con mayúsculas o espacios.
        station_name = msg.data.strip().lower()

        if station_name not in self.stations:

            self.get_logger().warn(
                f"Estación desconocida: '{station_name}'. "
                f"Disponibles: {list(self.stations.keys())}"
            )
            return

        station = self.stations[station_name]

        # ====================================================
        # 6. OBTENER Y VALIDAR LAS COORDENADAS
        # ====================================================

        try:

            x = float(station["x"])
            y = float(station["y"])
            yaw = float(station["yaw"])

            frame_id = station["frame_id"]

            if frame_id != "map":
                raise ValueError(
                    "Las estaciones deben estar en el frame 'map'."
                )

            if not all(math.isfinite(v) for v in (x, y, yaw)):
                raise ValueError(
                    "Las coordenadas deben ser finitas."
                )

        except (KeyError, TypeError, ValueError) as exc:

            self.get_logger().error(
                f"Configuración inválida en "
                f"'{station_name}': {exc}"
            )
            return

        # ====================================================
        # 7. CONSTRUIR LA POSE OBJETIVO
        # ====================================================

        goal = PoseStamped()

        goal.header.frame_id = frame_id

        # Utilizamos el reloj del nodo, sincronizado
        # con /clock de Gazebo mediante use_sim_time.
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        # Conversión de yaw a cuaternión.
        # El MiR100 navega sobre un plano horizontal,
        # por lo que roll = pitch = 0.
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        # ====================================================
        # 8. PUBLICAR EL OBJETIVO
        # ====================================================

        # Evitamos enviar una orden si todavía no existe
        # ningún nodo conectado a /amm/navigation_goal.
        if self.goal_publisher.get_subscription_count() == 0:

            self.get_logger().warn(
                "No hay suscriptores en /amm/navigation_goal. "
                "Comprueba que navigation_client esté activo."
            )
            return

        self.goal_publisher.publish(goal)

        self.get_logger().info(
            f"Objetivo enviado: {station_name} | "
            f"x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = StationManager()

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
