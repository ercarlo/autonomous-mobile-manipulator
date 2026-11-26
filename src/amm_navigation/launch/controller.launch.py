
#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - CONTROLLER LAUNCH
============================================================

Paquete: amm_navigation
Archivo: launch/controller.launch.py

Objetivo:
    Iniciar el Controller Server de Nav2 junto con
    su gestor de ciclo de vida.

Requisitos:
    - Gazebo y el robot en funcionamiento.
    - Localización AMCL activa.
    - Transformaciones TF y /odom disponibles.

En esta etapa no enviamos objetivos de movimiento.
============================================================
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # --------------------------------------------------------
    # 1. Archivo de configuración de Nav2
    # --------------------------------------------------------

    package_dir = get_package_share_directory(
        "amm_navigation"
    )

    params_file = os.path.join(
        package_dir,
        "config",
        "nav2_params.yaml"
    )

    # --------------------------------------------------------
    # 2. Controller Server
    # --------------------------------------------------------

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file]
    )

    # --------------------------------------------------------
    # 3. Lifecycle Manager del controlador
    # --------------------------------------------------------
    # Utilizamos un nombre diferente del gestor del Planner
    # para poder iniciar ambos de forma independiente.

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_controller",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": ["controller_server"]
        }]
    )

    return LaunchDescription([
        controller_server,
        lifecycle_manager
    ])
