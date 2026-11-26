
#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - PLANNER LAUNCH
============================================================

Paquete: amm_navigation
Archivo: launch/planner.launch.py

Objetivo:
    Iniciar el Planner Server de Nav2 y su gestor
    de ciclo de vida.

Requisitos:
    - Simulación Gazebo ejecutándose.
    - Map Server y AMCL funcionando.
    - Transformaciones TF disponibles.

Esta versión inicia únicamente la planificación global.
Todavía no incorpora el controlador de navegación.
============================================================
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # --------------------------------------------------------
    # 1. Localización del archivo de parámetros
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
    # 2. Planner Server
    # --------------------------------------------------------

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file]
    )

    # --------------------------------------------------------
    # 3. Lifecycle Manager
    # --------------------------------------------------------
    # Gestiona la configuración y activación automática
    # del servidor de planificación.

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": ["planner_server"]
        }]
    )

    # --------------------------------------------------------
    # 4. Descripción del lanzamiento
    # --------------------------------------------------------

    return LaunchDescription([
        planner_server,
        lifecycle_manager
    ])
