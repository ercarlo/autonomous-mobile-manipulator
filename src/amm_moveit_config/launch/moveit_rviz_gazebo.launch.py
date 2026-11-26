#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - MOVEIT RVIZ
============================================================

Paquete: amm_moveit_config
Archivo: moveit_rviz_gazebo.launch.py

Objetivo:
    Iniciar RViz2 con el plugin MotionPlanning de MoveIt 2.

    Cargar explícitamente las descripciones URDF y SRDF,
    los parámetros cinemáticos y los planificadores.

Arquitectura:
    - Gazebo ejecuta el robot y sus controladores.
    - move_group ejecuta la planificación.
    - Este launch ejecuta únicamente RViz2.

Importante:
    No iniciamos otro robot_state_publisher.
    No iniciamos nuevos controladores ros2_control.
    No iniciamos otra instancia de move_group.
============================================================
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # --------------------------------------------------------
    # 1. Cargar la configuración de MoveIt 2
    # --------------------------------------------------------

    # Reutilizamos el paquete generado mediante
    # MoveIt Setup Assistant para nuestro robot completo.

    moveit_config = (
        MoveItConfigsBuilder(
            "autonomous_mobile_manipulator",
            package_name="amm_moveit_config"
        )
        .to_moveit_configs()
    )

    # --------------------------------------------------------
    # 2. Localizar la configuración gráfica de RViz
    # --------------------------------------------------------

    package_share = get_package_share_directory(
        "amm_moveit_config"
    )

    rviz_config = os.path.join(
        package_share,
        "config",
        "moveit.rviz"
    )

    # --------------------------------------------------------
    # 3. Iniciar RViz con los parámetros de MoveIt
    # --------------------------------------------------------

    # Proporcionamos directamente el URDF y el SRDF.
    #
    # Esto evita depender exclusivamente de que RViz
    # reciba las descripciones mediante topics.
    #
    # También cargamos la configuración cinemática,
    # los límites articulares y los planificadores.

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",

        arguments=[
            "-d",
            rviz_config
        ],

        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits
        ]
    )

    # --------------------------------------------------------
    # 4. Construir la descripción del lanzamiento
    # --------------------------------------------------------

    return LaunchDescription([
        rviz_node
    ])