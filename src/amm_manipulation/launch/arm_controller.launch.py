#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - ARM CONTROLLER LAUNCH
============================================================

Objetivo:
    Iniciar nuestro controlador del UR5e.

Arquitectura:
    - Utiliza la acción /move_action.
    - Requiere move_group activo.
    - No inicializa MoveItPy.
    - Utiliza el tiempo simulado de Gazebo.

============================================================
"""

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # Configuración articular objetivo.

    argumento_objetivo = DeclareLaunchArgument(
        "target",
        default_value="observation",
        description="Configuración objetivo del UR5e.",
    )

    # Habilitar o deshabilitar la ejecución.

    argumento_ejecucion = DeclareLaunchArgument(
        "execute",
        default_value="false",
        description="Ejecutar la trayectoria planificada.",
    )

    # Nodo controlador del UR5e.

    controlador = Node(
        package="amm_manipulation",
        executable="arm_controller",
        name="amm_arm_controller",
        output="screen",

        parameters=[
            {"use_sim_time": True},
        ],

        arguments=[
            "--target",
            LaunchConfiguration("target"),

            "--execute",
            LaunchConfiguration("execute"),
        ],
    )

    return LaunchDescription([
        argumento_objetivo,
        argumento_ejecucion,
        controlador,
    ])