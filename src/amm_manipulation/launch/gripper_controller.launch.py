#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - GRIPPER LAUNCH
============================================================

Objetivo:
    Iniciar nuestro controlador de la pinza Robotiq.

Argumentos:
    position:
        Posición articular objetivo en radianes.

    execute:
        true / false

El controlador de la pinza debe estar activo en Gazebo.

============================================================
"""

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Posición articular objetivo.

    argumento_posicion = DeclareLaunchArgument(
        "position",
        default_value="0.4",
        description="Posición objetivo de la pinza en radianes.",
    )

    # Control de ejecución.

    argumento_ejecucion = DeclareLaunchArgument(
        "execute",
        default_value="false",
        description="Habilitar el movimiento.",
    )

    # Nuestro controlador ROS 2.

    controlador = Node(
        package="amm_manipulation",
        executable="gripper_controller",
        name="amm_gripper_controller",
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
        arguments=[
            "--position",
            LaunchConfiguration("position"),
            "--execute",
            LaunchConfiguration("execute"),
        ],
    )

    return LaunchDescription([
        argumento_posicion,
        argumento_ejecucion,
        controlador,
    ])