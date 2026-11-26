#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - MOVE GROUP
============================================================

Objetivo:
    Iniciar MoveIt 2 utilizando el reloj de Gazebo
    desde el comienzo del proceso.

    Se conserva la configuración generada mediante
    MoveIt Setup Assistant.
============================================================
"""

from launch import LaunchDescription
from launch_ros.actions import SetParameter

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():

    # Cargar la configuración del robot y sus planificadores.
    moveit_config = (
        MoveItConfigsBuilder(
            "autonomous_mobile_manipulator",
            package_name="amm_moveit_config"
        )
        .to_moveit_configs()
    )

    # Generar el lanzamiento habitual de MoveIt.
    moveit_launch = generate_move_group_launch(moveit_config)

    # IMPORTANTE:
    # Activar el tiempo simulado ANTES de iniciar move_group.
    # Así sus componentes internos utilizan /clock desde
    # el momento en que se crean.
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        *moveit_launch.entities,
    ])