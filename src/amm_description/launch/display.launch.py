
#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - DISPLAY LAUNCH
============================================================

Paquete: amm_description
Archivo: display.launch.py

Objetivo:
    Visualizar el robot MiR100 + UR5e en RViz2.

Nodos utilizados:
    1. robot_state_publisher
    2. joint_state_publisher_gui
    3. rviz2

Este launch es únicamente para visualización y pruebas
cinemáticas. No controla el robot físicamente en Gazebo.
============================================================
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.conditions import IfCondition


def generate_launch_description():

    # ========================================================
    # 1. LOCALIZACIÓN DE LA DESCRIPCIÓN DEL ROBOT
    # ========================================================
    # Obtenemos la ruta instalada de nuestro paquete.
    # Esto evita depender de rutas absolutas del ordenador.

    package_path = get_package_share_directory(
        "amm_description"
    )

    xacro_file = os.path.join(
        package_path,
        "urdf",
        "mobile_manipulator.urdf.xacro"
    )

    rviz_config_file = os.path.join(
        package_path,
        "rviz",
        "display.rviz"
    )   

    # ========================================================
    # 2. GENERACIÓN AUTOMÁTICA DEL URDF
    # ========================================================
    # Command ejecuta xacro y recoge el XML generado.
    #
    # ParameterValue(..., value_type=str) evita que ROS 2
    # interprete el XML como otro tipo de parámetro.

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file]),
            value_type=str
        )
    }

    # ========================================================
    # 3. ARGUMENTO PARA ACTIVAR O DESACTIVAR LA GUI
    # ========================================================
    # Nos permite lanzar el robot con o sin los deslizadores.
    #
    # Por defecto la GUI estará activada.

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Activar joint_state_publisher_gui"
    )

    # ========================================================
    # 4. ROBOT STATE PUBLISHER
    # ========================================================
    # Publica el URDF en /robot_description.
    #
    # Calcula las transformaciones TF a partir del modelo
    # y de los estados articulares recibidos en /joint_states.

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    # ========================================================
    # 5. JOINT STATE PUBLISHER GUI
    # ========================================================
    # Genera posiciones articulares mediante deslizadores.
    #
    # Se utiliza únicamente durante el desarrollo y las
    # pruebas de visualización, nunca como sustituto de los
    # controladores de Gazebo.

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[robot_description],
        condition=IfCondition(LaunchConfiguration("gui"))
    )

    # ========================================================
    # 6. RVIZ2
    # ========================================================
    # Abre RViz para visualizar la descripción del robot.
    #
    # Más adelante cargaremos automáticamente un archivo
    # de configuración .rviz con los displays preparados.

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file]
    )

    # ========================================================
    # 7. LANZAMIENTO DE LOS NODOS
    # ========================================================
    # LaunchDescription agrupa las acciones que ROS 2
    # ejecutará al iniciar el archivo launch.

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
