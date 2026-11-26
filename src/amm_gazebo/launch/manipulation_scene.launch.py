#!/usr/bin/env python3
"""
Autonomous Mobile Manipulator — Escena de recogida y reparto.

Conserva robot, sensores y controladores del launch original,
inserta una mesa central, cuatro objetos YCB/equivalentes y
cuatro mesas de destino distribuidas simétricamente.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

WORLD_NAME = "amm_warehouse_world"


def crear_spawn(nombre, directorio_modelos, posicion,
                rotacion=(0.0, 0.0, 0.0), modelo=None):
    """Crea una entidad SDF. 'modelo' permite reutilizar un SDF con otro nombre."""
    ruta_sdf = os.path.join(directorio_modelos, modelo or nombre, "model.sdf")
    x, y, z = posicion
    roll, pitch, yaw = rotacion

    return Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{nombre}",
        output="screen",
        arguments=[
            "-world", WORLD_NAME,
            "-file", ruta_sdf,
            "-name", nombre,
            "-x", str(x), "-y", str(y), "-z", str(z),
            "-R", str(roll), "-P", str(pitch), "-Y", str(yaw),
        ],
    )


def generate_launch_description():
    # ========================================================
    # 1. MODELOS Y LANZAMIENTO DEL ROBOT
    # ========================================================
    gazebo_share = get_package_share_directory("amm_gazebo")
    control_share = get_package_share_directory("amm_control")
    modelos = os.path.join(gazebo_share, "models")

    configurar_recursos = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=modelos,
        separator=os.pathsep,
    )

    # Se conserva el arranque probado del MiR100 + UR5e,
    # la cámara, los bridges y los controladores.
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_share, "launch", "robot_simulation.launch.py")
        )
    )

    # ========================================================
    # 2. MESA CENTRAL Y OBJETOS DE RECOGIDA
    # ========================================================
    spawn_mesa_central = crear_spawn(
        "industrial_workbench", modelos, (0.0, -0.90, 0.0)
    )

    # Acercamos los objetos SOLO 4 cm hacia el MiR100
    # (dirección Y positiva), manteniendo X, Z y orientación.
    # Su visibilidad y la ejecución del pick deberán revalidarse.
    spawn_mostaza = crear_spawn(
        "ycb_mustard_bottle", modelos, (-0.36, -0.92, 0.70)
    )
    spawn_manzana = crear_spawn(
        "ycb_apple", modelos, (-0.12, -0.80, 0.70)
    )
    spawn_taza = crear_spawn(
        "ycb_mug", modelos, (0.12, -0.92, 0.70)
    )
    spawn_cuenco = crear_spawn(
        "ycb_bowl", modelos, (0.36, -0.80, 0.70)
    )

    # ========================================================
    # 3. MESAS DE DESTINO: CUATRO ESQUINAS
    # ========================================================
    # Posiciones simétricas respecto al origen del mundo.
    # Mesa 1 conserva las coordenadas elegidas manualmente.
    # Las orientaciones se reflejan para mantener la simetría.
    mesas = [
        ("place_workbench",   ( 5.271960, -4.460220, 0.115244),  1.546950),
        ("place_workbench_2", (-5.271960, -4.460220, 0.115244),  1.594643),
        ("place_workbench_3", ( 5.271960,  4.460220, 0.115244), -1.546950),
        ("place_workbench_4", (-5.271960,  4.460220, 0.115244), -1.594643),
    ]

    spawns_mesas = [
        crear_spawn(
            nombre, modelos, posicion,
            rotacion=(0.0, 0.0, yaw),
            modelo="industrial_workbench",
        )
        for nombre, posicion, yaw in mesas
    ]

    # ========================================================
    # 4. SECUENCIA DE CREACIÓN CONTROLADA
    # ========================================================
    # Se conservan spawns secuenciales y 5 s antes del bol,
    # porque anteriormente no aparecía bien en Entity Tree.
    spawns = [
        spawn_mesa_central,
        spawn_mostaza,
        spawn_manzana,
        spawn_taza,
        spawn_cuenco,
        *spawns_mesas,
    ]

    eventos_spawn = []
    for anterior, siguiente in zip(spawns, spawns[1:]):
        acciones = (
            [TimerAction(period=5.0, actions=[siguiente])]
            if siguiente is spawn_cuenco else [siguiente]
        )
        eventos_spawn.append(
            RegisterEventHandler(
                OnProcessExit(target_action=anterior, on_exit=acciones)
            )
        )

    return LaunchDescription([
        configurar_recursos,
        *eventos_spawn,
        robot_launch,
        spawn_mesa_central,
    ])