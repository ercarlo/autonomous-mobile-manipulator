#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - SLAM LAUNCH
============================================================

Paquete: amm_slam
Archivo: online_async_slam.launch.py

Objetivo:
    Iniciar SLAM Toolbox en modo asíncrono para construir
    un mapa 2D del entorno mediante el lidar del MiR100.

Funcionamiento:
    1. Localizar nuestra configuración YAML.
    2. Localizar el launch oficial de SLAM Toolbox.
    3. Iniciar el nodo de SLAM en modo asíncrono.
    4. Activar automáticamente su ciclo de vida.
    5. Utilizar el reloj de simulación de Gazebo.

Requisitos:
    - Simulación del robot en funcionamiento.
    - Topic /scan disponible.
    - Transformación odom -> base_footprint disponible.
    - Cadena TF hasta front_laser_link disponible.

Este archivo NO inicia Gazebo. La simulación debe
ejecutarse previamente mediante amm_control.

============================================================
"""

import os

from ament_index_python.packages import (
    get_package_share_directory
)

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ========================================================
    # 1. LOCALIZAR LOS PAQUETES NECESARIOS
    # ========================================================
    #
    # amm_slam:
    #   Contiene nuestros parámetros y launch propios.
    #
    # slam_toolbox:
    #   Contiene los ejecutables y launch oficiales
    #   que implementan el algoritmo SLAM.
    #
    # Utilizamos rutas instaladas para evitar depender
    # del directorio desde el que se ejecute el comando.

    amm_slam_share = get_package_share_directory(
        "amm_slam"
    )

    slam_toolbox_share = get_package_share_directory(
        "slam_toolbox"
    )

    # ========================================================
    # 2. LOCALIZAR NUESTRA CONFIGURACIÓN
    # ========================================================
    #
    # Este archivo YAML contiene los parámetros adaptados
    # al robot MiR100:
    #
    #   - scan_topic: /scan
    #   - odom_frame: odom
    #   - base_frame: base_footprint
    #   - map_frame: map
    #
    # También define la resolución del mapa y los
    # parámetros de procesamiento y optimización.

    slam_params_file = os.path.join(
        amm_slam_share,
        "config",
        "slam_toolbox.yaml"
    )

    # ========================================================
    # 3. LOCALIZAR EL LAUNCH OFICIAL ASÍNCRONO
    # ========================================================
    #
    # El launch oficial de SLAM Toolbox gestiona:
    #
    #   - La creación del Lifecycle Node.
    #   - Su configuración inicial.
    #   - Su activación automática.
    #
    # Reutilizamos esa infraestructura estándar,
    # proporcionando nuestro propio archivo YAML.

    slam_toolbox_launch = os.path.join(
        slam_toolbox_share,
        "launch",
        "online_async_launch.py"
    )

    # ========================================================
    # 4. CONFIGURAR EL TIEMPO DE SIMULACIÓN
    # ========================================================
    #
    # El argumento use_sim_time permite decidir
    # si el nodo utiliza el reloj de Gazebo.
    #
    # Para nuestro proyecto el valor predeterminado
    # será true.
    #
    # Lo declaramos también como argumento para que
    # pueda modificarse desde la línea de comandos.

    use_sim_time = LaunchConfiguration(
        "use_sim_time"
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description=(
            "Utilizar el reloj de simulación de Gazebo"
        )
    )

    # ========================================================
    # 5. INICIAR SLAM TOOLBOX
    # ========================================================
    #
    # Pasamos cuatro argumentos al launch oficial:
    #
    # slam_params_file:
    #   Ruta de nuestra configuración YAML.
    #
    # use_sim_time:
    #   Sincronización con /clock.
    #
    # autostart:
    #   Configura y activa el Lifecycle Node
    #   automáticamente.
    #
    # use_lifecycle_manager:
    #   false porque el launch oficial gestionará
    #   directamente las transiciones de estado.
    #
    # No ejecutamos aquí AMCL: durante el mapeo,
    # SLAM Toolbox publicará map -> odom.

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            slam_toolbox_launch
        ),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "use_lifecycle_manager": "false",
        }.items(),
    )

    # ========================================================
    # 6. DESCRIPCIÓN FINAL DEL LANZAMIENTO
    # ========================================================
    #
    # Primero declaramos los argumentos y después
    # iniciamos SLAM Toolbox con nuestra configuración.

    return LaunchDescription([
        declare_use_sim_time,
        slam_toolbox,
    ])