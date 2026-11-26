
#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - LOCALIZATION LAUNCH
============================================================

Paquete: amm_slam
Archivo: localization.launch.py

Objetivo:
    Localizar el MiR100 dentro del almacén utilizando
    el mapa generado previamente con SLAM Toolbox.

Componentes:
    1. Map Server: carga y publica el mapa.
    2. AMCL: estima la posición mediante partículas.
    3. Lifecycle Manager: activa los dos nodos.

Requisitos:
    - Gazebo y robot_simulation.launch.py ejecutándose.
    - Lidar /scan y odometría disponibles.
    - SLAM Toolbox detenido.

IMPORTANTE:
    AMCL sustituye a SLAM Toolbox como responsable
    de publicar la transformación map -> odom.
============================================================
"""

import os

from ament_index_python.packages import (
    get_package_share_directory
)

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # ========================================================
    # 1. LOCALIZAR LOS ARCHIVOS DE NUESTRO PAQUETE
    # ========================================================
    #
    # Obtenemos la ruta instalada de amm_slam.
    # Así evitamos depender de rutas absolutas específicas
    # de nuestro ordenador.

    package_path = get_package_share_directory(
        "amm_slam"
    )

    # Mapa generado anteriormente con SLAM Toolbox.

    map_file = os.path.join(
        package_path,
        "maps",
        "warehouse_map.yaml"
    )

    # Configuración de nuestro filtro de partículas AMCL.

    amcl_params_file = os.path.join(
        package_path,
        "config",
        "amcl.yaml"
    )

    # ========================================================
    # 2. MAP SERVER
    # ========================================================
    #
    # Nav2 Map Server:
    #   - Lee el archivo YAML del mapa.
    #   - Carga la imagen PGM correspondiente.
    #   - Publica un OccupancyGrid en /map.
    #
    # No genera mapas nuevos: utiliza el mapa ya guardado.
    #
    # Es un Lifecycle Node y será activado por el
    # Lifecycle Manager definido más abajo.

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",

        parameters=[
            {
                "yaml_filename": map_file,
                "use_sim_time": True,
            }
        ],
    )

    # ========================================================
    # 3. LOCALIZACIÓN MEDIANTE AMCL
    # ========================================================
    #
    # AMCL estima la pose del robot utilizando:
    #
    #   - Mapa publicado por Map Server.
    #   - Mediciones del lidar en /scan.
    #   - Transformaciones de odometría TF.
    #
    # El filtro de partículas compara las mediciones
    # actuales con las paredes y obstáculos del mapa.
    #
    # Publicará:
    #   - /amcl_pose
    #   - map -> odom
    #
    # Inicializaremos la pose desde RViz utilizando
    # la herramienta 2D Pose Estimate.

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",

        parameters=[
            amcl_params_file,
        ],
    )

    # ========================================================
    # 4. LIFECYCLE MANAGER
    # ========================================================
    #
    # Los nodos Map Server y AMCL tienen estados:
    #
    #   unconfigured -> inactive -> active
    #
    # El Lifecycle Manager configura y activa ambos.
    #
    # autostart:
    #   Activa automáticamente los nodos al iniciar.
    #
    # node_names:
    #   Lista ordenada de los nodos administrados.
    #   Primero activamos Map Server y después AMCL.
    #
    # bond_timeout:
    #   Tiempo de espera para supervisar conexiones
    #   de los nodos gestionados. Utilizamos un margen
    #   de 10 segundos por trabajar en una máquina virtual.

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",

        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "map_server",
                    "amcl",
                ],
                "bond_timeout": 10.0,
            }
        ],
    )

    # ========================================================
    # 5. DESCRIPCIÓN FINAL DEL LANZAMIENTO
    # ========================================================
    #
    # Los tres procesos se ejecutan desde este launch.
    # Lifecycle Manager coordina su activación.
    #
    # Gazebo no se inicia aquí porque ya dispone
    # de su propio launch en el paquete amm_control.

    return LaunchDescription([
        map_server,
        amcl,
        lifecycle_manager,
    ])
