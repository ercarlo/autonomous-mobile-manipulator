
#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - NAVIGATION LAUNCH
============================================================

Paquete: amm_navigation
Archivo: launch/navigation.launch.py

Objetivo:
    Iniciar los componentes de navegación desarrollados
    hasta esta etapa del proyecto.

Componentes:
    - Planner Server (NavFn A*)
    - Controller Server (Regulated Pure Pursuit)
    - Lifecycle Manager
    - Cliente propio de planificación
    - Cliente propio de seguimiento
    - RViz (opcional)

Requisitos:
    - Simulación Gazebo activa.
    - Localización AMCL y Map Server activos.

NOTA:
    El BT Navigator y los comportamientos de recuperación
    se incorporarán en una etapa posterior.
============================================================
"""

import os

from ament_index_python.packages import (
    get_package_share_directory
)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # ========================================================
    # 1. RUTAS DEL PAQUETE
    # ========================================================

    package_dir = get_package_share_directory(
        "amm_navigation"
    )

    params_file = os.path.join(
        package_dir,
        "config",
        "nav2_params.yaml"
    )

    rviz_file = os.path.join(
        package_dir,
        "rviz",
        "navigation.rviz"
    )

    # ========================================================
    # 2. ARGUMENTOS DE LANZAMIENTO
    # ========================================================

    # Permite activar o desactivar RViz desde la terminal.
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Iniciar RViz con la navegación"
    )

    use_rviz = LaunchConfiguration("use_rviz")

    # ========================================================
    # 3. PLANNER SERVER
    # ========================================================

    # Calcula rutas utilizando el Global Costmap y NavFn A*.

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file]
    )

    # ========================================================
    # 4. CONTROLLER SERVER
    # ========================================================

    # Sigue las trayectorias mediante Regulated Pure Pursuit
    # y genera comandos de velocidad para el MiR100.

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file]
    )


    # ========================================================
    # BEHAVIOR SERVER
    # ========================================================
    #
    # Proporciona comportamientos de recuperación:
    #   - Giro sobre sí mismo.
    #   - Retroceso.
    #   - Espera.
    #
    # Utiliza los costmaps existentes para comprobar
    # posibles colisiones durante las maniobras.

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file]
    )




    # ========================================================
    # BT NAVIGATOR
    # ========================================================
    #
    # Coordina planificación, seguimiento y recuperación.
    #
    # Nav2 incorpora una suscripción propia a /goal_pose.
    # La redirigimos para evitar que RViz envíe objetivos
    # simultáneamente a BT Navigator y a nuestro cliente.
    #
    # Los objetivos normales pasan por navigation_client,
    # que utiliza la acción NavigateToPose.
    #
    # Conservamos una entrada directa separada para
    # posibles pruebas de diagnóstico.

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file],
        remappings=[
            ("goal_pose", "/amm/bt_direct_goal_pose")
        ]
    )



    # ========================================================
    # 5. LIFECYCLE MANAGER
    # ========================================================

    # Gestiona la configuración y activación de los servidores.
    #
    # Ya no utilizamos un Lifecycle Manager por servidor:
    # ambos quedan bajo un mismo gestor.
    #
    # La lista node_names determina el orden de activación.

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator"
            ]
        }]
    )

    # ========================================================
    # 6. CLIENTE PROPIO DE PLANIFICACIÓN
    # ========================================================

    # Recibe objetivos en /goal_pose y solicita rutas
    # mediante la acción ComputePathToPose.

    planning_client = Node(
        package="amm_navigation",
        executable="planning_client",
        name="planning_client",
        output="screen",
        parameters=[{
            "use_sim_time": True
        }]
    )

    # ========================================================
    # 7. CLIENTE PROPIO DE SEGUIMIENTO
    # ========================================================

    # Recibe las rutas publicadas por planning_client.
    # Solo solicita FollowPath cuando recibe una orden
    # explícita mediante /amm/execute_path.

    path_follower_client = Node(
        package="amm_navigation",
        executable="path_follower_client",
        name="path_follower_client",
        output="screen",
        parameters=[{
            "use_sim_time": True
        }]
    )

    # ========================================================
    # 8. RVIZ
    # ========================================================

    # Utiliza nuestra configuración de navegación.
    # Puede desactivarse mediante use_rviz:=false.

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
        parameters=[{
            "use_sim_time": True
        }],
        condition=IfCondition(use_rviz)
    )



    # ========================================================
    # CLIENTE DE NAVEGACIÓN AUTÓNOMA
    # ========================================================
    #
    # Envía objetivos al BT Navigator mediante NavigateToPose.
    # Permite recibir destinos manuales desde RViz o
    # destinos automáticos desde otros nodos ROS 2.

    navigation_client = Node(
        package="amm_navigation",
        executable="navigation_client",
        name="navigation_client",
        output="screen",
        parameters=[{
            "use_sim_time": True
        }]
    )


    # ========================================================
    # STATION MANAGER
    # ========================================================
    #
    # Gestiona las estaciones predefinidas del almacén.
    #
    # Recibe nombres mediante /amm/station_command,
    # obtiene sus coordenadas desde stations.yaml
    # y publica objetivos en /amm/navigation_goal.
    #
    # navigation_client recibe estos objetivos y solicita
    # la navegación autónoma a BT Navigator.

    station_manager = Node(
        package="amm_navigation",
        executable="station_manager",
        name="station_manager",
        output="screen",
        parameters=[{
            "use_sim_time": True
        }]
    )



    # ========================================================
    # 9. LANZAMIENTO COMPLETO
    # ========================================================

    return LaunchDescription([

        use_rviz_arg,

        # Servidores Nav2.
        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,

        # Activación automática de servidores.
        lifecycle_manager,

        navigation_client,
        station_manager,

        # Herramienta de visualización.
        rviz_node

    ])
