#!/usr/bin/env python3

"""
AUTONOMOUS MOBILE MANIPULATOR
Lanzamiento completo de simulación del robot.

Este launch integra:
    - Gazebo Harmonic y el mundo de pruebas.
    - Puente del reloj de simulación.
    - Descripción URDF/Xacro del robot.
    - Inserción automática en Gazebo.
    - Controladores del UR5e.

La navegación y el control de la pinza se añadirán
en etapas posteriores.
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit

from ament_index_python.packages import (
    get_package_share_directory
)


def generate_launch_description():

    # ========================================================
    # 1. LOCALIZACIÓN DE LOS PAQUETES
    # ========================================================
    # Obtenemos sus rutas instaladas mediante ROS 2.
    # Así evitamos escribir rutas absolutas dependientes
    # del nombre de usuario o del directorio del workspace.

    gazebo_share = get_package_share_directory(
        "amm_gazebo"
    )

    description_share = get_package_share_directory(
        "amm_description"
    )

    # ========================================================
    # 2. INICIAR EL MUNDO DE GAZEBO
    # ========================================================
    # Reutilizamos nuestro launch existente.
    #
    # Este ya configura:
    #   - Gazebo Harmonic.
    #   - El mundo amm_test_world.
    #   - Las rutas de los meshes.
    #   - El puente /clock de Gazebo a ROS 2.

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_share,
                "launch",
                "simulation.launch.py"
            )
        )
    )

    # ========================================================
    # 3. GENERAR LA DESCRIPCIÓN DEL ROBOT
    # ========================================================
    # Procesamos el Xacro con sim_gazebo:=true para incluir
    # el plugin gz_ros2_control.
    #
    # ParameterValue(..., value_type=str) garantiza que
    # el XML generado se interprete como una cadena.

    xacro_file = os.path.join(
        description_share,
        "urdf",
        "mobile_manipulator.urdf.xacro"
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            xacro_file,
            " sim_gazebo:=true"
        ]),
        value_type=str
    )

    # ========================================================
    # 4. ROBOT STATE PUBLISHER
    # ========================================================
    # Publica la descripción en /robot_description.
    #
    # También utilizará /joint_states para calcular
    # las transformaciones TF de las articulaciones.
    #
    # use_sim_time=True permite utilizar el reloj de Gazebo.

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True
            }
        ]
    )

    # ========================================================
    # 5. INSERTAR EL ROBOT EN GAZEBO
    # ========================================================
    # El modelo se obtiene del topic /robot_description.
    #
    # De este modo, Gazebo y robot_state_publisher utilizan
    # exactamente la misma descripción del robot.
    #
    # z=0.15 m es la altura inicial utilizada en las
    # pruebas anteriores.

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_amm_robot",
        output="screen",
        arguments=[
            "-world", "amm_warehouse_world",
            "-topic", "/robot_description",
            "-name", "amm_robot",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.15"
        ]
    )

    # ========================================================
    # 6. JOINT STATE BROADCASTER
    # ========================================================
    # Solicita a controller_manager la carga y activación
    # del broadcaster de estados articulares.
    #
    # spawner espera a que el servicio del gestor esté
    # disponible, sin necesitar una pausa fija.

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120"
        ]
    )

    # ========================================================
    # 7. CONTROLADOR DE TRAYECTORIAS DEL UR5e
    # ========================================================
    # Activa el controlador que ejecutará los movimientos
    # articulares y posteriormente las trayectorias
    # planificadas mediante MoveIt 2.

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_trajectory_controller",
        output="screen",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120"
        ]
    )


    # ========================================================
    # 8. CONTROLADOR DE LA PINZA ROBOTIQ 2F-85
    # ========================================================
    # Cargamos y activamos el controlador encargado de
    # gestionar la apertura y el cierre de la pinza.
    #
    # Utiliza el controlador de pinzas paralelas definido
    # en amm_control/config/controllers.yaml.
    #
    # Controla una única articulación principal:
    #   gripper_robotiq_85_left_knuckle_joint
    #
    # Las cinco articulaciones restantes siguen su
    # movimiento mediante las relaciones mimic del URDF.
    #
    # El spawner espera a que controller_manager esté
    # disponible antes de solicitar la activación.

    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_gripper_controller",
        output="screen",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120"
        ]
    )

    # ========================================================
    # 9. ARRANQUE SECUENCIAL DE LOS CONTROLADORES
    # ========================================================
    #
    # Los nodos ROS 2 se inician de forma asíncrona.
    # Su orden dentro de LaunchDescription no garantiza
    # que uno haya terminado antes de iniciar el siguiente.
    #
    # Utilizamos eventos OnProcessExit para establecer
    # dependencias entre las operaciones:
    #
    # 1. Inserción del robot.
    # 2. Activación del joint_state_broadcaster.
    # 3. Activación del controlador del UR5e.
    # 4. Activación del controlador de la Robotiq.
    #
    # Solo continuamos cuando el proceso anterior
    # termina correctamente (código de salida 0).
    # ========================================================

    def continuar_si_exito(event, context, siguiente, nombre):
        if event.returncode == 0:
            return [siguiente]

        return [
            LogInfo(
                msg=f"ERROR: Falló {nombre}. "
                    "Se detiene la secuencia de controladores."
            )
        ]

    # Después de insertar el robot, activar broadcaster.
    iniciar_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=lambda event, context: continuar_si_exito(
                event, context,
                joint_state_broadcaster,
                "la inserción del robot"
            )
        )
    )

    # Después del broadcaster, activar el UR5e.
    iniciar_control_brazo = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=lambda event, context: continuar_si_exito(
                event, context,
                joint_trajectory_controller,
                "joint_state_broadcaster"
            )
        )
    )

    # Después del UR5e, activar la Robotiq.
    iniciar_control_pinza = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller,
            on_exit=lambda event, context: continuar_si_exito(
                event, context,
                gripper_controller,
                "joint_trajectory_controller"
            )
        )
    )

    # ========================================================
    # 10. LANZAMIENTO COMPLETO
    # ========================================================
    #
    # Primero registramos los eventos para no perder
    # ninguna notificación de finalización.
    #
    # Solo iniciamos directamente:
    #   - Gazebo.
    #   - robot_state_publisher.
    #   - Inserción del robot.
    #
    # Los controladores se inician mediante los eventos.
    # ========================================================

    return LaunchDescription([
        iniciar_broadcaster,
        iniciar_control_brazo,
        iniciar_control_pinza,

        gazebo,
        robot_state_publisher,
        spawn_robot
    ])