
#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - SIMULATION LAUNCH
============================================================

Paquete: amm_gazebo
Archivo: simulation.launch.py

Objetivo:
    Iniciar Gazebo Harmonic y establecer las comunicaciones
    necesarias entre el simulador y ROS 2.

Funcionalidades:
    1. Cargar nuestro mundo de pruebas test_world.sdf.
    2. Iniciar Gazebo con el motor gráfico Ogre.
    3. Configurar las rutas de recursos 3D.
    4. Publicar el reloj de simulación en ROS 2.
    5. Enviar comandos de velocidad a la base MiR100.
    6. Publicar la odometría de Gazebo en ROS 2.
    7. Publicar la transformación TF de odometría.

Este launch se utiliza desde robot_simulation.launch.py,
que se encarga de insertar el robot y activar sus
controladores mediante ros2_control.

============================================================
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch_ros.actions import Node


def generate_launch_description():

    # ========================================================
    # 1. LOCALIZAR NUESTRO PAQUETE Y MUNDO
    # ========================================================
    #
    # Obtenemos la ruta instalada del paquete amm_gazebo.
    # De esta forma evitamos rutas absolutas dependientes
    # de nuestro ordenador.
    #
    # El archivo test_world.sdf define el escenario de
    # pruebas donde insertaremos el manipulador móvil.

    package_path = get_package_share_directory(
        "amm_gazebo"
    )

    world_path = os.path.join(
        package_path,
        "worlds",
        "warehouse_world.sdf"
    )

    # ========================================================
    # 2. LOCALIZAR EL LAUNCH OFICIAL DE GAZEBO
    # ========================================================
    #
    # El paquete ros_gz_sim proporciona un launch para
    # iniciar Gazebo Harmonic desde ROS 2.
    #
    # Lo reutilizamos en lugar de ejecutar Gazebo mediante
    # un proceso personalizado.

    ros_gz_sim_path = get_package_share_directory(
        "ros_gz_sim"
    )

    gazebo_launch_path = os.path.join(
        ros_gz_sim_path,
        "launch",
        "gz_sim.launch.py"
    )

    # ========================================================
    # 3. CONFIGURAR LOS RECURSOS DE GAZEBO
    # ========================================================
    #
    # Gazebo necesita localizar las geometrías 3D (meshes)
    # que utilizan los modelos de nuestro robot.
    #
    # Para ello emplea GZ_SIM_RESOURCE_PATH.
    #
    # Añadimos los directorios que contienen los paquetes
    # de descripción del MiR100, UR5e y Robotiq.
    #
    # Las rutas se obtienen dinámicamente, facilitando
    # la portabilidad del proyecto a otros ordenadores.

    description_packages = [
        "mir_description",
        "ur_description",
        "robotiq_description",
    ]

    resource_paths = []

    for package_name in description_packages:

        package_share = get_package_share_directory(
            package_name
        )

        # Añadimos el directorio padre del paquete.
        #
        # Esto permite que Gazebo resuelva recursos como:
        #
        # package://mir_description/meshes/...
        # package://ur_description/meshes/...

        resource_paths.append(
            os.path.dirname(package_share)
        )

    gazebo_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join(resource_paths),
        separator=os.pathsep,
    )

        # ========================================================
    # 3.1. CONFIGURAR RENDERIZADO POR SOFTWARE
    # ========================================================
    #
    # En nuestra máquina virtual VMware, el renderizado
    # mediante aceleración gráfica provocaba que el lidar
    # GPU devolviera constantemente la distancia mínima.
    #
    # Hemos comprobado experimentalmente que forzar
    # el renderizado por software permite obtener
    # mediciones correctas de los obstáculos.
    #
    # LIBGL_ALWAYS_SOFTWARE=1 obliga a Mesa a utilizar
    # renderizado por software para OpenGL.
    #
    # Esta configuración debe aplicarse antes de iniciar
    # Gazebo para que sus procesos la hereden.

    # ============================================================
    # PRUEBA DE RENDIMIENTO EN UBUNTU NATIVO
    # ============================================================
    # Desactivamos el renderizado forzado por software.
    # Mesa podrá utilizar la aceleración gráfica de la GPU Intel.
    #
    # En VMware utilizábamos "1" por problemas con el lidar.
    # En Ubuntu nativo probamos "0" para aprovechar la GPU.
    # ============================================================

    software_rendering = SetEnvironmentVariable(
        name="LIBGL_ALWAYS_SOFTWARE",
        value="0",
    )

    # ========================================================
    # 4. INICIAR GAZEBO HARMONIC
    # ========================================================
    #
    # Argumentos utilizados:
    #
    # -r:
    #   Inicia la simulación automáticamente.
    #
    # --render-engine ogre:
    #   Utiliza Ogre para mejorar la compatibilidad
    #   gráfica con nuestra máquina virtual VMware.
    #
    # world_path:
    #   Ruta del mundo de pruebas.
    #
    # El robot no se inserta aquí: esa operación se
    # realiza desde robot_simulation.launch.py.

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gazebo_launch_path
        ),
        launch_arguments={
            "gz_args": (
                f"-r --render-engine ogre2 {world_path}"
            )
        }.items(),
    )

    # ========================================================
    # 5. PUENTE DE COMUNICACIÓN GAZEBO <-> ROS 2
    # ========================================================
    #
    # Gazebo Harmonic utiliza Gazebo Transport.
    #
    # ROS 2 utiliza DDS para comunicar sus nodos.
    #
    # ros_gz_bridge permite intercambiar mensajes entre
    # ambos sistemas.
    #
    # Sintaxis:
    #
    # @  Comunicación bidireccional.
    # [  Gazebo -> ROS 2.
    # ]  ROS 2 -> Gazebo.
    #
    # Utilizamos un único parameter_bridge para el reloj,
    # los comandos de velocidad y la odometría.

    gazebo_ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_ros_bridge",

        arguments=[

            # =================================================
            # 5.1. RELOJ DE SIMULACIÓN
            # =================================================
            #
            # Gazebo -> ROS 2
            #
            # Sincroniza los nodos ROS 2 con el tiempo
            # de simulación.
            #
            # Los nodos que tengan use_sim_time=true
            # utilizarán este reloj.

            "/clock@rosgraph_msgs/msg/Clock"
            "[gz.msgs.Clock",

            # =================================================
            # 5.2. CONTROL DE VELOCIDAD DEL MiR100
            # =================================================
            #
            # ROS 2 -> Gazebo
            #
            # Recibe comandos geometry_msgs/msg/Twist
            # publicados en /cmd_vel.
            #
            # El plugin DiffDrive de Gazebo utiliza:
            #
            # linear.x:
            #   Velocidad de avance o retroceso.
            #
            # angular.z:
            #   Velocidad de giro de la plataforma.
            #
            # A partir de estos valores calcula las
            # velocidades de las ruedas motrices.

            "/cmd_vel@geometry_msgs/msg/Twist"
            "]gz.msgs.Twist",

            # =================================================
            # 5.3. ODOMETRÍA DEL MiR100
            # =================================================
            #
            # Gazebo -> ROS 2
            #
            # El plugin DiffDrive publica la odometría
            # estimada de la plataforma.
            #
            # Contiene:
            #   - Posición.
            #   - Orientación.
            #   - Velocidad lineal.
            #   - Velocidad angular.
            #
            # Nav2 necesitará esta información para
            # estimar el movimiento del robot.

            "/odom@nav_msgs/msg/Odometry"
            "[gz.msgs.Odometry",

            # =================================================
            # 5.4. SENSOR LIDAR SICK LMS100
            # =================================================
            #
            # Gazebo -> ROS 2
            #
            # El sensor lidar simulado publica medidas de
            # distancia a los obstáculos del entorno.
            #
            # Gazebo utiliza el mensaje:
            #   gz.msgs.LaserScan
            #
            # ROS 2 utiliza:
            #   sensor_msgs/msg/LaserScan
            #
            # El topic /scan será utilizado por:
            #   - SLAM Toolbox para construir el mapa.
            #   - AMCL para localizar el robot.
            #   - Nav2 para detectar obstáculos.
            #
            # La comunicación es unidireccional porque
            # las medidas se generan dentro de Gazebo.

            "/scan@sensor_msgs/msg/LaserScan"
            "[gz.msgs.LaserScan",

        

            # =================================================
            # 5.5. CÁMARA RGB-D DE MUÑECA DEL UR5e
            # =================================================
            #
            # La cámara está montada cerca de la pinza
            # Robotiq 2F-85 y permite obtener:
            #
            #   - Imagen RGB del entorno.
            #   - Imagen de profundidad.
            #   - Parámetros intrínsecos de la cámara.
            #
            # Utilizaremos estos datos para detectar objetos
            # y calcular sus posiciones tridimensionales.
            #
            # La comunicación es unidireccional:
            #
            #   Gazebo -> ROS 2
            #
            # No conectamos por ahora la nube de puntos,
            # para reducir el consumo de recursos en VMware.

            # -------------------------------------------------
            # 5.5.1. IMAGEN RGB
            # -------------------------------------------------
            #
            # Contiene la información de color necesaria
            # para realizar segmentación con OpenCV y,
            # posteriormente, detección mediante YOLO.

            "/wrist_camera/image@sensor_msgs/msg/Image"
            "[gz.msgs.Image",

            # -------------------------------------------------
            # 5.5.2. IMAGEN DE PROFUNDIDAD
            # -------------------------------------------------
            #
            # Contiene medidas de profundidad obtenidas
            # mediante la cámara RGB-D simulada.
            #
            # Combinaremos estos datos con CameraInfo
            # para reconstruir posiciones en 3D.

            "/wrist_camera/depth_image@sensor_msgs/msg/Image"
            "[gz.msgs.Image",

            # -------------------------------------------------
            # 5.5.3. PARÁMETROS INTRÍNSECOS
            # -------------------------------------------------
            #
            # CameraInfo proporciona la calibración de
            # la cámara, incluyendo fx, fy, cx y cy.
            #
            # Estos parámetros permiten convertir píxeles
            # y profundidad en coordenadas tridimensionales.

            "/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo"
            "[gz.msgs.CameraInfo",


        ],

        output="screen",
    )

    # ========================================================
    # 6. PUENTE DE TRANSFORMACIÓN TF DE ODOMETRÍA
    # ========================================================
    #
    # El plugin DiffDrive publica una transformación
    # dinámica en Gazebo Transport.
    #
    # Topic de Gazebo:
    #
    #   /model/amm_robot/tf
    #
    # Tipo de mensaje:
    #
    #   gz.msgs.Pose_V
    #
    # Esta transformación representa:
    #
    #   odom -> base_footprint
    #
    # ROS 2 utiliza mensajes tf2_msgs/msg/TFMessage
    # para publicar transformaciones en /tf.
    #
    # El puente realiza dos operaciones:
    #
    # 1. Convierte Pose_V a TFMessage.
    #
    # 2. Redirige el topic de salida a /tf.
    #
    # IMPORTANTE:
    #
    # No publicamos aquí las transformaciones fijas
    # del robot (base_footprint -> base_link, etc.).
    #
    # Esas transformaciones ya las publica
    # robot_state_publisher mediante /tf_static.
    #
    # Separar ambos sistemas evita introducir
    # publicadores duplicados para la misma TF.

    odom_tf_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="odom_tf_bridge",

        arguments=[
            "/model/amm_robot/tf@tf2_msgs/msg/TFMessage"
            "[gz.msgs.Pose_V",
        ],

        remappings=[
            (
                "/model/amm_robot/tf",
                "/tf",
            )
        ],

        output="screen",
    )

    # ========================================================
    # 7. LANZAMIENTO COMPLETO DE LA SIMULACIÓN
    # ========================================================
    #
    # Ejecutamos las siguientes acciones:
    #
    # 1. Configurar rutas de recursos de Gazebo.
    #
    # 2. Iniciar Gazebo Harmonic con nuestro mundo.
    #
    # 3. Establecer comunicación para:
    #      - /clock
    #      - /cmd_vel
    #      - /odom
    #
    # 4. Publicar la transformación de odometría en /tf.
    #
    # Este archivo prepara la simulación y los bridges.
    #
    # El lanzamiento robot_simulation.launch.py se
    # encarga posteriormente de:
    #
    #   - Publicar robot_description.
    #   - Insertar el robot.
    #   - Inicializar controller_manager.
    #   - Activar los controladores del brazo y pinza.

    return LaunchDescription([
        software_rendering,
        gazebo_resource_path,
        gazebo,
        gazebo_ros_bridge,
        odom_tf_bridge,
    ])
