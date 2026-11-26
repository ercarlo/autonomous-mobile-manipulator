#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - PICK SEQUENCE
============================================================

Paquete: amm_manipulation
Archivo: pick_sequence.py

Objetivo:
    Coordinar las diferentes operaciones necesarias para
    recoger un objeto con el UR5e y la Robotiq 2F-85.

Etapa actual:
    Automatización del movimiento posterior al agarre:

        1. Ir a observation.
        2. Ir a transport.

En próximas etapas incorporaremos:
    - Detección RGB-D.
    - Aproximación pre-pick.
    - Descenso cartesiano.
    - Attach de Gazebo.
    - Elevación vertical.
============================================================
"""

import argparse
import rclpy

from amm_manipulation.arm_controller import ArmController

import subprocess
import sys

from pathlib import Path

import re
import time
import os

from std_srvs.srv import SetBool
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents

import time

from std_msgs.msg import UInt64

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

# ============================================================
# CONFIGURACIÓN DE LA SECUENCIA
# ============================================================

# Posturas guardadas en el SRDF.
#
# El robot debe haber completado previamente el agarre
# y la elevación vertical antes de ejecutar esta secuencia.

POST_PICK_STATES = [
    "observation",
    "transport",
]


# ============================================================
# COORDINADOR DE MANIPULACIÓN
# ============================================================

class PickSequence:

    def __init__(self):

        # Reutilizamos el controlador MoveIt 2 existente.
        self.arm = ArmController()

        self.logger = self.arm.get_logger()

        self.logger.info(
            "Coordinador de manipulación inicializado."
        )

    def mover_a_estado(self, nombre_estado, ejecutar):

        """
        Planifica o ejecuta un movimiento hacia una
        configuración articular guardada en el SRDF.

        Devuelve True únicamente si MoveIt confirma
        que la operación ha finalizado correctamente.
        """

        self.logger.info(
            f"Iniciando movimiento hacia: {nombre_estado}"
        )

        correcto = self.arm.mover_a(
            objetivo=nombre_estado,
            ejecutar=ejecutar,
        )

        if not correcto:

            self.logger.error(
                f"Error en el movimiento hacia {nombre_estado}."
            )

            return False

        self.logger.info(
            f"Operación hacia {nombre_estado} completada."
        )

        return True


        # ========================================================
    # PLANIFICACIÓN DE APROXIMACIÓN PRE-PICK
    # ========================================================

    def planificar_prepick(self, ejecutar=False):

        """
        Ejecuta el programa de aproximación pre-pick.

        Reutiliza prepick_observation.py en un proceso
        independiente para evitar inicializar ROS 2
        dos veces dentro del mismo proceso.

        En esta etapa solo planificamos. La ejecución
        se habilitará cuando validemos la integración.
        """

        self.logger.info(
            "=== INICIO DE PLANIFICACIÓN PRE-PICK ==="
        )

                # ====================================================
        # ETAPA 1: POSICIÓN GENERAL DE OBSERVACIÓN
        # ====================================================

        if ejecutar:

            self.logger.info(
                "ETAPA 1: Moviendo el UR5e a observation."
            )

            correcto = self.mover_a_estado(
                nombre_estado="observation",
                ejecutar=True,
            )

            if not correcto:
                self.logger.error(
                    "No se ha alcanzado observation. "
                    "Se cancela la aproximación."
                )
                return False

        else:

            # Planificar observation sin ejecutarla no cambia
            # la posición de la cámara. Por tanto, este modo
            # requiere que la manzana ya sea visible.
            self.logger.warning(
                "Modo planificación: el brazo debe estar "
                "previamente en una posición desde la que "
                "pueda detectar la manzana."
            )

        # Localizamos el programa existente dentro
        # del mismo paquete Python.
        ruta_script = (
            Path(__file__).resolve().parent
            / "prepick_observation.py"
        )

        if not ruta_script.is_file():
            self.logger.error(
                f"No se encuentra el programa: {ruta_script}"
            )
            return False

        # Parámetros que validamos anteriormente para
        # observar la manzana desde una posición cercana.
        #
        # La posición XYZ de la manzana NO está fijada:
        # el programa utiliza los datos actuales RGB-D.

        comando = [
            sys.executable,
            str(ruta_script),

            "--object", "apple",
            "--distance", "0.40",
            "--offset-x", "0.10",
            "--offset-y", "-0.03",

            "--plan", "true",
            "--execute", "true" if ejecutar else "false",

            "--ros-args",
            "-p", "use_sim_time:=true",
        ]

        self.logger.info(
            "Solicitando planificación de aproximación."
        )

        try:
            resultado = subprocess.run(
                comando,
                check=False,
            )

        except OSError as error:
            self.logger.error(
                f"No se pudo iniciar pre-pick: {error}"
            )
            return False

        if resultado.returncode != 0:
            self.logger.error(
                "La planificación pre-pick ha fallado."
            )
            return False

        self.logger.info(
            "=== PLANIFICACIÓN PRE-PICK FINALIZADA ==="
        )

        return True


        # ========================================================
    # ETAPA: PRE-GRASP VERTICAL
    # ========================================================

    def ejecutar_pregrasp(self, ejecutar=False):
        """
        Planifica o ejecuta el posicionamiento vertical
        de la Robotiq sobre la manzana.

        Reutiliza vertical_grasp.py en un proceso separado.

        La posición del objeto procede de la percepción
        RGB-D actual, no de coordenadas predefinidas.

        Durante las pruebas, el programa original mantiene
        su confirmación manual antes de ejecutar.
        """

        self.logger.info(
            "=== INICIO DE PRE-GRASP VERTICAL ==="
        )

        ruta_script = (
            Path(__file__).resolve().parent
            / "vertical_grasp.py"
        )

        if not ruta_script.is_file():
            self.logger.error(
                f"No se encuentra el programa: {ruta_script}"
            )
            return False

        # Parámetros empleados en las pruebas anteriores.
        comando = [
            sys.executable,
            str(ruta_script),

            "--object", "apple",
            "--diameter", "0.075",
            "--clearance", "0.15",
            "--yaw-deg", "0",

            "--plan", "true",
            "--execute", "true" if ejecutar else "false",

            "--ros-args",
            "-p", "use_sim_time:=true",
        ]

        try:
            resultado = subprocess.run(
                comando,
                check=False,
            )

        except OSError as error:
            self.logger.error(
                f"Error iniciando vertical_grasp.py: {error}"
            )
            return False

        if resultado.returncode != 0:
            self.logger.error(
                "El pre-grasp vertical ha fallado."
            )
            return False

        self.logger.info(
            "=== PRE-GRASP VERTICAL COMPLETADO ==="
        )

        return True


        # ========================================================
    # DESCENSO AUTOMÁTICO COMPLETO
    # ========================================================

    def ejecutar_descenso_auto(self, referencia_xyz=None):
        """
        Ejecuta el descenso completo reutilizando
        cartesian_descent.py.

        El propio programa gestiona:
            - Cálculo de los tramos.
            - Planificación cartesiana.
            - Confirmación entre los dos primeros.
            - Verificación TF2.
            - Finalización al alcanzar la altura objetivo.

        La referencia de la manzana se obtiene de la
        Planning Scene actual, sin coordenadas fijas.
        """

        self.logger.info(
            "=== INICIO DE DESCENSO AUTOMÁTICO ==="
        )

        ruta_script = (
            Path(__file__).resolve().parent
            / "cartesian_descent.py"
        )

        if not ruta_script.is_file():

            self.logger.error(
                f"No se encuentra el script: {ruta_script}"
            )

            return False

        comando = [
            sys.executable,
            str(ruta_script),

            "--auto-height", "true",
            "--auto-sequence", "true",

            "--step", "0.002",
            "--execute", "true",

            "--ros-args",
            "-p", "use_sim_time:=true",
        ]

        if referencia_xyz is not None:

            indice_ros = comando.index("--ros-args")

            comando[indice_ros:indice_ros] = [
                "--reference-xyz",
                *[str(valor) for valor in referencia_xyz],
            ]

        try:

            resultado = subprocess.run(
                comando,
                check=False,
            )

        except OSError as error:

            self.logger.error(
                f"Error iniciando el descenso: {error}"
            )

            return False

        if resultado.returncode != 0:

            self.logger.error(
                "El descenso automático ha fallado "
                "o ha sido cancelado."
            )

            return False

        self.logger.info(
            "=== DESCENSO AUTOMÁTICO COMPLETADO ==="
        )

        return True


        # ========================================================
    # ETAPA 7.6.5: AGARRE SIMULADO EN GAZEBO
    # ========================================================

    def ejecutar_attach(self):
        """
        Une la manzana a la muñeca del UR5e utilizando
        el plugin DetachableJoint de Gazebo Harmonic.

        El procedimiento:
          1. Comprueba que existen el robot y la manzana.
          2. Obtiene el ID actual del robot.
          3. Comprueba que el plugin no esté ya cargado.
          4. Solicita confirmación al usuario.
          5. Carga DetachableJoint dinámicamente.
          6. Verifica el estado 'attached'.

        El brazo no se mueve durante esta operación.
        """

        self.logger.info(
            "=== INICIO DE ATTACH AUTOMÁTICO ==="
        )

        mundo = "amm_warehouse_world"
        modelo_robot = "amm_robot"
        modelo_objeto = "ycb_apple"

        topic_estado = "/amm/apple/attachment_state"
        topic_attach = "/amm/apple/attach"

        try:
            # ------------------------------------------------
            # 1. Consultar el modelo del robot en Gazebo.
            # ------------------------------------------------

            consulta_robot = subprocess.run(
                ["gz", "model", "-m", modelo_robot],
                capture_output=True,
                text=True,
                timeout=15,
                check=False,
            )

            if consulta_robot.returncode != 0:
                raise RuntimeError(
                    "No se ha podido consultar el modelo del robot."
                )

            # No utilizamos IDs fijos: pueden cambiar
            # cada vez que reiniciamos Gazebo.
            coincidencia = re.search(
                r"(?m)^Model:\s*\[(\d+)\]",
                consulta_robot.stdout,
            )

            if coincidencia is None:
                raise RuntimeError(
                    "No se ha encontrado el ID de amm_robot."
                )

            robot_id = int(coincidencia.group(1))

            self.logger.info(
                f"Robot identificado: {modelo_robot} "
                f"(ID={robot_id})."
            )

            # ------------------------------------------------
            # 2. Comprobar que existe la manzana.
            # ------------------------------------------------

            consulta_objeto = subprocess.run(
                ["gz", "model", "-m", modelo_objeto],
                capture_output=True,
                text=True,
                timeout=15,
                check=False,
            )

            if (
                consulta_objeto.returncode != 0
                or f"Name: {modelo_objeto}"
                not in consulta_objeto.stdout
            ):
                raise RuntimeError(
                    "No se encuentra ycb_apple en Gazebo."
                )

            # ------------------------------------------------
            # 3. Evitar cargar dos veces el mismo plugin.
            # ------------------------------------------------

            consulta_topics = subprocess.run(
                ["gz", "topic", "-l"],
                capture_output=True,
                text=True,
                timeout=10,
                check=False,
            )

            if consulta_topics.returncode != 0:
                raise RuntimeError(
                    "No se pudieron consultar los topics de Gazebo."
                )

            if topic_attach in consulta_topics.stdout.splitlines():
                raise RuntimeError(
                    "El plugin DetachableJoint ya parece estar "
                    "cargado. No se creará una segunda unión."
                )

            # ------------------------------------------------
            # 4. Confirmación antes del agarre.
            # ------------------------------------------------

            print("\n" + "=" * 55)
            print("ATTACH AUTOMÁTICO - MANZANA")
            print("=" * 55)

            print(f"Robot: {modelo_robot}")
            print(f"ID actual: {robot_id}")
            print(f"Objeto: {modelo_objeto}")
            print("Enlace robot: ur5e_wrist_3_link")
            print("Enlace objeto: apple_link")

            print(
                "\nComprueba en Gazebo que la manzana "
                "está correctamente situada entre los dedos."
            )

            decision = input(
                "\nEscribe ATTACH para crear la unión: "
            ).strip()

            if decision != "ATTACH":
                raise RuntimeError(
                    "Attach cancelado por el usuario."
                )

            # ------------------------------------------------
            # 5. Preparar la configuración del plugin.
            # ------------------------------------------------

            configuracion_xml = (
                "<parent_link>ur5e_wrist_3_link</parent_link>"
                "<child_model>ycb_apple</child_model>"
                "<child_link>apple_link</child_link>"
                "<detach_topic>/amm/apple/detach</detach_topic>"
                "<attach_topic>/amm/apple/attach</attach_topic>"
                "<output_topic>"
                "/amm/apple/attachment_state"
                "</output_topic>"
            )

            # Construimos el mensaje protobuf utilizando
            # el ID obtenido de la simulación actual.
            peticion = (
                f"entity {{ id: {robot_id} }} "
                "plugins { "
                'name: "gz::sim::systems::DetachableJoint" '
                'filename: "gz-sim-detachable-joint-system" '
                f'innerxml: "{configuracion_xml}" '
                "}"
            )

            servicio = (
                f"/world/{mundo}/entity/system/add"
            )

            # ------------------------------------------------
            # 6. Escuchar el estado antes de cargar el plugin.
            # ------------------------------------------------

            monitor = subprocess.Popen(
                ["gz", "topic", "-e", "-t", topic_estado],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )

            try:
                # Dar tiempo a establecer la suscripción.
                time.sleep(1.0)

                respuesta = subprocess.run(
                    [
                        "gz", "service",
                        "-s", servicio,
                        "--reqtype", "gz.msgs.EntityPlugin_V",
                        "--reptype", "gz.msgs.Boolean",
                        "--timeout", "10000",
                        "--req", peticion,
                    ],
                    capture_output=True,
                    text=True,
                    timeout=15,
                    check=False,
                )

                if (
                    respuesta.returncode != 0
                    or not re.search(
                        r"\bdata:\s*true\b",
                        respuesta.stdout,
                    )
                ):
                    raise RuntimeError(
                        "Gazebo no ha confirmado la carga "
                        "de DetachableJoint.\n"
                        + respuesta.stdout
                        + respuesta.stderr
                    )

                self.logger.info(
                    "Gazebo ha aceptado la carga del plugin."
                )

                # Esperar el mensaje de confirmación.
                time.sleep(2.0)

            finally:
                monitor.terminate()

                try:
                    salida_estado, _ = monitor.communicate(
                        timeout=3
                    )
                except subprocess.TimeoutExpired:
                    monitor.kill()
                    salida_estado, _ = monitor.communicate()

            # ------------------------------------------------
            # 7. Verificar el estado del agarre.
            # ------------------------------------------------

            if not re.search(
                r'data:\s*"?attached"?',
                salida_estado,
            ):
                raise RuntimeError(
                    "El plugin se ha cargado, pero no se "
                    "ha recibido la confirmación 'attached'. "
                    "Comprueba el estado en Gazebo antes "
                    "de intentar cargarlo otra vez."
                )

            self.logger.info(
                "=== ATTACH COMPLETADO CORRECTAMENTE ==="
            )

            return True

        except (
            OSError,
            RuntimeError,
            subprocess.TimeoutExpired,
            EOFError,
        ) as error:

            self.logger.error(
                f"Error durante el attach: {error}"
            )

            return False

        # ========================================================
    # ELEVACIÓN POSTERIOR AL AGARRE
    # ========================================================

    def ejecutar_lift(self):
        """
        Eleva el UR5e un total de 60 mm.

        Reutiliza cartesian_descent.py en su modo
        de elevación posterior al agarre.

        No necesita conocer la posición de la manzana.
        """

        self.logger.info(
            "=== INICIO DE ELEVACIÓN POST-PICK ==="
        )

        ruta_script = (
            Path(__file__).resolve().parent
            / "cartesian_descent.py"
        )

        comando = [
            sys.executable,
            str(ruta_script),

            "--lift-post-pick", "true",
            "--step", "0.002",
            "--execute", "true",

            "--ros-args",
            "-p", "use_sim_time:=true",
        ]

        try:
            resultado = subprocess.run(
                comando,
                check=False,
            )

        except OSError as error:
            self.logger.error(
                f"Error iniciando la elevación: {error}"
            )
            return False

        if resultado.returncode != 0:
            self.logger.error(
                "La elevación ha fallado."
            )
            return False

        self.logger.info(
            "Elevación de 60 mm completada."
        )

        return True

    def ejecutar_post_pick(self, ejecutar):

        """
        Secuencia posterior al agarre.

        PRECONDICIONES PARA EJECUTAR:
          - El objeto está unido a la Robotiq en Gazebo.
          - El brazo ha realizado la elevación vertical.
          - Las trayectorias previstas son seguras.

        Movimientos:
          1. observation
          2. transport
        """

        self.logger.info(
            "=== INICIO DE SECUENCIA POST-PICK ==="
        )

        total = len(POST_PICK_STATES)

        for indice, estado in enumerate(
            POST_PICK_STATES, start=1
        ):

            self.logger.info(
                f"Etapa {indice}/{total}: {estado}"
            )

            correcto = self.mover_a_estado(
                nombre_estado=estado,
                ejecutar=ejecutar,
            )

            if not correcto:

                self.logger.error(
                    "Secuencia detenida por error."
                )

                return False

        self.logger.info(
            "=== SECUENCIA POST-PICK COMPLETADA ==="
        )

        return True

        # ========================================================
    # ELEVACIÓN + POSICIÓN DE TRANSPORTE
    # ========================================================

    def ejecutar_lift_y_transporte(self):
        """
        Secuencia posterior al agarre:

            1. Elevación vertical de 60 mm.
            2. Movimiento a observation.
            3. Movimiento a transport.

        Se detiene si cualquiera de las operaciones falla.
        """

        self.logger.info(
            "=== INICIO DE LIFT Y TRANSPORTE ==="
        )

        # Primera operación: elevar el objeto.
        if not self.ejecutar_lift():
            self.logger.error(
                "No se ha completado la elevación."
            )
            return False

        # Segunda operación: reutilizar el método
        # que ya desarrollamos y validamos.
        if not self.ejecutar_post_pick(ejecutar=True):
            self.logger.error(
                "No se ha completado el movimiento "
                "hacia la posición de transporte."
            )
            return False

        self.logger.info(
            "=== PICK COMPLETADO: BRAZO EN TRANSPORT ==="
        )

        return True


        # ========================================================
    # GESTIÓN DE LA PLANNING SCENE
    # ========================================================

    def llamar_servicio(self, tipo, nombre, solicitud):
        """
        Realiza una llamada ROS 2 y comprueba que
        se recibe una respuesta válida.
        """

        cliente = self.arm.create_client(tipo, nombre)

        try:
            if not cliente.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(
                    f"Servicio no disponible: {nombre}"
                )

            futuro = cliente.call_async(solicitud)

            rclpy.spin_until_future_complete(
                self.arm,
                futuro,
                timeout_sec=10.0,
            )

            if not futuro.done():
                raise RuntimeError(
                    f"Tiempo de espera agotado: {nombre}"
                )

            if futuro.exception() is not None:
                raise RuntimeError(
                    f"Error en {nombre}: {futuro.exception()}"
                )

            respuesta = futuro.result()

            if respuesta is None:
                raise RuntimeError(
                    f"Respuesta vacía del servicio {nombre}"
                )

            return respuesta

        finally:
            self.arm.destroy_client(cliente)

    def cambiar_congelacion(self, congelar):
        """
        Congela o descongela los objetos detectados.

        True  -> Mantener los obstáculos actuales.
        False -> Permitir actualizaciones desde YOLO.
        """

        solicitud = SetBool.Request()
        solicitud.data = congelar

        respuesta = self.llamar_servicio(
            SetBool,
            "/amm_manipulation/freeze_scene",
            solicitud,
        )

        if not respuesta.success:
            raise RuntimeError(
                "No se ha podido modificar la congelación "
                f"de la Planning Scene: {respuesta.message}"
            )

        estado = "CONGELADA" if congelar else "DESCONGELADA"

        self.logger.info(
            f"Planning Scene {estado}."
        )

    def comprobar_manzana_en_scene(self):
        """
        Comprueba que MoveIt contiene el objeto
        de colisión correspondiente a la manzana.
        """

        solicitud = GetPlanningScene.Request()

        solicitud.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
        )

        respuesta = self.llamar_servicio(
            GetPlanningScene,
            "/get_planning_scene",
            solicitud,
        )

        objetos = {
            objeto.id
            for objeto in respuesta.scene.world.collision_objects
        }

        return "amm_detected_apple" in objetos

        # ========================================================
    # ACTUALIZACIÓN VERIFICADA DE LA MANZANA
    # ========================================================

    def actualizar_escena_manzana(self):
        """
        Solicita una observación nueva de la manzana.

        El coordinador:
          1. Obtiene la última revisión confirmada.
          2. Descongela la Planning Scene.
          3. Espera una revisión posterior.
          4. Congela nuevamente la escena.
          5. Comprueba que la manzana existe en MoveIt.

        No se aceptan únicamente cubos antiguos:
        debe producirse una actualización nueva.
        """

        self.logger.info(
            "=== ACTUALIZACIÓN VERIFICADA DE MANZANA ==="
        )

        revision_actual = None

        # Callback que registra la revisión publicada.
        def revision_callback(mensaje):
            nonlocal revision_actual
            revision_actual = int(mensaje.data)

        qos_revision = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        suscripcion = self.arm.create_subscription(
            UInt64,
            "/amm_manipulation/apple_scene_revision",
            revision_callback,
            qos_revision,
        )

        escena_descongelada = False

        try:

            # ------------------------------------------------
            # 1. Recuperar la revisión anterior.
            # ------------------------------------------------

            inicio = time.monotonic()

            while revision_actual is None:

                rclpy.spin_once(
                    self.arm,
                    timeout_sec=0.1,
                )

                if time.monotonic() - inicio > 5.0:
                    raise RuntimeError(
                        "No se recibe el contador de "
                        "actualizaciones de la manzana."
                    )

            revision_anterior = revision_actual

            self.logger.info(
                f"Revisión anterior: {revision_anterior}"
            )

            # ------------------------------------------------
            # 2. Habilitar nuevas observaciones.
            # ------------------------------------------------

            self.cambiar_congelacion(False)
            escena_descongelada = True

            # ------------------------------------------------
            # 3. Esperar una actualización confirmada.
            # ------------------------------------------------

            inicio = time.monotonic()
            actualizacion_confirmada = False

            while time.monotonic() - inicio < 10.0:

                rclpy.spin_once(
                    self.arm,
                    timeout_sec=0.1,
                )

                if (
                    revision_actual is not None
                    and revision_actual > revision_anterior
                ):

                    actualizacion_confirmada = True

                    self.logger.info(
                        "Nueva actualización confirmada: "
                        f"{revision_actual}"
                    )

                    break

            if not actualizacion_confirmada:

                raise RuntimeError(
                    "No se ha confirmado una actualización "
                    "nueva de la manzana en 10 segundos."
                )

        finally:

            # ------------------------------------------------
            # 4. Congelar antes de continuar.
            #
            # El servicio puede indicar que hay una petición
            # pendiente. En ese caso reintentamos.
            # ------------------------------------------------

            try:
                if escena_descongelada:

                    congelacion_correcta = False

                    for intento in range(3):

                        try:
                            self.cambiar_congelacion(True)
                            congelacion_correcta = True
                            break

                        except RuntimeError:

                            if intento < 2:
                                time.sleep(1.0)

                    if not congelacion_correcta:
                        raise RuntimeError(
                            "No se ha podido confirmar la "
                            "congelación de la Planning Scene."
                        )

            finally:
                self.arm.destroy_subscription(suscripcion)

        # ----------------------------------------------------
        # 5. Comprobar que existe la manzana en MoveIt.
        # ----------------------------------------------------

        if not self.comprobar_manzana_en_scene():

            raise RuntimeError(
                "MoveIt no contiene amm_detected_apple "
                "después de la actualización."
            )

        self.logger.info(
            "=== MANZANA ACTUALIZADA Y ESCENA CONGELADA ==="
        )

        return True


        # ========================================================
    # PICK COMPLETO
    # ========================================================

    def ejecutar_pick_completo(self):
        """
        Coordina toda la recogida de la manzana.

        Secuencia:
            1. Observation + Pre-pick.
            2. Actualización de la Planning Scene.
            3. Pre-grasp vertical.
            4. Descenso cartesiano.
            5. Attach en Gazebo.
            6. Elevación y transporte.

        Si falla cualquier etapa, se interrumpe
        la secuencia inmediatamente.
        """

        self.logger.info(
            "========== INICIO PICK COMPLETO =========="
        )

        # ----------------------------------------------------
        # ETAPA 1 - OBSERVACIÓN Y APROXIMACIÓN
        # ----------------------------------------------------

        self.logger.info(
            "ETAPA 1/6: Observation y Pre-pick."
        )

        if not self.planificar_prepick(ejecutar=True):
            return False

        # ----------------------------------------------------
        # ETAPA 2 - ACTUALIZAR LA MANZANA
        # ----------------------------------------------------

        self.logger.info(
            "ETAPA 2/6: Actualizar Planning Scene."
        )

        if not self.actualizar_escena_manzana():
            return False

        # ----------------------------------------------------
        # ETAPA 3 - PRE-GRASP VERTICAL
        # ----------------------------------------------------

        self.logger.info(
            "ETAPA 3/6: Pre-grasp vertical."
        )

        if not self.ejecutar_pregrasp(ejecutar=True):
            return False

        # ----------------------------------------------------
        # ETAPA 4 - DESCENSO CARTESIANO
        # ----------------------------------------------------

        self.logger.info(
            "ETAPA 4/6: Descenso cartesiano."
        )

        # El cubo se encuentra congelado, así que
        # el descenso podrá obtener su posición.
        #
        # No necesitamos introducir coordenadas RGB-D:
        # cartesian_descent.py conservará la referencia
        # cuando retire amm_detected_apple.

        if not self.ejecutar_descenso_auto(
            referencia_xyz=None
        ):
            return False

        # ----------------------------------------------------
        # ETAPA 5 - ATTACH GAZEBO
        # ----------------------------------------------------

        self.logger.info(
            "ETAPA 5/6: Attach de la manzana."
        )

        if not self.ejecutar_attach():
            return False

        # ----------------------------------------------------
        # ETAPA 6 - ELEVACIÓN Y TRANSPORTE
        # ----------------------------------------------------

        self.logger.info(
            "ETAPA 6/6: Elevación y transporte."
        )

        if not self.ejecutar_lift_y_transporte():
            return False

        # ----------------------------------------------------
        # FIN
        # ----------------------------------------------------

        self.logger.info(
            "========================================"
        )

        self.logger.info(
            "PICK COMPLETO FINALIZADO CORRECTAMENTE"
        )

        self.logger.info(
            "UR5e en posición TRANSPORT."
        )

        self.logger.info(
            "========================================"
        )

        return True

    def destruir(self):

        self.arm.destroy_node()





    

# ============================================================
# MAIN
# ============================================================

def main(args=None):

    parser = argparse.ArgumentParser(
        description="Coordinador de manipulación del AMM."
    )

    parser.add_argument(
        "--execute",
        choices=["true", "false"],
        default="false",
        help="Ejecutar movimientos o únicamente planificarlos.",
    )

    parser.add_argument(
        "--stage",
        choices=[
            "prepick",
            "pregrasp",
            "descent",
            "attach",
            "lift",
            "post_pick",
            "full",
        ],
        default="post_pick",
        help="Etapa del coordinador que se desea ejecutar.",
    )

    parser.add_argument(
        "--reference-xyz",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=None,
        help=(
            "Última posición RGB-D válida de la manzana "
            "en base_link, antes de la oclusión."
        ),
    )

    # ========================================================
    # INTERPRETACIÓN Y VALIDACIÓN DE ARGUMENTOS
    # ========================================================

    argumentos, _ = parser.parse_known_args(args)

    ejecutar = argumentos.execute == "true"

    # Estas etapas modifican el estado del robot o de Gazebo.
    # Por seguridad, no permitimos ejecutarlas sin habilitar
    # explícitamente el modo de ejecución.

    etapas_solo_ejecucion = [
        "descent",
        "attach",
        "lift",
        "full",
    ]

    if (
        argumentos.stage in etapas_solo_ejecucion
        and not ejecutar
    ):
        parser.error(
            f"La etapa {argumentos.stage} "
            "requiere --execute true."
        )

    # ========================================================
    # CONFIRMACIÓN GENERAL DE SEGURIDAD
    # ========================================================

    # Las etapas descent y attach gestionan sus propias
    # confirmaciones manuales.
    #
    # La etapa lift solicita una única confirmación general
    # antes de comenzar la elevación y el transporte.

    if ejecutar and argumentos.stage not in ("descent", "attach"):

        print(
            "\nATENCIÓN: se ejecutarán movimientos "
            "del UR5e en Gazebo."
        )

        autorizacion = input(
            "Escribe EJECUTAR para iniciar la secuencia: "
        ).strip()

        if autorizacion != "EJECUTAR":

            print("Ejecución cancelada.")
            return 1

    # ========================================================
    # INICIALIZACIÓN DE ROS 2
    # ========================================================

    rclpy.init(args=args)

    coordinador = None

    try:

        coordinador = PickSequence()

        # ====================================================
        # SELECCIÓN DE ETAPA
        # ====================================================

        if argumentos.stage == "prepick":

            # Posición de observación y aproximación.
            correcto = coordinador.planificar_prepick(
                ejecutar=ejecutar
            )

        elif argumentos.stage == "pregrasp":

            # Posicionamiento vertical de la Robotiq.
            correcto = coordinador.ejecutar_pregrasp(
                ejecutar=ejecutar
            )

        elif argumentos.stage == "descent":

            # Descenso cartesiano automático por tramos.
            # Se conserva la confirmación entre el primer
            # y el segundo movimiento.
            correcto = coordinador.ejecutar_descenso_auto(
                referencia_xyz=argumentos.reference_xyz
            )

        elif argumentos.stage == "attach":

            # Unión rígida entre la manzana y el UR5e.
            correcto = coordinador.ejecutar_attach()

        elif argumentos.stage == "lift":

            # Elevación vertical de 60 mm.
            #
            # Después reutiliza ejecutar_post_pick()
            # para mover el brazo a:
            #   1. observation
            #   2. transport

            correcto = coordinador.ejecutar_lift_y_transporte()

        elif argumentos.stage == "full":

            correcto = coordinador.ejecutar_pick_completo()

        elif argumentos.stage == "post_pick":

            # Movimientos observation -> transport.
            #
            # Se mantiene esta etapa independiente para
            # realizar pruebas o movimientos sin elevación.

            correcto = coordinador.ejecutar_post_pick(
                ejecutar=ejecutar
            )

        else:

            raise RuntimeError(
                f"Etapa desconocida: {argumentos.stage}"
            )

        # ====================================================
        # RESULTADO DE LA OPERACIÓN
        # ====================================================

        if correcto:

            coordinador.logger.info(
                f"Etapa '{argumentos.stage}' "
                "completada correctamente."
            )

            return 0

        coordinador.logger.error(
            f"La etapa '{argumentos.stage}' ha fallado."
        )

        return 1

    except Exception as error:

        print(f"Error durante la secuencia: {error}")

        return 1

    finally:

        if coordinador is not None:
            coordinador.destruir()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":

    raise SystemExit(main())