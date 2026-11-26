#!/usr/bin/env python3
"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - TASK COORDINATOR
============================================================
Paquete: amm_manipulation
Archivo: task_coordinator.py

Demostración validable: apple -> mesa1 -> origen.

Gestiona por orden:
  1. Inicialización de AMCL y registro de la pose de origen.
  2. Activación de YOLO, percepción RGB-D y Planning Scene.
  3. Pick completo reutilizando pick_sequence.py.
  4. Parada de los procesos de percepción gestionados aquí.
  5. Inicio de Nav2 y navegación a la estación configurada.
  6. Place reutilizando place_sequence.py.
  7. Regreso con Nav2 a la pose inicial.

IMPORTANTE:
  - Gazebo, ros2_control, TF2 y MoveIt deben estar arrancados.
  - NO inicia ni detiene Gazebo o MoveIt.
  - Solo detiene procesos de percepción que ÉL haya iniciado.
  - Mantiene las confirmaciones del pick y del place existentes.
  - La única combinación habilitada por defecto es apple/mesa1.
============================================================
"""

import argparse
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from std_srvs.srv import SetBool
from yaml import safe_load


NOMBRE_PAQUETE = "amm_manipulation"
TOPICO_YOLO = "/yolo/masks/apple"
SERVICIO_SCENE = "/amm_manipulation/freeze_scene"
NODOS_NAV2 = (
    "controller_server", "planner_server", "behavior_server", "bt_navigator"
)


def texto_booleano(valor):
    return valor.lower() == "true"


def angulo_yaw(q):
    """Obtiene yaw de un cuaternión normalizado (base sobre el plano XY)."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def error_angulo(a, b):
    """Diferencia angular mínima, en valor absoluto y radianes."""
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def cargar_estacion(ruta_yaml, destino):
    """Lee una estación habilitada, sin asumir posiciones por su nombre."""
    with open(ruta_yaml, "r", encoding="utf-8") as archivo:
        datos = safe_load(archivo) or {}
    estaciones = datos.get("stations", {})
    estacion = estaciones.get(destino)
    if not isinstance(estacion, dict) or not estacion.get("enabled", False):
        raise RuntimeError(
            f"Estación '{destino}' inexistente o deshabilitada en {ruta_yaml}."
        )
    navegacion = estacion.get("navigation_pose")
    if not isinstance(navegacion, dict) or navegacion.get("frame_id") != "map":
        raise RuntimeError(f"{destino}: se necesita navigation_pose en marco 'map'.")

    posicion = navegacion["position"]
    orientacion = navegacion["orientation"]
    xyz = [float(posicion[k]) for k in ("x", "y", "z")]
    xyzw = [float(orientacion[k]) for k in ("x", "y", "z", "w")]
    if not all(math.isfinite(v) for v in xyz + xyzw):
        raise RuntimeError("Las coordenadas de navegación deben ser finitas.")
    norma = math.sqrt(sum(v * v for v in xyzw))
    if not 0.95 <= norma <= 1.05:
        raise RuntimeError(f"Cuaternión de {destino} inválido (norma={norma:.3f}).")

    return {"xyz": xyz, "xyzw": [v / norma for v in xyzw]}


def archivo_estaciones_por_defecto():
    """Prioriza el archivo fuente, también al utilizar --symlink-install."""
    candidatos = [
        Path(__file__).resolve().parents[1] / "config" / "stations.yaml",
        Path.home() / "autonomous_mobile_manipulator" / "src" /
        "amm_manipulation" / "config" / "stations.yaml",
    ]
    for ruta in candidatos:
        if ruta.is_file():
            return ruta
    return candidatos[-1]


class ProcesosGestionados:
    """Arranca y detiene únicamente procesos hijos de este coordinador."""

    def __init__(self, workspace, logger):
        self.workspace = Path(workspace)
        self.logger = logger
        self.procesos = {}
        self.registros = {}
        carpeta = Path.home() / ".ros" / "amm_task_coordinator"
        carpeta.mkdir(parents=True, exist_ok=True)
        self.carpeta_logs = carpeta

    def iniciar(self, nombre, comando):
        if nombre in self.procesos:
            raise RuntimeError(f"El proceso '{nombre}' ya está gestionado.")
        ruta_log = self.carpeta_logs / f"{nombre}.log"
        registro = open(ruta_log, "w", encoding="utf-8")
        try:
            proceso = subprocess.Popen(
                comando,
                cwd=str(self.workspace),
                stdin=subprocess.DEVNULL,
                stdout=registro,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                env=os.environ.copy(),
            )
        except Exception:
            registro.close()
            raise
        self.procesos[nombre] = proceso
        self.registros[nombre] = registro
        self.logger.info(f"{nombre} iniciado (PID {proceso.pid}); log: {ruta_log}")

    def verificar_vivos(self, nombres):
        for nombre in nombres:
            proceso = self.procesos.get(nombre)
            if proceso is not None and proceso.poll() is not None:
                raise RuntimeError(
                    f"{nombre} terminó (código {proceso.returncode}). "
                    f"Consultar {self.carpeta_logs / (nombre + '.log')}"
                )

    def detener(self, nombre):
        proceso = self.procesos.pop(nombre, None)
        registro = self.registros.pop(nombre, None)
        if proceso is None:
            return
        if proceso.poll() is None:
            self.logger.info(f"Deteniendo {nombre} (grupo de procesos propio)...")
            try:
                os.killpg(proceso.pid, signal.SIGINT)
                proceso.wait(timeout=8.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                if proceso.poll() is None:
                    os.killpg(proceso.pid, signal.SIGTERM)
                    try:
                        proceso.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        os.killpg(proceso.pid, signal.SIGKILL)
                        proceso.wait(timeout=3.0)
        if registro is not None:
            registro.close()

    def detener_percepcion(self):
        # El nodo de escena se mantiene hasta detener el resto de las
        # publicaciones, evitando una actualización durante el cierre.
        for nombre in ("yolo", "rgbd_3d", "detected_objects_scene"):
            self.detener(nombre)

    def cerrar(self):
        for nombre in list(reversed(list(self.procesos))):
            self.detener(nombre)


class TaskCoordinator(Node):
    def __init__(self, workspace):
        super().__init__("amm_task_coordinator")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.procesos = ProcesosGestionados(workspace, self.get_logger())
        self.workspace = Path(workspace)
        self.goal_handle_actual = None
        self.ultimo_feedback = 0.0

    def esperar(self, condicion, descripcion, limite=60.0, procesos=()):
        """Espera sin bloquear indefinidamente y atiende callbacks ROS 2."""
        inicio = time.monotonic()
        while rclpy.ok() and time.monotonic() - inicio < limite:
            if condicion():
                return
            self.procesos.verificar_vivos(procesos)
            rclpy.spin_once(self, timeout_sec=0.2)
        raise RuntimeError(f"Timeout esperando: {descripcion} ({limite:.0f} s).")

    def pose_actual(self, timeout=30.0):
        """Obtiene la pose REAL estimada en map, mediante TF2/AMCL."""
        inicio = time.monotonic()
        while rclpy.ok() and time.monotonic() - inicio < timeout:
            try:
                transformacion = self.tf_buffer.lookup_transform(
                    "map", "base_link", rclpy.time.Time()
                )
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = transformacion.transform.translation.x
                pose.pose.position.y = transformacion.transform.translation.y
                pose.pose.position.z = 0.0
                pose.pose.orientation = transformacion.transform.rotation
                return pose
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.2)
        raise RuntimeError("No hay TF map -> base_link. Revisar AMCL y /tf.")

    def amcl_iniciado(self):
        """Se reutiliza la localización existente si ya proporciona TF."""
        try:
            self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            return True
        except TransformException:
            return False

    def asegurar_localizacion(self):
        """AMCL se inicia antes del pick para registrar la pose de origen."""
        # Una instancia externa puede estar arrancada, pero su TF aún no
        # haber llegado a nuestro Buffer recién creado. No duplicarla.
        nombres = {nombre for nombre, _ in self.get_node_names_and_namespaces()}
        if not self.amcl_iniciado() and "amcl" not in nombres:
            self.procesos.iniciar(
                "localizacion",
                ["ros2", "launch", "amm_slam", "localization.launch.py"],
            )
        self.esperar(
            self.amcl_iniciado, "TF de localización AMCL", 90.0,
            procesos=("localizacion",),
        )
        self.get_logger().info("AMCL disponible y TF map -> base_link activo.")

    def asegurar_nav2(self):
        """Activa Nav2 solo después del pick; espera nodos ACTIVE."""
        nombres = {nombre for nombre, _ in self.get_node_names_and_namespaces()}
        if not self.nav.wait_for_server(timeout_sec=1.0) and "bt_navigator" not in nombres:
            self.procesos.iniciar(
                "navegacion",
                ["ros2", "launch", "amm_navigation", "navigation.launch.py"],
            )

        def preparado():
            if not self.nav.wait_for_server(timeout_sec=0.05):
                return False
            # El action server puede existir antes de llegar a ACTIVE.
            for nombre in NODOS_NAV2:
                cliente = self.create_client(GetState, f"/{nombre}/get_state")
                try:
                    if not cliente.wait_for_service(timeout_sec=0.05):
                        return False
                    futuro = cliente.call_async(GetState.Request())
                    rclpy.spin_until_future_complete(self, futuro, timeout_sec=1.5)
                    if not futuro.done() or futuro.result() is None:
                        return False
                    if futuro.result().current_state.label != "active":
                        return False
                finally:
                    self.destroy_client(cliente)
            return True

        self.esperar(
            preparado, "Nav2 activo (controller, planner, behavior y BT)",
            120.0, procesos=("navegacion",),
        )
        self.get_logger().info("Nav2 ACTIVE y acción /navigate_to_pose disponible.")

    def iniciar_percepcion(self, limite=120.0):
        """Inicia los tres procesos de percepción con recursos propios."""
        # Evitar duplicados que el coordinador no podría detener de forma segura.
        if (self.get_publishers_info_by_topic(TOPICO_YOLO)
                or any(nombre == "amm_detected_objects_scene"
                       for nombre, _ in self.get_node_names_and_namespaces())):
            raise RuntimeError(
                "Ya hay percepción activa fuera del coordinador. Cierra primero "
                "YOLO, objects_position_3d y detected_objects_scene iniciados "
                "manualmente para que este programa pueda gestionarlos."
            )

        venv_python = self.workspace / ".venv_yolo" / "bin" / "python3"
        yolo_script = self.workspace / "src" / "amm_perception" / "scripts" / "yolo_segmentation.py"
        rgbd_script = self.workspace / "src" / "amm_perception" / "scripts" / "objects_position_3d.py"
        for ruta in (venv_python, yolo_script, rgbd_script):
            if not ruta.exists():
                raise RuntimeError(f"Recurso de percepción no encontrado: {ruta}")

        self.procesos.iniciar(
            "detected_objects_scene",
            ["ros2", "run", NOMBRE_PAQUETE, "detected_objects_scene",
             "--ros-args", "-p", "use_sim_time:=true"],
        )
        self.procesos.iniciar(
            "rgbd_3d",
            [str(venv_python), str(rgbd_script),
             "--ros-args", "-p", "use_sim_time:=true"],
        )
        self.procesos.iniciar(
            "yolo",
            [str(venv_python), str(yolo_script),
             "--ros-args", "-p", "use_sim_time:=true"],
        )

        cliente_escena = self.create_client(SetBool, SERVICIO_SCENE)
        try:
            self.esperar(
                lambda: cliente_escena.service_is_ready()
                and bool(self.get_publishers_info_by_topic(TOPICO_YOLO)),
                "YOLO y gestor de Planning Scene", limite,
                procesos=("yolo", "rgbd_3d", "detected_objects_scene"),
            )
        finally:
            self.destroy_client(cliente_escena)
        self.get_logger().info("Percepción activa: lista para pick.")

    def ejecutar_programa(self, ejecutable, argumentos):
        """Ejecuta el bloque previamente validado sin duplicar su lógica."""
        comando = ["ros2", "run", NOMBRE_PAQUETE, ejecutable, *argumentos]
        self.get_logger().info("Ejecutando: " + " ".join(comando))
        resultado = subprocess.run(comando, cwd=str(self.workspace), check=False)
        if resultado.returncode != 0:
            raise RuntimeError(
                f"El bloque {ejecutable} ha fallado (exit {resultado.returncode}). "
                "No se ejecutarán las siguientes etapas."
            )

    def feedback_navegacion(self, mensaje):
        ahora = time.monotonic()
        if ahora - self.ultimo_feedback >= 5.0:
            feedback = mensaje.feedback
            self.get_logger().info(
                f"Navegación: distancia restante "
                f"{feedback.distance_remaining:.2f} m; "
                f"recuperaciones={feedback.number_of_recoveries}"
            )
            self.ultimo_feedback = ahora

    def navegar(self, objetivo, etiqueta, tiempo_maximo=400.0):
        """Envía una meta de Nav2 y exige resultado SUCCEEDED."""
        if not self.nav.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Nav2 /navigate_to_pose no está disponible.")
        objetivo.header.stamp = self.get_clock().now().to_msg()
        meta = NavigateToPose.Goal()
        meta.pose = objetivo
        self.get_logger().info(
            f"Navegando a {etiqueta}: "
            f"({objetivo.pose.position.x:.3f}, {objetivo.pose.position.y:.3f})"
        )
        futuro_envio = self.nav.send_goal_async(
            meta, feedback_callback=self.feedback_navegacion
        )
        rclpy.spin_until_future_complete(self, futuro_envio, timeout_sec=15.0)
        if not futuro_envio.done() or futuro_envio.result() is None:
            raise RuntimeError("Nav2 no respondió al envío del objetivo.")
        manejador = futuro_envio.result()
        if not manejador.accepted:
            raise RuntimeError(f"Nav2 rechazó el objetivo: {etiqueta}.")
        self.goal_handle_actual = manejador
        futuro_resultado = manejador.get_result_async()
        rclpy.spin_until_future_complete(
            self, futuro_resultado, timeout_sec=tiempo_maximo
        )
        if not futuro_resultado.done() or futuro_resultado.result() is None:
            # Cancelar antes de informar del error; no empezar el place.
            cancelacion = manejador.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancelacion, timeout_sec=6.0)
            raise RuntimeError(f"Timeout navegando a {etiqueta}.")
        resultado = futuro_resultado.result()
        self.goal_handle_actual = None
        if resultado.status != GoalStatus.STATUS_SUCCEEDED or resultado.result.error_code != 0:
            raise RuntimeError(
                f"Nav2 no completó {etiqueta}: estado={resultado.status}, "
                f"error={resultado.result.error_code}, "
                f"detalle={resultado.result.error_msg!r}"
            )
        self.get_logger().info(f"Nav2 confirma llegada a {etiqueta}.")

    def cancelar_nav_si_activa(self):
        if self.goal_handle_actual is not None:
            try:
                futuro = self.goal_handle_actual.cancel_goal_async()
                rclpy.spin_until_future_complete(self, futuro, timeout_sec=4.0)
            except Exception as error:
                self.get_logger().warning(f"No se confirmó cancelación Nav2: {error}")
            finally:
                self.goal_handle_actual = None


def pose_estacion(nodo, estacion):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = nodo.get_clock().now().to_msg()
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = estacion["xyz"]
    q = estacion["xyzw"]
    pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
    pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
    return pose


def comprobar_llegada_para_place(nodo, destino, tolerancia_xy, tolerancia_yaw):
    """No autoriza el place si la base está demasiado lejos u orientada mal."""
    actual = nodo.pose_actual()
    distancia = math.hypot(
        actual.pose.position.x - destino.pose.position.x,
        actual.pose.position.y - destino.pose.position.y,
    )
    giro = error_angulo(
        angulo_yaw(actual.pose.orientation),
        angulo_yaw(destino.pose.orientation),
    )
    nodo.get_logger().info(
        f"Error final frente a mesa: {distancia:.3f} m / "
        f"{math.degrees(giro):.2f} grados."
    )
    if distancia > tolerancia_xy or giro > tolerancia_yaw:
        raise RuntimeError(
            "La base no está suficientemente alineada para ejecutar el "
            "place fijo sin percepción. Ajusta/valida Nav2 antes de continuar."
        )


def main(args=None):
    parser = argparse.ArgumentParser(
        description="AMM: pick -> Nav2 -> place -> regreso al origen."
    )
    parser.add_argument("--object", default="apple", help="Objeto solicitado.")
    parser.add_argument("--destination", default="mesa1", help="Mesa de destino.")
    parser.add_argument("--execute", choices=["true", "false"], default="false")
    parser.add_argument("--return-home", choices=["true", "false"], default="true")
    parser.add_argument(
        "--manage-perception", choices=["true", "false"], default="true",
        help="Iniciar y detener YOLO/RGB-D/Planning Scene en este proceso.",
    )
    parser.add_argument("--workspace", default=str(Path.home() / "autonomous_mobile_manipulator"))
    parser.add_argument("--stations-file", default=str(archivo_estaciones_por_defecto()))
    parser.add_argument("--max-place-xy-error", type=float, default=0.15)
    parser.add_argument("--max-place-yaw-deg", type=float, default=15.0)
    argumentos, _ = parser.parse_known_args(args)

    try:
        ruta_yaml = Path(argumentos.stations_file).expanduser().resolve()
        estacion = cargar_estacion(ruta_yaml, argumentos.destination)
        if (argumentos.object, argumentos.destination) != ("apple", "mesa1"):
            raise RuntimeError(
                "Solo apple/mesa1 está implementado y validado. "
                "Las demás órdenes se mantienen deshabilitadas."
            )
        workspace = Path(argumentos.workspace).expanduser().resolve()
        if not workspace.is_dir():
            raise RuntimeError(f"No existe el workspace: {workspace}")
        if shutil.which("ros2") is None:
            raise RuntimeError("No se encuentra ros2. Haz source del workspace.")
        if not (0 < argumentos.max_place_xy_error <= 0.3):
            raise RuntimeError("--max-place-xy-error debe estar en (0, 0.3].")
        if not (0 < argumentos.max_place_yaw_deg <= 30):
            raise RuntimeError("--max-place-yaw-deg debe estar en (0, 30].")
    except (OSError, KeyError, ValueError, RuntimeError) as error:
        print(f"[TASK] Configuración incorrecta: {error}", file=sys.stderr)
        return 1

    print("\n=========== MISIÓN AMM ===========")
    print(f"Orden: {argumentos.object},{argumentos.destination}")
    print(f"Objetivo de mesa: X={estacion['xyz'][0]:.3f}, Y={estacion['xyz'][1]:.3f}")
    print("Flujo: pick -> detener YOLO -> Nav2 -> place -> regreso.")
    print("ATENCIÓN: el place usa posturas fijas, SIN detectar la mesa.")
    if argumentos.execute != "true":
        print("[TASK] Modo sin ejecución: configuración comprobada; no se mueve el robot.")
        return 0
    if input("Escribe EJECUTAR para iniciar la misión: ").strip() != "EJECUTAR":
        print("[TASK] Cancelado.")
        return 1

    rclpy.init(args=args)
    nodo = None
    try:
        nodo = TaskCoordinator(workspace)
        logger = nodo.get_logger()

        # AMCL consume mucho menos que YOLO: se inicializa antes
        # del pick para guardar la pose inicial en el marco 'map'.
        logger.info("FASE 1: Localización y registro del origen.")
        nodo.asegurar_localizacion()
        inicio = nodo.pose_actual()
        logger.info(
            f"Pose inicial guardada: {inicio.pose.position.x:.3f}, "
            f"{inicio.pose.position.y:.3f}; "
            f"yaw={math.degrees(angulo_yaw(inicio.pose.orientation)):.1f}°"
        )

        # No matar YOLO externo: solo procesos iniciados aquí.
        logger.info("FASE 2: Percepción y recogida.")
        if texto_booleano(argumentos.manage_perception):
            nodo.iniciar_percepcion()
        else:
            logger.warning(
                "Percepción externa: el coordinador NO podrá detener esos procesos."
            )
        nodo.ejecutar_programa(
            "pick_sequence",
            ["--stage", "full", "--execute", "true", "--ros-args", "-p", "use_sim_time:=true"],
        )

        # Tras el pick no son necesarios YOLO ni cálculo RGB-D.
        logger.info("FASE 3: Liberación de recursos de percepción.")
        nodo.procesos.detener_percepcion()

        logger.info("FASE 4: Activación de Nav2 y transporte a mesa 1.")
        nodo.asegurar_nav2()
        destino = pose_estacion(nodo, estacion)
        nodo.navegar(destino, argumentos.destination)
        comprobar_llegada_para_place(
            nodo, destino,
            argumentos.max_place_xy_error,
            math.radians(argumentos.max_place_yaw_deg),
        )

        logger.info("FASE 5: Place de manzana (sin YOLO).")
        nodo.ejecutar_programa("place_sequence", ["--execute", "true"])

        if texto_booleano(argumentos.return_home):
            logger.info("FASE 6: Regreso autónomo a la posición inicial.")
            nodo.navegar(inicio, "origen")
        logger.info("=== MISIÓN FINALIZADA CORRECTAMENTE ===")
        return 0

    except (RuntimeError, OSError, subprocess.TimeoutExpired, KeyboardInterrupt) as error:
        print(f"\n[TASK] ERROR: {error}", file=sys.stderr)
        print("[TASK] Misión detenida; comprobar el estado físico en Gazebo.", file=sys.stderr)
        return 1
    finally:
        if nodo is not None:
            nodo.cancelar_nav_si_activa()
            nodo.procesos.cerrar()
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
