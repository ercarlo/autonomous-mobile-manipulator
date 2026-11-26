#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - WORKSTATION SCENE
============================================================

Paquete: amm_manipulation
Archivo: workstation_scene.py

Objetivo:
    Incorporar las estaciones de trabajo conocidas
    a la Planning Scene de MoveIt 2.

Funcionamiento:
    - Leer las estaciones desde un archivo YAML.
    - Validar sus geometrías y posiciones.
    - Añadirlas a MoveIt mediante PlanningSceneManager.
    - Consultar los objetos de colisión.
    - Eliminar las estaciones configuradas.

No utiliza las posiciones de los objetos manipulables.

Esas posiciones se obtendrán posteriormente mediante
YOLO, profundidad y TF2.

============================================================
"""

import argparse
import math
import sys
from pathlib import Path

import yaml
import rclpy

from ament_index_python.packages import get_package_share_directory

from amm_manipulation.planning_scene import PlanningSceneManager


# ============================================================
# CONFIGURACIÓN
# ============================================================

NOMBRE_CONFIGURACION = "workstations.yaml"

OPERACIONES = ["add", "remove", "list"]


# ============================================================
# LECTURA DE CONFIGURACIÓN
# ============================================================

def cargar_estaciones():

    """
    Lee la configuración de estaciones desde YAML.

    Devuelve:
        Lista de obstáculos con sus geometrías,
        posiciones y sistemas de referencia.
    """

    # Obtener la ruta instalada del paquete.

    ruta_paquete = Path(
        get_package_share_directory("amm_manipulation")
    )

    ruta_archivo = (
        ruta_paquete
        / "config"
        / NOMBRE_CONFIGURACION
    )

    # Comprobar la existencia del archivo.

    if not ruta_archivo.is_file():

        raise FileNotFoundError(
            f"No existe la configuración: {ruta_archivo}"
        )

    # Cargar el documento YAML.

    with ruta_archivo.open("r", encoding="utf-8") as archivo:

        configuracion = yaml.safe_load(archivo)

    if not isinstance(configuracion, dict):

        raise ValueError(
            "La configuración YAML no es válida."
        )

    # Sistema de referencia predeterminado.

    frame = configuracion.get("frame_id")

    if not isinstance(frame, str) or not frame:

        raise ValueError(
            "Debe especificarse un frame_id válido."
        )

    # Obtener los obstáculos configurados.

    obstaculos = configuracion.get("obstacles", [])

    if not isinstance(obstaculos, list):

        raise ValueError(
            "La sección obstacles debe ser una lista."
        )

    identificadores = set()

    # ========================================================
    # VALIDACIÓN DE OBSTÁCULOS
    # ========================================================

    for obstaculo in obstaculos:

        if not isinstance(obstaculo, dict):

            raise ValueError(
                "Cada obstáculo debe ser un diccionario."
            )

        identificador = obstaculo.get("id")

        # Evitar identificadores duplicados.

        if not isinstance(identificador, str):

            raise ValueError(
                "El identificador debe ser una cadena."
            )

        if not identificador.startswith("amm_"):

            raise ValueError(
                f"Identificador no permitido: {identificador}"
            )

        if identificador in identificadores:

            raise ValueError(
                f"Identificador duplicado: {identificador}"
            )

        identificadores.add(identificador)

        # La primera versión admite cajas.
        # Podremos incorporar otras geometrías más adelante.

        if obstaculo.get("geometry") != "box":

            raise ValueError(
                f"Geometría no soportada: {identificador}"
            )

        posicion = obstaculo.get("position")
        dimensiones = obstaculo.get("dimensions")

        # Comprobar vectores XYZ.

        if not isinstance(posicion, list) or len(posicion) != 3:

            raise ValueError(
                f"Posición incorrecta: {identificador}"
            )

        if not isinstance(dimensiones, list) or len(dimensiones) != 3:

            raise ValueError(
                f"Dimensiones incorrectas: {identificador}"
            )

        # Comprobar valores numéricos.

        valores = posicion + dimensiones

        if not all(
            isinstance(valor, (int, float))
            and not isinstance(valor, bool)
            and math.isfinite(valor)
            for valor in valores
        ):

            raise ValueError(
                f"Valores no válidos: {identificador}"
            )

        if any(valor <= 0 for valor in dimensiones):

            raise ValueError(
                f"Dimensiones no positivas: {identificador}"
            )

        # Asignar el frame correspondiente.

        obstaculo["frame_id"] = frame

    return obstaculos


# ============================================================
# GESTIÓN DE LAS ESTACIONES
# ============================================================

def ejecutar_operacion(gestor, operacion, estaciones):

    """
    Ejecuta la operación solicitada utilizando
    nuestro PlanningSceneManager.
    """

    # --------------------------------------------------------
    # CONSULTAR
    # --------------------------------------------------------

    if operacion == "list":

        return gestor.listar_objetos()

    # --------------------------------------------------------
    # AÑADIR ESTACIONES
    # --------------------------------------------------------

    if operacion == "add":

        correcto = True

        for estacion in estaciones:

            resultado = gestor.agregar_caja(
                identificador=estacion["id"],
                frame=estacion["frame_id"],
                posicion=estacion["position"],
                dimensiones=estacion["dimensions"],
            )

            if not resultado:

                correcto = False

                gestor.get_logger().error(
                    f"No se ha podido añadir: {estacion['id']}"
                )

        return correcto

    # --------------------------------------------------------
    # ELIMINAR ESTACIONES
    # --------------------------------------------------------

    if operacion == "remove":

        correcto = True

        for estacion in estaciones:

            resultado = gestor.eliminar_objeto(
                estacion["id"]
            )

            if not resultado:

                correcto = False

                gestor.get_logger().error(
                    f"No se ha podido eliminar: {estacion['id']}"
                )

        return correcto

    return False


# ============================================================
# FUNCIÓN PRINCIPAL
# ============================================================

def main(args=None):

    # --------------------------------------------------------
    # ARGUMENTOS
    # --------------------------------------------------------

    parser = argparse.ArgumentParser(
        description="Gestión de estaciones de manipulación."
    )

    parser.add_argument(
        "--op",
        choices=OPERACIONES,
        required=True,
        help="Operación: add, remove o list.",
    )

    argumentos, _ = parser.parse_known_args(args)

    # --------------------------------------------------------
    # CARGAR CONFIGURACIÓN
    # --------------------------------------------------------

    try:

        estaciones = cargar_estaciones()

    except Exception as error:

        print(
            f"Error cargando estaciones: {error}",
            file=sys.stderr,
        )

        return 1

    # --------------------------------------------------------
    # INICIALIZAR ROS 2
    # --------------------------------------------------------

    rclpy.init(args=args)

    gestor = None
    correcto = False

    try:

        gestor = PlanningSceneManager()

        gestor.get_logger().info(
            f"Estaciones configuradas: {len(estaciones)}"
        )

        correcto = ejecutar_operacion(
            gestor,
            argumentos.op,
            estaciones,
        )

    except Exception as error:

        if gestor is not None:

            gestor.get_logger().error(
                f"Error gestionando estaciones: {error}"
            )

        else:

            print(
                f"Error: {error}",
                file=sys.stderr,
            )

    finally:

        if gestor is not None:
            gestor.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0 if correcto else 1


if __name__ == "__main__":
    sys.exit(main())