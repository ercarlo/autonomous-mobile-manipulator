#!/usr/bin/env python3
"""
============================================================
AMM - PLACE SEQUENCE (APPLE -> MESA 1)
============================================================

Secuencia validada por pasos en Gazebo:
  1. transport -> place_approach_mesa1
  2. place_approach_mesa1 -> place_drop_mesa1
  3. Detach de la manzana (Gazebo DetachableJoint)
  4. Esperar a que la manzana caiga
  5. Retirada por place_approach_mesa1 -> transport

No ejecuta YOLO ni Nav2. Supone que Nav2 ya ha detenido
la base frente a la mesa y que la manzana sigue unida.

Durante la primera prueba, revisar la trayectoria y la mesa
para evitar colisiones: MoveIt no modela la manzana transportada.
============================================================
"""

import argparse
import re
import subprocess
import sys
import tempfile
import time


DETACH_TOPIC = "/amm/apple/detach"
STATE_TOPIC = "/amm/apple/attachment_state"


def ejecutar_movimiento(estado):
    """Reutiliza el controlador del UR5e y exige éxito real."""
    print(f"\n[PLACE] Moviendo UR5e a: {estado}", flush=True)

    comando = [
        "ros2", "run", "amm_manipulation", "arm_controller",
        "--target", estado,
        "--execute", "true",
        "--ros-args", "-p", "use_sim_time:=true",
    ]
    resultado = subprocess.run(comando, check=False)
    if resultado.returncode != 0:
        raise RuntimeError(
            f"Movimiento a {estado} fallido "
            f"(código {resultado.returncode})."
        )


def verificar_plugin():
    """Comprueba que el sistema de attach/detach ya está cargado."""
    resultado = subprocess.run(
        ["gz", "topic", "-l"],
        capture_output=True,
        text=True,
        timeout=15,
        check=True,
    )
    topicos = set(resultado.stdout.splitlines())
    if DETACH_TOPIC not in topicos or STATE_TOPIC not in topicos:
        raise RuntimeError(
            "No están disponibles los topics de DetachableJoint. "
            "Comprueba que la manzana sigue unida al UR5e."
        )


def soltar_manzana(timeout=8.0):
    """
    Solicita detach y espera la confirmación 'detached' en Gazebo.

    Suscribe ANTES de publicar porque el cambio de estado puede
    emitirse una sola vez. Si falta confirmación, detiene la
    secuencia sin ordenar movimientos posteriores.
    """
    verificar_plugin()
    print("\n[PLACE] Escuchando confirmación del DetachableJoint...", flush=True)

    # El archivo temporal evita bloquear el proceso de Gazebo
    # por un pipe lleno y permite comprobar el resultado al vuelo.
    with tempfile.TemporaryFile(mode="w+t", encoding="utf-8") as registro:
        observador = subprocess.Popen(
            ["gz", "topic", "-e", "-t", STATE_TOPIC],
            stdout=registro,
            stderr=subprocess.STDOUT,
        )
        try:
            # Tiempo para establecer la suscripción antes del detach.
            time.sleep(1.0)
            if observador.poll() is not None:
                raise RuntimeError(
                    "El observador del estado del joint terminó antes del detach."
                )

            resultado = subprocess.run(
                [
                    "gz", "topic", "-t", DETACH_TOPIC,
                    "-m", "gz.msgs.Empty", "-p", "",
                ],
                capture_output=True,
                text=True,
                timeout=10,
                check=False,
            )
            if resultado.returncode != 0:
                raise RuntimeError(
                    "Gazebo no pudo publicar el detach: "
                    + (resultado.stderr.strip() or resultado.stdout.strip())
                )

            print("[PLACE] Detach enviado. Esperando confirmación...", flush=True)
            inicio = time.monotonic()
            while time.monotonic() - inicio < timeout:
                registro.flush()
                registro.seek(0)
                salida = registro.read()
                if re.search(r'\bdata\s*:\s*"?detached"?\b', salida, re.IGNORECASE):
                    print("[PLACE] Gazebo confirma: detached", flush=True)
                    return
                if observador.poll() is not None:
                    raise RuntimeError(
                        "El observador de Gazebo terminó inesperadamente. "
                        + salida[-400:]
                    )
                time.sleep(0.15)

            raise RuntimeError(
                "No llegó confirmación 'detached'. Verifica Gazebo "
                "antes de reintentar: el objeto podría estar ya suelto."
            )
        finally:
            if observador.poll() is None:
                observador.terminate()
                try:
                    observador.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    observador.kill()
                    observador.wait()


def main():
    parser = argparse.ArgumentParser(
        description="Place automático validado para manzana y mesa 1."
    )
    parser.add_argument(
        "--execute", choices=["true", "false"], default="false",
        help="Debe ser true para ejecutar movimientos y detach.",
    )
    parser.add_argument(
        "--settle-seconds", type=float, default=2.5,
        help="Tiempo para que la manzana caiga antes de retirar el brazo.",
    )
    args, _ = parser.parse_known_args()

    if args.execute != "true":
        parser.error("Esta etapa requiere --execute true.")
    if not 0.5 <= args.settle_seconds <= 20.0:
        parser.error("--settle-seconds debe estar entre 0.5 y 20.0.")

    print("\n=== PLACE: APPLE -> MESA 1 ===")
    print("Precondiciones: MiR100 detenido en mesa1; manzana unida;")
    print("brazo en transport; trayectorias revisadas en RViz.")
    if input("Escribe EJECUTAR para comenzar: ").strip() != "EJECUTAR":
        print("[PLACE] Cancelado por el usuario.")
        return 1

    try:
        # Comprobar antes de mover el brazo que el plugin de agarre
        # permanece cargado y se dispone del topic de liberación.
        verificar_plugin()

        # Movimientos definidos previamente mediante MoveIt 2.
        ejecutar_movimiento("place_approach_mesa1")
        ejecutar_movimiento("place_drop_mesa1")

        # No continuamos si Gazebo no confirma la liberación.
        soltar_manzana()
        print(f"[PLACE] Esperando {args.settle_seconds:.1f} s para la caída...")
        time.sleep(args.settle_seconds)

        # Retirada conservadora por la postura intermedia validada.
        ejecutar_movimiento("place_approach_mesa1")
        ejecutar_movimiento("transport")

        print("\n=== PLACE COMPLETADO: BRAZO EN TRANSPORT ===")
        print("Verifica visualmente que la manzana permanece sobre la mesa 1.")
        return 0
    except (RuntimeError, OSError, subprocess.TimeoutExpired) as error:
        print(f"\n[PLACE] ERROR: {error}", file=sys.stderr)
        print(
            "[PLACE] Secuencia detenida. Comprueba la situación real "
            "antes de intentar nuevos movimientos.",
            file=sys.stderr,
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
