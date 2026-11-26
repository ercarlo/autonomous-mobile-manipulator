#!/usr/bin/env python3
"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - DESCENSO CARTESIANO
============================================================
Paquete: amm_manipulation
Archivo: cartesian_descent.py
Fase 7 / Etapa 7.5.4 - descenso inicial de 40 mm

Objetivo:
    1. Congelar las detecciones de la Planning Scene.
    2. Guardar la última referencia de la manzana en base_link.
    3. Retirar SOLO amm_detected_apple de MoveIt antes de planificar.
    4. Conservar el tablero, los demás obstáculos y las autocolisiones.
    5. Calcular una bajada cartesiana corta, con orientación fija.
    6. Mostrar el plan en RViz; ejecutarlo solo con autorización.

IMPORTANTE:
    - No elimina la manzana física de Gazebo.
    - La posición se guarda en ~/.ros/amm_manipulation/apple_target_locked.yaml.
    - Si la caja ya fue eliminada durante el diagnóstico, utilizar
      --reference-xyz X Y Z con los valores guardados previamente.
    - No realiza todavía el descenso de contacto, el cierre ni la elevación.
    - Solo se acepta un recorrido cartesiano del 100 %, sin colisiones
      frente a los obstáculos RESTANTES.
    - La geometría de la manzana no se comprueba como obstáculo una vez
      retirada; verificar visualmente la separación respecto a la real.
============================================================
"""

import argparse
import math
import sys
import time
from pathlib import Path

import yaml

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    CollisionObject, DisplayTrajectory, MoveItErrorCodes,
    PlanningScene, PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene
from sensor_msgs.msg import JointState

# Reutilizar el código que YA se validó para esta misma estación,
# evitando duplicar la lógica de TF2, robot, escena y controladores.
# Ambos ficheros deben estar en el mismo directorio.
from vertical_grasp import (
    VerticalGrasp,
    FRAME_BASE,
    FRAME_MUNECA,
    POSICION_ABIERTA,
    distancia,
    normalizar_cuaternion,
)


class CartesianDescent(VerticalGrasp):
    """Calcula y ejecuta un desplazamiento rectilíneo de la muñeca."""

    def __init__(self):
        super().__init__()
        self.cliente_cartesiano = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self.cliente_aplicar_escena = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

    def consultar_escena(self):
        """Consulta objetos de MoveIt; exige que la mesa siga protegida."""
        if not self.cliente_escena.wait_for_service(timeout_sec=8.0):
            raise RuntimeError("No está disponible /get_planning_scene.")
        solicitud = GetPlanningScene.Request()
        solicitud.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        futuro = self.cliente_escena.call_async(solicitud)
        rclpy.spin_until_future_complete(self, futuro, timeout_sec=8.0)
        if not futuro.done() or futuro.result() is None:
            raise RuntimeError("No hay respuesta de /get_planning_scene.")
        return {obj.id: obj for obj in futuro.result().scene.world.collision_objects}

    def registrar_referencia(self, punto, origen):
        """Conserva la referencia para el posterior descenso y agarre.

        Se guarda el centro de la caja procedente de RGB-D, NO el centro
        geométrico real de la manzana ni una posición consultada a Gazebo.
        """
        ruta = Path.home() / ".ros" / "amm_manipulation" / "apple_target_locked.yaml"
        ruta.parent.mkdir(parents=True, exist_ok=True)
        valores = {
            "object_id": "amm_detected_apple",
            "reference_frame": FRAME_BASE,
            "source": origen,
            "point_rgbd_approx": {"x": float(punto[0]), "y": float(punto[1]), "z": float(punto[2])},
            "notes": (
                "Referencia aproximada en base_link. Z corresponde al centro del "
                "obstáculo de percepción (superficie RGB-D), no al centro de la manzana."
            ),
        }
        ruta_temporal = ruta.with_suffix(".tmp")
        with ruta_temporal.open("w", encoding="utf-8") as archivo:
            yaml.safe_dump(valores, archivo, sort_keys=False, allow_unicode=True)
        ruta_temporal.replace(ruta)
        self.get_logger().info(f"Referencia de manzana guardada: {ruta}")

    def eliminar_solo_manzana(self):
        """Retira el obstáculo objetivo sin modificar ninguno de los demás."""
        if not self.cliente_aplicar_escena.wait_for_service(timeout_sec=8.0):
            raise RuntimeError("No está disponible /apply_planning_scene.")
        quitar = CollisionObject()
        quitar.id = "amm_detected_apple"
        quitar.header.frame_id = FRAME_BASE
        quitar.operation = CollisionObject.REMOVE
        escena = PlanningScene()
        escena.is_diff = True
        escena.robot_state.is_diff = True
        escena.world.collision_objects = [quitar]
        solicitud = ApplyPlanningScene.Request()
        solicitud.scene = escena
        futuro = self.cliente_aplicar_escena.call_async(solicitud)
        rclpy.spin_until_future_complete(self, futuro, timeout_sec=10.0)
        if not futuro.done() or futuro.result() is None or not futuro.result().success:
            raise RuntimeError("MoveIt no confirmó la eliminación de amm_detected_apple.")
        if "amm_detected_apple" in self.consultar_escena():
            raise RuntimeError("amm_detected_apple sigue en la Planning Scene.")
        self.get_logger().info(
            "Se retiró SOLO amm_detected_apple; los demás obstáculos permanecen."
        )

    def fijar_objeto_para_agarre(self, referencia_explicita):
        """Obtiene y guarda la referencia antes de retirar el obstáculo.

        Si el cubo ya no existe, exige coordenadas explícitas en base_link;
        nunca lee de forma silenciosa un objetivo almacenado de otro ensayo.
        """
        objetos = self.consultar_escena()
        if "amm_workbench_top" not in objetos:
            raise RuntimeError(
                "No está amm_workbench_top en la Planning Scene. "
                "No se autoriza bajar sin la protección de la mesa."
            )
        if "amm_detected_apple" in objetos:
            referencia = self.obtener_centro_cubo_congelado()
            origen = "planning_scene_congelada"
            if referencia_explicita is not None:
                diferencia = distancia(referencia, referencia_explicita)
                if diferencia > 0.02:
                    raise RuntimeError(
                        f"La referencia manual discrepa {diferencia * 1000:.1f} mm "
                        "del objeto en MoveIt. Revisa las posiciones."
                    )
        elif referencia_explicita is not None:
            referencia = tuple(referencia_explicita)
            origen = "referencia_manual_de_diagnostico"
            self.get_logger().warning(
                "La manzana ya está retirada de MoveIt. Se usará el punto "
                "RGB-D previamente verificado mediante --reference-xyz."
            )
        else:
            raise RuntimeError(
                "amm_detected_apple ya no está en MoveIt. Para continuar "
                "desde esta escena, usa --reference-xyz X Y Z con la "
                "referencia RGB-D que acabamos de comprobar."
            )

        if not all(math.isfinite(v) for v in referencia):
            raise RuntimeError("La referencia de manzana no es finita.")
        return referencia, origen, "amm_detected_apple" in objetos

    def esperar_pinza_abierta(self, timeout=8.0):
        """Espera hasta recibir una lectura real de la articulación Robotiq.

        Usamos un plazo medido con time.monotonic(), NO un número fijo de
        spin_once(): cuando hay muchos mensajes /tf, cada spin_once() puede
        procesar un mensaje y volver de inmediato. Contar 60 callbacks NO
        equivale a esperar 6 segundos.

        Conservamos la validación de apertura: si no llega /joint_states
        o la pinza está cerrada, no se permite mover el brazo.
        """
        limite = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < limite:
            if self.apertura_actual is not None:
                break
            # Procesar las suscripciones /joint_states, /tf y percepción.
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.apertura_actual is None:
            raise RuntimeError(
                f"No se recibió el joint '{JOINT_PINZA}' en /joint_states "
                f"durante {timeout:.1f} s. Verifica que Gazebo no esté en pausa, "
                "que se publica /joint_states y que contiene ese nombre. "
                "No se autoriza el descenso."
            )

        self.get_logger().info(
            f"Estado de Robotiq recibido en /joint_states: "
            f"{self.apertura_actual:.4f} rad."
        )
        if abs(self.apertura_actual - POSICION_ABIERTA) > 0.06:
            raise RuntimeError(
                f"Robotiq NO abierta (posición={self.apertura_actual:.3f} rad). "
                "No se autoriza el descenso."
            )

    def preparar_objetivo(self,metros,referencia_explicita=None,auto_height=False,):
        """Comprueba pre-grasp, fija referencia y retira la manzana.

        La retirada se hace SOLO tras validar centrado, altura y apertura;
        queda activa para los siguientes pasos del agarre.
        """
        self.congelar_scene()
        self.esperar_pinza_abierta()
        self.comprobar_verticalidad(max_inclinacion_grados=2.5)
        centro_cubo, origen, caja_presente = self.fijar_objeto_para_agarre(
            referencia_explicita
        )
        self.referencia_agarre = tuple(float(v) for v in centro_cubo)
        centro_dedos, muneca, orientacion = self.centro_dedos_actual()

        # ========================================================
        # CÁLCULO AUTOMÁTICO DE LA ALTURA DE AGARRE
        # ========================================================

        if auto_height:

            # Geometría utilizada en nuestra primera
            # prueba de agarre con la manzana YCB.
            altura_mesa = 0.600
            diametro_manzana = 0.075

            # Extensión aproximada de las puntas desde
            # sus respectivos enlaces de referencia.
            longitud_puntas = 0.051

            # Dejamos un margen inicial de 10 mm respecto
            # al centro estimado de la manzana.
            margen_seguridad = 0.010

            centro_manzana_z = (
                altura_mesa + diametro_manzana / 2.0
            )

            objetivo_dedos_z = (
                centro_manzana_z
                + longitud_puntas
                + margen_seguridad
            )
            self.objetivo_dedos_z = float(objetivo_dedos_z)

            # Distancia vertical pendiente.
            distancia_restante = (
                centro_dedos[2] - objetivo_dedos_z
            )

            if not math.isfinite(distancia_restante):
                raise RuntimeError(
                    "No se ha podido calcular una distancia "
                    "de descenso válida."
                )

            if distancia_restante < 0.005:
                raise RuntimeError(
                    "Los dedos ya están suficientemente cerca "
                    "de la altura objetivo. "
                    "No se autoriza otro descenso automático."
                )

            # El programa actual admite un máximo de
            # 40 mm por movimiento cartesiano.
            metros = min(0.040, distancia_restante)

            print("\n=== DESCENSO AUTOMÁTICO ===")
            print(
                f"  Altura actual dedos: "
                f"{centro_dedos[2]:.4f} m"
            )
            print(
                f"  Centro estimado manzana: "
                f"{centro_manzana_z:.4f} m"
            )
            print(
                f"  Altura objetivo dedos: "
                f"{objetivo_dedos_z:.4f} m"
            )
            print(
                f"  Distancia total pendiente: "
                f"{distancia_restante * 1000:.1f} mm"
            )
            print(
                f"  Descenso del tramo actual: "
                f"{metros * 1000:.1f} mm"
            )

        # Guardamos la distancia efectiva del tramo para que
        # la verificación TF2 utilice el valor realmente calculado.
        self.distancia_tramo = metros

        desviacion_xy = math.hypot(
            centro_dedos[0] - centro_cubo[0],
            centro_dedos[1] - centro_cubo[1],
        )
        distancia_z_cubo = centro_dedos[2] - centro_cubo[2]
        final_z_cubo = distancia_z_cubo - metros

        print("\n=== ETAPA 7.5.4: DESCENSO CARTESIANO SIN CONTACTO ===")
        print(f"  Error XY respecto a referencia: {desviacion_xy * 1000:.1f} mm")
        print(f"  Altura dedos sobre referencia: {distancia_z_cubo * 1000:.1f} mm")
        print(f"  Descenso solicitado: {metros * 1000:.1f} mm")
        print(f"  Altura final sobre referencia: {final_z_cubo * 1000:.1f} mm")
        print(f"  Muñeca inicial: [{muneca[0]:+.4f}, {muneca[1]:+.4f}, {muneca[2]:+.4f}] m")
        print("  X/Y y orientación constantes. Robotiq ABIERTA.")
        print("  Colisiones: activadas con mesa, otros objetos y robot.")
        print("  La manzana se retira SOLO de MoveIt, nunca de Gazebo.\n")

        if desviacion_xy > 0.008:
            raise RuntimeError("Descentrado >8 mm. No se autoriza el descenso.")

        if metros > 0 and not (0.03 <= distancia_z_cubo <= 0.17):
            raise RuntimeError(
                "El robot está fuera de la zona de aproximación permitida."
            )

        if metros > 0 and final_z_cubo < 0.025:
            raise RuntimeError(
                "Se ha alcanzado el límite de descenso permitido."
            )

                # Para ascender, comprobamos que seguimos cerca del objeto
        # y limitamos la altura final respecto a la referencia RGB-D.
        if metros < 0:
            if not (0.015 <= distancia_z_cubo <= 0.17):
                raise RuntimeError(
                    "Ascenso rechazado: altura inicial fuera del rango permitido."
                )

            if final_z_cubo > 0.22:
                raise RuntimeError(
                    "Ascenso rechazado: altura final excesiva."
                )

        # Guardar ANTES de eliminar: lo reutilizaremos en la fase de agarre.
        self.registrar_referencia(centro_cubo, origen)
        if caja_presente:
            self.eliminar_solo_manzana()
        else:
            self.get_logger().info(
                "amm_detected_apple ya estaba retirado: no se modifica la escena."
            )

        # Reconfirmar que seguimos con la protección de mesa y sin manzana.
        presentes = self.consultar_escena()
        if "amm_workbench_top" not in presentes or "amm_detected_apple" in presentes:
            raise RuntimeError("Escena final incorrecta. No se autoriza planificar.")

        objetivo = Pose()
        objetivo.position.x = float(muneca[0])
        objetivo.position.y = float(muneca[1])
        objetivo.position.z = float(muneca[2] - metros)
        q = normalizar_cuaternion(orientacion)
        (objetivo.orientation.x, objetivo.orientation.y,
         objetivo.orientation.z, objetivo.orientation.w) = q
        return objetivo, centro_dedos, q

    def planificar_cartesiano(self, objetivo, paso):
        """Pide a MoveIt una trayectoria cartesiana con colisiones activadas."""
        if not self.cliente_cartesiano.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                "No existe /compute_cartesian_path. Comprueba move_group."
            )

        solicitud = GetCartesianPath.Request()
        solicitud.header.frame_id = FRAME_BASE
        solicitud.header.stamp = self.get_clock().now().to_msg()
        solicitud.group_name = "ur5e_arm"
        solicitud.link_name = FRAME_MUNECA
        solicitud.waypoints = [objetivo]
        solicitud.max_step = paso
        solicitud.jump_threshold = 0.0  # Comprobación explícita de saltos abajo.
        solicitud.avoid_collisions = True
        solicitud.max_velocity_scaling_factor = 0.05
        solicitud.max_acceleration_scaling_factor = 0.05

        # Partir del estado articular real conocido por el nodo y
        # mezclarlo con el estado actual del monitor de MoveIt.
        solicitud.start_state.is_diff = True
        estado = JointState()
        estado.name = list(self.posiciones_articulares.keys())
        estado.position = [
            self.posiciones_articulares[nombre] for nombre in estado.name
        ]
        solicitud.start_state.joint_state = estado

        futuro = self.cliente_cartesiano.call_async(solicitud)
        rclpy.spin_until_future_complete(self, futuro, timeout_sec=20.0)
        if not futuro.done() or futuro.result() is None:
            raise RuntimeError("No llegó respuesta del planificador cartesiano.")

        respuesta = futuro.result()
        serie = respuesta.solution.joint_trajectory
        cantidad = len(serie.points)
        print(f"  Fracción cartesiana conseguida: {respuesta.fraction * 100:.1f}%")
        print(f"  Puntos articulares calculados: {cantidad}")
        print(f"  Código MoveIt: {respuesta.error_code.val}")

        # Nunca ejecutar una trayectoria parcial, ni con errores MoveIt.
        if respuesta.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError("MoveIt ha rechazado el cálculo cartesiano.")
        if respuesta.fraction < 0.999 or cantidad < 2:
            raise RuntimeError(
                "Trayectoria cartesiana incompleta. No se autoriza la ejecución. "
                "Revisa el final de trayectoria, IK y las colisiones."
            )
        if not serie.joint_names:
            raise RuntimeError("Trayectoria sin nombres articulares.")

        # Evitar soluciones de IK que salten entre ramas aunque
        # visualmente la muñeca siga un segmento corto.
        salto_maximo = 0.0
        for anterior, siguiente in zip(serie.points, serie.points[1:]):
            for a, b in zip(anterior.positions, siguiente.positions):
                salto_maximo = max(salto_maximo, abs(b - a))
        print(f"  Máximo cambio entre puntos articulares: {salto_maximo:.4f} rad")
        if salto_maximo > 0.20:
            raise RuntimeError(
                "Salto articular >0.20 rad en una interpolación de 5 mm. "
                "La trayectoria no es adecuada para el descenso."
            )

        # Comprobación de tiempos asignados por MoveIt.
        tiempos = [
            p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec
            for p in serie.points
        ]
        if tiempos[-1] <= 0.0 or any(
            t1 >= t2 for t1, t2 in zip(tiempos, tiempos[1:])
        ):
            raise RuntimeError("Trayectoria sin parametrización temporal válida.")

        visualizacion = DisplayTrajectory()
        visualizacion.trajectory_start = respuesta.start_state
        visualizacion.trajectory = [respuesta.solution]
        self.publicador_trayectoria.publish(visualizacion)
        self.get_logger().info(
            "DESCENSO CARTESIANO PLANIFICADO (100%). "
            "Revisa la trayectoria publicada en RViz."
        )
        return respuesta.solution

    def comprobar_descenso(self, inicio_dedos, orientacion_inicial, metros):
        """Confirma por TF2 una bajada sin desplazamientos laterales."""
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.1)
        centro_final, _, q_final = self.centro_dedos_actual()
        dx = centro_final[0] - inicio_dedos[0]
        dy = centro_final[1] - inicio_dedos[1]
        bajada_real = inicio_dedos[2] - centro_final[2]
        error_z = abs(bajada_real - metros)
        dot = abs(sum(a * b for a, b in zip(orientacion_inicial, q_final)))
        dot = max(0.0, min(1.0, dot))
        angulo = 2.0 * math.degrees(math.acos(dot))
        print("\n=== VERIFICACIÓN DEL DESCENSO ===")
        print(f"  Bajada real: {bajada_real * 1000:.1f} mm")
        print(f"  Error respecto a distancia ordenada: {error_z * 1000:.1f} mm")
        print(f"  Deriva horizontal: {math.hypot(dx, dy) * 1000:.1f} mm")
        print(f"  Variación angular de muñeca: {angulo:.2f} grados")
        if error_z > 0.008 or math.hypot(dx, dy) > 0.008 or angulo > 2.5:
            raise RuntimeError(
                "Movimiento terminado, pero la verificación TF2 excede "
                "las tolerancias. No continuar descendiendo."
            )
        self.get_logger().info(
            "PRIMER DESCENSO CORRECTO. Pinza abierta. "
            "Sin contacto intencionado con la manzana."
        )


# ============================================================
# DESCENSO CARTESIANO AUTOMÁTICO COMPLETO
# ============================================================

def ejecutar_descenso_automatico(nodo, argumentos):
    """
    Ejecuta el descenso en un máximo de tres tramos.

    Funcionamiento:
        1. Calcular y ejecutar el primer tramo.
        2. Solicitar confirmación antes del segundo.
        3. Ejecutar el segundo tramo.
        4. Ejecutar automáticamente el tramo restante.
        5. Verificar que se ha alcanzado la altura objetivo.

    Cada tramo utiliza:
        - TF2 para conocer la posición actual.
        - MoveIt 2 para planificar la trayectoria.
        - Verificación del inicio de trayectoria.
        - Verificación del movimiento realmente ejecutado.

    Se mantiene la misma referencia RGB-D de la manzana
    durante toda la secuencia.
    """

    referencia = argumentos.reference_xyz
    max_tramos = 3
    tolerancia_final = 0.005

    nodo.get_logger().info(
        "=== INICIO DEL DESCENSO AUTOMÁTICO ==="
    )

    for numero in range(1, max_tramos + 1):

        nodo.get_logger().info(
            f"Preparando tramo {numero}/{max_tramos}."
        )

        # ----------------------------------------------------
        # 1. Calcular el siguiente objetivo mediante TF2.
        #
        # En el primer tramo se utiliza la referencia presente
        # en MoveIt. Después conservamos sus coordenadas
        # originales, aunque el cubo ya se haya retirado.
        # ----------------------------------------------------

        objetivo, centro_inicial, q_inicial = (
            nodo.preparar_objetivo(
                metros=0.04,
                referencia_explicita=referencia,
                auto_height=True,
            )
        )

        referencia = nodo.referencia_agarre
        distancia_tramo = float(nodo.distancia_tramo)

        # La última aproximación se ha validado para
        # distancias pequeñas. Si aparece una distancia
        # inesperadamente grande, detenemos la secuencia.
        if numero == 3 and distancia_tramo > 0.030:
            raise RuntimeError(
                "El último tramo supera 30 mm. "
                "Revisar la posición del robot antes "
                "de continuar."
            )

        nodo.get_logger().info(
            f"Tramo {numero}: descenso calculado "
            f"{distancia_tramo * 1000:.1f} mm."
        )

        # ----------------------------------------------------
        # 2. Planificar la trayectoria cartesiana.
        # ----------------------------------------------------

        trayectoria = nodo.planificar_cartesiano(
            objetivo,
            argumentos.step,
        )

        # ----------------------------------------------------
        # 3. Confirmaciones de seguridad.
        # ----------------------------------------------------

        if numero == 1:

            print(
                "\nPRIMER DESCENSO"
                "\nRevisa en RViz la trayectoria calculada."
            )

            decision = input(
                "Escribe EJECUTAR para iniciar "
                "el primer tramo: "
            ).strip()

            if decision != "EJECUTAR":
                raise RuntimeError(
                    "Primer descenso cancelado."
                )

        elif numero == 2:

            # Pausa solicitada expresamente:
            # el primer tramo ya ha terminado y
            # se ha verificado mediante TF2.

            print("\n" + "=" * 55)
            print("PRIMER DESCENSO COMPLETADO")
            print("=" * 55)

            print(
                "\nEl robot está inmóvil."
                "\nSe ha calculado la trayectoria "
                "del segundo tramo."
                "\nRevisa la trayectoria en RViz."
            )

            decision = input(
                "\nEscribe EJECUTAR para autorizar "
                "el SEGUNDO descenso: "
            ).strip()

            if decision != "EJECUTAR":
                raise RuntimeError(
                    "Segundo descenso cancelado. "
                    "El robot permanece en la posición "
                    "alcanzada después del primer tramo."
                )

        # ----------------------------------------------------
        # 4. Comprobar que la trayectoria sigue siendo válida.
        # ----------------------------------------------------

        objetos = nodo.consultar_escena()

        if (
            "amm_detected_apple" in objetos
            or "amm_workbench_top" not in objetos
        ):
            raise RuntimeError(
                "La Planning Scene ha cambiado. "
                "Se cancela la ejecución."
            )

        nodo.verificar_inicio_trayectoria(
            trayectoria.joint_trajectory
        )

        # ----------------------------------------------------
        # 5. Ejecutar el movimiento previamente planificado.
        # ----------------------------------------------------

        nodo.get_logger().info(
            f"Ejecutando tramo {numero}..."
        )

        nodo.ejecutar_trayectoria(trayectoria)

        # ----------------------------------------------------
        # 6. Verificar el movimiento mediante TF2.
        # ----------------------------------------------------

        nodo.comprobar_descenso(
            centro_inicial,
            q_inicial,
            distancia_tramo,
        )

        # ----------------------------------------------------
        # 7. Comprobar la distancia restante.
        # ----------------------------------------------------

        centro_dedos, _, _ = nodo.centro_dedos_actual()

        distancia_restante = (
            float(centro_dedos[2])
            - nodo.objetivo_dedos_z
        )

        if not math.isfinite(distancia_restante):
            raise RuntimeError(
                "Distancia restante no válida."
            )

        nodo.get_logger().info(
            f"Tramo {numero} completado. "
            f"Restan {distancia_restante * 1000:.1f} mm."
        )

        # No admitimos un sobrepaso significativo
        # de la altura objetivo.
        if distancia_restante < -0.005:
            raise RuntimeError(
                "Se ha descendido por debajo "
                "de la altura objetivo."
            )

        if distancia_restante <= tolerancia_final:

            nodo.get_logger().info(
                "=== ALTURA DE AGARRE ALCANZADA ==="
            )

            nodo.get_logger().info(
                "Descenso automático completado. "
                "La Robotiq permanece abierta."
            )

            return 0

    raise RuntimeError(
        "Se ha alcanzado el número máximo de tramos "
        "sin llegar a la altura de agarre."
    )



# ============================================================
# ETAPA 7.6.6 - ELEVACIÓN POSTERIOR AL AGARRE
# ============================================================

def ejecutar_elevacion_post_pick(nodo, paso=0.002):
    """
    Eleva el UR5e un total de 60 mm.

    Secuencia:
        1. Ascender 40 mm.
        2. Verificar el movimiento mediante TF2.
        3. Ascender otros 20 mm.
        4. Verificar nuevamente el movimiento.

    No necesita conocer la posición de la manzana:
    se utiliza únicamente la pose actual de la muñeca.

    Precondición:
        El objeto debe estar correctamente unido a
        la Robotiq mediante DetachableJoint en Gazebo.
    """

    tramos = [0.040, 0.020]

    nodo.get_logger().info(
        "=== INICIO DE ELEVACIÓN POST-PICK ==="
    )

    # Impedir que las detecciones modifiquen la
    # Planning Scene durante la elevación.
    nodo.congelar_scene()

    for indice, distancia in enumerate(tramos, start=1):

        nodo.get_logger().info(
            f"Elevación {indice}/{len(tramos)}: "
            f"{distancia * 1000:.1f} mm"
        )

        # ----------------------------------------------------
        # 1. Obtener la posición actual mediante TF2.
        # ----------------------------------------------------

        centro_inicial, muneca, q_inicial = (
            nodo.centro_dedos_actual()
        )

        # ----------------------------------------------------
        # 2. Calcular el objetivo cartesiano.
        #
        # X/Y y orientación permanecen constantes.
        # Únicamente aumentamos la coordenada Z.
        # ----------------------------------------------------

        objetivo = Pose()

        objetivo.position.x = float(muneca[0])
        objetivo.position.y = float(muneca[1])
        objetivo.position.z = float(
            muneca[2] + distancia
        )

        q = normalizar_cuaternion(q_inicial)

        (
            objetivo.orientation.x,
            objetivo.orientation.y,
            objetivo.orientation.z,
            objetivo.orientation.w,
        ) = q

        # ----------------------------------------------------
        # 3. Comprobar que conservamos los obstáculos.
        # ----------------------------------------------------

        objetos = nodo.consultar_escena()

        if "amm_workbench_top" not in objetos:
            raise RuntimeError(
                "No se encuentra la mesa en MoveIt. "
                "Se cancela la elevación."
            )

        if "amm_detected_apple" in objetos:
            raise RuntimeError(
                "La manzana sigue presente como obstáculo "
                "en MoveIt. Revisar Planning Scene."
            )

        # ----------------------------------------------------
        # 4. Planificar el movimiento.
        # ----------------------------------------------------

        trayectoria = nodo.planificar_cartesiano(
            objetivo,
            paso,
        )

        # ----------------------------------------------------
        # 5. Verificar y ejecutar la trayectoria.
        # ----------------------------------------------------

        nodo.verificar_inicio_trayectoria(
            trayectoria.joint_trajectory
        )

        nodo.ejecutar_trayectoria(trayectoria)

        # ----------------------------------------------------
        # 6. Comprobar el movimiento real mediante TF2.
        #
        # La función original admite distancias
        # negativas para verificar movimientos ascendentes.
        # ----------------------------------------------------

        nodo.comprobar_descenso(
            centro_inicial,
            q_inicial,
            -distancia,
        )

        nodo.get_logger().info(
            f"Elevación {indice} completada."
        )

    nodo.get_logger().info(
        "=== ELEVACIÓN TOTAL DE 60 MM COMPLETADA ==="
    )

    return 0



def main(args=None):
    parser = argparse.ArgumentParser(
        description="Primera prueba de descenso vertical, sin contacto."
    )
    parser.add_argument(
        "--distance", type=float, default=0.04,
        help="Distancia vertical a descender (m). Para esta etapa: <= 0.04 m."
    )
    parser.add_argument(
        "--step", type=float, default=0.005,
        help="Tamaño de paso cartesiano (m)."
    )
    parser.add_argument(
        "--execute", choices=["true", "false"], default="false",
        help="true: pide autorización para ejecutar el plan revisado."
    )
    parser.add_argument(
        "--reference-xyz", type=float, nargs=3, metavar=("X", "Y", "Z"),
        default=None,
        help=(
            "Referencia RGB-D previa en base_link. Úsala si "
            "amm_detected_apple ya fue retirado. Se verifica la posición "
            "del brazo antes de autorizar cualquier ejecución."
        ),
    )

    parser.add_argument(
        "--auto-height",
        choices=["true", "false"],
        default="false",
        help=(
            "Calcula automáticamente el descenso necesario "
            "a partir de la altura actual de los dedos. "
            "Cada ejecución realiza como máximo 40 mm."
        ),
    )

    parser.add_argument(
        "--auto-sequence",
        choices=["true", "false"],
        default="false",
        help=(
            "Ejecuta todos los tramos del descenso automático, "
            "solicitando confirmación entre el primero y el segundo."
        ),
    )


    parser.add_argument(
        "--lift-post-pick",
        choices=["true", "false"],
        default="false",
        help=(
            "Eleva el UR5e 60 mm después del attach, "
            "sin necesitar la referencia RGB-D."
        ),
    )
    argumentos, _ = parser.parse_known_args(args)

        # La elevación post-pick requiere ejecución habilitada.
    if argumentos.lift_post_pick == "true":

        if argumentos.execute != "true":
            parser.error(
                "--lift-post-pick true requiere --execute true."
            )

        if (
            argumentos.auto_height == "true"
            or argumentos.auto_sequence == "true"
        ):
            parser.error(
                "La elevación post-pick no puede combinarse "
                "con los modos automáticos de descenso."
            )


    if argumentos.auto_sequence == "true":

        if (
            argumentos.auto_height != "true"
            or argumentos.execute != "true"
        ):
            parser.error(
                "--auto-sequence true requiere "
                "--auto-height true y --execute true."
            )

    if argumentos.reference_xyz is not None:
        if not all(math.isfinite(v) for v in argumentos.reference_xyz):
            parser.error("--reference-xyz debe contener tres números finitos.")

    if not (0.005 <= abs(argumentos.distance) <= 0.04):
        parser.error(
            "--distance debe tener una magnitud entre 0.005 y 0.04 m."
        )
    if not (0.001 <= argumentos.step <= 0.01):
        parser.error("--step debe estar entre 0.001 y 0.01 m.")

    rclpy.init(args=args)
    nodo = None
    try:
        nodo = CartesianDescent()

                # Modo de elevación independiente del descenso.
        # No necesita referencia de la manzana.
        if argumentos.lift_post_pick == "true":

            return ejecutar_elevacion_post_pick(
                nodo,
                paso=argumentos.step,
            )

        if argumentos.auto_sequence == "true":

            return ejecutar_descenso_automatico(
                nodo,
                argumentos,
            )

        objetivo, centro_inicial, q_inicial = nodo.preparar_objetivo(
            argumentos.distance,
            referencia_explicita=argumentos.reference_xyz,
            auto_height=(argumentos.auto_height == "true"),
        )

        # Distancia efectiva calculada por preparar_objetivo().
        # En modo manual coincide con --distance.
        distancia_real = nodo.distancia_tramo
        trayectoria = nodo.planificar_cartesiano(objetivo, argumentos.step)
        if argumentos.execute == "false":
            nodo.get_logger().info("SOLO PLANIFICACIÓN. El robot no se ha movido.")
            return 0

        print("\nREVISAR EN RVIZ: ¿trayectoria vertical y sin colisiones?")
        decision = input(
            "Escribe EJECUTAR para autorizar este DESCENSO CORTO "
            "(Enter cancela): "
        ).strip()
        if decision != "EJECUTAR":
            nodo.get_logger().warning(
                "Ejecución cancelada. Brazo inmóvil."
            )
            return 1

        # Una detección nueva no debe haber reintroducido la manzana:
        # la escena continúa congelada y solo la referencia se guarda.
        objetos = nodo.consultar_escena()
        if "amm_detected_apple" in objetos or "amm_workbench_top" not in objetos:
            raise RuntimeError(
                "La Planning Scene ha cambiado al revisar el plan. "
                "No se autoriza ejecutar."
            )
        nodo.verificar_inicio_trayectoria(trayectoria.joint_trajectory)
        nodo.ejecutar_trayectoria(trayectoria)
        nodo.comprobar_descenso(
            centro_inicial,
            q_inicial,
            distancia_real,
        )
        return 0
    except (RuntimeError, ValueError, OSError, EOFError) as error:
        if nodo is not None:
            nodo.get_logger().error(str(error))
        else:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    finally:
        if nodo is not None:
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())