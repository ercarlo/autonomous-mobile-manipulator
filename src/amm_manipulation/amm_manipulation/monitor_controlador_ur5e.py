#!/usr/bin/env python3
"""
Monitor de diagnóstico NO INTRUSIVO para ejecución de trayectorias UR5e.

ROS 2 Jazzy / Gazebo Harmonic / MoveIt 2.

Escucha el estado de seguimiento del JointTrajectoryController y consulta
la muñeca por TF2. No publica comandos, no altera la Planning Scene y no
mueve el robot. Permite comprobar si durante un pequeño descenso cartesiano
se desvían las articulaciones respecto a la referencia.

Uso, desde el workspace:
  python3 src/amm_manipulation/amm_manipulation/monitor_controlador_ur5e.py \
      --ros-args -p use_sim_time:=true

Mantenerlo abierto antes, durante y después del movimiento; Ctrl+C para
mostrar el resumen y guardar CSV.
"""

import csv
import math
import signal
import time
from pathlib import Path

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

TOPICO = '/joint_trajectory_controller/controller_state'
FRAME_BASE = 'base_link'
FRAME_MUNECA = 'ur5e_wrist_3_link'
FICHERO = Path.cwd() / 'diagnostico_controlador_ur5e.csv'


class Monitor(Node):
    def __init__(self):
        super().__init__('amm_monitor_controlador_ur5e')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.muestras = []
        self.muestra_reciente = None
        self.t_inicio = time.monotonic()
        self.t_muestra = -1.0
        self.contador_estados = 0
        self.contador_tf = 0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.sub = self.create_subscription(
            JointTrajectoryControllerState, TOPICO, self.recibir, qos
        )
        self.get_logger().info(
            f'Monitor iniciado. Guardará datos en {FICHERO}. '
            'No envía órdenes. Detener con Ctrl+C después de la prueba.'
        )

    def recibir(self, mensaje):
        # En ROS 2 Jazzy, reference/feedback sustituyen a desired/actual.
        referencia = list(mensaje.reference.positions)
        real = list(mensaje.feedback.positions)
        nombres = list(mensaje.joint_names)
        if not nombres or len(referencia) != len(nombres) or len(real) != len(nombres):
            return
        self.contador_estados += 1
        self.muestra_reciente = (nombres, referencia, real)

    def registrar(self):
        # Muestreo moderado (~20 Hz) mientras rclpy procesa los estados.
        ahora = time.monotonic() - self.t_inicio
        if ahora - self.t_muestra < 0.05 or self.muestra_reciente is None:
            return
        self.t_muestra = ahora
        nombres, ref, real = self.muestra_reciente
        error_max = max(abs(a - b) for a, b in zip(ref, real))
        xyz = [float('nan')] * 3
        try:
            tf = self.tf_buffer.lookup_transform(FRAME_BASE, FRAME_MUNECA, Time())
            t = tf.transform.translation
            xyz = [float(t.x), float(t.y), float(t.z)]
            self.contador_tf += 1
        except TransformException:
            pass
        self.muestras.append({
            't': ahora,
            'nombres': nombres,
            'referencia': ref[:],
            'real': real[:],
            'error_max': error_max,
            'x': xyz[0], 'y': xyz[1], 'z': xyz[2],
        })

    def guardar_y_resumir(self):
        if not self.muestras:
            print('\nSIN MUESTRAS. Comprueba el tópico:', TOPICO)
            return
        nombres = self.muestras[0]['nombres']
        with FICHERO.open('w', newline='', encoding='utf-8') as archivo:
            campos = ['tiempo_s', 'tf_x_m', 'tf_y_m', 'tf_z_m', 'error_max_rad']
            campos += [f'ref_{n}' for n in nombres]
            campos += [f'real_{n}' for n in nombres]
            escritor = csv.writer(archivo)
            escritor.writerow(campos)
            for dato in self.muestras:
                escritor.writerow([
                    f"{dato['t']:.4f}", dato['x'], dato['y'], dato['z'],
                    dato['error_max'], *dato['referencia'], *dato['real'],
                ])
        errores = [m['error_max'] for m in self.muestras]
        buenas = [m for m in self.muestras if math.isfinite(m['z'])]
        print('\n========== SEGUIMIENTO DE EJECUCIÓN UR5e ==========')
        print(f'Estados recibidos: {self.contador_estados}')
        print(f'Muestras registradas: {len(self.muestras)}')
        print(f'Muestras con TF2 válido: {len(buenas)}')
        print(f'Error articular máximo ref-feedback: {max(errores):.6f} rad')
        if len(buenas) >= 2:
            a, b = buenas[0], buenas[-1]
            print(f'Posición muñeca inicial (TF2): [{a["x"]:+.5f}, {a["y"]:+.5f}, {a["z"]:+.5f}] m')
            print(f'Posición muñeca final   (TF2): [{b["x"]:+.5f}, {b["y"]:+.5f}, {b["z"]:+.5f}] m')
            print(f'Cambio de Z, final-inicial: {(b["z"]-a["z"])*1000:+.2f} mm')
            max_xy = max(math.hypot(m['x']-a['x'],m['y']-a['y']) for m in buenas)
            print(f'Máxima excursión XY registrada: {max_xy*1000:.2f} mm')
            z_min = min(m['z'] for m in buenas)
            z_max = max(m['z'] for m in buenas)
            print(f'Z mínima/máxima registrada: {z_min:.5f} / {z_max:.5f} m')
            print(f'Descenso máximo desde posición inicial: {(a["z"]-z_min)*1000:.2f} mm')
        else:
            print('No hay suficientes transformaciones TF2 para calcular desplazamientos.')
        print(f'CSV: {FICHERO}')
        print('===================================================')


def main():
    rclpy.init()
    nodo = Monitor()
    try:
        while rclpy.ok():
            rclpy.spin_once(nodo, timeout_sec=0.05)
            nodo.registrar()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.guardar_y_resumir()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
