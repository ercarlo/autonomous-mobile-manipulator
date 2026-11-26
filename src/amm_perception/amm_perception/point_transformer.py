#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - POINT TRANSFORMER
============================================================

Paquete: amm_perception
Archivo: point_transformer.py

Objetivo:
    Transformar puntos 3D desde el frame óptico de la
    cámara hasta el sistema de referencia base_link.

Funcionamiento:
    1. Recibir un PointStamped.
    2. Buscar la transformación TF2 correspondiente
       al instante de la medición.
    3. Transformar el punto.
    4. Publicar sus coordenadas respecto a base_link.
============================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, TransformException

# Registra las conversiones de PointStamped para TF2.
import tf2_geometry_msgs


class PointTransformer(Node):

    def __init__(self):
        super().__init__("point_transformer")

        # Frame al que transformaremos los puntos.
        self.target_frame = "base_link"

        # =====================================================
        # 1. CONFIGURACIÓN TF2
        # =====================================================
        #
        # Buffer:
        #   Almacena las transformaciones recibidas.
        #
        # TransformListener:
        #   Recibe /tf y /tf_static para actualizar
        #   automáticamente el buffer.

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # =====================================================
        # 2. SUSCRIPTOR Y PUBLICADOR
        # =====================================================

        self.point_sub = self.create_subscription(
            PointStamped,
            "/perception/selected_point_camera",
            self.point_callback,
            10
        )

        self.point_pub = self.create_publisher(
            PointStamped,
            "/perception/selected_point_base",
            10
        )

        # =====================================================
        # 3. GESTIÓN DE TRANSFORMACIONES PENDIENTES
        # =====================================================
        #
        # Puede ocurrir que llegue un punto antes de
        # disponer de la TF correspondiente a su timestamp.
        #
        # Conservamos el último punto seleccionado y
        # comprobamos periódicamente si está disponible.
        #
        # En esta versión diagnóstica procesamos un
        # único punto pendiente cada vez.

        self.punto_pendiente = None
        self.inicio_espera = None

        self.timer = self.create_timer(
            0.05,
            self.process_pending_point
        )

        self.get_logger().info(
            "Point Transformer iniciado. Esperando puntos 3D..."
        )

    def point_callback(self, msg):

        # Guardamos el punto con su timestamp original.
        if not msg.header.frame_id:
            self.get_logger().warn(
                "Punto recibido sin frame_id."
            )
            return

        self.punto_pendiente = msg
        self.inicio_espera = self.get_clock().now()

    def process_pending_point(self):

        if self.punto_pendiente is None:
            return

        punto = self.punto_pendiente

        # Tiempo exacto de la detección.
        tiempo = Time.from_msg(punto.header.stamp)

        # ==================================================
        # 4. COMPROBACIÓN DE DISPONIBILIDAD TF2
        # ==================================================
        #
        # No utilizamos la transformación más reciente.
        # Buscamos la correspondiente al timestamp
        # original de la medición.

        disponible = self.tf_buffer.can_transform(
            self.target_frame,
            punto.header.frame_id,
            tiempo
        )

        if not disponible:

            # Si la transformación no aparece en dos
            # segundos de simulación, descartamos el punto.
            espera = (
                self.get_clock().now() - self.inicio_espera
            ).nanoseconds * 1e-9

            if espera > 2.0:
                self.get_logger().warn(
                    "No se encontro la transformacion TF2 "
                    "para el punto seleccionado."
                )
                self.punto_pendiente = None

            return

        # ==================================================
        # 5. TRANSFORMACIÓN DEL PUNTO
        # ==================================================

        try:
            punto_base = self.tf_buffer.transform(
                punto,
                self.target_frame
            )

        except TransformException as error:
            self.get_logger().warn(
                f"Error TF2: {error}"
            )
            self.punto_pendiente = None
            return

        # ==================================================
        # 6. PUBLICACIÓN
        # ==================================================

        self.point_pub.publish(punto_base)

        self.get_logger().info(
            f"Punto en {self.target_frame}: "
            f"X={punto_base.point.x:.3f}, "
            f"Y={punto_base.point.y:.3f}, "
            f"Z={punto_base.point.z:.3f} m"
        )

        self.punto_pendiente = None


def main(args=None):

    rclpy.init(args=args)
    node = PointTransformer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
