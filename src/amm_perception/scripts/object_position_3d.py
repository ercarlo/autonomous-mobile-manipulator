#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - OBJECT POSITION 3D
============================================================

Paquete: amm_perception
Archivo: object_position_3d.py

Objetivo:
    Estimar una posición tridimensional representativa
    de un objeto detectado por YOLO11s-seg.

Entradas:
    /yolo/masks/bottle
    /wrist_camera/depth_image
    /wrist_camera/camera_info

Salidas:
    /perception/bottle/point_camera
    /perception/bottle/point_base

Procesamiento:
    1. Sincronizar máscara y profundidad.
    2. Seleccionar píxeles pertenecientes al objeto.
    3. Filtrar profundidades inválidas.
    4. Proyectar los píxeles a coordenadas 3D.
    5. Calcular una posición representativa.
    6. Transformar el punto a base_link mediante TF2.

============================================================
"""

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge

from message_filters import (
    Subscriber,
    ApproximateTimeSynchronizer
)

from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException
)


class ObjectPosition3D(Node):

    def __init__(self):

        super().__init__("object_position_3d")

        self.bridge = CvBridge()

        # ----------------------------------------------------
        # 1. Parámetros intrínsecos de la RealSense
        # ----------------------------------------------------

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/wrist_camera/camera_info",
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        # ----------------------------------------------------
        # 2. Sincronización máscara-profundidad
        # ----------------------------------------------------

        # Las máscaras se publican a unos 2 Hz.
        # La profundidad llega aproximadamente a 26 Hz.
        #
        # Como YOLO introduce un retraso de procesamiento,
        # conservamos suficientes imágenes de profundidad
        # para recuperar la correspondiente a cada máscara.

        self.mask_sub = Subscriber(
            self,
            Image,
            "/yolo/masks/bottle",
            qos_profile=qos_profile_sensor_data
        )

        self.depth_sub = Subscriber(
            self,
            Image,
            "/wrist_camera/depth_image",
            qos_profile=qos_profile_sensor_data
        )

        self.synchronizer = ApproximateTimeSynchronizer(
            [
                self.mask_sub,
                self.depth_sub
            ],
            queue_size=80,
            slop=0.05
        )

        self.synchronizer.registerCallback(
            self.synchronized_callback
        )

        # ----------------------------------------------------
        # 3. TF2
        # ----------------------------------------------------

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ----------------------------------------------------
        # 4. Publicadores de puntos 3D
        # ----------------------------------------------------

        self.camera_point_pub = self.create_publisher(
            PointStamped,
            "/perception/bottle/point_camera",
            10
        )

        self.base_point_pub = self.create_publisher(
            PointStamped,
            "/perception/bottle/point_base",
            10
        )

        self.get_logger().info(
            "Nodo de estimación 3D iniciado"
        )

    def camera_info_callback(self, msg):

        # ----------------------------------------------------
        # 5. Obtener parámetros intrínsecos
        # ----------------------------------------------------

        # Matriz K:
        #
        # [fx  0 cx]
        # [ 0 fy cy]
        # [ 0  0  1]

        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def synchronized_callback(self, mask_msg, depth_msg):

        # ----------------------------------------------------
        # 6. Comprobar los datos disponibles
        # ----------------------------------------------------

        if self.fx is None or self.fy is None:
            return

        if self.fx <= 0 or self.fy <= 0:
            self.get_logger().warning(
                "Parámetros intrínsecos inválidos"
            )
            return

        if mask_msg.header.frame_id != depth_msg.header.frame_id:
            self.get_logger().warning(
                "Las imágenes utilizan frames diferentes"
            )
            return

        if depth_msg.encoding != "32FC1":
            self.get_logger().warning(
                f"Formato de profundidad inesperado: "
                f"{depth_msg.encoding}"
            )
            return

        # ----------------------------------------------------
        # 7. Convertir mensajes ROS a matrices NumPy
        # ----------------------------------------------------

        mascara = self.bridge.imgmsg_to_cv2(
            mask_msg,
            desired_encoding="mono8"
        )

        profundidad = self.bridge.imgmsg_to_cv2(
            depth_msg,
            desired_encoding="passthrough"
        )

        if mascara.shape != profundidad.shape:
            self.get_logger().warning(
                "Máscara y profundidad tienen tamaños diferentes"
            )
            return

        # ----------------------------------------------------
        # 8. Filtrar bordes de la máscara
        # ----------------------------------------------------

        # Una erosión ligera reduce la posibilidad de
        # incluir píxeles del fondo próximos al contorno
        # del objeto.

        kernel = np.ones(
            (3, 3),
            dtype=np.uint8
        )

        mascara_filtrada = cv2.erode(
            mascara,
            kernel,
            iterations=1
        )

        # ----------------------------------------------------
        # 9. Seleccionar píxeles válidos
        # ----------------------------------------------------

        # Seleccionamos únicamente los píxeles:
        #   - Pertenecientes al objeto.
        #   - Con profundidad finita.
        #   - Situados entre 0.1 y 3.0 metros.

        pixeles_validos = (
            (mascara_filtrada > 0)
            & np.isfinite(profundidad)
            & (profundidad > 0.1)
            & (profundidad < 3.0)
        )

        v, u = np.where(pixeles_validos)

        if len(u) < 30:
            self.get_logger().warning(
                "No hay suficientes píxeles válidos "
                "para estimar la posición"
            )
            return

        # ----------------------------------------------------
        # 10. Obtener las profundidades
        # ----------------------------------------------------

        z = profundidad[v, u].astype(np.float64)

        # ----------------------------------------------------
        # 11. Proyectar los píxeles a coordenadas 3D
        # ----------------------------------------------------

        x = (u - self.cx) * z / self.fx

        y = (v - self.cy) * z / self.fy

        # ----------------------------------------------------
        # 12. Calcular una posición representativa
        # ----------------------------------------------------

        # Utilizamos la mediana de las coordenadas 3D.
        #
        # Es más robusta frente a valores atípicos
        # que utilizar un único píxel.

        x_med = float(np.median(x))
        y_med = float(np.median(y))
        z_med = float(np.median(z))

        # ----------------------------------------------------
        # 13. Publicar el punto en el frame óptico
        # ----------------------------------------------------

        punto_camera = PointStamped()

        punto_camera.header = mask_msg.header

        punto_camera.point.x = x_med
        punto_camera.point.y = y_med
        punto_camera.point.z = z_med

        self.camera_point_pub.publish(
            punto_camera
        )

        # ----------------------------------------------------
        # 14. Transformar el punto a base_link
        # ----------------------------------------------------

        try:

            transformacion = self.tf_buffer.lookup_transform(
                "base_link",
                mask_msg.header.frame_id,
                Time.from_msg(mask_msg.header.stamp),
                timeout=Duration(seconds=0.0)
            )

        except TransformException as error:

            self.get_logger().warning(
                f"Transformación TF2 no disponible: {error}"
            )

            return

        # ----------------------------------------------------
        # 15. Aplicar rotación y traslación
        # ----------------------------------------------------

        # Representamos la orientación como un cuaternión
        # y aplicamos su rotación al punto 3D.
        #
        # La transformación obtenida mediante TF2
        # convierte coordenadas del frame óptico
        # al frame base_link.

        q = transformacion.transform.rotation

        q_vector = np.array([
            q.x,
            q.y,
            q.z
        ], dtype=np.float64)

        q_w = q.w

        punto = np.array([
            x_med,
            y_med,
            z_med
        ], dtype=np.float64)

        # Rotación mediante producto vectorial.
        auxiliar = 2.0 * np.cross(q_vector, punto)

        punto_rotado = (
            punto
            + q_w * auxiliar
            + np.cross(q_vector, auxiliar)
        )

        t = transformacion.transform.translation

        punto_base = punto_rotado + np.array([
            t.x,
            t.y,
            t.z
        ])

        # ----------------------------------------------------
        # 16. Publicar el punto en base_link
        # ----------------------------------------------------

        mensaje_base = PointStamped()

        mensaje_base.header.stamp = mask_msg.header.stamp
        mensaje_base.header.frame_id = "base_link"

        mensaje_base.point.x = float(punto_base[0])
        mensaje_base.point.y = float(punto_base[1])
        mensaje_base.point.z = float(punto_base[2])

        self.base_point_pub.publish(
            mensaje_base
        )

        # ----------------------------------------------------
        # 17. Diagnóstico
        # ----------------------------------------------------

        self.get_logger().info(
            f"BOTELLA | "
            f"Camera: [{x_med:.3f}, {y_med:.3f}, {z_med:.3f}] | "
            f"Base: [{punto_base[0]:.3f}, "
            f"{punto_base[1]:.3f}, "
            f"{punto_base[2]:.3f}] | "
            f"Píxeles: {len(u)}"
        )


def main(args=None):

    rclpy.init(args=args)

    nodo = ObjectPosition3D()

    try:
        rclpy.spin(nodo)

    except KeyboardInterrupt:
        pass

    finally:

        nodo.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()