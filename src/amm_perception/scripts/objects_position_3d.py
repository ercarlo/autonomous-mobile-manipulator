#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - OBJECTS POSITION 3D
============================================================

Paquete: amm_perception
Archivo: objects_position_3d.py

Objetivo:
    Estimar las posiciones 3D de los cuatro objetos
    segmentados mediante YOLO11s-seg.

Entradas:
    /yolo/masks/bowl
    /yolo/masks/cup
    /yolo/masks/apple
    /yolo/masks/bottle
    /wrist_camera/depth_image
    /wrist_camera/camera_info

Salidas:
    /perception/<objeto>/point_camera
    /perception/<objeto>/point_base

Procesamiento:
    1. Sincronizar las cuatro máscaras y profundidad.
    2. Filtrar los píxeles válidos de cada objeto.
    3. Proyectar los píxeles a coordenadas 3D.
    4. Estimar un punto representativo mediante medianas.
    5. Transformar los puntos a base_link utilizando TF2.

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

from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener, TransformException


class ObjectsPosition3D(Node):

    def __init__(self):

        super().__init__("objects_position_3d")

        self.bridge = CvBridge()

        # ----------------------------------------------------
        # 1. Objetos presentes en nuestro escenario
        # ----------------------------------------------------

        self.objetos = [
            "bowl",
            "cup",
            "apple",
            "bottle"
        ]

        # ----------------------------------------------------
        # 2. Parámetros intrínsecos de la cámara
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
        # 3. Suscripciones a las máscaras de YOLO
        # ----------------------------------------------------

        # El nodo YOLO publica las cuatro máscaras con
        # el mismo timestamp de la imagen RGB original.

        self.mask_subscribers = []

        for objeto in self.objetos:

            subscriber = Subscriber(
                self,
                Image,
                f"/yolo/masks/{objeto}",
                qos_profile=qos_profile_sensor_data
            )

            self.mask_subscribers.append(subscriber)

        # ----------------------------------------------------
        # 4. Suscripción a profundidad
        # ----------------------------------------------------

        self.depth_sub = Subscriber(
            self,
            Image,
            "/wrist_camera/depth_image",
            qos_profile=qos_profile_sensor_data
        )

        # ----------------------------------------------------
        # 5. Sincronización temporal
        # ----------------------------------------------------

        # Sincronizamos cinco mensajes:
        #   - Cuatro máscaras del mismo fotograma.
        #   - Una imagen de profundidad cercana en tiempo.
        #
        # La cola permite conservar profundidad mientras
        # YOLO completa la inferencia en CPU.
        #
        # slop = 0.05 s limita la diferencia temporal
        # máxima entre los mensajes sincronizados.

        self.synchronizer = ApproximateTimeSynchronizer(
            self.mask_subscribers + [self.depth_sub],
            queue_size=80,
            slop=0.05
        )

        self.synchronizer.registerCallback(
            self.synchronized_callback
        )

        # ----------------------------------------------------
        # 6. TF2
        # ----------------------------------------------------

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ----------------------------------------------------
        # 7. Publicadores de posiciones 3D
        # ----------------------------------------------------

        self.camera_publishers = {}
        self.base_publishers = {}

        for objeto in self.objetos:

            self.camera_publishers[objeto] = (
                self.create_publisher(
                    PointStamped,
                    f"/perception/{objeto}/point_camera",
                    10
                )
            )

            self.base_publishers[objeto] = (
                self.create_publisher(
                    PointStamped,
                    f"/perception/{objeto}/point_base",
                    10
                )
            )

        self.get_logger().info(
            "Estimación 3D iniciada para bowl, cup, apple y bottle"
        )

    def camera_info_callback(self, msg):

        # Matriz intrínseca K de la cámara.
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def synchronized_callback(self, *mensajes):

        # ----------------------------------------------------
        # 8. Separar máscaras y profundidad
        # ----------------------------------------------------

        # El orden coincide con self.objetos.
        mensajes_mascaras = mensajes[:-1]

        depth_msg = mensajes[-1]

        if self.fx is None or self.fy is None:
            return

        if self.fx <= 0 or self.fy <= 0:
            return

        if depth_msg.encoding != "32FC1":
            self.get_logger().warning(
                "Formato de profundidad no compatible"
            )
            return

        # Comprobar que las máscaras corresponden
        # al mismo fotograma RGB.

        stamp_referencia = mensajes_mascaras[0].header.stamp
        frame_referencia = mensajes_mascaras[0].header.frame_id

        for msg in mensajes_mascaras:

            if msg.header.stamp != stamp_referencia:
                self.get_logger().warning(
                    "Las máscaras no corresponden al mismo fotograma"
                )
                return

            if msg.header.frame_id != frame_referencia:
                return

        if depth_msg.header.frame_id != frame_referencia:
            self.get_logger().warning(
                "Frame de profundidad diferente al de las máscaras"
            )
            return

        # ----------------------------------------------------
        # 9. Obtener imagen de profundidad
        # ----------------------------------------------------

        profundidad = self.bridge.imgmsg_to_cv2(
            depth_msg,
            desired_encoding="passthrough"
        )

        # ----------------------------------------------------
        # 10. Obtener transformación TF2
        # ----------------------------------------------------

        # La transformación se consulta una sola vez
        # porque los cuatro objetos pertenecen al
        # mismo fotograma.

        try:

            transformacion = self.tf_buffer.lookup_transform(
                "base_link",
                frame_referencia,
                Time.from_msg(stamp_referencia),
                timeout=Duration(seconds=0.0)
            )

        except TransformException as error:

            self.get_logger().warning(
                f"TF2 no disponible: {error}"
            )
            return

        # Obtener rotación y traslación.
        q = transformacion.transform.rotation
        t = transformacion.transform.translation

        q_vector = np.array(
            [q.x, q.y, q.z],
            dtype=np.float64
        )

        traslacion = np.array(
            [t.x, t.y, t.z],
            dtype=np.float64
        )

        # ----------------------------------------------------
        # 11. Procesar los cuatro objetos
        # ----------------------------------------------------

        resultados = []

        kernel = np.ones(
            (3, 3),
            dtype=np.uint8
        )

        for objeto, mask_msg in zip(
            self.objetos,
            mensajes_mascaras
        ):

            mascara = self.bridge.imgmsg_to_cv2(
                mask_msg,
                desired_encoding="mono8"
            )

            if mascara.shape != profundidad.shape:
                continue

            # Erosión ligera para reducir píxeles de fondo.
            mascara = cv2.erode(
                mascara,
                kernel,
                iterations=1
            )

            # Seleccionar profundidades válidas.
            validos = (
                (mascara > 0)
                & np.isfinite(profundidad)
                & (profundidad > 0.1)
                & (profundidad < 3.0)
            )

            v, u = np.where(validos)

            # Si YOLO no detecta el objeto, su máscara
            # estará vacía y no publicaremos ningún punto.
            if len(u) < 30:
                continue

            # ------------------------------------------------
            # 12. Proyección de píxeles a coordenadas 3D
            # ------------------------------------------------

            z = profundidad[v, u].astype(np.float64)

            x = (u - self.cx) * z / self.fx
            y = (v - self.cy) * z / self.fy

            # Mediana 3D de la superficie visible.
            punto_camera = np.array([
                np.median(x),
                np.median(y),
                np.median(z)
            ], dtype=np.float64)

            # ------------------------------------------------
            # 13. Transformación al frame base_link
            # ------------------------------------------------

            auxiliar = 2.0 * np.cross(
                q_vector,
                punto_camera
            )

            punto_rotado = (
                punto_camera
                + q.w * auxiliar
                + np.cross(q_vector, auxiliar)
            )

            punto_base = punto_rotado + traslacion

            # ------------------------------------------------
            # 14. Publicar punto en el frame de la cámara
            # ------------------------------------------------

            mensaje_camera = PointStamped()

            mensaje_camera.header = mask_msg.header

            mensaje_camera.point.x = float(punto_camera[0])
            mensaje_camera.point.y = float(punto_camera[1])
            mensaje_camera.point.z = float(punto_camera[2])

            self.camera_publishers[objeto].publish(
                mensaje_camera
            )

            # ------------------------------------------------
            # 15. Publicar punto en base_link
            # ------------------------------------------------

            mensaje_base = PointStamped()

            mensaje_base.header.stamp = stamp_referencia
            mensaje_base.header.frame_id = "base_link"

            mensaje_base.point.x = float(punto_base[0])
            mensaje_base.point.y = float(punto_base[1])
            mensaje_base.point.z = float(punto_base[2])

            self.base_publishers[objeto].publish(
                mensaje_base
            )

            resultados.append(
                f"{objeto}: "
                f"[{punto_base[0]:.3f}, "
                f"{punto_base[1]:.3f}, "
                f"{punto_base[2]:.3f}]"
            )

        # ----------------------------------------------------
        # 16. Mostrar resultados
        # ----------------------------------------------------

        if resultados:

            self.get_logger().info(
                " | ".join(resultados)
            )


def main(args=None):

    rclpy.init(args=args)

    nodo = ObjectsPosition3D()

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