#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - YOLO SEGMENTATION
============================================================

Paquete: amm_perception
Archivo: yolo_segmentation.py

Objetivo:
    Detectar y segmentar objetos mediante YOLO11s-seg
    utilizando las imágenes RGB de la RealSense D435.

Entrada:
    /wrist_camera/image

Salidas:
    /yolo/segmented_image
    /yolo/masks/bowl
    /yolo/masks/cup
    /yolo/masks/apple
    /yolo/masks/bottle

Las máscaras:
    - Son imágenes mono8.
    - Tienen la misma resolución que la imagen RGB.
    - Contienen 255 en los píxeles del objeto.
    - Contienen 0 en el resto.
    - Conservan el timestamp de la imagen original.

============================================================
"""

import time

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO


class YoloSegmentation(Node):

    def __init__(self):

        super().__init__("yolo_segmentation")

        # ----------------------------------------------------
        # 1. Modelo de segmentación
        # ----------------------------------------------------

        self.modelo = YOLO("models/yolo11s-seg.pt")

        self.bridge = CvBridge()

        # Clases presentes en nuestro escenario.
        self.clases_objetivo = (
            "bowl",
            "cup",
            "apple",
            "bottle"
        )

        self.ultima_imagen = None
        self.imagen_nueva = False

        # ----------------------------------------------------
        # 2. Suscripción a la cámara
        # ----------------------------------------------------

        self.subscription = self.create_subscription(
            Image,
            "/wrist_camera/image",
            self.image_callback,
            qos_profile_sensor_data
        )

        # ----------------------------------------------------
        # 3. Publicador de imagen segmentada
        # ----------------------------------------------------

        self.image_publisher = self.create_publisher(
            Image,
            "/yolo/segmented_image",
            qos_profile_sensor_data
        )

        # ----------------------------------------------------
        # 4. Publicadores de máscaras individuales
        # ----------------------------------------------------

        # Creamos un topic por clase.
        #
        # En este escenario tenemos una instancia de cada
        # clase. Si aparecen varias detecciones de una misma
        # clase, conservaremos la de mayor confianza.

        self.mask_publishers = {}

        for nombre in self.clases_objetivo:

            self.mask_publishers[nombre] = (
                self.create_publisher(
                    Image,
                    f"/yolo/masks/{nombre}",
                    qos_profile_sensor_data
                )
            )

        # ----------------------------------------------------
        # 5. Temporizador de inferencia
        # ----------------------------------------------------

        # Frecuencia objetivo: 2 Hz.
        # Procesamos únicamente la imagen más reciente.

        self.timer = self.create_timer(
            0.5,
            self.process_image
        )

        self.get_logger().info(
            "YOLO11s-seg iniciado: imagen y máscaras binarias"
        )

    def image_callback(self, msg):

        # Almacenamos la imagen recibida.
        # La inferencia se ejecuta desde el temporizador.

        self.ultima_imagen = msg
        self.imagen_nueva = True

    def process_image(self):

        # ----------------------------------------------------
        # 6. Obtener imagen RGB
        # ----------------------------------------------------

        if self.ultima_imagen is None:
            return

        if not self.imagen_nueva:
            return

        msg = self.ultima_imagen
        self.imagen_nueva = False

        imagen = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        alto, ancho = imagen.shape[:2]

        # ----------------------------------------------------
        # 7. Ejecutar YOLO
        # ----------------------------------------------------

        inicio = time.perf_counter()

        resultado = self.modelo.predict(
            source=imagen,
            imgsz=960,
            conf=0.15,
            iou=0.45,
            agnostic_nms=True,
            device="cpu",
            verbose=False
        , classes=[39, 41, 45, 47])[0]

        duracion = time.perf_counter() - inicio

        # ----------------------------------------------------
        # 8. Generar imagen con segmentaciones
        # ----------------------------------------------------

        imagen_segmentada = resultado.plot()

        mensaje_segmentado = self.bridge.cv2_to_imgmsg(
            imagen_segmentada,
            encoding="bgr8"
        )

        mensaje_segmentado.header = msg.header

        self.image_publisher.publish(
            mensaje_segmentado
        )

        # ----------------------------------------------------
        # 9. Preparar máscaras binarias
        # ----------------------------------------------------

        # Creamos inicialmente una máscara vacía para
        # cada clase.
        #
        # Así, si un objeto no se detecta en un fotograma,
        # publicaremos una máscara completamente negra.
        #
        # Todas las máscaras tendrán las mismas dimensiones
        # que la imagen RGB original.

        mascaras = {
            nombre: np.zeros(
                (alto, ancho),
                dtype=np.uint8
            )
            for nombre in self.clases_objetivo
        }

        # Guardamos la mayor confianza encontrada para
        # cada clase, por si hubiera detecciones repetidas.

        mejores_confianzas = {
            nombre: -1.0
            for nombre in self.clases_objetivo
        }

        detecciones = []

        # ----------------------------------------------------
        # 10. Extraer las máscaras de YOLO
        # ----------------------------------------------------

        if resultado.masks is not None:

            # masks.xy devuelve polígonos en coordenadas
            # de píxeles de la imagen original.
            #
            # Utilizamos estos polígonos para generar
            # máscaras binarias con la resolución exacta
            # de nuestra cámara.

            poligonos = resultado.masks.xy

            for indice, caja in enumerate(resultado.boxes):

                clase = int(caja.cls.item())
                confianza = float(caja.conf.item())

                nombre = self.modelo.names[clase]

                # Ignorar clases ajenas a nuestro escenario.
                if nombre not in self.clases_objetivo:
                    continue

                # Si hubiera varias detecciones de la misma
                # clase, utilizar la de mayor confianza.
                if confianza <= mejores_confianzas[nombre]:
                    continue

                poligono = poligonos[indice]

                if len(poligono) < 3:
                    continue

                # Convertir las coordenadas del contorno
                # a enteros para trabajar con OpenCV.

                contorno = np.rint(
                    poligono
                ).astype(np.int32)

                contorno = contorno.reshape(
                    (-1, 1, 2)
                )

                # Crear la máscara binaria del objeto.
                mascara = np.zeros(
                    (alto, ancho),
                    dtype=np.uint8
                )

                cv2.fillPoly(
                    mascara,
                    [contorno],
                    color=255
                )

                mascaras[nombre] = mascara
                mejores_confianzas[nombre] = confianza

        # ----------------------------------------------------
        # 11. Publicar máscaras ROS 2
        # ----------------------------------------------------

        for nombre, mascara in mascaras.items():

            mensaje_mascara = self.bridge.cv2_to_imgmsg(
                mascara,
                encoding="mono8"
            )

            # El timestamp y el frame_id son los mismos
            # que los de la imagen RGB procesada.
            #
            # Esto permitirá relacionar posteriormente
            # la segmentación con la profundidad.

            mensaje_mascara.header = msg.header

            self.mask_publishers[nombre].publish(
                mensaje_mascara
            )

            if mejores_confianzas[nombre] >= 0:

                detecciones.append(
                    f"{nombre}: "
                    f"{mejores_confianzas[nombre]:.2f}"
                )

        # ----------------------------------------------------
        # 12. Diagnóstico
        # ----------------------------------------------------

        self.get_logger().info(
            f"Detecciones: {detecciones} | "
            f"Inferencia: {duracion:.3f} s"
        )


def main(args=None):

    rclpy.init(args=args)

    nodo = YoloSegmentation()

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