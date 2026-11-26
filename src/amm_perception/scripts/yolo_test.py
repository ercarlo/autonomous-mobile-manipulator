#!/usr/bin/env python3

"""
Prueba de segmentación YOLO11s con la RealSense D435.

Captura una imagen RGB de Gazebo, ejecuta YOLO
y guarda las detecciones con sus máscaras.
"""

import time
import cv2
import rclpy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from ultralytics import YOLO


# --------------------------------------------------
# 1. Recibir una imagen de ROS 2
# --------------------------------------------------

rclpy.init()
node = rclpy.create_node("yolo_test")

bridge = CvBridge()
imagen = None


def callback(msg):
    global imagen

    if imagen is None:
        imagen = bridge.imgmsg_to_cv2(
            msg, desired_encoding="bgr8"
        )


node.create_subscription(
    Image,
    "/wrist_camera/image",
    callback,
    qos_profile_sensor_data
)

print("Esperando imagen de la RealSense...")

inicio = time.monotonic()

while rclpy.ok() and imagen is None:
    rclpy.spin_once(node, timeout_sec=0.5)

    if time.monotonic() - inicio > 20:
        raise TimeoutError("No se ha recibido ninguna imagen")

node.destroy_node()
rclpy.shutdown()


# --------------------------------------------------
# 2. Ejecutar YOLO11s-seg
# --------------------------------------------------

modelo = YOLO("models/yolo11s-seg.pt")

resultado = modelo.predict(
    imagen,
    imgsz=960,
    conf=0.15,

    # Suprimir detecciones solapadas aunque tengan
    # clases diferentes.
    agnostic_nms=True,

    # Umbral de IoU utilizado por NMS.
    # Si dos cajas se solapan más del 45 %,
    # se puede suprimir la de menor confianza.
    iou=0.45,

    device="cpu",
    verbose=False
)[0]


# --------------------------------------------------
# 3. Mostrar las detecciones
# --------------------------------------------------

print("\nDETECCIONES YOLO:")

for caja in resultado.boxes:
    clase = int(caja.cls.item())
    confianza = float(caja.conf.item())

    print(f"{modelo.names[clase]}: {confianza:.3f}")


# --------------------------------------------------
# 4. Guardar imágenes
# --------------------------------------------------

cv2.imwrite("yolo_original.png", imagen)

cv2.imwrite(
    "yolo_segmentation_test_s.png",
    resultado.plot()
)

num_mascaras = (
    len(resultado.masks.data)
    if resultado.masks is not None
    else 0
)

print(f"\nMáscaras generadas: {num_mascaras}")
print("Imagen guardada: yolo_segmentation_test_s.png")