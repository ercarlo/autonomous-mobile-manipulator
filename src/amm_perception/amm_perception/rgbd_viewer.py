#!/usr/bin/env python3

"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - RGB-D VIEWER
============================================================

Paquete: amm_perception
Archivo: rgbd_viewer.py

Objetivo:
    Recibir imágenes RGB de la cámara de muñeca
    del UR5e y convertirlas a formato OpenCV.

Primera versión:
    - Suscripción al topic RGB.
    - Conversión mediante cv_bridge.
    - Comprobación de dimensiones y tipo de datos.
============================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

from message_filters import Subscriber, ApproximateTimeSynchronizer

from geometry_msgs.msg import PointStamped


import cv2
import numpy as np


class RGBDViewer(Node):

    def __init__(self):
        super().__init__("rgbd_viewer")

        # Puente de conversión entre imágenes ROS 2 y OpenCV.
        self.bridge = CvBridge()

        # Almacenamos las últimas imágenes recibidas.
        # Todavía no están sincronizadas temporalmente.
        self.rgb_image = None
        self.depth_image = None

        # =====================================================
        # SELECCIÓN INTERACTIVA DE PÍXELES
        # =====================================================
        #
        # Guardaremos las coordenadas del último píxel
        # seleccionado por el usuario sobre la imagen RGB.

        self.punto_seleccionado = None

        # Nombre de la ventana de OpenCV.
        self.nombre_ventana = "AMM - Camara RGB-D de muneca"

        # Creamos la ventana con el tamaño de la imagen
        # para mantener correctamente las coordenadas del ratón.
        cv2.namedWindow(
            self.nombre_ventana,
            cv2.WINDOW_AUTOSIZE
        )

        # Asociamos los eventos del ratón a nuestro callback.
        cv2.setMouseCallback(
            self.nombre_ventana,
            self.mouse_callback
        )


        # Evitamos imprimir información en cada imagen.
        self.rgb_recibida = False

        # Suscripción a la cámara de muñeca.
        # Utilizamos el perfil QoS apropiado para sensores.

        # =====================================================
        # SUSCRIPTORES RGB-D SINCRONIZADOS
        # =====================================================
        #
        # Utilizamos Subscriber de message_filters en lugar
        # de create_subscription de rclpy.
        #
        # Esto permite que el sincronizador reciba
        # directamente los mensajes de ambas cámaras.
        #
        # Mantenemos el QoS apropiado para sensores.

        self.rgb_sub = Subscriber(
            self,
            Image,
            "/wrist_camera/image",
            qos_profile=qos_profile_sensor_data
        )

        self.depth_sub = Subscriber(
            self,
            Image,
            "/wrist_camera/depth_image",
            qos_profile=qos_profile_sensor_data
        )


        # =====================================================
        # PARÁMETROS INTRÍNSECOS DE LA CÁMARA
        # =====================================================
        #
        # Almacenaremos fx, fy, cx y cy obtenidos
        # directamente del mensaje CameraInfo.
        #
        # Estos parámetros permanecen constantes mientras
        # no cambie la calibración de nuestra cámara.

        self.intrinsics = None

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/wrist_camera/camera_info",
            self.camera_info_callback,
            qos_profile_sensor_data
        )

                # Publicador de puntos 3D respecto al frame óptico.
        self.point_pub = self.create_publisher(
            PointStamped,
            "/perception/selected_point_camera",
            10
        )

        # Cabecera de la última pareja RGB-D sincronizada.
        # Nos permitirá conservar el instante de la medición.
        self.depth_header = None


        # =====================================================
        # SINCRONIZADOR TEMPORAL
        # =====================================================
        #
        # queue_size:
        #   Número máximo de mensajes almacenados
        #   temporalmente por cada entrada.
        #
        # slop:
        #   Máxima diferencia temporal permitida
        #   entre imágenes, expresada en segundos.
        #
        # Utilizamos inicialmente 20 ms, menos que
        # el periodo aproximado de una cámara a 30 Hz.

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=5,
            slop=0.02
        )

        # Ambos mensajes llegarán juntos a este callback.
        self.sync.registerCallback(self.rgbd_callback)

        # Controlamos la impresión de información inicial.
        self.primera_pareja = True


        # Temporizador de visualización a 5 Hz.
        # Limitamos la frecuencia para reducir la carga gráfica
        # de la máquina virtual.
        self.visualization_timer = self.create_timer(
            0.2,
            self.display_images
        )


        self.get_logger().info("Esperando imágenes RGB...")

 
    def rgbd_callback(self, rgb_msg, depth_msg):

        # ==================================================
        # 1. COMPROBACIÓN TEMPORAL
        # ==================================================
        #
        # Calculamos la diferencia entre los timestamps
        # de las imágenes RGB y profundidad.
        #
        # sec: segundos
        # nanosec: nanosegundos

        tiempo_rgb = (
            rgb_msg.header.stamp.sec +
            rgb_msg.header.stamp.nanosec * 1e-9
        )

        tiempo_depth = (
            depth_msg.header.stamp.sec +
            depth_msg.header.stamp.nanosec * 1e-9
        )

        diferencia_ms = abs(
            tiempo_rgb - tiempo_depth
        ) * 1000.0

        # ==================================================
        # 2. CONVERSIÓN A OPENCV
        # ==================================================

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(
                rgb_msg,
                desired_encoding="bgr8"
            )

            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough"
            )

        except CvBridgeError as error:
            self.get_logger().error(
                f"Error convirtiendo imágenes: {error}"
            )
            return

        # Comprobamos que ambas imágenes tienen
        # la misma resolución antes de combinarlas.
        if rgb_image.shape[:2] != depth_image.shape:
            self.get_logger().error(
                "RGB y profundidad tienen resoluciones diferentes."
            )
            return

        # ==================================================
        # 3. ALMACENAMIENTO DEL PAR SINCRONIZADO
        # ==================================================
        #
        # El temporizador de visualización utilizará
        # estas dos imágenes, que pertenecen al mismo
        # par temporal.

        self.rgb_image = rgb_image
        self.depth_image = depth_image

                # Conservamos el timestamp y el frame de profundidad.
        # Es importante utilizar el instante de la medición,
        # especialmente cuando el brazo está moviéndose.
        self.depth_header = depth_msg.header

        # ==================================================
        # 4. VALIDACIÓN INICIAL
        # ==================================================

        if self.primera_pareja:

            self.get_logger().info(
                "Primer par RGB-D sincronizado recibido."
            )

            self.get_logger().info(
                f"RGB: {rgb_image.shape}, {rgb_image.dtype}"
            )

            self.get_logger().info(
                f"Profundidad: {depth_image.shape}, "
                f"{depth_image.dtype}"
            )

            self.get_logger().info(
                f"Diferencia temporal: {diferencia_ms:.3f} ms"
            )

            self.primera_pareja = False


    def camera_info_callback(self, msg):

        # ==================================================
        # EXTRACCIÓN DE LOS PARÁMETROS INTRÍNSECOS
        # ==================================================
        #
        # CameraInfo contiene una matriz K de 3x3:
        #
        #       [ fx   0   cx ]
        #   K = [  0  fy   cy ]
        #       [  0   0    1 ]
        #
        # ROS 2 almacena la matriz como un vector
        # de nueve elementos, ordenado por filas.

        fx = msg.k[0]
        fy = msg.k[4]
        cx = msg.k[2]
        cy = msg.k[5]

        # Evitamos utilizar una calibración no válida.
        if fx <= 0 or fy <= 0:
            self.get_logger().warn(
                "CameraInfo contiene focales no validas."
            )
            return

        # Registramos los parámetros una sola vez.
        if self.intrinsics is None:

            self.intrinsics = (fx, fy, cx, cy)

            self.get_logger().info(
                f"Intrinsecos recibidos: "
                f"fx={fx:.3f}, fy={fy:.3f}, "
                f"cx={cx:.1f}, cy={cy:.1f}"
            )

            self.get_logger().info(
                f"Frame optico: {msg.header.frame_id}"
            )


    def display_images(self):

        # No visualizamos hasta recibir las dos imágenes.
        if self.rgb_image is None or self.depth_image is None:
            return

        # ==================================================
        # 1. PROCESAMIENTO VISUAL DE PROFUNDIDAD
        # ==================================================
        #
        # La imagen de profundidad contiene metros (float32).
        # Para mostrarla con OpenCV, generamos una copia
        # normalizada entre 0 y 255 (uint8).
        #
        # Esta conversión es SOLO para visualización.
        # La profundidad original no se modifica.

        profundidad = self.depth_image


        # ==================================================
        # FILTRADO POR RANGO DE PROFUNDIDAD
        # ==================================================
        #
        # Seleccionamos regiones situadas dentro del
        # intervalo de trabajo definido.
        #
        # Esta máscara no identifica objetos:
        # únicamente determina qué píxeles pertenecen
        # al rango de profundidad seleccionado.

        distancia_min = 0.20  # metros
        distancia_max = 1.00  # metros

        # Comprobamos que la medida sea válida
        # y esté dentro del intervalo establecido.
        mascara = (
            np.isfinite(profundidad) &
            (profundidad >= distancia_min) &
            (profundidad <= distancia_max)
        )

        # Convertimos la máscara booleana en una
        # imagen binaria compatible con OpenCV.
        #
        # 255 = región aceptada (blanco)
        #   0 = región descartada (negro)

        mascara_binaria = mascara.astype(np.uint8) * 255



        # ==================================================
        # APLICACIÓN DE LA MÁSCARA SOBRE LA IMAGEN RGB
        # ==================================================
        #
        # Conservamos los píxeles RGB cuya profundidad
        # está dentro del rango de observación.
        #
        # 255 en la máscara: conservamos el píxel RGB.
        #   0 en la máscara: el píxel aparece negro.
        #
        # No modificamos la imagen RGB original.

        rgb_filtrada = cv2.bitwise_and(
            self.rgb_image,
            self.rgb_image,
            mask=mascara_binaria
        )

        # Reducimos la imagen para visualizarla.
        rgb_filtrada_display = cv2.resize(
            rgb_filtrada,
            (480, 360)
        )

        cv2.imshow(
            "AMM - RGB filtrada por profundidad",
            rgb_filtrada_display
        )



        # Adaptamos la resolución exclusivamente
        # para mostrar la imagen.
        mascara_display = cv2.resize(
            mascara_binaria,
            (480, 360),
            interpolation=cv2.INTER_NEAREST
        )

        cv2.imshow(
            "AMM - Filtro de profundidad",
            mascara_display
        )


        # Identificamos píxeles con profundidad válida.
        validos = np.isfinite(profundidad) & (profundidad > 0)

        # Sustituimos temporalmente los píxeles inválidos
        # para evitar problemas durante la conversión.
        profundidad_segura = np.where(
            validos,
            profundidad,
            2.0
        )

        # Rango de visualización: entre 0.1 y 2 metros.
        # Los objetos cercanos se mostrarán más claros.
        profundidad_limitada = np.clip(
            profundidad_segura,
            0.1,
            2.0
        )

        profundidad_gris = (
            255.0 * (2.0 - profundidad_limitada) / 1.9
        ).astype(np.uint8)

        # Aplicamos una escala de colores para distinguir
        # mejor objetos situados a diferentes distancias.
        profundidad_color = cv2.applyColorMap(
            profundidad_gris,
            cv2.COLORMAP_TURBO
        )

        # Representamos las profundidades inválidas
        # mediante píxeles negros.
        profundidad_color[~validos] = (0, 0, 0)

        # ==================================================
        # 2. PREPARACIÓN DE LAS IMÁGENES
        # ==================================================

        # Reducimos la resolución de visualización
        # para no sobrecargar la interfaz de VMware.
        rgb_display = cv2.resize(
            self.rgb_image,
            (480, 360)
        )

        depth_display = cv2.resize(
            profundidad_color,
            (480, 360)
        )

        # Etiquetas identificativas.
        cv2.putText(
            rgb_display,
            "RGB",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.putText(
            depth_display,
            "Profundidad (0.1 - 2 m)",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )


        # ==================================================
        # REPRESENTACIÓN DEL PÍXEL SELECCIONADO
        # ==================================================
        #
        # Si el usuario ha seleccionado un punto,
        # dibujamos su posición en ambas imágenes.

        if self.punto_seleccionado is not None:

            u, v = self.punto_seleccionado

            # Transformamos las coordenadas originales
            # a la resolución de visualización.
            x_vis = int(u * 480 / self.rgb_image.shape[1])
            y_vis = int(v * 360 / self.rgb_image.shape[0])

            # Dibujamos un círculo sobre el mismo píxel
            # en RGB y profundidad.
            cv2.circle(
                rgb_display,
                (x_vis, y_vis),
                5,
                (0, 0, 255),
                -1
            )

            cv2.circle(
                depth_display,
                (x_vis, y_vis),
                5,
                (0, 0, 255),
                -1
            )

            # Consultamos la profundidad actual.
            z = float(self.depth_image[v, u])

            if np.isfinite(z) and z > 0:
                texto = f"({u}, {v})  Z={z:.3f} m"
            else:
                texto = f"({u}, {v})  Z=no valida"

            # Mostramos las coordenadas y la profundidad
            # sobre la imagen RGB.
            cv2.putText(
                rgb_display,
                texto,
                (10, 340),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2
            )

        # ==================================================
        # 3. VISUALIZACIÓN CONJUNTA
        # ==================================================

        # Unimos ambas imágenes horizontalmente.
        imagen_conjunta = np.hstack((
            rgb_display,
            depth_display
        ))

        # Mostramos la imagen en una ventana.
        cv2.imshow(
            self.nombre_ventana,
            imagen_conjunta
        )

        # Procesamos los eventos de la ventana.
        cv2.waitKey(1)



    def mouse_callback(self, event, x, y, flags, param):

        # Solo procesamos clics con el botón izquierdo.
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        # Comprobamos que existan imágenes disponibles.
        if self.rgb_image is None or self.depth_image is None:
            return

        # ==================================================
        # 1. COMPROBAR LA ZONA SELECCIONADA
        # ==================================================
        #
        # La ventana contiene dos imágenes:
        #
        #   RGB:         x = 0 ... 479
        #   Profundidad: x = 480 ... 959
        #
        # Por ahora solo permitimos seleccionar puntos
        # en la imagen RGB.

        if not (0 <= x < 480 and 0 <= y < 360):
            return

        # ==================================================
        # 2. RECUPERAR LAS COORDENADAS ORIGINALES
        # ==================================================
        #
        # Las imágenes originales tienen 640x480 píxeles,
        # pero las visualizamos a 480x360.
        #
        # Debemos convertir las coordenadas del clic
        # a las coordenadas de la imagen original.

        alto, ancho = self.depth_image.shape

        u = min(ancho - 1, int(x * ancho / 480))
        v = min(alto - 1, int(y * alto / 360))

        # Guardamos el píxel seleccionado.
        self.punto_seleccionado = (u, v)

        # ==================================================
        # 3. CONSULTAR LA PROFUNDIDAD
        # ==================================================
        #
        # NumPy utiliza el orden:
        #
        #   imagen[fila, columna] = imagen[v, u]
        #
        # Obtenemos la profundidad del píxel
        # correspondiente al objeto observado.

        profundidad = float(self.depth_image[v, u])

        # ==================================================
        # 4. VALIDAR Y MOSTRAR EL RESULTADO
        # ==================================================

        
        # ==================================================
        # VALIDACIÓN DE PROFUNDIDAD
        # ==================================================

        if not np.isfinite(profundidad) or profundidad <= 0:
            self.get_logger().warn(
                f"Pixel ({u}, {v}): profundidad no valida."
            )
            return

        # Utilizamos el mismo intervalo que hemos
        # definido para nuestro filtro de profundidad.
        if not (0.20 <= profundidad <= 1.00):
            self.get_logger().info(
                f"Pixel ({u}, {v}): Z={profundidad:.3f} m "
                "fuera del rango de observacion."
            )
            return

        # Comprobamos que CameraInfo haya llegado.
        if self.intrinsics is None:
            self.get_logger().warn(
                "Esperando parametros de CameraInfo..."
            )
            return

        # ==================================================
        # PROYECCIÓN INVERSA: PÍXEL 2D -> PUNTO 3D
        # ==================================================
        #
        # Obtenemos los parámetros intrínsecos.
        fx, fy, cx, cy = self.intrinsics

        # La profundidad Z está expresada en metros.
        z = profundidad

        # Calculamos las coordenadas en el frame óptico.
        x_cam = (u - cx) * z / fx
        y_cam = (v - cy) * z / fy


                # ==================================================
        # PUBLICACIÓN DEL PUNTO 3D
        # ==================================================
        #
        # PointStamped combina:
        #   - Header: tiempo y sistema de referencia.
        #   - Point: coordenadas X, Y, Z.
        #
        # Todavía no aplicamos ninguna transformación TF2.

        if self.depth_header is None:
            return

        punto = PointStamped()

        # Conservamos el frame y el timestamp de la medición.
        punto.header = self.depth_header

        # Coordenadas en metros respecto a la cámara.
        punto.point.x = float(x_cam)
        punto.point.y = float(y_cam)
        punto.point.z = float(z)

        # Publicamos el punto seleccionado.
        self.point_pub.publish(punto)
        # ==================================================
        # RESULTADO
        # ==================================================

        self.get_logger().info(
            f"Pixel ({u}, {v}) | "
            f"Posicion 3D en camara: "
            f"X={x_cam:.3f}, "
            f"Y={y_cam:.3f}, "
            f"Z={z:.3f} m"
        )




def main(args=None):

    rclpy.init(args=args)

    node = RGBDViewer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # Cerramos las ventanas de OpenCV.
        cv2.destroyAllWindows()

        # Liberamos los recursos de ROS 2.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
