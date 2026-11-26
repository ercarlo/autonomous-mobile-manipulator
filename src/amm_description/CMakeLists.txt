
cmake_minimum_required(VERSION 3.8)

# ============================================================
# AUTONOMOUS MOBILE MANIPULATOR - DESCRIPTION PACKAGE
# ============================================================
#
# Este paquete contiene la descripción del robot:
#   - Archivos URDF/Xacro.
#   - Archivos launch de visualización.
#   - Configuraciones de RViz.
#   - Parámetros asociados al modelo.
#
# No contiene ejecutables C++.
# ============================================================

project(amm_description)

# ------------------------------------------------------------
# 1. SISTEMA DE COMPILACIÓN
# ------------------------------------------------------------
# ament_cmake permite integrar el paquete en ROS 2.

find_package(ament_cmake REQUIRED)

# ------------------------------------------------------------
# 2. INSTALACIÓN DE RECURSOS
# ------------------------------------------------------------
# Copiamos nuestros directorios al espacio install/.
#
# De esta forma podremos localizar los archivos mediante
# get_package_share_directory("amm_description").

install(
  DIRECTORY
    urdf
    launch
    config
    rviz
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------
# 3. REGISTRO DEL PAQUETE
# ------------------------------------------------------------
# Registra el paquete en el índice de ROS 2.

ament_package()
