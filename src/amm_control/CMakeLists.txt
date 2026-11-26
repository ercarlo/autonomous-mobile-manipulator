
cmake_minimum_required(VERSION 3.8)

# ============================================================
# AUTONOMOUS MOBILE MANIPULATOR - CONTROL PACKAGE
# ============================================================
#
# Este paquete contendrá:
#   - Interfaces ros2_control.
#   - Configuración YAML de controladores.
#   - Launch de gestión de controladores.
#
# No implementa hardware físico ni ejecutables C++.
# ============================================================

project(amm_control)

# Sistema de construcción de paquetes ROS 2.
find_package(ament_cmake REQUIRED)

# Instalamos los archivos de configuración necesarios.
# Otros paquetes podrán encontrarlos mediante el índice
# de paquetes de ROS 2.
install(
  DIRECTORY
    config
    urdf
    launch
  DESTINATION share/${PROJECT_NAME}
)

# Registrar el paquete en ROS 2.
ament_package()
