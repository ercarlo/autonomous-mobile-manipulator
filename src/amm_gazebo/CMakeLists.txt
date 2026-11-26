
cmake_minimum_required(VERSION 3.8)

# ============================================================
# AUTONOMOUS MOBILE MANIPULATOR - GAZEBO PACKAGE
# ============================================================
#
# Contiene los recursos necesarios para la simulación:
#   - Mundos SDF.
#   - Archivos launch.
#   - Configuraciones de simulación.
#
# Este paquete no contiene ejecutables C++.
# ============================================================

project(amm_gazebo)

# Sistema de compilación de ROS 2.
find_package(ament_cmake REQUIRED)

# Instalamos los recursos del paquete para que otros
# componentes puedan localizarlos desde cualquier workspace
# que tenga cargado nuestro entorno.
install(
  DIRECTORY
    launch
    worlds
    config
    models
  DESTINATION share/${PROJECT_NAME}
)

# Registro del paquete en ROS 2.
ament_package()
