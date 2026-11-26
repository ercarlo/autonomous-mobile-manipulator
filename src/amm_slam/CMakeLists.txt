cmake_minimum_required(VERSION 3.8)
project(amm_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ============================================================
# INSTALACIÓN DE LOS RECURSOS DEL PAQUETE AMM_SLAM
# ============================================================
#
# Este paquete contiene las configuraciones necesarias
# para realizar el mapeo y la localización del MiR100.
#
# Inicialmente no compilamos ejecutables propios:
# utilizaremos SLAM Toolbox y, posteriormente, AMCL.
#
# Instalamos los siguientes directorios:
#
# config:
#   Parámetros YAML de SLAM Toolbox y AMCL.
#
# launch:
#   Archivos Python para iniciar los nodos.
#
# maps:
#   Mapas del almacén generados durante el mapeo.
#
# Los recursos se instalan en:
#   install/amm_slam/share/amm_slam/
#
# De esta forma ROS 2 puede encontrarlos utilizando
# get_package_share_directory("amm_slam").
# ============================================================

install(
  DIRECTORY
    config
    launch
    maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
