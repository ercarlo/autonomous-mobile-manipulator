"""
============================================================
AUTONOMOUS MOBILE MANIPULATOR - PACKAGE SETUP
============================================================

Paquete: amm_manipulation

Nodos:
    - arm_controller
    - gripper_controller
    - planning_scene

============================================================
"""

import os
from glob import glob

from setuptools import find_packages, setup


package_name = "amm_manipulation"


setup(
    name=package_name,

    version="0.0.0",

    packages=find_packages(
        exclude=["test"]
    ),

    data_files=[

        # Registrar el paquete en el índice de ROS 2.

        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name]
        ),

        # Instalar la descripción del paquete.

        (
            "share/" + package_name,
            ["package.xml"]
        ),

        # Instalar archivos launch.

        (
            os.path.join(
                "share",
                package_name,
                "launch"
            ),
            glob("launch/*.launch.py")
        ),

        # Instalar archivos de configuración.

        (
            os.path.join(
                "share",
                package_name,
                "config"
            ),
            glob("config/*.yaml")
        ),
    ],

    install_requires=["setuptools"],

    zip_safe=True,

    maintainer="carlo",

    maintainer_email="carlo@todo.todo",

    description=(
        "Sistema de manipulación autónoma "
        "con MoveIt 2 y ROS 2."
    ),

    license="Apache-2.0",

    tests_require=["pytest"],

    entry_points={

        "console_scripts": [

            # Control del brazo UR5e.

            "arm_controller = "
            "amm_manipulation.arm_controller:main",

            # Control de la pinza Robotiq.

            "gripper_controller = "
            "amm_manipulation.gripper_controller:main",

            # Gestión de obstáculos de MoveIt 2.

            "planning_scene = "
            "amm_manipulation.planning_scene:main",


            # Configuración de estaciones de trabajo.
            "workstation_scene = amm_manipulation.workstation_scene:main",

            "detected_objects_scene = amm_manipulation.detected_objects_scene:main",

             # Coordinador de las operaciones de recogida.
            "pick_sequence = amm_manipulation.pick_sequence:main",

            'place_sequence = amm_manipulation.place_sequence:main',

            'task_coordinator = amm_manipulation.task_coordinator:main',
        ],
    },
)