import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amm_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carlo',
    maintainer_email='carlo@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'planning_client = amm_navigation.planning_client:main',
            'path_follower_client = amm_navigation.path_follower_client:main',
            'navigation_client = amm_navigation.navigation_client:main',
            'station_manager = amm_navigation.station_manager:main',


        ],
    },
)
