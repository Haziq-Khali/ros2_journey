from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gotham_mission_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=[
        'setuptools', 
        'rosidl_runtime_py',
    ],
    zip_safe=True,
    maintainer='enders',
    maintainer_email='enders@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'qr_node = gotham_mission_control.qr_node:main',
            'color_node = gotham_mission_control.color_node:main',
            'text_node = gotham_mission_control.text_node:main',
            'object_node = gotham_mission_control.object_node2:main',
            'hunger_games = gotham_mission_control.hunger_games2:main',
            'mission_manager = gotham_mission_control.mission_manager:main',
            'patrol_action_server = gotham_mission_control.patrol_action_server:main',
        ],
    },
)
