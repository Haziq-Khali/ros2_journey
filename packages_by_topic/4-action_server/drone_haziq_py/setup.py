from setuptools import find_packages, setup

package_name = 'drone_haziq_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haziq',
    maintainer_email='haziq@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_altitude_server = drone_haziq_py.drone_altitude_server:main',
            'drone_altitude_client = drone_haziq_py.drone_altitude_client:main',
            'drone_altitude = drone_haziq_py.drone_altitude:main',
        ],
    },
)
