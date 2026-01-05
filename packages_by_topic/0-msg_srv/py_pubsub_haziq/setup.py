from setuptools import find_packages, setup

package_name = 'py_pubsub_haziq'

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
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'customer = py_pubsub_haziq.pub_order:main',
            'waiter = py_pubsub_haziq.sub_order:main',
        ],
    },
)
