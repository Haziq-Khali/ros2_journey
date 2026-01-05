from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='python_parameters_haziq',          #CHANGE
            executable='haziq_param_node',              #CHANGE
            name='custom_haziq_param_node',             #CHANGE
            output='screen',
            emulate_tty=True,
            parameters=[
                {'my_parameter': 'auto launcher'},
                {'my_quantity' : 91},
                {'my_item' : 'Ikan Siakap'}
            ]
        )
    ])