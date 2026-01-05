from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='tf2_echo',
            name='tf2_echo_pointC_to_pointB',
            output='screen',
            arguments=['pointC', 'pointB']
        ),

                
    ])
