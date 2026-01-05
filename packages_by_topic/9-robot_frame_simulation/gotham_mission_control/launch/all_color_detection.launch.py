from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gotham_mission_control',
            executable='color_node2',
            name='red_detector',
            namespace='red',
            remappings=[('/image_raw', '/patrol_cam/image_raw')],
        ),
        Node(
            package='gotham_mission_control',
            executable='color_node2',
            name='green_detector',
            namespace='green',
            remappings=[('/image_raw', '/patrol_cam/image_raw')],
        ),
        Node(
            package='gotham_mission_control',
            executable='color_node2',
            name='blue_detector',
            namespace='blue',
            remappings=[('/image_raw', '/patrol_cam/image_raw')],
        ),
        Node(
            package='gotham_mission_control',
            executable='color_node2',
            name='purple_detector',
            namespace='purple',
            remappings=[('/image_raw', '/patrol_cam/image_raw')],
        ),
        Node(
            package='gotham_mission_control',
            executable='color_node2',
            name='orange_detector',
            namespace='orange',
            remappings=[('/image_raw', '/patrol_cam/image_raw')],
        ),
    ])

