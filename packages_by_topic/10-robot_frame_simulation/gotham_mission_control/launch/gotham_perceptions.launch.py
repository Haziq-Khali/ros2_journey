from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    color_nodes = [
        Node(
            package='gotham_mission_control',
            executable='color_node',
            name=f'{color}_detector',
            namespace=color,
            output='screen',
            remappings=[
                ('/camera_sensor/image_raw', '/camera_sensor/image_raw')
            ],
        )
        for color in ['red', 'green', 'blue', 'purple', 'orange', 'pink']
    ]

    #object_node = Node(
        #package='gotham_mission_control',
        #executable='object_node',
        #name='object_detector',
        #output='screen',
        #remappings=[
            #('/camera_sensor/image_raw', '/camera_sensor/image_raw')
        #],
    #)

    qr_node = Node(
        package='gotham_mission_control',
        executable='qr_node',
        name='qr_detector',
        output='screen',
        remappings=[
            ('/camera_sensor/image_raw', '/camera_sensor/image_raw')
        ],
    )

    text_node = Node(
        package='gotham_mission_control',
        executable='text_node',
        name='text_detector',
        output='screen',
        remappings=[
            ('/camera_sensor/image_raw', '/camera_sensor/image_raw')
        ],
    )

    return LaunchDescription(
        color_nodes + [
            #object_node,
            qr_node,
            text_node,
        ]
    )

