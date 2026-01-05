from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='haziq_excavator',
            executable='frame_excavator_tf2_broadcaster',
            name='excavator1_broadcaster',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('haziq_excavator'), 'rviz', 'excavator_view.rviz'
            ])],
        ),

    
    ])
