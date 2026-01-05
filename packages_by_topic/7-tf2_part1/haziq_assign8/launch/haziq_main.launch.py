from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare('haziq_assign8'), 'launch'])

    return LaunchDescription([

        # Launch static frame broadcaster
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'haziq_static_publisher.launch.py'])
        ),

        # Launch tf_echo
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'haziq_tf_echo.launch.py'])
        ),

    ])