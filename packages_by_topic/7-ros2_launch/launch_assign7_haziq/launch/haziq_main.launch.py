from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare('launch_assign7_haziq'), 'launch'])
    return LaunchDescription([

        ## python_parameters_haziq.py
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'python_parameters_haziq.launch.py'])
        ),

        ## drone composable node
        IncludeLaunchDescription(
          PathJoinSubstitution([launch_dir, 'action_drone.launch.py'])  
        ),

    ])