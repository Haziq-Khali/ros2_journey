from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    ## for launch args
    #my_parameter = LaunchConfiguration('my_parameter')

    return LaunchDescription([

        ## declare default arguments
        DeclareLaunchArgument(
            'my_parameter', 
            default_value= 'Pelanggan Tetap'
        ),
        DeclareLaunchArgument(
            'my_quantity',
            default_value= '2'
        ),
        DeclareLaunchArgument(
            'my_item',
            default_value= 'Ikan Siakap'
        ),

        Node(
            package='python_parameters_haziq',          #CHANGE
            executable='haziq_param_node',              #CHANGE
            name='custom_haziq_param_node',             #CHANGE
            output='screen',
            emulate_tty=True,
            #parameters=[{'my_parameter': my_parameter}],
            parameters=[PathJoinSubstitution([
               FindPackageShare('launch_assign7_haziq'), 'config', 'python_params.yaml'])
            ],
        ),






        
    ])