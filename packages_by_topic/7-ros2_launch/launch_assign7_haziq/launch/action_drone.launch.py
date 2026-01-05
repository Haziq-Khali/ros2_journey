from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    # Launch arguments
    target_altitude = LaunchConfiguration('target_altitude')
    step = LaunchConfiguration('step')
    rate = LaunchConfiguration('rate')

    # --- Node: Drone Altitude Action Server ---
    drone_altitude_node = Node(
        package='drone_haziq_py',                       # package name
        executable='drone_altitude_server',             # must match your executable name
        name='drone_altitude',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'target_altitude': target_altitude,
            'step': step,
            'rate': rate
        }]
    )

    # --- Correct ExecuteProcess ---
    send_goal_process = ExecuteProcess(
        cmd=[[
            'ros2 action send_goal -f', 
            ' /drone_altitude',
            ' action_haziq_interfaces/action/DroneAltitude',
            ' "{target_altitude: ', 
            target_altitude,
            '}"'
        ]],
        shell=True
    )

    # --- Delay before sending goal ---
    send_goal_timer = TimerAction(
        period=5.0,
        actions=[send_goal_process]
    )

    # --- LaunchDescription ---
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_altitude',
            default_value='10.0',
            description='Target altitude for the drone'
        ),
        DeclareLaunchArgument(
            'step',
            default_value='0.5',
            description='Step size for altitude change'
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='0.5',
            description='Rate (seconds) between altitude updates'
        ),
        drone_altitude_node,
        send_goal_timer
    ])
