from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='tf2_ros',
			executable='static_transform_publisher',
			arguments=[
				'--x', '4',
				'--y', '3',
				'--z', '5.4',
				'--yaw', '0',
				'--pitch', '0',
				'--roll', '3.142',
				'--frame-id', 'world',
				'--child-frame-id', 'pointA']
        ),
		Node(
			package='tf2_ros',
			executable='static_transform_publisher',
			arguments=[
				'--x', '-2',
				'--y', '2',
				'--z', '-3',
				'--yaw', '3.14',
				'--pitch', '0.4',
				'--roll', '2.1',
				'--frame-id', 'pointA',
				'--child-frame-id', 'pointB']
        ),
		Node(
			package='tf2_ros',
			executable='static_transform_publisher',
			arguments=[
				'--x', '-2',
				'--y', '2',
				'--z', '9',
				'--yaw', '0',
				'--pitch', '0',
				'--roll', '0',
				'--frame-id', 'pointA',
				'--child-frame-id', 'pointC']
        ),

		# Launch rviz2
		Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_assign8',
			arguments=['--d', PathJoinSubstitution([
				FindPackageShare('haziq_assign8'), 'rviz', 'orbit_view.rviz'
			])]
        ),

    ])