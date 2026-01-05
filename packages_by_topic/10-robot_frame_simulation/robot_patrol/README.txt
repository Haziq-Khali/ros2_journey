Let the world begins:
  ros2 launch robot_patrol robot_patrol.launch.py 

List of CLI:

    1. Move neck: ros2 topic pub /neck_controller/commands std_msgs/msg/Float64MultiArray \
"{data: [-0.5]}"

    2. Move head: ros2 topic pub /head_controller/commands std_msgs/msg/Float64MultiArray \ "{data: [1.1]}"

To move using keyboard: ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped


To move the batmobile using controller. added configuration for joystick (remap depend on joystick output).
	- Run the joy_node: ros2 run joy joy_node
	- Echo the topic to configure/remap the button: ros2 topic echo /joy 
	- Run the teleop function: ros2 run teleop_twist_joy teleop_node --ros-args --params-file ~/ros2_ws/src/robot_patrol/config/joy_control.yaml -r cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped


==========================
  Gotham Mission Control
==========================

~ rqt imave view
  ros2 run rqt_image_view rqt_image_view

~ Neck and head joystick controller
  ros2 run gotham_mission_control joy_buttons 

~ Color detection
  ros2 launch gotham_mission_control color_detection.launch.py 

~ Color detection multiple
  ros2 launch gotham_mission_control all_color_detection.launch.py 

~ Text detection
  ros2 run gotham_mission_control text_node

~ QR detection
  ros2 run gotham_mission_control qr_node

~ Obstacle avoidance 
  ros2 run gotham_mission_control hunger_games 

