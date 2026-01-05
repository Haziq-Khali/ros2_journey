==============================
    haziq_assign8 pacakge
==============================

This package contains only launch files to launch the static_transform_publisher 
node from tf2 built in package. Besides it also have rviz folder that contain 
saved configuration fro rviz2 model.

~ How to compile?
    1. Download the package and put it into ros2 workspace.
    2. Compile the package.

~ Use the next CLI to launch
    1. ros2 launch haziq_assign8 haziq_static_publisher.launch.py 
    2. ros2 run tf2_ros tf2_echo world pointA
    3. ros2 run tf2_ros tf2_echo pointA pointB
    4. ros2 run tf2_ros tf2_echo pointA pointC
    5. ros2 run tf2_ros tf2_echo pointC world
    6. ros2 run tf2_ros tf2_echo world pointB
    7. ros2 run tf2_ros tf2_echo pointC pointB