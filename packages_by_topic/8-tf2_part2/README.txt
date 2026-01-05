==============================
    haziq_excavator pacakge
==============================

This package contains three types of frames which are frame_excavator, frame_robot_cleaner
and ext_frame_robot_cleaner plus a listener for frame_excavator. For this assignment I will use frame_excavator as it is not so complicated
to visualize and understand. For the launch file will only launch frame_excavator and rviz2.

~ How to compile?
    1. Download the package and put it into ros2 workspace.
    2. Compile the package.
    3. Refresh the bash file: source ~/.bashrc

~ Use the next CLI to launch
    1. $ ros2 run haziq_excavator frame_excavator_tf2_broadcaster 
    2. $ ros2 launch haziq_excavator frame.launch.py


If you are interested to see the other xample which is robot vacuum cleaner,
you can use the below CLI. There is two types: basic one that no dynamic frame and extended one (ext) that 
has rotation for brush and mop.

    1. $ ros2 run haziq_excavator frame_robot_cleaner_broadcaster 
    2. $ ros2 run haziq_excavator ext_frame_robot_cleaner_broadcaster