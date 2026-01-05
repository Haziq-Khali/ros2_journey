===========================
    launch_assign7_haziq
===========================

How to compile?
~ First download the launch_assign7_haziq package and extract it into your ROS2 workspace.
Then build the package. This package contain 4 main code:
    1. /config/python_params.yaml
    2. /launch/haziq_main.launch.py
    3. /launch/python_parameters_haziq.launch.py 
    4. /launch/action_drone.launch.py 

~ By launching the haziq_main.launch.py, it will run the two sub launch files, python_parameters_haziq.launch.py and 
action_drone.launch.py. Here I am using the previous assignment examples: ASSIGNMENT 2 and ASSIGNMET 4.

~ For the python_parameters_haziq.launch.py was included a changing parameter using yaml file.
While for action_drone.launch.py was included a substitution to change the parameter which is target_altitude.

Below is the list of CLI that used to try the launch package:
    $ ros2 launch launch_assign7_haziq haziq_main.launch.py 
    $ ros2 launch launch_assign7_haziq python_parameters_haziq.launch.py --show-args
    $ ros2 launch launch_assign7_haziq action_drone.launch.py
    $ ros2 launch launch_assign7_haziq action_drone.launch.py target_altitude:=2.0
