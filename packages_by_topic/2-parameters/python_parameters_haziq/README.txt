=======================================
python_parameters_haziq - ROS2 Parameters
=======================================

This package demonstrates how does parameters work in ROS2 in Python 
using CLI (ros1 param set), custom parameters (custom_param.yaml) and by rqt. 
Then we demonstrate via ros2 launch. 

## 1. Installation

1. Go to your ROS2 workspace 'src' folder:
    cd ~/ros2_ws/src

2. Copy the python_parameters_haziq package folder into the 'src'.

3. Go back to the workspace and build the package:
    cd ~/ros2_ws
    colcon build --packages-select python_parameters_haziq

4. Restart the terminal by open the new terminal or running the line below in the old terminal:
    source ~/.bashrc

---

## 2. Running the node (focus on Graphic card names)

### In one terminal run the package:
    $ ros2 run python_parameters_haziq haziq_param_node

### In another terminal set the parameter using CLI:
    $ ros2 param set /haziq_param_node my_quantity 1

---

We can also dump parameters and load it in terminal. The parameters are already dumped in a file 
named 'custom_param.yaml' in a folder named 'config_param'. For notes only below is the command line to dump parameters:

    $ ros2 param dump /haziq_param_node > ~/ros2_ws/src/python_parameters_haziq/config_param/custom_param.yaml

### In one terminal run the packages

### In another terminal load the parameters file:
    $ ros2 param load /haziq_param_node ~/ros2_ws/src/python_parameters_haziq/config_param/custom_param.yaml


We can also use rqt to cahnge the parameters.

## In another terminal run the command:
    rqt

====================================================================================================================

# 3. Running the node via ros2 launch (focus on food names)

### In one terminal
    $ ros2 launch python_parameters_haziq python_parameters_haziq_launch.py

### In another terminal we can set the parameters but the command will be a bit different as we using
different node (custom_haziq_param_node):
    $ ros2 param set /custom_haziq_param_node my_item 'Nasi Lemak'

We can load the dump file which was already included in the packages by using below command:
    $ ros2 param dump /custom_haziq_param_node > ~/ros2_ws/src/python_parameters_haziq/config_param/custom_param_launch.yaml


### In another terminal load the dump file:
    $ ros2 param load /custom_haziq_param_node ~/ros2_ws/src/python_parameters_haziq/config_param/custom_param_launch.yaml 

Then we can also change parameters using rqt.



