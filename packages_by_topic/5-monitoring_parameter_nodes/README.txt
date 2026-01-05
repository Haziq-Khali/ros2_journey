=================================================
haziq_cpp_parameter_event_handler - ROS2 Tutorial
=================================================

This package demonstrates a parameter event handler in Cpp.
Basically there are two packages: cpp_parameters_haziq and haziq_cpp_parameter_event_handler.
The cpp_parameters_haziq is taken from assignment 2 but the code was rewrote in Cpp (before was in Python).

---

## 1. Prerequisites
- ROS2 Humble (or compatible) installed
- Colcon build system
- A ROS2 workspace (e.g., ~/ros2_ws)

---

## 2. Installation

1. Go to your ROS2 workspace `src` folder:

   cd ~/ros2_ws/src

2. Clone or copy the two packages 'cpp_parameters_haziq' and 'haziq_cpp_parameter_event_handler' folder into `src`.

3. Go back to the workspace root and build:

   cd ~/ros2_ws
   colcon build --packages-select cpp_parameters_haziq haziq_cpp_parameter_event_handler

4. Source the setup file:

   source install/setup.bash

---

## 3. Running the Nodes

### Run the node:
In one terminal:
   ros2 run cpp_parameters_haziq cpp_param_haziq

This will publish "Hello world! We only sell 9 Nvidia Graphic".

### Run the parameter event handler:
In another terminal:
   ros2 run haziq_cpp_parameter_event_handler manyparam_node_event_handler 

This will listen to any changes of parameter in the node and updated it in the terminal.

### Change the parameter:
In another terminal:
   ros2 param set /haziq_param_node my_quantity 87
   ros2 param set /haziq_param_node my_item "Radeon Graphic"
   ros2 param set /haziq_param_node my_parameter "Buyer"

---

---

## 5. Notes
- Make sure you always `source install/setup.bash` in each new terminal.
- If you change the `.msg` file, run `colcon build` again.
- Check available parameter:

   ros2 param list


