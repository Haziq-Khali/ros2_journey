=======================================
order_haziq_base - ROS2 Plugins & Parameters
=======================================

This package demonstrates how to use ROS2 **pluginlib** with parameters. 
We define a base interface (`order_interfaces.hpp`) for two types of plugins:
  1. OrderSet plugins (e.g. different computer specifications)
  2. OrderCalculator plugins (e.g. with/without discounts)

The package shows how to load these plugins dynamically at runtime, 
and how to control them via ROS2 parameters (using CLI or rqt).

## 1. Installation

1. Go to your ROS2 workspace 'src' folder:
    cd ~/ros2_ws/src

2. Copy the order_haziq_base package folder into the 'src'.

3. Go back to the workspace and build the package:
    cd ~/ros2_ws
    colcon build --packages-select order_haziq_base

4. Restart the terminal by opening a new terminal or running:
    source ~/.bashrc

---

## 2. Running the node (focus on computer sets and calculators)

### In one terminal run the node:
    $ ros2 run order_haziq_base order_node
    			or
    $ ros2 run order_haziq_base order_node_latch

By default this will:
- Load **SetA** (computer with NVIDIA RTX 3060)
- Use **BasicCalculator** (no discount)
- Quantity = 1

### In another terminal set parameters using CLI:
    $ ros2 param set /order_node order_set SetB
    $ ros2 param set /order_node calculator DiscountCalculator
    $ ros2 param set /order_node quantity 3

Each time a parameter is updated, the node will immediately recompute 
and print the new total order price.

---

We can also use **rqt** to change the parameters dynamically.

### Run the command in another terminal:
    $ rqt

---









================
    SUMMARY
================

Run code that continuosly publish:
$ ros2 run order_haziq_base order_node

Run code only publish after set parameter or plugins:
$ ros2 run order_haziq_base order_node_latch


Set parameters:
    1. Calculator type --> $ ros2 param set /order_node calculator DiscountCalculator
    2. Set order type ---> $ ros2 param set /order_node order_set SetB
    3. Quantity of set --> $ ros2 param set /order_node quantity 8

Set parameters using rqt GUI:
    $ rqt
    
    
