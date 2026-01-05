=======================================
order_haziq_plugins - ROS2 Plugin Implementations
=======================================

This package provides the actual implementations of plugins for the 
`order_haziq_base` package. It defines two types of plugins:

  1. OrderSet plugins (different computer specification sets)
  2. OrderCalculator plugins (different calculation strategies)

These plugins can be dynamically loaded at runtime using `pluginlib`, 
based on ROS2 parameters.

---

## 1. Installation

1. Go to your ROS2 workspace 'src' folder:
    cd ~/ros2_ws/src

2. Copy the order_haziq_plugins package folder into the 'src'.

3. Go back to the workspace and build the package:
    cd ~/ros2_ws
    colcon build --packages-select order_haziq_plugins

4. Restart the terminal by opening a new terminal or running:
    source ~/.bashrc

---

## 2. Plugin Types

### A) OrderSet Plugins (Computer Configurations)

- **SetA**: Gaming PC with NVIDIA RTX 3060 GPU
- **SetB**: Gaming PC with AMD Radeon GPU (same performance tier as SetA)
- **SetC**: Budget PC for low-end gaming

Each set provides a list of `Item`s (with name, price, and quantity).

---

### B) OrderCalculator Plugins (Price Calculation)

- **BasicCalculator**  
  Simple total: sum of (price × quantity)

- **DiscountCalculator**  
  Applies a 10% discount to the total

---

## 3. Usage with Base Package

The plugins are not meant to be run directly.  
They are loaded by the `order_node` from the `order_haziq_base` package.

Example workflow:

1. Run the base node:
    $ ros2 run order_haziq_base order_node
        		or
    $ ros2 run order_haziq_base order_node_latch

2. Change the parameters to switch plugins:

    $ ros2 param set /order_node order_set SetB
    $ ros2 param set /order_node calculator DiscountCalculator
    $ ros2 param set /order_node quantity 2

Expected output:

    [INFO] Order: 2 x SetB | Calculator: Discount Calculator (10% off) | Total = 4410.00

---

## 4. Plugin XML Registration

This package provides a `order_plugins.xml` file, which registers 
the available plugins with `pluginlib`. This file is exported and 
installed automatically by CMake.

The XML describes which classes can be loaded at runtime by 
the base package.

---

## 5. Summary

- `order_haziq_base` = defines the **interfaces** and provides the **node**
- `order_haziq_plugins` = implements the **plugins** (computer sets + calculators)

Together, they demonstrate how to build a modular ROS2 system 
using **pluginlib** and **parameters**.


