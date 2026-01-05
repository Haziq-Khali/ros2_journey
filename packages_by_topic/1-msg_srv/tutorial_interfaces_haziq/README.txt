=====================================
tutorial_interfaces_haziq - ROS2 Interfaces
=====================================

This package defines custom ROS2 message and service types
for use in publisher/subscriber and service/client examples.

---

## 1. Prerequisites
- ROS2 Humble (or compatible) installed
- Colcon build system
- A ROS2 workspace (e.g., ~/ros2_ws)

---

## 2. Message Definitions

### msg/Order.msg
Custom message for food orders:
    string customer_name
    string order
    int64 quantity

### srv/SetOrder.srv
Custom service definition:
    # Request
    string name
    string set
    int64 quantity
    ---
    # Response
    string order
    int64 total_item

---

## 3. Installation

1. Go to your ROS2 workspace `src` folder:

   cd ~/ros2_ws/src

2. Clone or copy the `tutorial_interfaces_haziq` folder into `src`.

   Your workspace should look like:
   ~/ros2_ws/src/tutorial_interfaces_haziq/

3. Go back to the workspace root and build:

   cd ~/ros2_ws
   colcon build --packages-select tutorial_interfaces_haziq

4. Source the setup file:

   source install/setup.bash

---

## 4. Usage in Other Packages

Other packages (e.g., py_pubsub or py_srvcli_haziq) can use these
interfaces by adding this dependency in their `package.xml`:

   <depend>tutorial_interfaces_haziq</depend>

And updating `CMakeLists.txt` or `setup.py` to include the dependency.

---

## 5. Testing the Interfaces

After building, you can test your custom types directly in the terminal.

### Publish an Order message:
   ros2 topic pub /item tutorial_interfaces_haziq/msg/Order \
   "{customer_name: 'Imran', order: 'Ikan keli bakar', quantity: 2}"

### Call the SetOrder service:
   ros2 service call /set_order tutorial_interfaces_haziq/srv/SetOrder \
   "{name: 'Haziq', set: 'Nasi Lemak', quantity: 3}"

---

## 6. Notes
- Always rebuild with `colcon build` after modifying `.msg` or `.srv` files.
- Always `source install/setup.bash` in each new terminal.
- To check available custom message types:

   ros2 interface list | grep tutorial_interfaces_haziq

- To show the details of a type:

   ros2 interface show tutorial_interfaces_haziq/msg/Order
   ros2 interface show tutorial_interfaces_haziq/srv/SetOrder

