==============================
py_srvcli_haziq - ROS2 Service/Client
==============================

This package demonstrates how to use a Service and Client in Python
with a custom service definition (SetOrder.srv).

---

## 1. Prerequisites
- ROS2 Humble (or compatible) installed
- Colcon build system
- A ROS2 workspace (e.g., ~/ros2_ws)
- The `tutorial_interfaces_haziq` package must be built (contains Order.msg and SetOrder.srv)

---

## 2. Installation

1. Go to your ROS2 workspace `src` folder:

   cd ~/ros2_ws/src

2. Clone or copy the `py_srvcli_haziq` folder into `src`.

   Your workspace should look like:
   ~/ros2_ws/src/py_srvcli_haziq/

3. Go back to the workspace root and build:

   cd ~/ros2_ws
   colcon build --packages-select py_srvcli_haziq

4. Source the setup file:

   source install/setup.bash

---

## 3. Running the Nodes

### Start the Service (Server)
In one terminal:
   ros2 run py_srvcli_haziq service

This node provides the `set_order` service using `SetOrder.srv`.

### Call the Client
In another terminal:
   ros2 run py_srvcli_haziq client Imran "Ikan keli bakar" 2

This will send a request to the service with:
- name = "Imran"
- set = "Ikan keli bakar"
- quantity = 2

The service will respond with a confirmation message and total items.

---

## 4. Testing with ROS2 CLI (Optional)

You can also call the service manually using the ROS2 CLI:

   ros2 service call /set_order tutorial_interfaces_haziq/srv/SetOrder \
   "{name: 'Imran', set: 'Ikan keli bakar', quantity: 2}"

---

## 5. Notes
- Always run `source install/setup.bash` in each new terminal.
- Make sure the service server is running **before** calling the client.
- If you change the `.srv` file, rebuild with `colcon build`.
- To see all services:

   ros2 service list

- To check the type of a service:

   ros2 service type /set_order

