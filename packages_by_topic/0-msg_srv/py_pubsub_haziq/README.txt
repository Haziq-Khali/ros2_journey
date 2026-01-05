===========================
py_pubsub_haziq - ROS2 Tutorial
===========================

This package demonstrates a simple Publisher and Subscriber in Python
using a custom message (Order.msg).

---

## 1. Prerequisites
- ROS2 Humble (or compatible) installed
- Colcon build system
- A ROS2 workspace (e.g., ~/ros2_ws)

---

## 2. Installation

1. Go to your ROS2 workspace `src` folder:

   cd ~/ros2_ws/src

2. Clone or copy the `py_pubsub_haziq` folder into `src`.

   Your workspace should look like:
   ~/ros2_ws/src/py_pubsub_haziq/

3. Go back to the workspace root and build:

   cd ~/ros2_ws
   colcon build --packages-select py_pubsub_haziq

4. Source the setup file:

   source install/setup.bash

---

## 3. Running the Nodes

### Start the Publisher
In one terminal:
   ros2 run py_pubsub_haziq customer

This will publish `Order` messages on the topic `/food_order`.

### Start the Subscriber
In another terminal:
   ros2 run py_pubsub_haziq waiter

This will listen to the `/food_order` topic and print out received messages.

---

## 4. Sending Your Own Messages (Optional)

Instead of running the publisher, you can use ROS2 CLI to send a message:

   ros2 topic pub /food_order tutorial_interfaces_haziq/msg/Order \
   "{customer_name: 'Imran', order: 'Ikan keli bakar', quantity: 2}"

---

## 5. Notes
- Make sure you always `source install/setup.bash` in each new terminal.
- If you change the `.msg` file, run `colcon build` again.
- Check available topics:

   ros2 topic list

- Check message type of a topic:

   ros2 topic info /item

