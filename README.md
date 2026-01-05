=================================
	ROS2 Journey Startup 
=================================

This repository contain many packages and my journal sheet. To begin the journey, the following prerequisites is necessary.
Inside the packages_by_topic folder includes many sub topic that covered during the learning. Each of the sub topic package have their respective readme.txt to help understanding the packages.
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

