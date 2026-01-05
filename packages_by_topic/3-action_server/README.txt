=======================================
drone_haziq - ROS2 Action Server
=======================================

This package demonstrates how to create a **ROS2 Action Server** 
that simulates controlling a drone’s altitude in Python. 

It uses a custom action definition (`DroneAltitude.action`) 
and supports feedback, canceling, and single-goal execution.

---

## 1. Installation

1. Go to your ROS2 workspace 'src' folder:
    cd ~/ros2_ws/src

2. Copy both packages:
    - action_haziq_interfaces
    - drone_haziq_py

3. Build the packages:
    cd ~/ros2_ws
    colcon build --packages-select action_haziq_interfaces drone_haziq_py

4. Source your environment:
    source ~/.bashrc

---

## 2. Action Interface (DroneAltitude.action)

Located in:
    action_haziq_interfaces/action/DroneAltitude.action

### Content:
# Goal
float64 target_altitude
---
# Result
bool success
float64 final_altitude
---
# Feedback
float64 current_altitude

This defines the data exchanged between the client and the server
during an altitude control operation.

---

## 3. Running the Node

### Start the Action Server:
    $ ros2 run drone_haziq_py drone_altitude_server

When started, the node prints:
    🚁 Drone Altitude Server Ready

The node will accept one goal at a time and publish feedback
until the drone reaches the target altitude or is canceled.

---

## 4. Sending Goals (Clients)

You can send a goal using the command-line interface:

### Example:
    $ ros2 action send_goal /drone_altitude action_haziq_interfaces/action/DroneAltitude "{target_altitude: 10.0}"

Expected output:
    - Server starts climbing or descending
    - Publishes feedback (altitude progress)
    - Returns success when target is reached

### Example Output:
    📡 Goal received: target altitude = 10.00 m  
    🪶 Altitude: 0.50 m  
    🪶 Altitude: 1.00 m  
    ...  
    ✅ Target reached at 10.00 m  

---

## 5. Canceling a Goal

You can cancel an ongoing goal:

    $ ros2 service call /drone_altitude/_action/cancel_goal action_msgs/srv/CancelGoal {}

If successful, the server prints:
    ⚠️ Cancel request received...
    🛑 Goal canceled at altitude X.XX m

---

## 6. Node Features

- Supports **feedback** and **cancel** requests  
- Only one goal runs at a time (new goals are rejected)  
- Uses `rclpy.spin_once()` for timing (non-async loop)  
- Clean shutdown on Ctrl + C  

---

## 7. Package Summary

Package Name:  drone_haziq_py  
Main Node:     drone_altitude_server.py  
Action Used:   DroneAltitude  
Depends On:    action_haziq_interfaces  

---

================
     SUMMARY
================

Start the Action Server:
    $ ros2 run drone_haziq_py drone_altitude_server

Send goal from terminal:
    $ ros2 action send_goal /drone_altitude action_haziq_interfaces/action/DroneAltitude "{target_altitude: 8.0}"

Cancel goal:
    $ ros2 service call /drone_altitude/_action/cancel_goal action_msgs/srv/CancelGoal {}

Expected behavior:
    - Drone climbs/descends step-by-step
    - Feedback published continuously
    - Cancels smoothly if requested
    - Succeeds when target reached

---


