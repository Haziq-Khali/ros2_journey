#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from action_haziq_interfaces.action import DroneAltitude


class DroneAltitudeServer(Node):
    def __init__(self):
        super().__init__('drone_altitude_server')

        self._action_server = ActionServer(
            self,
            DroneAltitude,
            'drone_altitude',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            goal_callback=self.goal_callback
        )

        self.current_altitude = 0.0
        self.active_goal_handle = None
        self.stop_current_goal = False

        self.get_logger().info('🚁 Drone Altitude Action Server Ready')

    # Handle new goals (abort previous)
    def goal_callback(self, goal_request):
        if self.active_goal_handle and self.active_goal_handle.is_active:
            self.get_logger().warn('⚠️ Aborting previous goal to start new one!')
            self.stop_current_goal = True
            self.active_goal_handle.abort()
        self.stop_current_goal = False
        return GoalResponse.ACCEPT

    # Handle cancel request
    def cancel_callback(self, goal_handle):
        self.get_logger().info('⚠️ Cancel request received...')
        self.stop_current_goal = True
        return CancelResponse.ACCEPT

    # Execute the goal
    async def execute_callback(self, goal_handle):
        self.active_goal_handle = goal_handle
        target = goal_handle.request.target_altitude
        self.get_logger().info(f'📡 Executing goal: {target:.2f} m')

        feedback = DroneAltitude.Feedback()
        result = DroneAltitude.Result()

        step = 0.5   # meters per step
        rate = 0.5   # seconds delay between steps

        while rclpy.ok() and not self.stop_current_goal and abs(self.current_altitude - target) > 0.1:
            # Check if canceled by user
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn(f'🛑 Goal canceled at {self.current_altitude:.2f} m')
                result.success = False
                result.final_altitude = self.current_altitude
                return result

            # Move toward target
            if self.current_altitude < target:
                self.current_altitude += step
            else:
                self.current_altitude -= step

            # Send feedback
            feedback.current_altitude = self.current_altitude
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'🪶 Altitude: {self.current_altitude:.2f} m')

            ##time.sleep(rate)
            rclpy.spin_once(self, timeout_sec=rate)

        if self.stop_current_goal:
            self.get_logger().warn(f'🚫 Stopping current goal at {self.current_altitude:.2f} m')
            result.success = False
            result.final_altitude = self.current_altitude
            goal_handle.abort()
            return result

        # Success
        goal_handle.succeed()
        result.success = True
        result.final_altitude = self.current_altitude
        self.get_logger().info(f'✅ Target reached at {self.current_altitude:.2f} m')

        return result


def main(args=None):
    rclpy.init(args=args)
    node = DroneAltitudeServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
