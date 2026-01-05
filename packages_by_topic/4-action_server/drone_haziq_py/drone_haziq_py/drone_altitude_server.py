import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from action_haziq_interfaces.action import DroneAltitude  

import time

class DroneAltitudeServer(Node):
    def __init__(self):
        super().__init__('drone_altitude_server')      ## ROS2 service list name
        self._action_server = ActionServer(
            self,
            DroneAltitude,
            'drone_altitude',                  ## ROS2 action list name
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,   ## ENABLE cancel support
            #goal_callback=self.goal_callback,       ## ENABLE focus on a goal
            )
        
        self.current_altitude = 0.0                                     ##STORE current altitude
        self.get_logger().info('Drone Altitude Server Ready')

    async def execute_callback(self, goal_handle):
        target = goal_handle.request.target_altitude
        self.get_logger().info(f'Goal received: target altitude = {target:.2f} m')

        feedback = DroneAltitude.Feedback()
        ##self.current_altitude = 0.0                              ##REMOVE to store current alt.
        step = 0.5  # meters per iteration
        rate = 0.5  # seconds

        while abs(self.current_altitude - target) > 0.1:
            ## 🛑 Check if user requested cancel
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn(f'Goal canceled at altitude {self.current_altitude:.2f} m')

                result = DroneAltitude.Result()
                result.success = False
                result.final_altitude = self.current_altitude
                return result

            # Simulate climb or descent
            if self.current_altitude < target:
                self.current_altitude += step
            else:
                self.current_altitude -= step

            # Feedback
            feedback.current_altitude = self.current_altitude
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'Altitude: {self.current_altitude:.2f} m')
            
            ##time.sleep(rate)
            rclpy.spin_once(self, timeout_sec=rate)

        goal_handle.succeed()

        result = DroneAltitude.Result()
        result.success = True
        result.final_altitude = self.current_altitude
        self.get_logger().info(f'Target reached at {self.current_altitude:.2f} m')

        return result

    ## 🧩 Handle cancel requests
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received, stopping altitude change...')
        return CancelResponse.ACCEPT
    


def main(args=None):
    rclpy.init(args=args)
    node = DroneAltitudeServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
