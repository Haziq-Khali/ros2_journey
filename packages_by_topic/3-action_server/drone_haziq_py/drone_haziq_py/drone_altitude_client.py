import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_haziq_interfaces.action import DroneAltitude  # change to your package

class DroneAltitudeClient(Node):
    def __init__(self):
        super().__init__('drone_altitude_client')
        self._client = ActionClient(self, DroneAltitude, 'drone_altitude')

    def send_goal(self, target_altitude):
        self.get_logger().info('Connecting to action server...')
        self._client.wait_for_server()

        goal_msg = DroneAltitude.Goal()
        goal_msg.target_altitude = float(target_altitude)

        self.get_logger().info(f'Sending goal: target altitude = {target_altitude:.2f} m')
        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('❌ Goal rejected')
            return

        self.get_logger().info('✅ Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        altitude = feedback_msg.feedback.current_altitude
        self.get_logger().info(f'📈 Current Altitude: {altitude:.2f} m')

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'🎯 Final Altitude: {result.final_altitude:.2f} m (Target Reached)')
        else:
            self.get_logger().info(f'⚠️ Could not reach target')


def main(args=None):
    rclpy.init(args=args)
    node = DroneAltitudeClient()
    node.send_goal(5.0)  # set target altitude here
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
