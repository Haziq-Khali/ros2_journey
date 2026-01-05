#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math

class WallAvoidance(Node):

    def __init__(self):
        super().__init__('wall_avoidance_node')

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/diff_drive_base_controller/cmd_vel_unstamped', 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.autonomy_sub = self.create_subscription(
            Bool,
            '/autonomy_enabled',
            self.autonomy_callback,
            10
        )

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Internal state
        self.scan_data = None
        self.autonomy_enabled = True   # START ENABLED
        self.safe_distance = 1.4        # meters

        self.get_logger().info("Wall avoidance ready. Waiting for autonomy command.")

    # ================= CALLBACKS =================

    def autonomy_callback(self, msg):
        self.autonomy_enabled = msg.data
        state = "ENABLED" if msg.data else "DISABLED"
        self.get_logger().info(f"Autonomy {state}")

    def scan_callback(self, msg):
        self.scan_data = msg

    # ================= UTILS =================

    def get_range(self, start_angle, end_angle):
        ranges = []
        for i, r in enumerate(self.scan_data.ranges):
            angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
            if start_angle <= angle <= end_angle:
                if not math.isinf(r) and not math.isnan(r):
                    ranges.append(r)
        return min(ranges) if ranges else float('inf')

    # ================= CONTROL =================

    def control_loop(self):
        # DO NOTHING if autonomy disabled
        if not self.autonomy_enabled:
            return

        if self.scan_data is None:
            return

        front = self.get_range(-0.2, 0.2)
        left  = self.get_range(0.5, 1.0)
        right = self.get_range(-1.0, -0.5)

        cmd = Twist()

        if front > self.safe_distance:
            cmd.linear.x = 0.6
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = -1.0 if left > right else 1.0

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = WallAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

