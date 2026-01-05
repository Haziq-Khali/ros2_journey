#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray


class GothamPerceptionCommander(Node):

    def __init__(self):
        super().__init__('gotham_perception_commander')

        # ---------- SUBSCRIBER ----------
        self.sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # ---------- PUBLISHERS ----------
        # Perception command publisher
        self.cmd_pub = self.create_publisher(
            String,
            '/bot_commands',
            10
        )

        # Head & neck controller publishers
        self.head_pub = self.create_publisher(
            Float64MultiArray,
            '/head_controller/commands',
            10
        )

        self.neck_pub = self.create_publisher(
            Float64MultiArray,
            '/neck_controller/commands',
            10
        )

        # ---------- JOINT STATE ----------
        self.head_pos = 0.0   # radians
        self.neck_pos = 0.0   # radians

        # ---------- LIMITS ----------
        self.HEAD_MIN = -2.356
        self.HEAD_MAX =  2.356
        self.NECK_MIN = -0.21
        self.NECK_MAX =  0.0

        self.STEP = 0.05  # smooth movement

        self.get_logger().info("🦇 Gotham Commander ONLINE")


    def joy_callback(self, msg):

        # =====================================================
        # PERCEPTION MODES
        # =====================================================
        if msg.buttons[3] == 1:   # Y
            self.publish_cmd("detect_object", "🧍 Object Detection")

        elif msg.buttons[0] == 1: # A
            self.publish_cmd("detect_qr", "🔢 QR Detection")

        elif msg.buttons[1] == 1: # B
            self.publish_cmd("detect_text", "📝 Text Detection")

        elif msg.buttons[2] == 1: # X
            self.publish_cmd("detect_color", "🎨 Color Detection")

        # =====================================================
        # HEAD CONTROL
        # =====================================================
        elif msg.buttons[8] == 1:  # LB → head left
            self.head_pos += self.STEP
            self.head_pos = min(self.head_pos, self.HEAD_MAX)
            self.publish_head()

        elif msg.buttons[9] == 1:  # RB → head right
            self.head_pos -= self.STEP
            self.head_pos = max(self.head_pos, self.HEAD_MIN)
            self.publish_head()

        # =====================================================
        # NECK CONTROL
        # =====================================================
        elif msg.buttons[6] == 1:  # BACK → neck down
            self.neck_pos -= self.STEP
            self.neck_pos = max(self.neck_pos, self.NECK_MIN)
            self.publish_neck()

        elif msg.buttons[7] == 1:  # START → neck up
            self.neck_pos += self.STEP
            self.neck_pos = min(self.neck_pos, self.NECK_MAX)
            self.publish_neck()


    # -------------------------------------------------------
    # HELPERS
    # -------------------------------------------------------
    def publish_cmd(self, command, log_msg):
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self.get_logger().info(log_msg)

    def publish_head(self):
        msg = Float64MultiArray()
        msg.data = [self.head_pos]
        self.head_pub.publish(msg)
        self.get_logger().info(f"🧠 Head → {self.head_pos:.2f} rad")

    def publish_neck(self):
        msg = Float64MultiArray()
        msg.data = [self.neck_pos]
        self.neck_pub.publish(msg)
        self.get_logger().info(f"🦒 Neck → {self.neck_pos:.2f} rad")


# =========================================================
# MAIN
# =========================================================
def main():
    rclpy.init()
    node = GothamPerceptionCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

