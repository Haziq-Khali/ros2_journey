#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2DArray

from rclpy.action import ActionServer
from action_tutorials_interfaces.action import PatrolMission


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        # ================= FSM STATE =================
        self.current_mission = "JOKER"
        self.waiting_for_qr = False
        self.waiting_for_action = False

        # ================= PUBLISHERS =================
        self.autonomy_pub = self.create_publisher(Bool, '/autonomy_enabled', 10)
        self.event_pub = self.create_publisher(String, '/mission_event', 10)

        # ================= SUBSCRIBERS =================
        self.create_subscription(
            String, '/color_detected', self.color_callback, 10
        )

        self.create_subscription(
            Detection2DArray, '/yolo_detections', self.object_callback, 10
        )

        self.create_subscription(
            String, '/easyocr_text', self.text_callback, 10
        )

        self.create_subscription(
            String, '/qr_detected', self.qr_callback, 10
        )

        self.create_subscription(
            PointStamped, '/villain_location', self.location_callback, 10
        )

        # ================= ACTION SERVER =================
        self.action_server = ActionServer(
            self,
            PatrolMission,
            'patrol_mission',
            self.execute_action
        )

        self.get_logger().info("🦇 Mission start. Lets save Gotham, DC Forever.")
        self.start_mission("JOKER")

    # ==================================================
    #                     FSM
    # ==================================================

    def start_mission(self, mission):
        self.current_mission = mission
        self.waiting_for_qr = False
        self.waiting_for_action = False

        self.get_logger().info(f"🚨 MISSION STARTED: {mission}")
        self.autonomy_pub.publish(Bool(data=True))

    # ==================================================
    #                 JOKER (COLOR)
    # ==================================================

    def color_callback(self, msg):
        if self.current_mission != "JOKER":
            return

        if msg.data == "JOKER_ZONE":
            self.get_logger().info("🟣 Joker zone detected → stopping robot")
            self.autonomy_pub.publish(Bool(data=False))
            self.waiting_for_qr = True

    # ==================================================
    #               PENGUIN (YOLO)
    # ==================================================

    def object_callback(self, msg: Detection2DArray):
        if self.current_mission != "PENGUIN":
            return

        for det in msg.detections:
            class_id = det.results[0].hypothesis.class_id.lower()

            if class_id in ["bird", "vase"]:
                self.get_logger().info("🐧 Penguin zone detected (bird/vase)")
                self.autonomy_pub.publish(Bool(data=False))
                self.waiting_for_qr = True
                return

    # ==================================================
    #               RIDDLER (OCR)
    # ==================================================

    def text_callback(self, msg):
        if self.current_mission != "RIDDLER":
            return

        text = msg.data.strip().lower()

        if text == "riddle me this":
            self.get_logger().info("❓ Riddler zone detected via OCR")
            self.autonomy_pub.publish(Bool(data=False))
            self.waiting_for_qr = True

    # ==================================================
    #                    QR DONE
    # ==================================================

    def qr_callback(self, msg):
        if msg.data.strip().upper() != "DONE":
            return

        if not self.waiting_for_qr:
            return

        event = String()
        event.data = f"{self.current_mission}_DONE"
        self.event_pub.publish(event)

        self.get_logger().info(f"✅ {self.current_mission} mission completed")

        self.waiting_for_qr = False
        self.waiting_for_action = True  #This was True

    # ==================================================
    #               LOCATION LOGGING
    # ==================================================

    def location_callback(self, msg: PointStamped):
        if self.current_mission == "JOKER":
            self.get_logger().info(
                f"🟣 JOKER located at x={msg.point.x:.2f}, y={msg.point.y:.2f}"
            )

        elif self.current_mission == "PENGUIN":
            self.get_logger().info(
                f"🐧 PENGUIN located at x={msg.point.x:.2f}, y={msg.point.y:.2f}"
            )

        elif self.current_mission == "RIDDLER":
            self.get_logger().info(
                f"❓ RIDDLER located at x={msg.point.x:.2f}, y={msg.point.y:.2f}"
            )

    # ==================================================
    #                 ACTION SERVER
    # ==================================================

    async def execute_action(self, goal_handle):

        if not self.waiting_for_action:
            goal_handle.abort()
            return PatrolMission.Result(
                success=False,
                message="No mission waiting"
            )

        self.get_logger().info("▶ Action received → continuing patrol")

        if self.current_mission == "JOKER":
            self.start_mission("PENGUIN")

        elif self.current_mission == "PENGUIN":
            self.start_mission("RIDDLER")

        elif self.current_mission == "RIDDLER":
            self.get_logger().info("🏁 ALL MISSIONS COMPLETED")
            self.autonomy_pub.publish(Bool(data=False))
            self.current_mission = "COMPLETED"

        self.waiting_for_action = False
        goal_handle.succeed()

        return PatrolMission.Result(
            success=True,
            message="Mission advanced"
        )


def main():
    rclpy.init()
    node = MissionManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

