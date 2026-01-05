#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from pyzbar import pyzbar
import cv2

class QRCodeDetector(Node):

    def __init__(self):
        super().__init__('qr_code_detector')

        # ===== STATE =====
        self.done_published = False

        # ===== SUB =====
        self.create_subscription(
            Image,
            '/patrol_cam/image_raw',
            self.image_callback,
            10
        )

        # ===== PUB =====
        self.qr_pub = self.create_publisher(String, '/qr_detected', 10)
        self.marker_pub = self.create_publisher(Marker, '/qr_marker', 10)

        self.bridge = CvBridge()
        self.get_logger().info("📸 QR Code Detector ready")

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        qrcodes = pyzbar.decode(frame)

        for qr in qrcodes:
            qr_text = qr.data.decode('utf-8').strip()
            x, y, w, h = qr.rect

            self.get_logger().info(f"📦 QR detected: {qr_text}")

            # ========== RViz Marker ==========
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = msg.header.frame_id or "camera_link"
            marker.ns = "qr_codes"
            marker.id = hash(qr_text) % 1000
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD

            marker.pose.position.x = (x + w / 2) * 0.001
            marker.pose.position.y = (y + h / 2) * 0.001
            marker.pose.position.z = 1.0
            marker.pose.orientation.w = 1.0

            marker.scale.z = 0.25
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.text = qr_text

            self.marker_pub.publish(marker)

            # ========== SEMANTIC SIGNAL ==========
            if qr_text == "DONE" and not self.done_published:
                self.qr_pub.publish(String(data="DONE"))
                self.done_published = True
                self.get_logger().info("✅ DONE QR published to mission manager")

def main():
    rclpy.init()
    node = QRCodeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

