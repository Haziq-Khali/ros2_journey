#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
import easyocr
import cv2


class EasyOCRNode(Node):

    def __init__(self):
        super().__init__('easyocr_node')

        # Publishers
        self.text_pub = self.create_publisher(String, '/easyocr_text', 10)
        self.loc_pub = self.create_publisher(PointStamped, '/villain_location', 10)

        # Subscriber
        self.create_subscription(
            Image,
            '/patrol_cam/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.reader = easyocr.Reader(['en'], gpu=False)

        self.last_text = ""
        self.get_logger().info("❓ EasyOCR Riddler detector online")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            results = self.reader.readtext(rgb)

            h, w, _ = frame.shape

            for (_, text, conf) in results:
                if conf < 0.8:
                    continue

                normalized = text.strip().lower()

                if normalized == self.last_text:
                    return

                self.last_text = normalized

                self.get_logger().info(
                    f"📖 OCR detected: '{text}' (conf={conf:.2f})"
                )

                self.text_pub.publish(String(data=text))

                # ===== Riddler trigger =====
                if normalized == "riddle me this":
                    loc = PointStamped()
                    loc.header = msg.header
                    loc.point.x = w / 2.0
                    loc.point.y = h / 2.0
                    loc.point.z = 0.0

                    self.loc_pub.publish(loc)

                    self.get_logger().info(
                        f"❓ Riddler located at image center x={loc.point.x:.1f}, y={loc.point.y:.1f}"
                    )

        except Exception as e:
            self.get_logger().error(f"OCR error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = EasyOCRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

