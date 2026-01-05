#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):

    def __init__(self):
        super().__init__('color_detector')

        self.bridge = CvBridge()

        # Semantic outputs
        self.color_pub = self.create_publisher(String, '/color_detected', 10)
        self.loc_pub = self.create_publisher(PointStamped, '/villain_location', 10)

        # Image I/O
        self.create_subscription(
            Image,
            '/patrol_cam/image_raw',
            self.image_callback,
            10
        )
        self.image_pub = self.create_publisher(Image, 'annotated', 10)

        self.get_logger().info("🎨 Color Detector ready (Pink wall = Joker Zone)")

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        ns = self.get_namespace().replace('/', '')

        # ================= COLOR DEFINITIONS =================
        color_ranges = {
            "red":    ([0,120,70],  [10,255,255], [170,120,70], [180,255,255], "RED",    (0,0,255)),
            "green":  ([35,50,70],  [85,255,255], None,        None,          "GREEN",  (0,255,0)),
            "blue":   ([90,50,50],  [130,255,255],None,        None,          "BLUE",   (255,0,0)),
            "purple": ([124,45,100],[140,255,255],None,        None,          "PURPLE", (128,0,128)),
            "orange": ([10,100,100],[25,255,255], None,        None,          "ORANGE", (0,165,255)),
            "pink":   ([145,50,150],[170,255,255],None,        None,          "PINK",   (255,0,255))
        }

        if ns not in color_ranges:
            return

        lower1, upper1, lower2, upper2, label, box_color = color_ranges[ns]

        # ================= MASK =================
        mask = cv2.inRange(hsv, np.array(lower1), np.array(upper1))
        if lower2 is not None:
            mask |= cv2.inRange(hsv, np.array(lower2), np.array(upper2))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        joker_zone_detected = False

        # ================= PROCESS CONTOURS =================
        for cnt in contours:
            area = cv2.contourArea(cnt)
            x, y, w, h = cv2.boundingRect(cnt)

            # ---------- Visual annotation (ALL colors) ----------
            if area > 500:
                cv2.rectangle(frame, (x, y), (x+w, y+h), box_color, 2)
                cv2.putText(
                    frame,
                    label,
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    box_color,
                    2
                )

            # ---------- Joker Zone logic (ONLY PINK + LARGE) ----------
            if ns == "pink" and area > 150000:
                joker_zone_detected = True

                cv2.rectangle(frame, (x, y), (x+w, y+h), box_color, 4)
                cv2.putText(
                    frame,
                    "JOKER'S ZONE",
                    (x, y - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    box_color,
                    3
                )

                # ---------- Approx location ----------
                img_h, img_w, _ = frame.shape
                cx = x + w / 2.0
                cy = y + h / 2.0

                loc = PointStamped()
                loc.header.stamp = self.get_clock().now().to_msg()
                loc.header.frame_id = "camera_frame"
                loc.point.x = (cx - img_w / 2.0) / (img_w / 2.0)
                loc.point.y = (cy - img_h / 2.0) / (img_h / 2.0)
                loc.point.z = 0.0

                self.loc_pub.publish(loc)

        # ================= SEMANTIC OUTPUT =================
        if joker_zone_detected:
            self.color_pub.publish(String(data="JOKER_ZONE"))
            self.get_logger().info("🟣 JOKER ZONE detected + location published")

        # Publish annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        annotated_msg.header = msg.header
        self.image_pub.publish(annotated_msg)

def main():
    rclpy.init()
    node = ColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

