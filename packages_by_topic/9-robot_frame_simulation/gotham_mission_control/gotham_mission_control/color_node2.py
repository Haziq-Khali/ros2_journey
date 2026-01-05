#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        # Subscriber to camera image
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',   # change this topic to match your camera
            self.image_callback,
            10)

        # Publisher for annotated image
        self.publisher_ = self.create_publisher(Image, 'annotated', 10)

        # CV Bridge
        self.bridge = CvBridge()

        self.get_logger().info("Color Detector Node started.")

    def image_callback(self, msg: Image):
        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Namespace to decide color
        ns = self.get_namespace().replace('/', '')
        
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define red color range (two ranges because red wraps around HSV hue)
        if ns == "red":
            lower1 = np.array([0, 120, 70])
            upper1 = np.array([10, 255, 255])
            lower2 = np.array([170, 120, 70])
            upper2 = np.array([180, 255, 255])
            color_text = "RED"
            box_color = (0, 0, 255)
            
        elif ns == "green":
            lower1 = np.array([35, 50, 70])
            upper1 = np.array([85, 255, 255])
            lower2 = None
            color_text = "GREEN"
            box_color = (0, 255, 0)

        elif ns == "blue":
            lower1 = np.array([90, 50, 50])
            upper1 = np.array([130, 255, 255])
            lower2 = None
            color_text = "BLUE"
            box_color = (255, 0, 0)

        elif ns == "purple":
            lower1 = np.array([125, 80, 60])
            upper1 = np.array([150, 255, 255])
            lower2 = None
            color_text = "JOKER's ZONE"
            box_color = (255, 0, 255)  # purple in BGR

        elif ns == "orange":
            lower1 = np.array([10, 120, 70])
            upper1 = np.array([25, 255, 255])
            lower2 = None
            color_text = "JOKER's Henchmen"
            box_color = (0, 165, 255)  # orange in BGR

            
        else:
            return

        # Create masks
        mask = cv2.inRange(hsv, lower1, upper1)
        if lower2 is not None:
            mask |= cv2.inRange(hsv, lower2, upper2)

        # Find contours of red regions
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:  # filter noise
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.putText(frame, color_text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.9, (0, 0, 255), 2)

        # Publish annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        annotated_msg.header = msg.header
        self.publisher_.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
