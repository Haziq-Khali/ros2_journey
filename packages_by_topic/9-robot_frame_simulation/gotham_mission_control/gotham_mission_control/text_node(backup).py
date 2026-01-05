#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import easyocr
import cv2


class EasyOCRNode(Node):
    def __init__(self):
        super().__init__('easyocr_node')

        # Create a publisher for recognized text
        self.publisher_ = self.create_publisher(String, 'easyocr_text', 10)

        # Subscribe to a camera topic
        self.subscription = self.create_subscription(
            Image,
            '/patrol_cam/image_raw',  # Change to your camera topic
            self.image_callback,10)

        # Initialize CV bridge and EasyOCR reader
        self.bridge = CvBridge()
        self.reader = easyocr.Reader(['en'])  # You can add other languages
        self.get_logger().info('EasyOCR Node has started.')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert image to RGB (EasyOCR expects RGB)
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Run OCR
            results = self.reader.readtext(rgb_image)

            for (bbox, text, prob) in results:
                if prob > 0.8:  # Filter low-confidence results
                    self.get_logger().info(f"Detected: {text} ({prob:.2f})")
                    self.publisher_.publish(String(data=text))

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = EasyOCRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


