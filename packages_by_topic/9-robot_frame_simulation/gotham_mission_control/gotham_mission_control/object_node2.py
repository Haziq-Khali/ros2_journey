#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from vision_msgs.msg import BoundingBox2D, Pose2D
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import torch


class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo_node')

        # ===== SUB =====
        self.create_subscription(
            Image,
            "/patrol_cam/image_raw",
            self.image_callback,
            10
        )

        # ===== PUB =====
        self.det_pub = self.create_publisher(Detection2DArray, "/yolo_detections", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/yolo_markers", 10)
        self.image_pub = self.create_publisher(Image, "/yolo/annotated", 10)
        self.loc_pub = self.create_publisher(PointStamped, "/villain_location", 10)

        self.bridge = CvBridge()

        # ===== YOLO =====
        self.model = torch.hub.load("ultralytics/yolov5", "yolov5s", pretrained=True)
        self.model.conf = 0.4

        self.get_logger().info("🐧 YOLO Penguin detector online")

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(frame)

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        markers = MarkerArray()
        annotated = frame.copy()

        for i, det in enumerate(results.xyxy[0]):
            x1, y1, x2, y2, conf, cls = det.tolist()
            label = self.model.names[int(cls)].lower()

            # Draw
            cv2.rectangle(
                annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2
            )
            cv2.putText(
                annotated,
                f"{label} ({conf:.2f})",
                (int(x1), int(y1) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                2
            )

            # ===== Detection2D =====
            detection = Detection2D()
            detection.header = msg.header

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = label
            hyp.hypothesis.score = float(conf)
            detection.results.append(hyp)

            bbox = BoundingBox2D()
            bbox.center = Pose2D()
            bbox.center.position.x = float((x1 + x2) / 2.0)
            bbox.center.position.y = float((y1 + y2) / 2.0)
            bbox.size_x = float(x2 - x1)
            bbox.size_y = float(y2 - y1)
            detection.bbox = bbox

            detections_msg.detections.append(detection)

            # ===== Penguin trigger =====
            if label in ["bird", "vase"]:
                loc = PointStamped()
                loc.header = msg.header
                loc.point.x = bbox.center.position.x
                loc.point.y = bbox.center.position.y
                loc.point.z = 0.0

                self.loc_pub.publish(loc)

                self.get_logger().info(
                    f"🐧 Penguin located at image x={loc.point.x:.1f}, y={loc.point.y:.1f}"
                )

            # ===== Marker =====
            marker = Marker()
            marker.header = msg.header
            marker.ns = "yolo"
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = 0.5
            marker.pose.position.y = bbox.center.position.x / 200.0
            marker.pose.position.z = bbox.center.position.y / 200.0
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.text = label
            markers.markers.append(marker)

        # Publish
        img_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
        img_msg.header = msg.header

        self.image_pub.publish(img_msg)
        self.det_pub.publish(detections_msg)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

