#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import base64
import numpy as np
from ultralytics import YOLO
import time
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.sub = self.create_subscription(Image, 'image_raw', self.listener_callback, 10)
        self.cv_bridge = CvBridge()
        self.cmd_pub = self.create_publisher(String, 'motor_cmd', 10)

        device = "cpu"
        self.get_logger().info(f"Using device: {device}")

        try:
            self.model = YOLO("/home/sheldonvt/workspace/src/pkg/camera_pkg/best_test_ok.pt").to(device)
            self.get_logger().info("YOLO model loaded successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            self.model = None

    def listener_callback(self, msg):
        self.get_logger().info("Receiving video frame")
        image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        # results = self.model(image, conf=0.7)
        # annotated_frame = results[0].plot()

        # for box in results[0].boxes:
        #     cls_id = int(box.cls)
        #     confidence = box.conf
        #     label = self.model.names[cls_id]
        #     print(label)


        # cv2.imshow("YOLOv8 Live Detection", annotated_frame)
        cv2.imshow("no yolo", image)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            self.get_logger().info("Shutting down...")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()