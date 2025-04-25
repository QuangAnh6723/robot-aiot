#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage  # Sử dụng CompressedImage thay vì Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self, name):
        super().__init__(name)
        self.publisher_ = self.create_publisher(CompressedImage, 'image_compressed', 10)  # Sử dụng topic 'image_compressed'
        self.timer = self.create_timer(0.1, self.timer_callback)  # Gửi ảnh mỗi 0.1 giây (~10 FPS)
        self.cap = cv2.VideoCapture(0)  # Mở camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)  # Đặt độ phân giải
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cv_bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Nén ảnh thành JPEG
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])  # Chất lượng JPEG (80%)
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()  # Gắn timestamp
            msg.format = "jpeg"  # Định dạng ảnh
            msg.data = buffer.tobytes()  # Chuyển buffer thành bytes
            self.publisher_.publish(msg)  # Gửi ảnh qua topic
            self.get_logger().info('Publishing compressed video frame...')

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher("topic_webcam_pub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()