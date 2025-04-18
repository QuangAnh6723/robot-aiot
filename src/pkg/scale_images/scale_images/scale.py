#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

'''đang bị nhược điểm là gửi từ lap sang camera thì nó sẽ bị delay, nhưng mà gửi từ camera sang lap thì ko bị delay'''

class CameraScalerNode(Node):
    def __init__(self):
        super().__init__('camera_scaler_node')
        
        # Subscriber nhận kích thước (width, height)
        self.size_subscription = self.create_subscription(
            Int32MultiArray,
            'desired_size',
            self.size_callback,
            10
        )
        
        # Publisher gửi ảnh đã scale
        self.scaled_image_publisher = self.create_publisher(Image, 'scaled_image', 10)
        
        self.bridge = CvBridge()

        # Kích thước mặc định
        self.desired_width = 128
        self.desired_height = 480
        
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Dell the mo dc cam nha thg lol')
        
        # Đặt tần số đọc camera (10Hz)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.capture_and_scale)
        
        self.get_logger().info('Ready to scale images!')
        self.get_logger().info(f'Default: {self.desired_width}x{self.desired_height}')

    def size_callback(self, msg):
        # check xem du 2 kich thuoc ko 
        if len(msg.data) != 2:
            self.get_logger().error('can 2 gia tri nha: [width, height].')
            return
        
        self.desired_width, self.desired_height = msg.data
        self.get_logger().info(f'Ok: {self.desired_width}x{self.desired_height}')

    def capture_and_scale(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('dont get frame from camera')
            return
        
        # Scale ảnh
        scaled_frame = cv2.resize(
            frame,
            (self.desired_width, self.desired_height),
            interpolation=cv2.INTER_LINEAR
        )
        
        # Chuyển ảnh sang message và publish
        img_msg = self.bridge.cv2_to_imgmsg(scaled_frame, encoding='bgr8')
        self.scaled_image_publisher.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraScalerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
