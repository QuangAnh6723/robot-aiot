#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from ultralytics.utils.plotting import Annotator
import time
# from cfg import TURN_LEFT, TURN_RIGHT, STOP, FORWARD
orange_flag = False
blue_flag = False
green_flag = False
class ImageSubscriber(Node):
    def __init__(self,):
        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(
            CompressedImage,
            'image_compressed',
            self.listener_callback,
            1
        )
        self.subscription  # Ngăn chặn việc bị xóa bởi garbage collector
        self.queue_object = ["blue", "green", "orange"]
        


        self.cv_bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Int8, 'motor_detect', 1)
        device = "cpu"
        print("Loading YOLO model...")
        try:
            self.model = YOLO("/home/shelvt/robot-aiot/src/pkg/camera_pkg/best_test_ok.pt").to(device)
            self.get_logger().info("YOLO model loaded successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            self.model = None

    def process_logic(self, label, percent, distance, position):    #1:turn left, 2:turn right, 0:stop
        # msg = In  
        global green_flag
        global orange_flag
        print(f"percent : {percent:.4f}")
        if label == "green":
            green_flag = False
            print("------------------------------- Tawts cow xanh")
        
        if label == "orange" and green_flag == False:
            orange_flag = False


        if percent <= 0.07 and percent > 0: # ở xa object va ddi thang
            msg = Int8()
            msg.data = 1
            self.cmd_pub.publish(msg)


        elif percent > 0.07:                       # Ở gần object
            if label == "blue":               # queo trai
                msg = Int8()
                msg.data = 3
                self.cmd_pub.publish(msg)
                green_flag = True

            elif label == "green":              # Turn right
                msg = Int8()
                msg.data = 4
                self.cmd_pub.publish(msg)
                print("GREEN - rẻ phải thường")
                green_flag = False
                orange_flag = True

            elif label == "orange":               # Stop
                msg = Int8()
                msg.data = 0
                self.cmd_pub.publish(msg)
                print("Stop 0.05")
                time.sleep(10)

        if green_flag == True:
            msg = Int8()
            msg.data = 3
            self.cmd_pub.publish(msg)

        elif orange_flag == True:
            msg = Int8()
            msg.data = 4
            self.cmd_pub.publish(msg)

        time.sleep(0.2)
        print(f"=========== Sleep = {green_flag} ")
        msg = Int8()
        msg.data = 0
        self.cmd_pub.publish(msg)

        if distance > 50 and position == "left":
            msg = Int8()
            msg.data = 5
            self.cmd_pub.publish(msg)
        elif distance > 50 and position == "right":
            msg = Int8()
            msg.data = 6
            self.cmd_pub.publish(msg)
        
        time.sleep(0.5)
        msg.data = 0
        self.cmd_pub.publish(msg)

    
    def calculate_distance_and_position(self, amidW, object_center_x):
        # Calculate the distance between the center of the image and the object's center
        distance = abs(object_center_x - amidW)
        
        # Determine whether the object is on the left or right side of the image center
        position = "left" if object_center_x < amidW else "right"
        return distance, position
    
    def listener_callback(self, msg):
        label = None
        percent = -1
        self.get_logger().info("Receiving video frame")
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame_height, frame_width, _ = frame.shape
        amidW = frame_width // 2
        amidH = frame_height // 2
        cv2.circle(frame, (amidW, amidH), 5, (0, 255, 0), -1)
        distance = 0, position = ""
        
        results = self.model(frame, conf = 0.4)
        annotated_frame = results[0].plot()
        annotator = Annotator(frame, example=self.model.names)
        for box in results[0].boxes.xyxy.cpu():
            width, height, area = annotator.get_bbox_dimension(box)
            
            label = results[0].names[int(results[0].boxes.cls[0])]
            # print(f"{label.frame}")
            # print(f"Detected label: {label}")

            percent = area/(frame_width * frame_height)
            print(f"Percent: {percent:.4f}")
            object_center_x = int((box[0] + box[2]) / 2)
            distance, position = self.calculate_distance_and_position(amidW, object_center_x)
        cv2.imshow("YOLOv8 Live Detection", annotated_frame)
        print(f"            Label: {label}")
        self.process_logic(label, percent, distance, position)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Shutting down...")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()