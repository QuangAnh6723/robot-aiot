import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLineEdit, QLabel

class SimpleClient(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'simple_client')
        QWidget.__init__(self)
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        self.setWindowTitle("ROS2 PyQt Client")
        self.setGeometry(200, 200, 300, 150)
        
        self.layout = QVBoxLayout()
        
        self.input_a = QLineEdit(self)
        self.input_a.setPlaceholderText("Nhập số A")
        self.layout.addWidget(self.input_a)
        
        self.input_b = QLineEdit(self)
        self.input_b.setPlaceholderText("Nhập số B")
        self.layout.addWidget(self.input_b)
        
        self.button = QPushButton("Gửi yêu cầu", self)
        self.button.clicked.connect(self.send_request)
        self.layout.addWidget(self.button)
        
        self.result_label = QLabel("Kết quả: ", self)
        self.layout.addWidget(self.result_label)
        
        self.setLayout(self.layout)

    def send_request(self):
        if not self.client.wait_for_service(timeout_sec=2.0):
            self.result_label.setText("Service không khả dụng!")
            return
        
        try:
            a = int(self.input_a.text())
            b = int(self.input_b.text())
        except ValueError:
            self.result_label.setText("Vui lòng nhập số hợp lệ!")
            return

        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            self.result_label.setText(f"Kết quả: {response.sum}")
        except Exception as e:
            self.result_label.setText(f"Lỗi: {e}")

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    client_node = SimpleClient()
    client_node.show()
    
    def spin_ros2():
        rclpy.spin(client_node)

    from threading import Thread
    thread = Thread(target=spin_ros2, daemon=True)
    thread.start()
    
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
