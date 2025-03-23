import sys
import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import WifiConnect
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLineEdit, QLabel
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import Qt
import sys


class WifiClient(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'wifi_client')
        QWidget.__init__(self)
        self.client = self.create_client(WifiConnect, 'wifi_connect')

        self.setWindowTitle("WiFi Configuration")
        self.setWindowFlags(Qt.WindowType.Window)
        self.setGeometry(200, 200, 600, 200)

        self.layout = QVBoxLayout()

        self.ssid_input = QLineEdit(self)
        self.ssid_input.setPlaceholderText("Nhập SSID WiFi")
        self.layout.addWidget(self.ssid_input)

        self.password_input = QLineEdit(self)
        self.password_input.setPlaceholderText("Nhập Password")
        self.password_input.setEchoMode(QLineEdit.Password)  # Ẩn mật khẩu
        self.layout.addWidget(self.password_input)

        self.button = QPushButton("Kết nối WiFi", self)
        self.button.clicked.connect(self.send_request)
        self.layout.addWidget(self.button)

        self.result_label = QLabel("Trạng thái: Chưa kết nối", self)
        self.layout.addWidget(self.result_label)

        self.setLayout(self.layout)

    def send_request(self):
        if not self.client.wait_for_service(timeout_sec=2.0):
            self.result_label.setText("Service không khả dụng!")
            return

        ssid = self.ssid_input.text()
        password = self.password_input.text()

        if not ssid or not password:
            self.result_label.setText("Vui lòng nhập đầy đủ thông tin!")
            return

        request = WifiConnect.Request()
        request.ssid = ssid
        request.password = password
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.result_label.setText(f"✅ {response.message}")
            else:
                self.result_label.setText(f"❌ {response.message}")
        except Exception as e:
            self.result_label.setText(f"Lỗi: {e}")

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    client_node = WifiClient()
    client_node.show()

    def spin_ros2():
        rclpy.spin(client_node)

    from threading import Thread
    thread = Thread(target=spin_ros2, daemon=True)
    thread.start()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()
