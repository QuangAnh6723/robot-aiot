import rclpy
from rclpy.node import Node
# from example_interfaces.srv import AddTwoInts  # Dùng tạm service này để gửi SSID & Password như 2 số
# from tutorial_interfaces.srv import AddThreeInts
from tutorial_interfaces.srv import WifiConnect



class WifiService(Node):
    def __init__(self):
        super().__init__('wifi_service')
        self.srv = self.create_service(WifiConnect, 'wifi_connect', self.handle_request)
        self.get_logger().info('WiFi Service is ready...')

    def handle_request(self, request, response):
        ssid = str(request.ssid)  # Giả lập chuyển đổi số thành chuỗi (thực tế cần custom service)
        password = str(request.password)

        # self.get_logger().info(f"📡 Nhận yêu cầu kết nối WiFi:")
        self.get_logger().info(f"   ➤ SSID: {ssid}")
        self.get_logger().info(f"   ➤ Password: {password}")

        import subprocess

        script_path = "/home/aiot/workspace/robot-aiot/scripts/wifii_run.bash"
        args = [ssid, password]  # Danh sách các đối số

        subprocess.run(["bash", script_path] + args)

        response.success = True  # Giả lập kết nối thành công
        response.message = 'vo dc wifi'
        return response

def main():
    rclpy.init()
    node = WifiService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
