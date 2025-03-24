import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # Dùng service ROS 2 có sẵn

class SimpleService(Node):
    def __init__(self):
        super().__init__('simple_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)
        self.get_logger().info('Service is ready...')

    def add_callback(self, request, response):
        self.get_logger().info(f'Received request: {request.a} + {request.b}')
        response.sum = request.a + request.b
        return response

def main():
    rclpy.init()
    node = SimpleService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
