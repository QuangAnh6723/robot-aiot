import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys, select, termios, tty

class MotorPublisher(Node):
    def __init__(self):
        super().__init__('Motor_publisher')
        self.publisher_ = self.create_publisher(String, 'Motor_publisher', 10)
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_message(self, key):
        msg = String()
        msg.data = key
        self.publisher_.publish(msg)
        print(f'Sending: {msg.data}')

    def run(self):
        while rclpy.ok():
            key = self.getKey()
            if key == 'A' or key == 'a':
                self.publish_message('A')
            elif key == 'S' or key == 's':
                self.publish_message('S')
            elif key == 'D' or key == 'd':
                self.publish_message('D')
            elif key == 'W' or key == 'w':
                self.publish_message('W')
            elif key == 'Q' or key == 'q':
                break
            elif key != '':
                self.publish_message('NONE')

def main(args=None):
    rclpy.init(args=args)
    motor_publisher = MotorPublisher()
    motor_publisher.run()
    motor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
