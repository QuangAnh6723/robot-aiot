import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
from time import sleep

class Motor:
    def __init__(self, in1_pin, in2_pin, pwm_pin, pwm_freq=100):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        self.in1 = in1_pin
        self.in2 = in2_pin
        self.pwm_pin = pwm_pin
        self.pwm_freq = pwm_freq

        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.pwm_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(self.pwm_pin, self.pwm_freq)
        self.stop_motor()

    def forward(self, speed=25):
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.start(speed)

    def backward(self, speed=25):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)
        self.pwm.start(speed)

    def stop_motor(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.stop()

class MotorSubscriber(Node):
    def __init__(self):
        super().__init__('Motor_subscriber')

        self.motor1 = Motor(31, 37, 33)
        self.motor2 = Motor(16, 18, 12)
        self.motor3 = Motor(40, 38, 36)
        self.motor4 = Motor(24, 26, 22)

        # Create subscription to the topic 'Motor_publisher'.
        self.subscription = self.create_subscription(
            String,
            'motor_cmd',
            self.control_motor,
            10
        )

    def control_motor(self, msg):

        command = msg.data.strip()
        self.get_logger().info(f"Command received: {command}")

        if command == 'xanh_la':
            # Đi thẳng
            self.forward(speed=95)
            print("a")
        elif command == 'xanh_lam':
            # Quẹo phải
            self.turn_right(speed=95)
            print("b")
        elif command == 'cam':
            # Quẹo trái
            self.turn_left(speed=95)
            print("c")
        else:
            self.get_logger().info('Invalid or unhandled command. Stopping...')
            self.stop()

    def forward(self, speed):
        self.motor1.forward(speed)
        self.motor2.forward(speed)
        self.motor3.forward(speed)
        self.motor4.forward(speed)

    def backward(self, speed):
        self.motor1.backward(speed)
        self.motor2.backward(speed)
        self.motor3.backward(speed)
        self.motor4.backward(speed)

    def turn_left(self, speed):
        self.motor1.stop_motor()
        self.motor2.forward(speed)
        self.motor3.forward(speed)
        self.motor4.stop_motor()

    def turn_right(self, speed):
        self.motor1.forward(speed)
        self.motor2.stop_motor()
        self.motor3.stop_motor()
        self.motor4.forward(speed)

    def stop(self):
        self.motor1.stop_motor()
        self.motor2.stop_motor()
        self.motor3.stop_motor()
        self.motor4.stop_motor()

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    motor_subscriber = MotorSubscriber()

    try:
        rclpy.spin(motor_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        motor_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
