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
        self.motor1 = Motor(37, 31, 33)
        self.motor2 = Motor(18, 16, 12) 
        self.motor3 = Motor(40, 38, 36)
        self.motor4 = Motor(24, 26, 22)

        self.subscription = self.create_subscription(
            String,
            'Motor_publisher',
            self.control_motor,
            10)

    def forward(self, speed=25):
        self.motor1.forward(speed)
        self.motor2.forward(speed)
        self.motor3.forward(speed)
        self.motor4.forward(speed)

    def backward(self, speed=25):
        self.motor1.backward(speed)
        self.motor2.backward(speed)
        self.motor3.backward(speed)
        self.motor4.backward(speed)

    def turn_left(self, speed=25):
        self.motor1.stop_motor()
        self.motor2.forward(speed)
        self.motor3.forward(speed)
        self.motor4.stop_motor()

    def turn_right(self, speed=25):
        self.motor1.forward(speed)
        self.motor2.stop_motor()
        self.motor3.stop_motor()
        self.motor4.forward(speed)

    def stop(self):
        self.motor1.stop_motor()
        self.motor2.stop_motor()
        self.motor3.stop_motor()
        self.motor4.stop_motor()

    def test1(self, speed =25):
        self.motor1.forward(speed)
        self.motor2.forward(speed)
        self.motor3.forward(speed)
        self.motor4.forward(speed)
    def test2(self, speed =25):
        # self.motor1.backward(speed)
        # self.motor2.backward(speed) 
        # self.motor3.backward(speed)
        # self.motor4.backward(speed)
        GPIO.setup(29, GPIO.OUT)
        GPIO.setup(31, GPIO.OUT)
        GPIO.setup(33, GPIO.OUT)
        GPIO.output(29, GPIO.LOW)
        GPIO.output(31, GPIO.HIGH)
        pwm = GPIO.PWM(33, 100)
        pwm.start(speed) 
        
    def control_motor(self, msg):
        command = msg.data
        print(f" command : {command}")
        if command == 'W':
            self.forward(speed=80)
        elif command == 'S':
            self.backward(speed=80)
        elif command == 'A':
            self.turn_left(speed=70)
        elif command == 'D':
            self.turn_right(speed=70)
        elif command == 'X':
            self.stop()
        elif command == 'M':
            self.test1(100)
        elif command == 'N':
            self.test2(100)
            self.get_logger().info('Invalid command')

    def destroy_node(self):
        self.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_subscriber = MotorSubscriber()
    rclpy.spin(motor_subscriber)
    motor_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()