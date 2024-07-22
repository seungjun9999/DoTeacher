# joy_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame

class JoyNode(Node):
    def __init__(self):
        super().__init__('joy_node')
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        
        # Pygame 초기화
        pygame.init()
        pygame.joystick.init()
        
        # 조이스틱 연결 확인
        if pygame.joystick.get_count() == 0:
            self.get_logger().error('No joystick found')
            return
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        self.get_logger().info(f'Joystick initialized: {self.joystick.get_name()}')
        
        # 타이머 설정 (100Hz로 조이스틱 상태 확인)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        pygame.event.pump()
        
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 축 상태 읽기
        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        joy_msg.axes = axes
        
        # 버튼 상태 읽기
        buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        joy_msg.buttons = buttons
        
        self.publisher_.publish(joy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()
    
    
# joy_teleop_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.linear_axis = 1  # 왼쪽 스틱의 수직 축
        self.angular_axis = 0  # 왼쪽 스틱의 수평 축
        self.linear_scale = 2.0
        self.angular_scale = 2.0

    def joy_callback(self, msg):
        twist = Twist()
        twist.linear.x = msg.axes[self.linear_axis] * self.linear_scale
        twist.angular.z = msg.axes[self.angular_axis] * self.angular_scale
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
# motor_control_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from adafruit_motor import motor
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
import board
import busio

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # I2C 버스 설정
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 500  # PCA9685 주파수 설정

        # PWMThrottleHat 클래스 정의
        class PWMThrottleHat:
            def __init__(self, pwm, channel):
                self.pwm = pwm
                self.channel = channel
                self.pwm.frequency = 500

            def set_throttle(self, throttle):
                pulse = int(0xFFFF * abs(throttle))
                if throttle > 0:
                    self.pwm.channels[self.channel + 5].duty_cycle = pulse
                    self.pwm.channels[self.channel + 4].duty_cycle = 0
                    self.pwm.channels[self.channel + 3].duty_cycle = 0xFFFF
                elif throttle < 0:
                    self.pwm.channels[self.channel + 5].duty_cycle = pulse
                    self.pwm.channels[self.channel + 4].duty_cycle = 0xFFFF
                    self.pwm.channels[self.channel + 3].duty_cycle = 0
                else:
                    self.pwm.channels[self.channel + 5].duty_cycle = 0
                    self.pwm.channels[self.channel + 4].duty_cycle = 0
                    self.pwm.channels[self.channel + 3].duty_cycle = 0

        # PWMThrottleHat 인스턴스 생성
        self.motor_hat = PWMThrottleHat(self.pca, channel=0)

        # 서보 모터 제어 설정
        self.kit = ServoKit(channels=16, i2c=i2c, address=0x60)
        self.pan = 100  # 서보 모터 초기 위치 설정
        self.kit.servo[0].angle = self.pan

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        self.motor_hat.set_throttle(msg.linear.x)
        self.pan = 100 + msg.angular.z * 50  # 예시로 각도 조정
        self.kit.servo[0].angle = self.pan

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    try:
        rclpy.spin(motor_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_control_node.destroy_node()
        rclpy.shutdown()
        motor_control_node.motor_hat.set_throttle(0)  # 모터 정지
        motor_control_node.kit.servo[0].angle = 100  # 서보 모터 초기 위치로 리셋
        motor_control_node.pca.deinit()  # PCA9685 정리
        print("Program stopped and motor stopped.")

if __name__ == '__main__':
    main()