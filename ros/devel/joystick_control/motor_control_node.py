import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import busio
import board
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit

class PWMThrottleHat:
    def __init__(self, pwm, channel):
        self.pwm = pwm
        self.channel = channel
        self.pwm.frequency = 60

    def set_throttle(self, throttle):
        pulse = int(0xFFFF * abs(throttle))
        if throttle > 0:
            self._set_channel_values(pulse, 0, 0xFFFF)
        elif throttle < 0:
            self._set_channel_values(pulse, 0xFFFF, 0)
        else:
            self._set_channel_values(0, 0, 0)

    def _set_channel_values(self, ch5, ch4, ch3):
        self.pwm.channels[self.channel + 5].duty_cycle = ch5
        self.pwm.channels[self.channel + 4].duty_cycle = ch4
        self.pwm.channels[self.channel + 3].duty_cycle = ch3

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 60

        self.motor_hat = PWMThrottleHat(self.pca, channel=0)
        self.kit = ServoKit(channels=16, i2c=i2c, address=0x60)
        
        self.setup_steering()

    def setup_steering(self):
        self.steering_servo_channel = 0
        self.steering_center = 100
        self.steering_min = 60
        self.steering_max = 140
        self.steering_step = 10
        self.current_angle = self.steering_center
        self.kit.servo[self.steering_servo_channel].angle = self.current_angle

    def cmd_vel_callback(self, msg):
        self.update_steering(msg.angular.z)
        self.update_drive(msg.linear.x)
        self.log_status(msg.linear.x, msg.angular.z)

    def update_steering(self, angular_velocity):
        if angular_velocity > 0:
            self.current_angle = min(self.steering_max, self.current_angle + self.steering_step)
        elif angular_velocity < 0:
            self.current_angle = max(self.steering_min, self.current_angle - self.steering_step)
        self.control_steering(self.current_angle)

    def update_drive(self, linear_velocity):
        throttle = -linear_velocity
        self.control_drive(throttle)

    def control_steering(self, angle):
        self.kit.servo[self.steering_servo_channel].angle = angle
        duty_cycle = self.calculate_duty_cycle(angle)
        self.get_logger().info(f'Servo duty cycle: {duty_cycle:.2f}%')

    def calculate_duty_cycle(self, angle):
        min_duty, max_duty = 2.5, 12.5
        return min_duty + (angle / 180.0) * (max_duty - min_duty)

    def control_drive(self, throttle):
        self.motor_hat.set_throttle(throttle)

    def log_status(self, linear_velocity, angular_velocity):
        self.get_logger().info(f'Steering angle: {self.current_angle}, Throttle: {-linear_velocity}, '
                               f'Angular velocity: {angular_velocity}, Linear velocity: {linear_velocity}')

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()