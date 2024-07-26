import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from adafruit_pca9685 import PCA9685
import board
import busio
import logging
import sys
class PWMThrottleHat:
    def __init__(self, pwm, channel):
        self.pwm = pwm
        self.channel = channel
        self.pwm.frequency = 600

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

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.subscription = self.create_subscription(
            Float32,
            'ultrasonic_distance',
            self.distance_callback,
            10)
        
        # I2C 및 PCA9685 설정
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 600

        self.motor_hat = PWMThrottleHat(self.pca, channel=0)
        self.moving_forward = False
        self.current_distance = None
        self.timer = self.create_timer(0.05, self.control_loop)  # 50ms 주기 (20Hz)

    def distance_callback(self, msg):
        self.current_distance = msg.data
        self.get_logger().info(f"Current distance: {self.current_distance:.2f} cm")

    def control_loop(self):
        if self.current_distance is not None:
            if self.current_distance < 10:
                self.stop_motor()
            elif self.current_distance < 20:
                self.set_motor_speed(0.2)  # 20cm 이하
            elif self.current_distance < 30:
                self.set_motor_speed(0.3)  # 30cm 이하
            else:
                self.set_motor_speed(0.4)  # 30cm 초과

    def set_motor_speed(self, speed):
        self.get_logger().info(f"Setting motor speed: {speed}")
        self.motor_hat.set_throttle(speed)
        self.moving_forward = True if speed > 0 else False

    def stop_motor(self):
        if self.moving_forward:
            self.get_logger().info("Stopping motor")
            self.motor_hat.set_throttle(0.0)  # 정지
            self.moving_forward = False

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.set_motor_speed(0)
        sys.exit(0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
