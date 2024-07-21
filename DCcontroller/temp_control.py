import time
import math
import board
import busio
from adafruit_pca9685 import PCA9685
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# I2C 버스 설정
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 60  # PCA9685 주파수 설정

# PWM 제어
class PWMThrottleHat:
    def __init__(self, pwm, channel):
        self.pwm = pwm
        self.channel = channel
        self.pwm.frequency = 60

    def set_throttle(self, throttle):
        try:
            pulse = int(0xFFFF * abs(throttle))
            if throttle > 0: # 전진
                self.pwm.channels[self.channel + 5].duty_cycle = pulse
                self.pwm.channels[self.channel + 4].duty_cycle = 0
                self.pwm.channels[self.channel + 3].duty_cycle = 0xFFFF
            elif throttle < 0: # 후진
                self.pwm.channels[self.channel + 5].duty_cycle = pulse
                self.pwm.channels[self.channel + 4].duty_cycle = 0xFFFF
                self.pwm.channels[self.channel + 3].duty_cycle = 0
            else: # 정지
                self.pwm.channels[self.channel + 5].duty_cycle = 0
                self.pwm.channels[self.channel + 4].duty_cycle = 0
                self.pwm.channels[self.channel + 3].duty_cycle = 0
                
        except Exception as e:
            # get_logger() : ROS2에서 Python 활용 개발 시 로깅 위해 사용
            self.get_logger().error(f"PWM throttle ERROR: {e}")

class AdaptiveDCMotorController(Node):
    def __init__(self, channel):
        super().__init__('adaptive_dc_motor_controller')
        self.motor = PWMThrottleHat(pca, channel)

        # self.wheel_diameter = 0.06  # 0.06 미터 단위의 바퀴 지름 (예: 6cm)
        # self.gear_ratio = 1.5  # 기어 비 (모터 회전 대 바퀴 회전)
        # self.max_rpm = 330  # 모터의 최대 RPM
        # self.scaling_factor = 1.0  # 스케일링 계수 초기값 => 무게 등의 변화가 생기면 실험을 통해 수정 필요!!!!!!!!
        # self.distance_traveled = 0
        # self.last_update_time = time.time()
        # self.current_speed = 0

        # ROS2 파라미터 선언
        self.declare_parameter('wheel_diameter', 0.06)
        self.declare_parameter('gear_ratio', 1.5)
        self.declare_parameter('max_rpm', 330)
        self.declare_parameter('scaling_factor', 1.0)

        # 파라미터 값 가져오기
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.scaling_factor = self.get_parameter('scaling_factor').value

        # 나머지 초기화
        self.motor = PWMThrottleHat(pca, channel)
        self.distance_traveled = 0
        self.last_update_time = self.get_clock().now().to_msg().sec
        self.current_speed = 0

        # 파라미터 변경 콜백 추가
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.subscription = self.create_subscription(
            Float32,
            'target_distance',
            self.move_distance_callback,
            10
        )
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'wheel_diameter':
                self.wheel_diameter = param.value
            elif param.name == 'gear_ratio':
                self.gear_ratio = param.value
            elif param.name == 'max_rpm':
                self.max_rpm = param.value
            elif param.name == 'scaling_factor':
                self.scaling_factor = param.value
        
        self.get_logger().info('Parameters updated')
        return SetParametersResult(successful=True)

    def set_speed(self, speed):
        self.motor.set_throttle(speed)
        self.current_speed = speed
        self.last_update_time = time.time()

    def stop(self):
        self.motor.set_throttle(0)
        self.current_speed = 0

    def update_distance(self):
        current_time = time.time()
        elapsed_time = current_time - self.last_update_time

        if self.current_speed != 0:
            # 현재 속도에 따른 RPM 계산
            current_rpm = abs(self.current_speed) * self.max_rpm

            # 바퀴의 회전 속도 계산 (RPM)
            wheel_rpm = current_rpm / self.gear_ratio

            # 이동 거리 계산 (미터)
            distance = (wheel_rpm / 60) * elapsed_time * math.pi * self.wheel_diameter * self.scaling_factor

            # 방향에 따라 거리 더하기 또는 빼기
            self.distance_traveled += distance if self.current_speed > 0 else -distance

        self.last_update_time = current_time

    def move_distance(self, target_distance, speed=0.5):
        start_distance = self.distance_traveled
        self.set_speed(speed if target_distance > 0 else -speed)

        while abs(self.distance_traveled - start_distance) < abs(target_distance):
            self.update_distance()
            time.sleep(0.01)  # 10ms 간격으로 업데이트

        self.stop()

    def move_distance_callback(self, msg):
        target_distance = msg.data
        self.get_logger().info(f"Received target distance: {target_distance} meters")
        self.move_distance(target_distance)

    def calibrate(self, actual_distance):
        # 실제 이동 거리를 기반으로 파라미터 조정
        estimated_distance = abs(self.distance_traveled)
        self.scaling_factor = actual_distance / estimated_distance
        self.scaling_factor = actual_distance / estimated_distance

        self.get_logger().info(f"Calibrated: New scaling factor = {self.scaling_factor:.4f}")

        # 거리 재설정
        self.distance_traveled = 0

def main(args=None):
    rclpy.init(args=args)
    controller = AdaptiveDCMotorController(channel=0)

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        pca.deinit()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
