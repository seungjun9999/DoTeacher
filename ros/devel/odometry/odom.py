
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf_transformations import quaternion_from_euler

class DifferentialDriveOdometry(Node):
    def __init__(self, wheel_base, wheel_radius):
        super().__init__('differential_drive_odometry')
        
        # 노드 파라미터 초기화
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 퍼블리셔와 서브스크라이버 초기화
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # 주기적으로 오도메트리 업데이트를 위한 타이머
        self.timer = self.create_timer(0.1, self.publish_odometry)

        # 이전 시간 초기화
        self.previous_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        # 시간 간격 계산
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9
        self.previous_time = current_time
        
        # 바퀴 속도 계산
        left_wheel_speed = msg.linear.x - (msg.angular.z * self.wheel_base / 2)
        right_wheel_speed = msg.linear.x + (msg.angular.z * self.wheel_base / 2)
        
        # 각속도 계산 (라디안/초)
        omega_left = left_wheel_speed / self.wheel_radius
        omega_right = right_wheel_speed / self.wheel_radius
        
        # 차량의 선속도 v와 각속도 omega 계산
        v = (self.wheel_radius * (omega_left + omega_right)) / 2
        omega = (self.wheel_radius * (omega_right - omega_left)) / self.wheel_base
        
        # 오른손 법칙 적용에 따라 차량의 새 위치와 방향 계산
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        
        # theta 범위 조정 (0에서 2π 사이로)
        self.theta = self.theta % (2 * math.pi)
        
        # 업데이트된 위치와 방향을 로그로 출력
        self.get_logger().info(f'Updated position: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f} radians')

    def publish_odometry(self):
        # 오도메트리 메시지 생성 및 퍼블리시
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        # 위치 설정
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y

        # 방향 설정 (쿼터니언)
        quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # 퍼블리시
        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    wheel_base = 13.7 / 100.0  # 휠 베이스 (m)
    wheel_radius = 3 / 100.0  # 휠 반경 (m)
    odometry_node = DifferentialDriveOdometry(wheel_base, wheel_radius)
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

