import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class AntiAckermannSteeringController(Node):

    def __init__(self):
        super().__init__('anti_ackermann_steering_controller')
        self.declare_parameter('wheelbase', 0.137)
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.ackermann_cmd_vel_publisher = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd_vel', 10)

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'Received cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}')
        try:
            ackermann_cmd = AckermannDriveStamped()
            ackermann_cmd.header.stamp = self.get_clock().now().to_msg()
            
            # AckermannDriveStamped 메시지의 필드 설정
            ackermann_cmd.drive.speed = msg.linear.x
            ackermann_cmd.drive.steering_angle = self.convert_twist_to_steering_angle(msg.linear.x, msg.angular.z)
            
            self.get_logger().info(f'Publishing ackermann_cmd: speed={ackermann_cmd.drive.speed}, steering_angle={ackermann_cmd.drive.steering_angle}')
            # 처리된 명령을 발행합니다.
            self.ackermann_cmd_vel_publisher.publish(ackermann_cmd)
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel_callback: {str(e)}')

    def convert_twist_to_steering_angle(self, linear_velocity, angular_velocity):
        wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        
        try:
            if np.isclose(linear_velocity, 0.0):
                if np.isclose(angular_velocity, 0.0):
                    steering_angle = 0.0
                else:
                    steering_angle = -np.pi / 2
            else:
                radius = linear_velocity / angular_velocity
                steering_angle = -np.arctan(wheelbase / radius)
                
            return steering_angle
        except Exception as e:
            self.get_logger().error(f'Error in convert_twist_to_steering_angle: {str(e)}')
            return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = AntiAckermannSteeringController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
