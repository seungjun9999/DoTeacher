import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class CombinedJoyNode(Node):
    def __init__(self):
        super().__init__('combined_joy_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        self.twist_msg = Twist()
 
    def joy_callback(self, msg):
        # Joy 메시지 로깅
        self.get_logger().info(f'Received Joy message: {msg}')
        
        # Initialize with zero values
        angular_z = 0.0
        linear_x = 0.0

        # Check for joystick input (LEFTX and LEFTY)
        if len(msg.axes) >= 2:
            angular_z = msg.axes[0] * 1.0  # 좌우 회전 (왼쪽이 양수)
            linear_x = msg.axes[1] * 1.0  # 전진/후진 (위가 양수)

        # Check for directional pad input
        if len(msg.axes) >= 8:  # 방향키는 일반적으로 axes[6]과 axes[7]에 매핑됩니다
            if msg.axes[6] != 0.0:  # 좌우 회전 (왼쪽이 양수)
                angular_z = msg.axes[6] * 1.0
            if msg.axes[7] != 0.0:  # 전진/후진 (위가 양수)
                linear_x = msg.axes[7] * 1.0

        # Update twist message
        self.twist_msg.angular.z = angular_z
        self.twist_msg.linear.x = linear_x
        
        # Publish the twist message
        self.publisher_.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CombinedJoyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
