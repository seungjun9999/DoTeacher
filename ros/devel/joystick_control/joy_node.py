import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class XboxJoyNode(Node):
    def __init__(self):
        super().__init__('xbox_joy_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        self.twist_msg = Twist()

    def joy_callback(self, msg):
        # Joy 메시지 로깅
        self.get_logger().info(f'Received Joy message: {msg}')
        
        # 방향키 입력 처리
        if len(msg.axes) >= 7:  # 방향키는 일반적으로 axes[6]과 axes[7]에 매핑됩니다
            self.twist_msg.angular.z = -msg.axes[6] * 1.0  # 좌우 회전 (왼쪽이 양수)
            self.twist_msg.linear.x = -msg.axes[7] * 1.0  # 전진/후진 (위가 양수)
        
        self.publisher_.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = XboxJoyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
