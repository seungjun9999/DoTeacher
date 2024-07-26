#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
