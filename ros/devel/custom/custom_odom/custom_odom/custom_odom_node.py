import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import transforms3d.euler as euler
from tf2_ros import TransformBroadcaster

class CustomOdomNode(Node):
    def __init__(self):
        super().__init__('custom_odom_node')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.odom_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0

        self.get_logger().info('Odom Transform Publisher Node has been started')

    def timer_callback(self):
        current_time = self.get_clock().now()

        # Update pose
        self.x += self.vx * 0.1
        self.y += self.vy * 0.1
        self.theta += self.vtheta * 0.1

        odom_quat = self.create_quaternion_from_yaw(self.theta)

        # Create the TransformStamped message
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = odom_quat[0]
        t.transform.rotation.y = odom_quat[1]
        t.transform.rotation.z = odom_quat[2]
        t.transform.rotation.w = odom_quat[3]

        # Send the transform
        self.odom_broadcaster.sendTransform(t)

        # Create the Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]
        odom.child_frame_id = 'base_footprint'
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vtheta

        # Publish the message
        self.odom_pub.publish(odom)

    def create_quaternion_from_yaw(self, yaw):
        return euler.euler2quat(0, 0, yaw)

def main(args=None):
    rclpy.init(args=args)
    node = CustomOdomNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
