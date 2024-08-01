import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class CustomTFNode(Node):

    def __init__(self):
        super().__init__('frame_publisher')

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.rx = 0.0
        self.ry = 0.0
        self.rz = 0.0
        self.rw = 0.0

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, 'scan_odom', self.update_pos_callback, 10)

        # Create a timer that will call the timer_callback function every second
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Custom TF Node was started')


    def update_pos_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        self.rx = msg.pose.pose.orientation.x
        self.ry = msg.pose.pose.orientation.y
        self.rz = msg.pose.pose.orientation.z
        self.rw = msg.pose.pose.orientation.w
        self.get_logger().info(f'pos x:{self.x:.6f} y:{self.y :.6f} z:{self.z:.6f} ori x:{self.rx:.6f} y:{self.ry:.6f} z:{self.rz:.6f} w:{self.rw:.6f}')


    def timer_callback(self):
        # Create a transform message
        t = TransformStamped()

        # Set the time and frame IDs
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'# 'base_footprint'
        t.child_frame_id = 'base_footprint' # 'laser_frame'

        # Set the translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z

        # Set the rotation (quaternion)
        t.transform.rotation.x = self.rx
        t.transform.rotation.y = self.ry
        t.transform.rotation.z = self.rz
        t.transform.rotation.w = self.rw

        # Send the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = CustomTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
