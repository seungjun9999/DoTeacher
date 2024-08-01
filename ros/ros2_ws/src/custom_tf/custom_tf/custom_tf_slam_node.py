import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class CustomTFSLAMNode(Node):

    def __init__(self):
        super().__init__('frame_slam_publisher')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a timer that will call the timer_callback function every second
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Create a transform message
        t = TransformStamped()

        # Set the time and frame IDs
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Set the translation
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Set the rotation (quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Send the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = CustomTFSLAMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
