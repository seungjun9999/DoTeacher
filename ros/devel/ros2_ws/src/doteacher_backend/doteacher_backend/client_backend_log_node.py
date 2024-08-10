import rclpy
from rclpy.node import Node
import requests
from std_msgs.msg import String

class ClientBackendLogNode(Node):
    def __init__(self):
        super().__init__('client_backend_db_node')
        self.publisher_ = self.create_publisher(String, 'backend_db_resp', 10)
        # self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info('Log Client Node started')
        
        self.timer_callback()
        
        self.get_logger().info('Log Client Node finished')

    def timer_callback(self):
        url = 'http://i11d102.p.ssafy.io:8081/photo'  # API endpoint
        try:
            response = requests.get(url)
            if response.status_code == 200:
                msg = String()
                msg.data = response.text
                self.publisher_.publish(msg)
                self.get_logger().info(f'Successfully fetched data: {response.text}')
            else:
                self.get_logger().error(f'Failed to fetch data: {response.status_code}')
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Log server request failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ClientBackendLogNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
