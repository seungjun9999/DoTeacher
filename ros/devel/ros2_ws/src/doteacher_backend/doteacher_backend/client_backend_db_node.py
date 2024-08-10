import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json

class ClientBackendDBNode(Node):
    def __init__(self):
        super().__init__('client_backend_db_node')
        self.publisher_ = self.create_publisher(String, 'backend_db_resp', 10)
        # self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info('DB Client Node started')
        
        self.timer_callback()
        
        self.get_logger().info('DB Client Node finished')

    def timer_callback(self):
        url = 'http://i11d102.p.ssafy.io:8081/photo'  # API endpoint
        file_path = '/home/jeongjae/Downloads/doteacher'  # Path to the file to be uploaded
        description = 'test'
        user_id = 2
        
        with open(file_path, 'rb') as f:
            files = {'file': f}
            data = {
                'description': description,
                'userId': user_id
            }
            try:
                response = requests.post(url, files=files, data=data)
                if response.status_code == 200:
                    response_data = response.json()
                    image_url = response_data['data'].get('imageUrl', 'No imageUrl found')
                    msg = String()
                    msg.data = image_url
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Successfully uploaded file. Image URL: {image_url}')
                else:
                    self.get_logger().error(f'Failed to upload file: {response.status_code}')
            except requests.exceptions.RequestException as e:
                self.get_logger().error(f'DB Client request failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ClientBackendDBNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
