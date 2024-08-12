import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from doteacher_interfaces.srv import UploadPicture
import requests

class UploadPictureService(Node):
    def __init__(self):
        super().__init__('upload_picture_service')
        self.publisher_ = self.create_publisher(String, 'backend_db_resp', 10)
        self.service = self.create_service(UploadPicture, 'upload_picture', self.handle_upload_request)
        self.get_logger().info('Upload picture server is ready to upload pictures.')

    def handle_upload_request(self, request, response):
        self.get_logger().info(f'Received request to upload picture: {request.filename}, description: {request.description}, user_id: {request.user_id}')
        
        success, image_url = self.upload(request.user_id, request.filename, request.description)
        
        if success:
            response.success = True
            response.message = f'Successfully uploaded file. Image URL: {image_url}'
        else:
            response.success = False
            response.message = 'Failed to upload file.'

        return response

    def upload(self, user_id, filename, desc):
        url = 'http://i11d102.p.ssafy.io:8081/photo'
        file_path = f'{filename}'

        description = desc
        
        try:
            with open(file_path, 'rb') as f:
                files = {'file': f}
                data = {
                    'description': description,
                    'userId': user_id
                }
                response = requests.post(url, files=files, data=data)
                
                if response.status_code == 200:
                    response_data = response.json()
                    image_url = response_data['data'].get('imageUrl', 'No imageUrl found')
                    msg = String()
                    msg.data = image_url
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Successfully uploaded file. Image URL: {image_url}')
                    return True, image_url
                else:
                    self.get_logger().error(f'Failed to upload file: {response.status_code}')
                    return False, ''
        
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Upload picture request failed: {e}')
            return False, ''

def main(args=None):
    rclpy.init(args=args)
    node = UploadPictureService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
