import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# detect_picture.py의 내용을 가져옵니다
import sys
sys.path.append('/mnt/data/')
from detect_picture import detect_objects  # 예: detect_objects 함수를 포함한다고 가정

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Image, 'image_processed', 10)
        self.br = CvBridge()

    def listener_callback(self, data):
        self.get_logger().info('Receiving image data...')
        current_frame = self.br.imgmsg_to_cv2(data)

        # detect_picture.py의 함수를 사용하여 이미지 처리
        processed_frame = detect_objects(current_frame)

        # 처리된 이미지를 다시 image_processed 토픽으로 발행
        self.publisher_.publish(self.br.cv2_to_imgmsg(processed_frame))
        self.get_logger().info('Processed image published')

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
