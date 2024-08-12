import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
from ultralytics.utils.plotting import colors

from doteacher_interfaces.srv import UploadPicture

import sys
sys.path.append('/mnt/data/')


# 예를 들어, 이미지를 /home/jetson/Images/ 디렉토리에 저장하려면:
save_directory = '/home/jetson/doteacher/images'

class DetectPose(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(3, self.detect_pose)
        self.image_publisher = self.create_publisher(Image, 'image_pose', 10)

        self.bridge = CvBridge()
        package_share_directory = get_package_share_directory('doteacher_vision')
        model_path = os.path.join(package_share_directory, 'models/yolov8n-pose.pt')

        # YOLOv8 Pose 모델 로드
        self.model = YOLO(model_path)

        # Create a service client
        self.client = self.create_client(UploadPicture, 'upload_picture')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info("Detect Pose node has been started.")


    def listener_callback(self, data):
        self.frame = data


    def detect_pose(self):
        self.get_logger().info('Detect pose started')

        src = self.bridge.imgmsg_to_cv2(self.frame)

        results = self.model(src)
        result = results[0]

        boxes = result.boxes.xyxy.to('cpu').numpy()
        if len(boxes) > 0:
            areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
            max_area_index = areas.argmax()
            largest_box = boxes[max_area_index]

        keypoints = result.keypoints
        if keypoints.xy.shape[1] > 10 and keypoints.conf.shape[1] > 10:
            rwrist, rwrist_prob = keypoints.xy[0, 9].to('cpu').numpy(), keypoints.conf[0, 9].item()
            rsholder, rsholder_prob = keypoints.xy[0, 5].to('cpu').numpy(), keypoints.conf[0, 5].item()
            lwrist, lwrist_prob = keypoints.xy[0, 10].to('cpu').numpy(), keypoints.conf[0, 10].item()
            lsholder, lsholder_prob = keypoints.xy[0, 6].to('cpu').numpy(), keypoints.conf[0, 6].item()

            if rwrist[1] != 0 and rsholder[1] != 0 and rwrist[1] < rsholder[1] and lwrist[1] != 0 and lsholder[1] != 0 and lwrist[1] < lsholder[1]:
                self.get_logger().info("Cheeze")
                filename = datetime.now().strftime('%Y%m%d_%H%M%S') + '.jpg'
                filename2 = datetime.now().strftime('%Y%m%d_%H%M%S') + '_2' + '.jpg'

                abs_path = os.path.join(save_directory, filename)
                cv2.imwrite(abs_path, src)

                result_image = result.plot()
                abs_path2 = os.path.join(save_directory, filename2)
                cv2.imwrite(abs_path2, result_image)

                img_msg = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
                self.image_publisher.publish(img_msg)
                self.get_logger().warn(f"Image saving complete {filename}")

                # Call the service to upload the picture
                self.call_service(3, abs_path, "Pose detection image")
                # self.call_service(3, abs_path2, "Pose detection image2")
                self.get_logger().info(f"upload complete {filename}")


    def call_service(self, user_id, filename, description):
        request = UploadPicture.Request()
        request.user_id = user_id
        request.filename = filename
        request.description = description
        
        future = self.client.call_async(request)

        # Non-blocking spin until the service call is completed
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {str(e)}')
                else:
                    self.get_logger().info(f'Service response: {response.message}')
                break

def main(args=None):
    rclpy.init(args=args)
    node = DetectPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
