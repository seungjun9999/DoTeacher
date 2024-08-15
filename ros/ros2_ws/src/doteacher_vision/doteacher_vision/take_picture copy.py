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
from doteacher_interfaces.srv import UploadPicture, RPiCommand

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
        self.timer = self.create_timer(1, self.detect_pose)
        self.blocking = False
        self.image_publisher = self.create_publisher(Image, 'image_pose', 10)

        self.bridge = CvBridge()
        package_share_directory = get_package_share_directory('doteacher_vision')
        model_path = os.path.join(package_share_directory, 'models/yolov8n-pose.pt')

        # YOLOv8 Pose 모델 로드
        self.model = YOLO(model_path)

        self.user_id = 12  # param 으로 수정 필요

        # Create a service client
        self.client = self.create_client(UploadPicture, 'upload_picture')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Upload Picture Service not available, waiting...')
        self.get_logger().info('Upload Picture Service is available')

        self.cli = self.create_client(RPiCommand, 'rpi_cmd_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('RPi Command Service is available')

        self.get_logger().info("Detect Pose node has been started.")
        
        # frame 초기화
        self.frame = None
    
    def listener_callback(self, data):
        self.frame = data

    def detect_pose(self):
        if self.blocking:
            return
        
        # frame이 초기화되지 않은 경우 예외 처리
        if self.frame is None:
            self.get_logger().warn('Frame is not yet received.')
            return

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

            # 왼 손목과 왼 어깨가 있으며 손목이 어깨 위로 올라갈 경우
            # 이미지에서의 오른쪽을 의미
            if rwrist[1] != 0 and rsholder[1] != 0 and rwrist[1] < rsholder[1]:
                self.blocking = True
                self.get_logger().info("Cheeze")
                
                # 사운드 요청 서비스
                self.call_service_sound('capture_sound')
                
                # 5초 후 이미지를 저장하는 작업을 한 번만 실행
                self.get_clock().call_later(5, lambda: self.save_image(src, result))

    def save_image(self, src, result):
        # 이미지 저장 처리
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
        self.call_service(self.user_id, abs_path, "Pose detection image")
        self.get_logger().info(f"Upload complete {filename}")
        self.blocking = False

    def call_service(self, user_id, filename, description):
        request = UploadPicture.Request()
        request.user_id = user_id
        request.filename = filename
        request.description = description
        
        future = self.client.call_async(request)
        future.add_done_callback(self.upload_callback)

    def upload_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service response: {response.message}")
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            
    def call_service_sound(self, command):
        # ROS2 서비스 요청 생성 및 전송
        req = RPiCommand.Request()
        req.command = command
        future = self.cli.call_async(req)
        future.add_done_callback(self.service_sound_callback)

    def service_sound_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service response: success={response.success}, message='{response.message}'")
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

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
