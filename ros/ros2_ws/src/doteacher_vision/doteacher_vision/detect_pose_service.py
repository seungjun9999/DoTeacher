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
from doteacher_interfaces.srv import DetectPoseService, UploadPicture  # 사용자 정의 서비스 가져오기


class DetectPose(Node):
    def __init__(self):
        super().__init__('detect_pose_service')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        package_share_directory = get_package_share_directory('doteacher_vision')
        model_path = os.path.join(package_share_directory, 'models/yolov8n-pose.pt')

        # YOLOv8 Pose 모델 로드
        self.model = YOLO(model_path)

        self.latest_frame = None  # 가장 최근에 수신된 이미지를 저장할 변수

        # 서비스 생성
        self.srv = self.create_service(DetectPoseService, 'detect_pose', self.handle_detect_pose)

        # Create the UploadPicture service client
        self.upload_picture_client = self.create_client(UploadPicture, 'upload_picture')
        while not self.upload_picture_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('UploadPictureService not available, waiting again...')


        self.get_logger().info("Detect Pose Service has been started.")

    def listener_callback(self, data):
        # 수신된 최신 이미지를 저장
        self.latest_frame = data

    def handle_detect_pose(self, request, response):
        if self.latest_frame is None:
            response.success = False
            response.message = "No image has been received yet."
            self.get_logger().warn(response.message)
            return response

        src = self.bridge.imgmsg_to_cv2(self.latest_frame)

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

            if rwrist[1] != 0 and rsholder[1] != 0 and rwrist[1] < rsholder[1]:
                self.get_logger().info("Cheeze")
                filename = datetime.now().strftime('%Y%m%d_%H%M%S') + '.jpg'
                filename2 = datetime.now().strftime('%Y%m%d_%H%M%S') + '_2' + '.jpg'
                cv2.imwrite(filename, src)

                result_image = result.plot()
                cv2.imwrite(filename2, result_image)

                img_msg = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
                self.get_logger().warn(f"Image saving complete {filename}")

                # Call UploadPictureService here
                self.call_upload_picture_service(filename, "Detected pose description", request.user_id)

                response.success = True
                response.message = f"Image saved as {filename} and {filename2}."
            else:
                response.success = False
                response.message = "No valid pose detected."

        return response


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
