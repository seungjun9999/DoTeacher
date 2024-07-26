#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()

        # 모델 파일의 절대 경로를 가져옵니다.
        package_share_directory = get_package_share_directory('usb_cam_pkg')
        model_path = os.path.join(package_share_directory, 'resource', 'yolov8n-pose.pt')

        # YOLOv8 Pose 모델 로드
        self.model = YOLO(model_path)

        self.get_logger().info("Image subscriber node has been started.")

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.recognize_marker(cv_image)

        except CvBridge.CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

    def recognize_marker(self, frame):
        # 모델을 사용하여 포즈 인식
        results = self.model(frame)

        # 여러 사람이 있을때 가장 큰 박스 내 사람 기준 
        boxes = results[0].boxes.xyxy.to('cpu').numpy()  # (x1, y1, x2, y2) 형식
        
        if len(boxes) > 0:
            areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])  # 박스의 넓이 계산
            max_area_index = areas.argmax()  # 가장 큰 박스의 인덱스
            largest_box = boxes[max_area_index]

        keypoints = results[0].keypoints

        # 결과를 화면에 표시
        annotated_frame = results[0].plot()  # 결과를 이미지에 그립니다.
        cv2.imshow('Pose Detection', annotated_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
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
