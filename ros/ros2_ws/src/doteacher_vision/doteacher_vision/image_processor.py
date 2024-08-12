import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as Img
from cv_bridge import CvBridge
import cv2
import numpy as np

import os
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
from ultralytics.utils.plotting import colors
from PIL import Image, ImageDraw, ImageFont

import sys
sys.path.append('/mnt/data/')

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Img,
            'image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Img, 'image_processed', 10)

        self.bridge = CvBridge()
        package_share_directory = get_package_share_directory('doteacher_vision')
        model_path = os.path.join(package_share_directory, 'models/best.pt')

        # YOLOv8 Pose 모델 로드
        self.model = YOLO(model_path)

        self.get_logger().info("Image Processor node has been started.")


    def listener_callback(self, data):
        # self.get_logger().info('Receiving image data...')
        current_frame = self.bridge.imgmsg_to_cv2(data)

        # detect_picture.py의 함수를 사용하여 이미지 처리
        processed_frame = self.detect_target(current_frame)

        # 처리된 이미지를 다시 image_processed 토픽으로 발행
        self.publisher_.publish(self.bridge.cv2_to_imgmsg(processed_frame))
        # self.get_logger().info('Processed image published')


    def detect_target(self, src):
        font_path = "/usr/share/fonts/nanum/NanumGothic.ttf"
        font_size = 24
        font = ImageFont.truetype(font_path, font_size)

        # 현재 프레임에서 객체 추적 수행 (커스텀 모델)
        custom_results = self.model.track(src, persist=True)

        # OpenCV 이미지를 Pillow 이미지로 변환
        im_pil = Image.fromarray(cv2.cvtColor(src, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(im_pil)
        
        # 경계 상자와 추적 ID, 클래스 레이블 추출 및 주석 처리 (커스텀 모델)
        if custom_results[0].boxes.id is not None:
            bboxes = custom_results[0].boxes.xyxy.cpu().numpy()
            track_ids = custom_results[0].boxes.id.cpu().numpy()
            classes = custom_results[0].boxes.cls.cpu().numpy()
            confs = custom_results[0].boxes.conf.cpu().numpy()  # 신뢰도 추가
            for bbox, track_id, class_idx, conf in zip(bboxes, track_ids, classes, confs):
                if conf > 0.3:  # 신뢰도 임계값 설정
                    x1, y1, x2, y2 = [int(coord) for coord in bbox]
                    label = f"{self.model.names[int(class_idx)]} {int(track_id)} ({conf:.2f})"
                    color = colors(int(track_id), True)
                    draw.rectangle([x1, y1, x2, y2], outline=color, width=2)
                    draw.text((x1, y1 - font_size), label, font=font, fill=color)

            self.get_logger().warn(f"{self.model.names[int(classes[0])]} {int(track_ids[0])} ({conf:.2f})")
            

        # Pillow 이미지를 다시 OpenCV 이미지로 변환
        result = cv2.cvtColor(np.array(im_pil), cv2.COLOR_RGB2BGR)
        
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
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
