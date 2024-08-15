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
from doteacher_interfaces.srv import Nav2Index, DetectedImage

class DetectPictureService(Node):
    def __init__(self):
        super().__init__('detect_picture_service')
        self.subscription = self.create_subscription(
            Img,
            'image_raw',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Img, 'detect_picture', 10)
        self.bridge = CvBridge()
        model_path = os.path.join(get_package_share_directory('doteacher_vision'), 'models/best.pt')

        # YOLOv8 모델 로드
        self.model = YOLO(model_path)

        self.latest_frame = None  # 가장 최근에 수신된 이미지를 저장할 변수

        # 서비스 생성
        self.srv = self.create_service(DetectedImage, 'detect_image', self.handle_detect_image)

        self.get_logger().info("Detect Picture Service has been started.")

    def listener_callback(self, data):
        # 수신된 최신 이미지를 저장
        self.latest_frame = data

    def handle_detect_image(self, request, response):
        if self.latest_frame is None:
            response.success = False
            response.message = "No image has been received yet."
            self.get_logger().warn(response.message)
            return response

        current_frame = self.bridge.imgmsg_to_cv2(self.latest_frame)

        # 이미지를 처리
        processed_frame, track_id = self.detect_target(current_frame)

        # 처리된 이미지를 ROS 이미지로 변환
        detected_image = self.bridge.cv2_to_imgmsg(processed_frame)

        # 결과를 응답에 설정
        response.success = True
        response.message = "Image processed successfully."
        response.detected_image = detected_image
        response.track_id = track_id

        # 처리된 이미지를 토픽으로 발행
        self.publisher_.publish(detected_image)

        return response

    def detect_target(self, src):
        font_path = "/usr/share/fonts/nanum/NanumGothic.ttf"
        font_size = 24
        font = ImageFont.truetype(font_path, font_size)

        # 현재 프레임에서 객체 추적 수행
        custom_results = self.model.track(src, persist=True)

        # OpenCV 이미지를 Pillow 이미지로 변환
        im_pil = Image.fromarray(cv2.cvtColor(src, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(im_pil)
        
        result_track_id = -1
        
        # 경계 상자와 추적 ID, 클래스 레이블 추출 및 주석 처리
        if custom_results[0].boxes.id is not None:
            bboxes = custom_results[0].boxes.xyxy.cpu().numpy()
            track_ids = custom_results[0].boxes.id.cpu().numpy()
            classes = custom_results[0].boxes.cls.cpu().numpy()
            confs = custom_results[0].boxes.conf.cpu().numpy()  # 신뢰도 추가
            for bbox, track_id, class_idx, conf in zip(bboxes, track_ids, classes, confs):
                if conf > 0.65:  # 신뢰도 임계값 설정
                    x1, y1, x2, y2 = [int(coord) for coord in bbox]
                    label = f"{self.model.names[int(class_idx)]} {int(track_id)} ({conf:.2f})"
                    color = colors(int(track_id), True)
                    draw.rectangle([x1, y1, x2, y2], outline=color, width=2)
                    draw.text((x1, y1 - font_size), label, font=font, fill=color)

            self.get_logger().warn(f"{self.model.names[int(classes[0])]} {int(track_ids[0])} ({conf:.2f})")
            result_track_id = int(track_ids[0])
            

        # Pillow 이미지를 다시 OpenCV 이미지로 변환
        result = cv2.cvtColor(np.array(im_pil), cv2.COLOR_RGB2BGR)
        
        return result, result_track_id

def main(args=None):
    rclpy.init(args=args)
    node = DetectPictureService()
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
