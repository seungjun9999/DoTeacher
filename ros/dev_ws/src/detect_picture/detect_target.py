from collections import defaultdict
import cv2
from ultralytics import YOLO
from ultralytics.utils.plotting import colors
from PIL import Image, ImageDraw, ImageFont
import numpy as np

# # 가상환경 활성화
# venv_path = os.path.expanduser('~/dev_ws/.venv/bin/activate_this.py')
# with open(venv_path) as f:
#     exec(f.read(), {'__file__': venv_path})


def detect_target(src):
    # load model
    custom_model = YOLO("/home/jetson/dev_ws/src/detect_picture/best.pt")

    font_path = "/usr/share/fonts/nanum/NanumGothic.ttf"
    font_size = 24
    font = ImageFont.truetype(font_path, font_size)

    # 현재 프레임에서 객체 추적 수행 (커스텀 모델)
    custom_results = custom_model.track(src, persist=True)

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
                label = f"{custom_model.names[int(class_idx)]} {int(track_id)} ({conf:.2f})"
                color = colors(int(track_id), True)
                draw.rectangle([x1, y1, x2, y2], outline=color, width=2)
                draw.text((x1, y1 - font_size), label, font=font, fill=color)

    # Pillow 이미지를 다시 OpenCV 이미지로 변환
    result = cv2.cvtColor(np.array(im_pil), cv2.COLOR_RGB2BGR)
    return result
