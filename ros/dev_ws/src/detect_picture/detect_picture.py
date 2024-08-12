from collections import defaultdict
import cv2
from ultralytics import YOLO
from ultralytics.utils.plotting import colors
from PIL import Image, ImageDraw, ImageFont
import numpy as np

# 추적 기록을 저장할 딕셔너리 초기화 (기본값은 빈 리스트)
track_history = defaultdict(lambda: [])

# YOLO 모델 로드
custom_model = YOLO("best.pt")

# 비디오 파일 열기
cap = cv2.VideoCapture(0)

# 한글을 지원하는 폰트 경로 (예: 나눔고딕 폰트 경로, 시스템에 맞는 폰트 파일을 사용하세요)
font_path = "/usr/share/fonts/nanum/NanumGothic.ttf"  # 윈도우즈의 기본 폰트 예시

# Pillow에서 사용할 폰트 로드
font_size = 24
font = ImageFont.truetype(font_path, font_size)

while True:
    # 비디오에서 프레임 읽기
    ret, im0 = cap.read()
    if not ret:
        print("비디오 프레임이 비었거나 비디오 처리가 성공적으로 완료되었습니다.")
        break

    # 현재 프레임에서 객체 추적 수행 (커스텀 모델)
    custom_results = custom_model.track(im0, persist=True)
 
    # OpenCV 이미지를 Pillow 이미지로 변환
    im_pil = Image.fromarray(cv2.cvtColor(im0, cv2.COLOR_BGR2RGB))
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
    im0 = cv2.cvtColor(np.array(im_pil), cv2.COLOR_RGB2BGR)

    # 주석이 추가된 프레임을 표시
    cv2.imshow("instance-segmentation-object-tracking", im0)

    # 'q' 키가 눌리면 루프 종료
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# 캡처 객체 해제, 모든 OpenCV 창 닫기
cap.release()
cv2.destroyAllWindows()
