from collections import defaultdict
import cv2
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors

# 추적 기록을 저장할 딕셔너리 초기화 (기본값은 빈 리스트)
track_history = defaultdict(lambda: [])

# YOLO 모델 로드
custom_model = YOLO("C:/Users/SSAFY/Desktop/yolo/yolov8train/runs/detect/train/weights/best.pt")
base_model = YOLO("yolov8n.pt")  # 기본 모델 경로

# 비디오 파일 열기
cap = cv2.VideoCapture(0)

while True:
    # 비디오에서 프레임 읽기
    ret, im0 = cap.read()
    if not ret:
        print("비디오 프레임이 비었거나 비디오 처리가 성공적으로 완료되었습니다.")
        break

    # 현재 프레임에서 객체 추적 수행 (커스텀 모델)
    custom_results = custom_model.track(im0, persist=True)
    
    # 현재 프레임에서 객체 추적 수행 (기본 모델)
    base_results = base_model.track(im0, persist=True)
    
    # 경계 상자와 추적 ID, 클래스 레이블 추출 및 주석 처리 (커스텀 모델)
    if custom_results[0].boxes.id is not None:
        bboxes = custom_results[0].boxes.xyxy.cpu().numpy()
        track_ids = custom_results[0].boxes.id.cpu().numpy()
        classes = custom_results[0].boxes.cls.cpu().numpy()
        for bbox, track_id, class_idx in zip(bboxes, track_ids, classes):
            x1, y1, x2, y2 = [int(coord) for coord in bbox]
            label = f"{custom_results[0].names[int(class_idx)]} {int(track_id)}"
            color = colors(int(track_id), True)
            cv2.rectangle(im0, (x1, y1), (x2, y2), color, 2)
            cv2.putText(im0, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # 경계 상자와 추적 ID, 클래스 레이블 추출 및 주석 처리 (기본 모델)
    if base_results[0].boxes.id is not None:
        bboxes = base_results[0].boxes.xyxy.cpu().numpy()
        track_ids = base_results[0].boxes.id.cpu().numpy()
        classes = base_results[0].boxes.cls.cpu().numpy()
        for bbox, track_id, class_idx in zip(bboxes, track_ids, classes):
            x1, y1, x2, y2 = [int(coord) for coord in bbox]
            label = f"{base_results[0].names[int(class_idx)]} {int(track_id)}"
            color = colors(int(track_id) + 1000, True)  # 색상을 다르게 하기 위해 +1000
            cv2.rectangle(im0, (x1, y1), (x2, y2), color, 2)
            cv2.putText(im0, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # 주석이 추가된 프레임을 표시
    cv2.imshow("instance-segmentation-object-tracking", im0)

    # 'q' 키가 눌리면 루프 종료
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# 비디오 캡처 객체 해제, 모든 OpenCV 창 닫기
cap.release()
cv2.destroyAllWindows()
