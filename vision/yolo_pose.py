import cv2
from ultralytics import YOLO
import mysql.connector
from playsound import playsound
from datetime import datetime
import requests
from datetime import datetime

# MySQL 데이터베이스에 연결

conn = mysql.connector.connect(
    host="localhost",
    user="root",
    password="1234",
    database="d102"
)
cursor = conn.cursor()

# 이미지 데이터를 바이너리로 읽는 함수
def read_image(file_path):
    with open(file_path, 'rb') as file:
        binary_data = file.read()
    return binary_data

# 이미지 데이터를 삽입하는 함수
def insert_image(binary_data):
    sql = "INSERT INTO images (image_data, saved_time) VALUES (%s, %s)"
    val = (binary_data, datetime.now())
    cursor.execute(sql, val)
    conn.commit()
    print(cursor.rowcount, "record inserted.")

url = 'http://localhost:8080/photo'
description = "This is a sample photo"

# 현재 user id session
user_id = 123
timestamp = datetime.now()

# 데이터 S3에 보내기
def send_image(image_path):
    with open(image_path, 'rb') as img:
        files = {'file': img}
        data = {
            'description': description,
            'userId': user_id
        }
        response = requests.post(url, files=files, data=data)


# YOLOv8 Pose 모델 로드
model = YOLO('yolov8n-pose.pt')

# 비디오 캡처 설정 (웹캠 사용)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # 모델을 사용하여 포즈 인식
    results = model(frame)
    
    #for result in results:
    result = results[0]

    #사람인지 아닌지?
    class_labels = result.names
    print(class_labels)

    #if class_labels[1] == 'person':
    #   print("사람")

    # 여러 사람이 있을때 가장 큰 박스 내 사람 기준 
    boxes = result.boxes.xyxy.to('cpu').numpy()  # (x1, y1, x2, y2) 형식
    #print("Boxes:", boxes[0])
    if len(boxes) > 0:
        areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])  # 박스의 넓이 계산
        max_area_index = areas.argmax()  # 가장 큰 박스의 인덱스
        largest_box = boxes[max_area_index]
        #print("Largest box:", largest_box)
    

    # *********************************************    
    # 사람 박스 사이즈가 일정 이하면 인식 안하는 로직 필요함
    # *********************************************    

    # 0번 == 코
    # 1번 == 오른쪽 눈
    # 2번 == 왼쪽 눈
    # 3번 == 오른쪽 귀
    # 4번 == 왼쪽 귀
    # 5번 == 오른쪽 어깨
    # 6번 == 왼쪽 어깨
    # 7번 == 오른쪽 팔꿈치
    # 8번 == 왼쪽 팔꿈치
    # 9번 == 오른쪽 손목
    # 10번 == 왼쪽 손목
    # 11번 == 오른쪽 골반
    # 12번 == 왼쪽 골반
    # 13번 == 오른쪽 무릎
    # 14번 == 왼쪽 무릎
    # 15번 == 오른쪽 발
    # 16번 == 왼쪽 발

    keypoints = result.keypoints
    #print(keypoints.xyn)
    rwrist, rwrist_prob =keypoints.xy[0,9].to('cpu').numpy(),keypoints.conf[0,9].item()
    rsholder, rsholder_prob = keypoints.xy[0,5].to('cpu').numpy(),keypoints.conf[0,5].item()
    
    cnt=0

    # 현재 오른쪽 손목이 오른쪽 어깨 위로 올라갔을경우 (카메라 바라보는 기준 화면상 오른쪽, 실제는 왼쪽)
    if rwrist[1] != 0 and rsholder[1] != 0 and rwrist[1] < rsholder[1]: 
        print("찰칵찰칵찰칵찰칵찰칵찰칵찰칵찰칵")
        playsound("C:/Users/SSAFY/Desktop/test1.mp3")
        ret, frame = cap.read()
        cnt +=1
        filename = 'captured_image_' + timestamp.strftime('%Y%m%d_%H%M%S') + '.jpg'
        cv2.imwrite(filename, frame)
        #send_image(filename)

         # 이미지를 바이너리 데이터로 변환
        #_, buffer = cv2.imencode('.jpg',frame)
        #binary_image = buffer.tobytes()
        #insert_image(binary_image)

    # *********************************************    
    # 정면을 보고있는 로직도 필요함
    # 촬영된 데이터 서버로 보내는 로직 필요함
    # *********************************************    


    # 결과를 화면에 표시
    annotated_frame = results[0].plot()  # 결과를 이미지에 그립니다.
    cv2.imshow('Pose Detection', annotated_frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 캡처 종료 및 모든 창 닫기
cap.release()
cv2.destroyAllWindows()
