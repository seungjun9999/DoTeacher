import cv2


def main():
    # 장치 인덱스를 변경해가며 시도
    device_index = 0  # 또는 1로 변경해서 시도해 보세요

    # 웹캠을 캡처할 VideoCapture 객체 생성
    cap = cv2.VideoCapture(device_index)

    if not cap.isOpened():
        print(f"웹캠을 열 수 없습니다. 장치 인덱스: {device_index}")
        exit()

    while True:
        # 프레임 읽기
        ret, frame = cap.read()

        if not ret:
            print("프레임을 가져올 수 없습니다.")
            break

        # 프레임을 윈도우에 표시
        cv2.imshow('Webcam', frame)

        # 'q' 키를 누르면 루프 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 리소스 해제
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
