# README

생성자: 이재영
생성 일시: 2024년 8월 16일 오후 4:52
상태: 진행 중
최종 편집자: 이재영

# 🚗전시회장 내 자율 주행 안내 로봇 서비스: “도선생”

![image](/uploads/309c169e26006e19c4c7b3d31cf69edf/image.png)

### 🎞️소개 영상 :  [도선생 UCC](https://drive.google.com/file/d/14d044L9bcnZX_znx0uH11mlCFfubB7W1/view)

---

## 📆프로젝트 진행기간

SSAFY 11기 2학기 공통프로젝트 “도선생”

2024.07.02 ~ 2024.08.16

## 🌟핵심 기능


- 실내 공간 자율주행 기능
    - LiDAR 기반 2D 데이터로 장애물 회피 기능
    - SLAM 기반 Mapping
    - 모바일 상호 작용 기반으로 다음 작품 이동
    - 메뉴얼 조작 기능 (Keyboard, Joystick)
- 추천 경로 기반 안내 기능
    - 사용자 선호도 기반 추천 작품 리스트에 따른 경로 생성
    - 해당 작품 좌표 기반 작품 음성 설명
- 음성 인식 기능
    - 특정 키워드 기반 상호작용 기능
- 영상 처리 기능
    - 특정 포즈를 통한 사진 촬영 기능
    - 커스텀 데이터 셋을 만들어서 객체 인식
- 모바일 어플리케이션
    - 로그인, 로그아웃 기능
    - QR 코드 활용 사용자-로봇 등록
    - 촬영된 사진 확인 가능
    - AI Chatbot 기능

## 주요 기술

- **SOFTWARE**
    - ROS2
        - ros2_control
        - Nav2
    - Computer-vision
        - Yolov8
        - CUDA
        - Pytorch
    - Speech-recognition
        - Google cloud speech(SST, TTS)
        - OpenAI gpt4o-mini
        - pyaudio
        - pygame
    - Mobile Application
        - Android
    - Backend
        - Spring Boot
        - DateBase : MySQL, Redis
    - CI/CD
        - Jenkins
        - Docker
    - Server : Amazon EC2
        
        
- **HARDWARE**
    - Jetson Orin Nano
    - Raspberry Pi 4/5
    - YDLIDAR
    - USB Speaker(ABKO speaker slp20)
    - USB microphone(ABKO microphone mp16)

## 협업 툴 및 환경

- Gitlab
    - 팀원별 Branch 활용
    - ROS2 패키지 기반 시스템 통합 및  기능별 관리
- JIRA
    - 매주 40시간의 백로그를 설정 후 Sprint 진행
- Notion
    - 기능 학습 및 구현 과정 정리
- Figma
    - 목업 및 PPT 제작
- MatterMost
    - 소통
    - 파일 공유

## 팀원 역할 분배

- 이정재(팀장) :  시스템 통합, ROS2, 발표, hardware
- 박재영(팀원) :  영상처리, 음성처리, UCC
- 박준수(팀원) :  ROS2, hardware
- 이재영(팀원) :  ROS2, hardware
- 조민기(팀원) : 영상처리, 백엔드
- 최승준(팀원) : 모바일 어플리케이션, 백엔드, CI/CD

## 프로젝트 산출물

- ERD

![image_1](/uploads/f02a82c98bd6eba95ddff7af1cd2d408/image_1.png)

- 기능명세서
- 포팅메뉴얼