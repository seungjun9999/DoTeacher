# 🚗 **도선생: 전시회장 내 자율주행 안내 로봇**  
> **실내 자율주행, AI 기반 경로 안내, 음성 및 영상 인식 기능을 탑재한 지능형 가이드 로봇 & 어플리케이션**  

![도선생 메인](https://github.com/user-attachments/assets/b4d65cd5-5e8b-4bcd-8d4b-3c5e781d6a1d)  

🔗 **🎞️ 소개 영상**: [도선생 UCC](https://drive.google.com/file/d/14d044L9bcnZX_znx0uH11mlCFfubB7W1/view)  

---

## **📆 프로젝트 개요**  
- **진행 기간**: 2024.07.02 ~ 2024.08.16 (SSAFY 11기 공통 프로젝트)  
- **목표**: 전시회장에서 **사용자의 동선을 최적화하고, AI 기반 인터랙션을 제공하는 자율주행 로봇 & 어플리케이션** 개발  

---

## **🌟 주요 기능**  

### 🔹 **실내 자율주행 및 경로 안내**  
✅ **LiDAR 기반 장애물 회피 & SLAM Mapping**  
✅ **사용자 입력 기반 이동 (모바일 & Joystick 조작 지원)**  
✅ **전시 작품 추천 경로 생성 및 음성 가이드 제공**  

### 🔹 **AI 기반 사용자 인터랙션**  
✅ **음성 인식 기반 사용자 상호작용** (Google Cloud Speech, OpenAI GPT4o)  
✅ **객체 인식 및 특정 포즈 인식 기능** (YOLOv8 활용)  
✅ **사용자 맞춤 전시 추천 및 사진 촬영 기능**  

### 🔹 **모바일 애플리케이션 & 백엔드 시스템**  
✅ **QR 코드 기반 사용자-로봇 연결**  
✅ **전시 가이드 챗봇 기능 & 촬영된 사진 확인 가능**  
✅ **Spring Boot 기반 서버 구축 & MySQL / Redis 데이터베이스 관리**  

---

## **🛠 기술 스택**  

### **📌 Software**  
| 분류 | 기술 스택 |
|------|--------------------------------|
| **로봇 시스템** | ROS2 (ros2_control, Nav2) |
| **컴퓨터 비전** | YOLOv8, CUDA, Pytorch |
| **음성 인식** | Google Cloud Speech, OpenAI GPT4o-mini, Pyaudio |
| **모바일 앱** | Android (Hilt, Kotlin, MVVM, Retrofit) |
| **백엔드** | Spring Boot, MySQL, Redis |
| **CI/CD** | Jenkins, Docker |
| **클라우드 서버** | AWS EC2 |

### **📌 Hardware**  
| 분류 | 장비 |
|------|--------------------------------|
| **로봇 제어** | Jetson Orin Nano, Raspberry Pi 4/5 |
| **센서** | YDLIDAR (LiDAR 기반 자율주행) |
| **입출력 장치** | ABKO Speaker SLP20, ABKO Microphone MP16 |

---

## **🤝 협업 환경 및 관리**  

| 협업 툴 | 활용 내용 |
|------|--------------------------------|
| **Gitlab** | 팀원별 Branch 관리 & 기능별 ROS2 패키지 관리 |
| **JIRA** | 주 단위 Sprint 관리 (40시간 백로그 설정 후 진행) |
| **Notion** | 기능 학습 & 개발 과정 정리 |
| **Figma** | UI 목업 및 발표 자료 제작 |
| **MatterMost** | 팀 내 실시간 소통 및 파일 공유 |

---

## **👥 팀원 역할 분배**  

| 팀원 | 담당 역할 |
|------|--------------------------------|
| **이정재** (팀장) | 시스템 통합, ROS2, 하드웨어, 발표 |
| **박재영** | 영상 처리, 음성 처리, UCC 제작 |
| **박준수** | ROS2, 하드웨어 |
| **이재영** | ROS2, 하드웨어 |
| **조민기** | 영상 처리, 백엔드 |
| **최승준** | 모바일 애플리케이션, 백엔드, CI/CD |

---

## **📑 프로젝트 산출물**  

📌 **ERD 설계도**  
![도선생 ERD](https://github.com/user-attachments/assets/7dc5f7ad-6aae-4ca4-8827-36a75b81db1d)  

📌 **기능 명세서 & 포팅 매뉴얼 제공**

[포팅 매뉴얼](https://ssafy-d102.notion.site/d7948fd9c9d746878f0c7bfa6d56358f?pvs=74)

[기능 명세서](https://ssafy-d102.notion.site/21858dfbedb64c619d0a052ffbae95eb?pvs=4)
