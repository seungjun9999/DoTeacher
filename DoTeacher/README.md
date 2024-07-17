# SSAFY D102 내가 왕이될 상이조
## 프로젝트 기획

<center>

### 전시회장 내 자율 주행 안내 로봇 서비스


[<img src="https://img.shields.io/badge/gitlab-FC6D26?style=for-the-badge&logo=gitlab&logoColor=white">](https://lab.ssafy.com/s11-webmobile3-sub1/S11P11D102) [<img src="https://img.shields.io/badge/jira-0052CC?style=for-the-badge&logo=jira&logoColor=white">](https://ssafy.atlassian.net/jira/software/c/projects/S11P11D102/boards/6854) [<img src="https://img.shields.io/badge/notion-000000?style=for-the-badge&logo=notion&logoColor=white">](https://www.notion.so/ssafy-d102/SSAFY-D102-6859abc68ab94acf90dce7437171dadf
)[<img src="https://img.shields.io/badge/figma-F24E1E?style=for-the-badge&logo=figma&logoColor=white">](https://www.figma.com/files/team/1392453004303357605/all-projects?fuid=1372739892237720013)<br>
<img src="https://img.shields.io/badge/python-3776AB?style=for-the-badge&logo=python&logoColor=white">
<img src="https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white">
<img src="https://img.shields.io/badge/ROS-22314E?style=for-the-badge&logo=ros&logoColor=white">
<img src="https://img.shields.io/badge/ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white">
</center>

#### 핵심 기능
1. 실내 공간 자율 주행 기능
- 사람, 사물, 장애물을 실시간으로 인식하고 회피하는 기능
- 출발, 도착을 앱을 통해 명령하는 기능
2. 추천 경로 기반 안내 기능
- 사용자가 모바일 앱으로 선호 경로를 선택하는 기능(동선 또는 선호 작품 순서)
- 화장실, 엘리베이터 등 부대시설 경로 안내 기능
3. 영상 처리 기능
- 사람과 사물을 인식하는 기능
- 자율 주행으로 작품의 위치로 이동 후 작품을 인식하는 기능
4. 음성 설명 기능
- 영상 처리로 작품을 인식하고 작품에 대한 설명을 음성으로 전달하는 기능
5. 사용자의 경로 이탈 시 추적 기능(예정)
- 사용자가 경로를 이탈했을 때, 사용자를 추적하여 따라가는 기능
- 사용자가 경로에서 이탈하면 새로운 경로를 생성하는 기능

#### 요구사항 정의
1. 자율 주행
- 특정 위치로 이동
- 위치별 경로 생성
    - 위치들에 대한 최적 경로(물리적 가까운 거리)
    - 사용자가 제시한 경로대로 진행하다 취소(폐지)
- 장애물 회피
    - 2D/3D LiDAR 기반 장애물 회피 경로 생성
- 하나의 대상을 기준으로 진행
    - 음성 또는 터치 인터페이스를 사용해서 다음 작품이나 위치로 이동
    - 인터페이스를 사용해서 일시정지 기능
- 매뉴얼 조작 기능(Bluetooth)
2. 음성 안내
- 음성 TTS 기능
- 작품 설명 다시 듣기 기능
- 말하고 있는 텍스트를 표시(디스플레이 또는 모바일 앱)
- Input Voice To Text 변환 기능
- (추가 기능)작품에 대한 세부 대화 기능 - AI
3. 영상 처리
- 사람 및 사물 인식 기능
- Maker 기반의 영상 처리로 작품의 위치를 인식하는 기능
4. 모바일 어플리케이션
- 사용자 정보 입력 기능
    - 로그인 기능
- 안내 로봇 서비스 시작 기능
    - 블루투스 연동
- 다시 듣기 기능
- 말하고 있는 텍스트 표시

## Confirm
### 24.07.08 Mon
#### 스마트 옷장
- 핵심 기능: 착장 추천 및 옷 관리 기능
    - 날씨 데이터 기준 착장 추천
    - 과거 정보 기반 추천
    - 홀드 케이스 처리, 사용자의 입력 데이터 기반 추천
    - 나와 비슷한 성향의 사람들에 대한 정보 기준(빅데이터 추천)
    - 현재 입은 옷에 대한 정보에 따라 모델링
- 우려점
    - 옷장 안의 옷 학습의 어려움
    - 옷의 종류(타입)
        - 사용자에 따른 Fit에 매칭
        - 옷장에 걸려 있는 옷에 대한 인식 및 처리
        - 사용자의 선호 스타일 처리
        - 옷 보관보다 추천에 가까운 기능으로 개발될 수 있음
        - 추천 알고리즘이 사용자 옷에 대한 정보 기반인지, 빅데이터 기반인지
    - 모바일에 집중
        - 어플리케이션에서 과거 착장 정보 확인
        - UI/UX 특화 필요
        - 사용자에 따라 다르게 적용할 수 있는 기능을 특화
        - 스마트 거울 같은 시스템
    - 다른 착장에 대한 모델링 처리 시 예외 발생(예시 긴팔 -> 반팔)

#### 자율 주행 택배 이송 시스템
- 핵심 기능: 실내 공간에서의 자율 주행
    - 실내 공간 정보를 맵핑
    - 맵핑 정보를 기반으로 자율 주행 기능
    - 사용자와 상호 작용할 수 있는 AI 음성 모델 및 명령 처리
- 우려점
    - 사용자가 직접 받아야하는 구조인가?
    - 시스템이 물건을 내려놓는 기능이 있는가?
    - 단순히 물건을 옮기는 기능만 수행하는가?
    - 경로 상에서 이득이 되는 다른 기능은 없는가?

#### 보완 요구사항
- 설치형 아이디어도 고려
- 기능의 추가가 필요
- AI를 기반으로 IoT에 연동

### 24.07.10 Wed
#### 무인 택배 이송 모빌리티
- 프로젝트 기획 보완점
    - 사용자의 역할에 따른 기능 분류 필요
    - 기술적 목표가 아닌 서비스 중심의 대주제 선정
    - 서비스 기능에 따른 업무 재분류 필요
    - 페르소나 설정
        - 페르소나를 구체화하여 기획 아이디어를 확정
        - 예시 거동이 불편한 사용자를 대상으로한 물품 전달 등
    - 프로젝트의 배경과 의문에 대한 해소 필요
    - 서비스 어플리케이션 기반 아이디어 논의 필요


## History
### 무인 택배 배달로봇 (가제) 

### SSAFY 11기 구미 1반 D102 (AIoT) 
| 이름   | 역할 |
|--------|------|
| 이정재 (팀장) |ROS SLAM|
| 박재영 |영상 처리|
| 박준수 |모터 제어 임베디드|
| 이재영 |ROS SLAM|
| 조민기 |영상 처리|
| 최승준 |모바일| 
<br>
- 실내 공간에서의 택배 배달 과정을 개선하여, 배달 시간 단축, 인건비 절감, 그리고 사용자 만족도 향상을 이루고자 합니다.
- 해당 기술이 다양한 실내 서비스 분야로 확장될 수 있는 기반을 마련

## 목표

- 자율주행 로봇 설계 및 개발
- 인간-로봇 상호작용 인터페이스 개발
- 안정성 평가 및 테스트 

## 업무 분담
- ### S/W
    - 영상 처리 (face, object detection)
    - ROS SLAM
    - 모터 제어 임베디드
    - 주행 알고리즘
    - 모바일 : 상태 확인 및 알림
    - DB (예정)
- ### H/W
    - 외형 설계 및 조립
    - Lidar 설치, H/W 보완