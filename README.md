# 🚙 Jackal-ORCA Project

**Jackal UGV project for Obstacle Recognition and Clustering-based Annihilation.**
ROS2 Humble 기반의 전술적 개척 UGV. Livox LiDAR와 2대의 RealSense 카메라를 융합하여 고밀도 맵을 생성하고, 클러스터링 알고리즘으로 가장 밀집된 장애물 구역을 찾아낸 뒤, 최적의 타격 지점으로 자율주행하여 자폭(개척) 임무를 수행합니다.

## 🌟 Overview

- **Sensor Fusion:** 2대의 Intel Depth Camera와 Livox LiDAR의 3D 포인트 클라우드 데이터 병합
- **Perception:** 노이즈 필터링, 장애물 인식 및 클러스터링(Clustering) 알고리즘 적용
- **Navigation:** SLAM 맵핑 및 ORCA 알고리즘 기반 능동형 회피 주행
- **Platform:** Clearpath Jackal UGV

## 🛠️ Environment

- **OS:** Ubuntu 22.04 LTS (Native or WSL2)
- **ROS Version:** ROS2 Humble
- **Hardware:** Intel RealSense (x2), Livox LiDAR

## 📂 Directory Structure

본 프로젝트는 커스텀 ROS2 패키지, 튜토리얼 샌드박스, 외부 오픈소스 의존성을 단일 레포지토리에서 통합 관리합니다. 레포지토리 최상단이 곧 ROS2 워크스페이스 역할을 합니다.

```text
Jackal-ORCA/
├── docs/                        # 시스템 아키텍처 다이어그램, 회로도, API 문서
├── tutorials/                   # 샌드박스 및 튜토리얼 (센서 구동 예제, 데이터 분석 등)
│   ├── livox_tutorial/
│   ├── realsense_tutorial/
│   └── slam_nav2_tutorial/
├── third_party/                 # 외부 오픈소스 패키지 관리 (.repos 파일 등)
└── src/                         # 메인 ROS2 커스텀 패키지
    ├── jackal_orca_bringup/     # 전체 시스템 및 센서 구동용 launch / yaml 파라미터
    ├── jackal_orca_description/ # 센서가 부착된 Jackal URDF / Xacro 로봇 모델링 파일
    ├── jackal_orca_perception/  # 센서 데이터 전처리, 장애물 탐지 및 클러스터링 노드
    ├── jackal_orca_navigation/  # SLAM, Nav2 및 회피 알고리즘 (ORCA) 노드
    ├── jackal_orca_core/        # 상태 머신 및 최상위 제어 노드
    └── jackal_orca_msgs/        # 커스텀 Message, Service, Action 정의
```

## 🚀 Getting Started

### 1. Clone the Repository

```bash
# 레포지토리를 복제하고 해당 폴더로 이동합니다.
git clone [https://github.com/](https://github.com/)[본인 깃허브 계정]/Jackal-ORCA.git
cd Jackal-ORCA
```

### 2. Install Dependencies

외부 오픈소스 패키지(RealSense, Livox 드라이버 등)는 `third_party` 폴더 내의 `.repos` 파일을 통해 관리됩니다. (추후 추가 예정)

```bash
# ROS2 환경 로드
source /opt/ros/humble/setup.bash
```

### 3. Build Workspace

본 레포지토리 자체가 ROS2 워크스페이스입니다. `Jackal-ORCA` 최상단 디렉토리에서 빌드를 진행합니다.

```bash
# 전체 패키지 빌드
colcon build --symlink-install

# 빌드된 환경 적용
source install/setup.bash
```
