# 🚙 Jackal-ORCA — Jetson Perception

**Jackal UGV 프로젝트의 Jetson(Orin) 인식 파트.**
RealSense D435i 2대(앞/뒤)의 컬러 영상으로 **AprilTag 검출 + YOLO 태그 탐지**를 수행한다.
SLAM·주행·미션 제어는 별도 Jackal mini PC가 담당하며, **이 레포는 Jetson에서 돌아가는
인식 노드/토픽만** 포함한다.

> 컴퓨트 분리: **Jetson** = GPU 추론(AprilTag·YOLO), **mini PC** = LiDAR·FAST-LIO2 SLAM·
> reactive_explorer·mission_node. Jetson은 TF/맵 프레임을 사용하지 않고 검출 결과만 토픽으로 보낸다.

## 🌟 처리 경로

- **apriltag_ros 경로** — 기하학적 정밀 검출 (태그 ID·자세)
- **YOLO 경로** — 학습 모델(YOLOv8s, TensorRT FP16) 기반 빠른 방향(bearing) 추정 →
  mini PC 시각 서보잉 피드백 (`/yolo/tag_candidate`)

자세한 노드/토픽/학습 설명은 **[`src/jackal_orca_perception/README.md`](src/jackal_orca_perception/README.md)** 참조.

## 🛠️ 환경

- **OS:** Ubuntu 22.04 / **ROS:** ROS 2 Humble / **HW:** Jetson Orin + Intel RealSense D435i ×2
- **추론 가속:** TensorRT FP16 엔진 (Jetson GPU 전용 — 기기에서 직접 변환)

## 📂 디렉토리 구조

레포 최상단이 곧 ROS 2 워크스페이스다.

```text
jackal-ORCA/
├── src/
│   ├── jackal_orca_perception/   # AprilTag + YOLO 인식 노드 (메인 패키지)
│   └── custom_msgs/              # TagCandidate / TagPose / TagPoseArray / MissionState
├── apriltag_print/               # 실물 인쇄용 태그 (36h11 150mm, A4 PDF/PNG)
└── yolo_train_kit/               # PC용 YOLO 학습 키트 (데이터 생성·학습·TensorRT 변환)
```

## 🚀 빌드 & 실행

```bash
# 빌드
cd ~/ros2_ws/jackal-ORCA
colcon build --packages-select custom_msgs jackal_orca_perception
source install/setup.bash

# 실제 미션용 파이프라인 (카메라 + apriltag + YOLO)
ros2 launch jackal_orca_perception apriltag_pipeline.launch.py enable_yolo:=true

# bbox 웹 모니터까지 (브라우저로 검출 영상 확인)
ros2 launch jackal_orca_perception yolo_web_monitor.launch.py
```

> ⚠️ `custom_msgs`(특히 `TagCandidate`)를 먼저 빌드해야 YOLO 노드가 빌드/실행된다.

## 🌐 웹페이지로 검출 영상 보기

`yolo_web_monitor.launch.py`가 카메라+AprilTag+YOLO와 함께 **`web_video_server`**를 띄운다.
모니터가 없는 Jetson을 SSH로만 붙어도, 같은 네트워크의 브라우저에서 검출 영상을 실시간으로 볼 수 있다.

```bash
ros2 launch jackal_orca_perception yolo_web_monitor.launch.py        # 포트 변경: port:=9090
```

브라우저 접속 (USB 직결 `192.168.55.1`, Tailscale `100.75.100.71`, 또는 Jetson IP):

| 화면 | URL |
|---|---|
| 전체 토픽 목록 | `http://<jetson-ip>:8080` |
| 앞 카메라 bbox | `http://<jetson-ip>:8080/stream?topic=/yolo/debug_image_front&type=mjpeg&quality=50&width=640` |
| 뒤 카메라 bbox | `http://<jetson-ip>:8080/stream?topic=/yolo/debug_image_back&type=mjpeg&quality=50&width=640` |

- bbox 색상: **녹색 = 확정**(conf ≥ 0.55) / 노란색 = 후보(≥ 0.30) / 회색 = 미달.
- 화질/해상도는 **URL 쿼리 파라미터**다 (`quality` 1–100, `width`/`height` 다운스케일).
- `/yolo/debug_image_*`는 **구독자가 있을 때만(=브라우저 탭을 열었을 때만)** 그려 발행하므로,
  평상시(탭 닫음)에는 오버레이 연산 부하가 0이다.
