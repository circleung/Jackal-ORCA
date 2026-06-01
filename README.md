# 민석 파트 — YOLO AprilTag 탐지 및 기록 노드

ROS2 Humble 기반 Jackal UGV 프로젝트에서 **앞뒤 RealSense 카메라**로 AprilTag를 탐지하고, 태그 ID와 SLAM 좌표를 기록하는 파트입니다.

---

## 담당 역할

```
[앞/뒤 RealSense 카메라]
        ↓
[tag_yolo_detector_node]   ← YOLO로 태그 존재 여부 탐지 → confidence 발행
        ↓
  /tag_detected (Bool)     ← 제어팀 Goal Manager가 구독 → 목적지를 태그로 변경
        ↓
[태그 앞 도착 후]
        ↓
[tag_recorder_node]        ← apriltag_ros로 태그 ID 읽고 SLAM 좌표와 함께 저장
```

---

## 파일 목록

| 파일 | 설명 |
|---|---|
| `tag_yolo_detector_node.py` | 앞/뒤 카메라 이미지를 YOLO로 추론, confidence 발행 |
| `tag_recorder_node.py` | 태그 앞 도착 신호 수신 시 ID + 좌표 CSV 저장 |
| `apriltag_yolo.pt` | 학습된 YOLOv8 모델 (mAP50 93.8%) |
| `generate_dataset.py` | 합성 학습 데이터셋 생성 스크립트 |
| `train_yolo.py` | YOLOv8 학습 스크립트 |

---

## 토픽 인터페이스

### tag_yolo_detector_node 발행 토픽

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/tag_confidence` | `std_msgs/Float32` | YOLO confidence 값 (0.0 ~ 1.0) |
| `/tag_detected` | `std_msgs/Bool` | threshold(0.35) 초과 시 True 발행 |
| `/tag_bbox` | `std_msgs/String` | JSON 형태의 bbox 위치 정보 (디버그용) |

### tag_yolo_detector_node 구독 토픽

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/camera_front/color/image_raw` | `sensor_msgs/Image` | 앞 카메라 |
| `/camera_rear/color/image_raw` | `sensor_msgs/Image` | 뒤 카메라 |

### tag_recorder_node 구독 토픽

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/at_tag_position` | `std_msgs/Bool` | 제어팀 → 태그 앞 도착 신호 |
| `/apriltag/detections` | `apriltag_msgs/AprilTagDetectionArray` | apriltag_ros 탐지 결과 |

### tag_recorder_node 발행 토픽

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/recorded_tag_positions` | `geometry_msgs/PoseArray` | 저장된 태그 좌표 목록 |

---

## 설치 방법

### 의존 패키지

```bash
sudo apt install ros-humble-apriltag-ros ros-humble-apriltag-msgs
pip3 install ultralytics
```

### 패키지 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select jackal_mine_detection --symlink-install
source install/setup.bash
```

---

## YOLO 모델 학습 방법 (재학습이 필요한 경우)

### 1단계: 데이터셋 생성

네트워크 없이 로컬에서 합성 데이터셋을 자동 생성합니다.

```bash
pip3 install opencv-python numpy
python3 generate_dataset.py
```

- 생성 위치: `datasets/apriltag/`
- 소요 시간: 약 1분
- 생성 이미지 수: 2000장 (학습 1700장 / 검증 300장)

실제 AprilTag36h11 이미지를 GitHub에서 다운로드해 사용할 수도 있습니다:

```bash
python3 -c "
import urllib.request
from pathlib import Path
Path('datasets/tag_src').mkdir(parents=True, exist_ok=True)
for i in range(10):
    url = f'https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/tag36_11_{i:05d}.png'
    out = f'datasets/tag_src/tag36_11_{i:05d}.png'
    try:
        urllib.request.urlretrieve(url, out)
        print(f'tag {i} 완료')
    except Exception as e:
        print(f'tag {i} 실패: {e}')
"
```

### 2단계: 학습

```bash
pip3 install ultralytics
python3 train_yolo.py
```

- GPU 사용 (RTX 5080 기준 약 10~20분)
- 학습 완료 후 모델 저장 위치: `models/apriltag_yolo.pt`

### 학습 결과

| 지표 | 값 |
|---|---|
| mAP50 | **93.8%** |
| mAP50-95 | 84.8% |
| Precision | 83.2% |
| Recall | 89.8% |

---

## 실행 방법

### 탐지 노드 실행

```bash
# 모델 경로를 지정해서 실행
ros2 run jackal_mine_detection tag_yolo_detector_node --ros-args \
  -p model_path:=/home/minseok/ros2_ws/src/jackal_mine_detection/models/apriltag_yolo.pt \
  -p confidence_threshold:=0.35 \
  -p front_topic:=/camera_front/color/image_raw \
  -p rear_topic:=/camera_rear/color/image_raw
```

### 기록 노드 실행

```bash
ros2 run jackal_mine_detection tag_recorder_node
```

- 기록 파일 저장 위치: `~/ros2_ws/tag_records_MMDD_HHMM.csv`
- CSV 형식: `tag_id, map_x, map_y`

---

## 전체 시스템 연동 흐름

```
[Fast-LIO2 3D SLAM]
  → TF (map → base_link)
  → /map (OccupancyGrid)

[tag_yolo_detector_node]  ← 민석
  → /tag_detected (Bool)
         ↓
[Goal Manager]            ← 제어팀
  → 태그 위치를 /goal_pose로 발행

[Global Planner (A*)]     ← 제어팀
  → /path 발행

[Local Planner (Pure Pursuit)] ← 제어팀
  → /cmd_vel 발행 (태그 앞까지 이동)

[태그 앞 도착]
  → /at_tag_position (Bool) 발행  ← 제어팀

[tag_recorder_node]       ← 민석
  → apriltag_ros로 tag ID 읽기
  → SLAM 좌표 조회 (TF)
  → CSV 저장 + /recorded_tag_positions 발행

[탐사 재개]
  → Goal Manager가 다시 Frontier 목적지로 전환
```

---

## 주의사항

1. **confidence threshold**: 기본값 `0.35` — 낮을수록 민감하게 탐지 (오탐지 증가), 높을수록 확실할 때만 탐지
2. **apriltag_ros 필요**: tag_recorder_node가 실제 태그 ID를 읽을 때 apriltag_ros 패키지 사용
3. **카메라 TF 확인**: `camera_front_color_optical_frame`, `camera_rear_color_optical_frame`이 TF 트리에 있어야 함
4. **SLAM 좌표계**: TF에서 `map → base_link` 변환이 있어야 좌표 저장 가능 (Fast-LIO2 실행 중이어야 함)
