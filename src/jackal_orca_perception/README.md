# jackal_orca_perception

Jetson(Orin) 측 **AprilTag 인식 + YOLO 태그 탐지** 패키지.
RealSense D435i 2대(앞/뒤)의 컬러 영상을 받아 두 경로로 처리한다.

- **apriltag_ros 경로** — 기하학적 정밀 검출 (태그 ID·자세)
- **YOLO 경로** — 학습 모델 기반 빠른 방향(bearing) 추정 → mini PC 시각 서보잉 피드백

> Jetson은 **TF/맵 프레임을 사용하지 않는다.** bearing은 camera_info intrinsic과
> 카메라 장착각만으로 계산하며, 맵 프레임 매핑·서보잉 제어는 mini PC가 담당한다.

---

## 1. 빠른 시작 (Quick Start)

### 1-1. bbox 웹 모니터 (앞/뒤 영상 + 바운딩박스)

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/jackal-ORCA/install/setup.bash
ros2 launch jackal_orca_perception yolo_web_monitor.launch.py
```

카메라(color-only) + apriltag + YOLO + web_video_server 를 한 번에 기동한다.

브라우저 접속 (USB 직결 `192.168.55.1`, Tailscale `100.75.100.71`). **경량(저품질 MJPEG) 표준 URL**:

| 화면 | URL |
|---|---|
| 앞 카메라 bbox | `http://<jetson-ip>:8080/stream?topic=/yolo/debug_image_front&type=mjpeg&quality=50&width=640` |
| 뒤 카메라 bbox | `http://<jetson-ip>:8080/stream?topic=/yolo/debug_image_back&type=mjpeg&quality=50&width=640` |
| 전체 토픽 목록 | `http://<jetson-ip>:8080` |

bbox 색상: **녹색 = 확정**(conf ≥ 0.55) / 노란색 = 후보(≥ 0.30) / 회색 = 미달.

> **경량화 메모**: 런치는 `default_stream_type=mjpeg`로 고정(png는 무손실이라 더 무겁다).
> JPEG 품질·해상도는 web_video_server의 **URL 쿼리 파라미터**다 (노드 파라미터 아님):
> `quality`(1–100, 기본 80), `width`/`height`(다운스케일). 위 표준 URL이 `quality=50&width=640`.
> 브라우저 탭을 닫으면 디버그 영상은 *구독자 0 → 발행 중단*이라 부하가 0이 된다.

옵션:
```bash
# 포트 변경 / 뒤 카메라 끄기
ros2 launch jackal_orca_perception yolo_web_monitor.launch.py port:=9090 enable_back:=false
```

### 1-2. 실제 미션용 파이프라인 (웹서버 없이)

```bash
ros2 launch jackal_orca_perception apriltag_pipeline.launch.py enable_yolo:=true
```

| 인자 | 기본값 | 설명 |
|---|---|---|
| `enable_yolo` | `false` | tag_yolo_detector_node 기동 (ultralytics 필요) |
| `enable_back` | `true` | 뒤 카메라/apriltag 스택 |
| `auto_record` | `true` | `/at_tag_position` 신호 없이 검출만으로 저장 |
| `front_serial` / `back_serial` | 아래 표 | RealSense 시리얼 |

### 1-3. 빌드

```bash
cd ~/ros2_ws/jackal-ORCA
colcon build --packages-select custom_msgs jackal_orca_perception
```

> ⚠️ `custom_msgs` 는 `/home/jetson/jackal_project_shared/custom_msgs` 에 대한 심볼릭 링크다.
> `TagCandidate` 메시지를 정의하므로 **삭제하면 YOLO 노드가 빌드/실행되지 않는다.**

---

## 2. 하드웨어 구성

| 항목 | 내용 |
|---|---|
| 카메라 | Intel RealSense D435i ×2 |
| 앞(front) 시리얼 | `344522070059` (mount_yaw = 0) |
| 뒤(back) 시리얼 | `344522070202` (mount_yaw = π) |
| 스트림 | **color only** — depth/infra/IMU 비활성 (대역폭·연산 절약) |
| 추론 가속 | TensorRT FP16 엔진 (Jetson GPU) |

---

## 3. 노드 구성

```
RealSense D435i ×2 (color only)
  /camera_front/color/image_raw ──┬───────────────┐
  /camera_back/color/image_raw  ──┤               │
        + .../camera_info         ▼               ▼
                    ┌──────────────────┐  ┌────────────────────────┐
                    │ apriltag_node ×2 │  │ tag_yolo_detector_node │
                    │ (apriltag_ros)   │  │ (YOLOv8 + TensorRT)    │
                    └────────┬─────────┘  └───────────┬────────────┘
   /apriltag_{front,back}/detections     /yolo/tag_candidate ─→ (mini PC servoing)
                             │            /tag_detected, /tag_confidence
                             │            /yolo/debug_image_{front,back} ─→ web
                             ▼
                    ┌──────────────────┐
                    │ tag_recorder_node│ → CSV + /recorded_tag_positions
                    └──────────────────┘
```

### 3-1. `apriltag_node` ×2 (apriltag_ros)
- 설정: `config/apriltag.yaml` — family **36h11**, tag size **0.15 m**, decimate 2.0, threads 2, refine on
- remap: `image_rect`←`/camera_{side}/color/image_raw`, `camera_info`←동일, `detections`→`/apriltag_{side}/detections`
- 출력: `apriltag_msgs/AprilTagDetectionArray` (정확한 태그 ID·코너·자세)

### 3-2. `tag_yolo_detector_node.py` (학습 모델 경로)
`scripts/tag_yolo_detector_node.py`
- **모델 자동 선택**: `.engine`(TensorRT)이 `.pt`보다 최신이면 우선 로드, 아니면 `.pt` 폴백 (mtime 비교)
- 앞/뒤 컬러 + camera_info 구독, **`inference_every_n_frames`(기본 1)** 프레임마다 추론 (1=매 프레임, 카메라 FPS와 동일)
- **bearing 계산**: `bearing = mount_yaw + atan2(cx − u, fx)` — bbox 중심 픽셀 u, intrinsic(fx, cx), 장착각. camera_info 수신 전에는 candidate 미발행.
- 발행:
  - `/yolo/tag_candidate` (`custom_msgs/TagCandidate`: header, source_camera, bearing_rad, range_m=−1, confidence) → **mini PC mission_node 서보잉 피드백**
  - `/tag_detected` (Bool, conf ≥ 0.55 이벤트), `/tag_confidence` (Float32)
  - `/yolo/debug_image_{front,back}` (Image) — bbox 오버레이. **구독자가 있을 때만 그려 발행**(평시 부하 0)

주요 파라미터:

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `model_path` | 자동(.engine 우선) | 모델 경로 |
| `confidence_threshold` | 0.55 | `/tag_detected` 이벤트 임계 |
| `candidate_threshold` | 0.30 | `/yolo/tag_candidate` 발행 임계 |
| `inference_every_n_frames` | 1 | 추론 주기 (1=매 프레임, 부하 ↔ 반응성) |
| `front_mount_yaw` / `back_mount_yaw` | 0 / π | base_link 기준 카메라 장착각 |

### 3-3. `tag_recorder_node.py` (디버그 백업 기록)
`scripts/tag_recorder_node.py`
- 구독: `/apriltag_{front,back}/detections`, `/at_tag_position`(Bool, 제어팀 도착 신호 — `auto_record=true`면 무시)
- 검출 순간 map→base_link 위치를 TF로 읽어 **CSV(`~/ros2_ws/tag_records_MMDD_HHMM.csv`: tag_id, map_x, map_y)** 저장
- 발행: `/recorded_tag_positions`(PoseArray), `/tag_saved_event`(UInt32 — 자칼 사운드 트리거)
- 정식 맵-프레임 기록은 mini PC `tag_mapper` 가 담당. 이 노드는 단순 백업이며 mini PC SLAM TF가 DDS로 보여야 동작한다.

---

## 4. 토픽 인터페이스 요약

| 토픽 | 타입 | 방향 |
|---|---|---|
| `/camera_{front,back}/color/image_raw` | sensor_msgs/Image | RealSense → |
| `/camera_{front,back}/color/camera_info` | sensor_msgs/CameraInfo | RealSense → |
| `/apriltag_{front,back}/detections` | apriltag_msgs/AprilTagDetectionArray | apriltag_node → |
| `/yolo/tag_candidate` | custom_msgs/TagCandidate | YOLO → mini PC |
| `/tag_detected` `/tag_confidence` | std_msgs/Bool, Float32 | YOLO → (디버그) |
| `/yolo/debug_image_{front,back}` | sensor_msgs/Image | YOLO → web |
| `/recorded_tag_positions` | geometry_msgs/PoseArray | recorder → |
| `/tag_saved_event` | std_msgs/UInt32 | recorder → 사운드 |

---

## 5. YOLO 모델 & 학습

### 5-1. 모델 사양

| 항목 | 값 |
|---|---|
| 베이스 모델 | **YOLOv8s** (Ultralytics, small) |
| 태스크 | object detection (AprilTag bbox) |
| 클래스 | 10개 — tag_id 0~9 |
| 입력 해상도 | 640 × 640 |
| 학습 | epochs 100, batch 16, imgsz 640, lr0 0.01 |
| 배포 모델 (PyTorch) | `models/apriltag_yolo.pt` (≈22 MB) |
| 배포 모델 (TensorRT FP16) | `models/apriltag_yolo.engine` (≈24 MB, Jetson 전용) |

> TensorRT 엔진은 **빌드한 디바이스(GPU/TensorRT 버전)에 종속** — PC에서 만든 `.engine`은 Jetson에서 동작하지 않으므로 반드시 Jetson에서 변환한다.

### 5-2. 데이터셋
두 종류를 병합해 사용 (`prepare_dataset.py`가 있으면 combined, 없으면 합성만):

- **합성 데이터** (`generate_dataset.py`): 네트워크 없이 로컬에서 AprilTag(36h11 스타일) 패턴을 생성하고 원근 warp·배경 합성으로 2000장 자동 생성. 어그멘테이션: fliplr 0.5, 회전 ±5°, HSV 지터, mosaic 0.5, mixup 0.1 (상하 반전은 끔 — 벽 부착 태그는 뒤집히지 않음).
- **실측 데이터** (`dataset_collector_node.py`): 실제 주행 중 수집·자동 라벨링.
  - `mode:=positive` — 태그 부착 주행, apriltag_ros가 **해독 성공한 프레임만** 저장하고 corner 4점 → YOLO bbox 라벨 자동 생성 (검출 실패 프레임은 버려 오염 방지)
  - `mode:=negative` — 태그 제거 주행, 빈 라벨로 저장 (false positive 억제)
- **분할**: pos/neg 각각 85:15 계층 분할, 정렬 후 균등 간격 추출(재현성).

### 5-3. 재학습 워크플로우

```bash
# ── (Jetson) 실측 데이터 수집 — 파이프라인이 떠 있는 상태에서 ──
ros2 run jackal_orca_perception dataset_collector_node.py --ros-args \
    -p mode:=positive \
    -p image_topic:=/camera_front/color/image_raw \
    -p detections_topic:=/apriltag_front/detections
#   (런타임 입력: pause / resume / save / status)
#   태그 떼고 mode:=negative 로도 한 번 더 수집

# ── (PC) 데이터 합치고 학습 (RTX급 GPU) ──
scp -r jetson@<IP>:~/datasets/apriltag_real datasets/
python generate_dataset.py     # (선택) 합성 데이터도 섞으려면
python prepare_dataset.py       # → datasets/apriltag_combined/
python train_yolo.py            # → models/apriltag_yolo.pt (mAP 출력)

# ── (Jetson) 배포 ──
scp models/apriltag_yolo.pt jetson@<IP>:~/ros2_ws/jackal-ORCA/src/jackal_orca_perception/models/
ssh jetson@<IP>
cd ~/ros2_ws/jackal-ORCA/src/jackal_orca_perception
python3 scripts/dev_tools/export_trt.py     # .pt → .engine (FP16, 수 분)
cd ~/ros2_ws/jackal-ORCA && colcon build --packages-select jackal_orca_perception
#   tag_yolo_detector_node 가 새 .engine 을 자동 우선 로드
```

`scripts/dev_tools/` 도구:

| 스크립트 | 위치 | 역할 |
|---|---|---|
| `generate_dataset.py` | PC | 합성 데이터셋 생성 |
| `prepare_dataset.py` | PC | 실측+합성 병합, train/val 분할 |
| `train_yolo.py` | PC | YOLOv8s 학습 → `apriltag_yolo.pt` |
| `export_trt.py` | **Jetson** | `.pt` → `.engine` (FP16) |
| `apriltag_cam.py` / `yolo_cam.py` | - | 단독 카메라 테스트 뷰어 |
| `apriltag_file_test.py` | - | 이미지 파일 검출 테스트 |

---

## 6. 디렉토리 구조

```
jackal_orca_perception/
├── config/apriltag.yaml              # apriltag_ros 파라미터
├── launch/
│   ├── apriltag_pipeline.launch.py   # 카메라+apriltag+YOLO+recorder
│   └── yolo_web_monitor.launch.py    # 위 + web_video_server (bbox 웹뷰)
├── models/
│   ├── apriltag_yolo.pt              # 학습 가중치 (PyTorch)
│   └── apriltag_yolo.engine          # TensorRT FP16 (Jetson 배포)
├── scripts/
│   ├── tag_yolo_detector_node.py     # YOLO 탐지 노드
│   ├── tag_recorder_node.py          # 태그 위치 기록 (백업)
│   ├── dataset_collector_node.py     # 재학습 데이터 수집
│   └── dev_tools/                    # 학습/변환/테스트 도구
├── package.xml
└── CMakeLists.txt
```

---

## 7. 트러블슈팅

- **bbox 영상이 안 보임** — `/yolo/debug_image_*` 는 *구독자가 있을 때만* 발행된다. 브라우저로 stream URL을 열면 그때부터 그려진다.
- **카메라 충돌** (`device busy`) — 다른 RealSense 노드가 이미 장치를 점유 중. 기존 노드를 먼저 종료할 것.
- **YOLO 로드 실패** — `ultralytics` 미설치 또는 `.engine`/`.pt` 부재. `pip3 install ultralytics`, 모델 파일 확인.
- **`.pt`만 갱신했는데 옛 결과** — `export_trt.py` 로 `.engine` 재생성 필요. (노드는 더 최신 파일을 자동 선택하므로, `.pt`가 더 새것이면 `.pt`로 폴백한다.)
- **`TagCandidate` import 오류** — `custom_msgs` 빌드 안 됨. `colcon build --packages-select custom_msgs` 먼저.
```
