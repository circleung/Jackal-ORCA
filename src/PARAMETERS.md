# Jackal-ORCA 파라미터 설정 가이드

각종 검출 동작과 데이터 수집을 어떻게 조정하는지 정리한 문서입니다.

---

## 🗂️ 어디 파라미터를 만지나

| 무엇을 바꾸고 싶나 | 어느 파일 |
|---|---|
| 카메라 검출 거리, 박스 크기, 바닥 영역 | `src/jackal_orca_perception/launch/depth_detector.launch.py` |
| LiDAR 검출 범위, 클러스터링 민감도 | `src/jackal_orca_perception/launch/lidar_detector.launch.py` |
| 사운드 볼륨, 파일, timeout | `~/colcon_ws/src/jackal_audio/launch/audio_player.launch.py` (Mini PC) |
| 카메라 해상도, FPS | `src/jackal_orca_bringup/launch/bringup_all.launch.py` |
| LiDAR 동작 모드 (스캔 패턴 등) | `tutorials/livox_tutorial/config_backup/MID360_config.json` |

> 💡 **모든 파라미터는 launch 파일에 정의되어 있어 코드 수정 불필요**. launch 파일만 수정 후 재시작하면 됩니다.

---

## 1️⃣ 카메라 깊이 검출 — Phase 1.1

**파일**: `~/ros2_ws/jackal-ORCA/src/jackal_orca_perception/launch/depth_detector.launch.py`

### 파라미터 표

| 파라미터 | 현재값 | 단위 | 효과 | 조정 권장 |
|---|---|---|---|---|
| `min_distance_m` | `0.3` | m | 이보다 가까우면 무시 (노이즈) | 0.2 ~ 0.5 |
| `max_distance_m` | `5.0` | m | 이보다 멀면 무시 | 3.0 ~ 8.0 (실내 좁으면 3) |
| `min_area_px` | `200` | 픽셀 | bbox가 이 크기보다 작으면 버림 | 100 ~ 500 |
| `warn_dist_m` | `1.5` | m | 주황색 박스 경계 (이보다 가까우면 주황) | 1.0 ~ 2.0 |
| `danger_dist_m` | `0.8` | m | 빨강 박스 경계 (이보다 가까우면 빨강) | 0.5 ~ 1.0 |
| `morph_kernel` | `5` | px | 형태학 OPEN 커널 크기 | 3 ~ 7 |
| `floor_crop_ratio` | `0.55` | 0~1 | 화면 하단 이 비율은 바닥으로 간주 (마스킹) | 0.45 ~ 0.70 |

### 의미 자세히

#### `min_distance_m` / `max_distance_m`
검출 거리 범위. 이 밖의 픽셀은 마스크에서 제외.
- **줄이면**: 멀리 있는 객체 무시 → 검출 안정
- **늘리면**: 더 멀리 검출, 노이즈 증가 가능

#### `min_area_px`
검출된 영역의 픽셀 수가 이보다 작으면 무시.
- **줄이면**: 작은 물체도 검출 (신발, 작은 박스) → 노이즈 ↑
- **늘리면**: 큰 물체만 (가구, 사람) → 안정적

> 💡 해상도가 424×240이라 작음. 만약 bringup에서 더 큰 해상도(640×480 등) 사용하면 이 값도 비례해서 키워야 함.

#### `warn_dist_m` / `danger_dist_m`
박스 색상 결정. `danger < warn < ∞`:
- **🔴 빨강**: `distance < danger_dist_m`
- **🟠 주황**: `danger_dist_m ≤ distance < warn_dist_m`
- **🟢 녹색**: `distance ≥ warn_dist_m`

#### `morph_kernel`
모폴로지 OPEN (작은 노이즈 제거) 커널 크기.
- **3**: 작은 노이즈도 보존 (예민함)
- **5~7**: 강한 노이즈 제거, 작은 객체도 같이 사라짐

> ⚠️ MORPH_CLOSE는 코드상에서 제거됨 (객체끼리 합쳐지지 않도록). 변경 비추천.

#### `floor_crop_ratio`
하단 이 비율을 바닥으로 간주하고 마스크에서 제외 (검출 안 함).
- **0.45**: 화면 위쪽 45%만 검출 → 멀리 있는 물체 위주
- **0.55** (기본): 위쪽 55%만 검출, 하단 45% 마스크
- **0.70**: 화면 거의 다 검출 (바닥에 있는 작은 물체도 포함)

> 💡 카메라가 살짝 아래로 향하면 바닥이 화면 하단을 차지함. 이걸 마스크해서 "바닥이 거대한 객체로 잡히는 문제" 해결.

### 변경 방법

#### 방법 A: 직접 편집
```bash
nano ~/ros2_ws/jackal-ORCA/src/jackal_orca_perception/launch/depth_detector.launch.py
```

`common_params` 딕셔너리 값을 수정:
```python
common_params = {
    'min_distance_m': 0.3,    # 여기 수정
    'max_distance_m': 5.0,
    ...
}
```

#### 방법 B: sed 한 줄
```bash
# 예: max_distance_m을 5.0 → 3.0으로 변경
sed -i "s/'max_distance_m': 5.0/'max_distance_m': 3.0/" \
  ~/ros2_ws/jackal-ORCA/src/jackal_orca_perception/launch/depth_detector.launch.py
```

### 변경 후 재시작
```bash
# detector 죽이기
sudo pkill -9 -f depth_object && sleep 2

# 다시 실행 (launch 변경은 빌드 불필요 — symlink-install 덕분)
source ~/ros2_ws/jackal-ORCA/install/setup.bash
ros2 launch jackal_orca_perception depth_detector.launch.py
```

### 흔한 튜닝 시나리오

| 상황 | 조정 |
|---|---|
| 박스가 화면 전체를 덮음 | `floor_crop_ratio: 0.50` ↓ (바닥 더 많이 제거) |
| 작은 물체도 검출하고 싶음 | `min_area_px: 200 → 100` |
| 너무 많은 객체가 잡혀 복잡함 | `min_area_px: 200 → 400` |
| 가까운 객체가 너무 자주 빨강 | `danger_dist_m: 0.8 → 0.5` |
| 실내가 좁아 5m 검출 의미 없음 | `max_distance_m: 5.0 → 3.0` |
| 객체끼리 박스가 붙어있음 | `morph_kernel: 5 → 3` |

---

## 2️⃣ LiDAR 클러스터링 — Phase 1.2

**파일**: `~/ros2_ws/jackal-ORCA/src/jackal_orca_perception/launch/lidar_detector.launch.py`

### 파라미터 표

| 파라미터 | 현재값 | 단위 | 효과 |
|---|---|---|---|
| `input_topic` | `/livox/lidar` | — | 구독할 점군 토픽 |
| `range_min_m` | `0.3` | m | 수평거리 이보다 가까운 점 제외 (자기감지 방지) |
| `range_max_m` | `10.0` | m | 수평거리 이보다 먼 점 제외 |
| `height_min_m` | `0.1` | m | 지면 위 이 높이보다 낮으면 제외 (바닥 점 제거) |
| `height_max_m` | `2.0` | m | 이 높이보다 위 제외 (천장 제거) |
| `voxel_size_m` | `0.05` | m | 5cm 격자 다운샘플링 |
| `dbscan_eps_m` | `0.30` | m | DBSCAN 반경 (이내 점들은 같은 클러스터) |
| `dbscan_min_samples` | `10` | 개 | 한 클러스터의 최소 점 개수 (DBSCAN 정의) |
| `min_cluster_points` | `30` | 개 | 시각화할 클러스터의 최소 점 개수 |

### 의미 자세히

#### `range_min_m` / `range_max_m` (수평 거리 필터)
로봇 중심에서 수평면 거리(`√(x² + y²)`) 기준.
- **range_min_m**: 너무 가까운 점 제외 (LiDAR 자체 마운트나 자칼 몸체)
  - 0.3m → 0.5m: 더 적극적으로 자기감지 제거
- **range_max_m**: 멀리 있는 점 제외 (입력 데이터 양 줄임)
  - 실내 좁으면 5m로 줄여 부하 감소

#### `height_min_m` / `height_max_m` (높이 필터)
z 좌표 기준.
- **height_min_m**: 바닥의 점들 제외. 기본 0.1m면 지면 위 10cm 이상만.
  - 0.05: 작은 물체도 검출, 바닥 노이즈 ↑
  - 0.20: 책상, 의자 위쪽만 (낮은 물체 무시)
- **height_max_m**: 천장 제외. 실내라면 2m 이하로.

#### `voxel_size_m`
점군 다운샘플링. 같은 5cm 격자에 들어가는 점들은 1개로 합침.
- **0.03 (3cm)**: 더 세밀, DBSCAN 부하 ↑ (점 ~3배 증가)
- **0.05 (5cm, 기본)**: 균형
- **0.10 (10cm)**: 빠름, 작은 물체 놓침

> 💡 부하 줄이고 싶으면 이 값을 키우면 효과적.

#### `dbscan_eps_m` (가장 중요한 클러스터링 파라미터)
같은 클러스터로 묶일 점들의 최대 거리.
- **0.20m (작음)**: 객체 더 잘게 분리. 사람과 가까운 박스가 다른 클러스터.
- **0.30m (기본)**: 균형
- **0.50m (큼)**: 인접한 객체들을 하나로 묶음. 큰 영역만 검출.

#### `dbscan_min_samples`
DBSCAN의 "core point" 정의 — 한 점이 eps 반경 내에 이 개수 이상의 점이 있어야 클러스터 시작.
- **5~7**: 더 작은 객체도 클러스터로 인정
- **10 (기본)**: 안정적
- **15~20**: 큰 객체 위주 (노이즈 강건)

#### `min_cluster_points`
DBSCAN이 만든 클러스터 중 이 개수보다 점이 적으면 시각화/publish 안 함 (필터링).
- 노이즈로 인한 작은 클러스터 제거용
- DBSCAN 자체와 별도 (`dbscan_min_samples`는 군집 형성 조건, 이건 군집 표시 조건)

### 변경 방법

#### 직접 편집
```bash
nano ~/ros2_ws/jackal-ORCA/src/jackal_orca_perception/launch/lidar_detector.launch.py
```

`parameters` 딕셔너리 수정:
```python
parameters=[{
    'input_topic': '/livox/lidar',
    'range_min_m': 0.3,           # 여기 수정
    'range_max_m': 10.0,
    'height_min_m': 0.1,
    'height_max_m': 2.0,
    'voxel_size_m': 0.05,
    'dbscan_eps_m': 0.30,
    'dbscan_min_samples': 10,
    'min_cluster_points': 30,
}],
```

#### sed 한 줄 예시
```bash
# 예: dbscan_eps_m을 0.30 → 0.50으로
sed -i "s/'dbscan_eps_m': 0.30/'dbscan_eps_m': 0.50/" \
  ~/ros2_ws/jackal-ORCA/src/jackal_orca_perception/launch/lidar_detector.launch.py
```

### 변경 후 재시작
```bash
sudo pkill -9 -f lidar_obstacle && sleep 2
source ~/ros2_ws/jackal-ORCA/install/setup.bash
ros2 launch jackal_orca_perception lidar_detector.launch.py
```

### 흔한 튜닝 시나리오

| 상황 | 조정 |
|---|---|
| LiDAR 마운트가 자기 감지됨 (0.1m 박스) | `range_min_m: 0.3 → 0.5` |
| 바닥의 노이즈가 박스로 잡힘 | `height_min_m: 0.1 → 0.15` |
| 클러스터가 너무 잘게 쪼개짐 | `dbscan_eps_m: 0.30 → 0.45` |
| 객체들이 하나의 거대 박스로 묶임 | `dbscan_eps_m: 0.30 → 0.20` |
| 작은 물체(공, 신발) 검출 | `dbscan_min_samples: 10 → 5`, `min_cluster_points: 30 → 10` |
| CPU 부하 줄이고 싶음 | `voxel_size_m: 0.05 → 0.08`, `range_max_m: 10 → 5` |
| 박스가 자주 깜빡임 | (코드) marker lifetime 늘리기 (아래 참고) |

### 박스 깜빡임 — 코드 수정 필요

깜빡임은 DBSCAN의 프레임별 독립 실행 + 추적 없음 때문. 임시 완화:

**파일**: `~/ros2_ws/jackal-ORCA/src/jackal_orca_perception/scripts/lidar_obstacle_detector.py`

`200_000_000` 검색해서 `500_000_000` 으로 변경 (두 곳, 박스용 + 라벨용):
```bash
sed -i 's/200_000_000/500_000_000/g' \
  ~/ros2_ws/jackal-ORCA/src/jackal_orca_perception/scripts/lidar_obstacle_detector.py
```

→ 박스 lifetime이 0.2초 → 0.5초로 늘어나 깜빡임 ↓

---

## 3️⃣ 사운드 피드백 — Mini PC

**파일**: `~/colcon_ws/src/jackal_audio/launch/audio_player.launch.py` (Mini PC에서)

### 파라미터 표

| 파라미터 | 현재값 | 효과 |
|---|---|---|
| `volume` | `16384` | 0~32768 사이 (32768=100%) — 재생 볼륨 |
| `timeout_sec` | `2.0` | heartbeat 끊긴 후 N초 뒤 사운드 정지 |
| `audio_file` | `~/colcon_ws/.../sounds/Scan_Sound.mp3` | 재생할 MP3 |
| `topic` | `/perception/depth_active` | 구독할 heartbeat 토픽 |

### 변경 방법

#### A. launch 파일에서 (Mini PC SSH 들어가서)
```bash
ssh jackal@192.168.55.100
nano ~/colcon_ws/src/jackal_audio/launch/audio_player.launch.py
```

`parameters` 안의 값 수정:
```python
parameters=[{
    'volume': 16384,           # 여기 수정
    'timeout_sec': 2.0,
    # 'audio_file': '/home/jackal/colcon_ws/.../Scan_Sound.mp3',
}],
```

#### B. depth_detector launch에서 직접 (Jetson)
**파일**: `~/ros2_ws/jackal-ORCA/src/jackal_orca_perception/launch/depth_detector.launch.py`

SSH 부분에 ROS 파라미터로 전달:
```python
ExecuteProcess(
    cmd=['ssh', ...,
         'python3 ' + AUDIO_SCRIPT +
         ' --ros-args -p volume:=8192'],  # 25%
    ...
)
```

### 볼륨 값 가이드

| volume | % | 용도 |
|---|---|---|
| 4096 | 12.5% | 매우 조용 (테스트) |
| 8192 | 25% | 조용 (실내) |
| 16384 | 50% | 기본 |
| 24576 | 75% | 크게 |
| 32768 | 100% | 최대 |

### 사운드 파일 변경
새 MP3로 바꾸려면:
```bash
# Mac에서 Jetson으로 전송
scp ~/Downloads/MySound.mp3 jetson@100.75.100.71:~/

# Jetson에서 Mini PC로 전송
scp ~/MySound.mp3 jackal@192.168.55.100:~/colcon_ws/src/jackal_audio/sounds/

# Mini PC에서 launch 파일 수정해서 새 파일 경로 지정 (필요 시)
```

또는 기존 파일을 단순히 덮어쓰기:
```bash
scp ~/Downloads/MyNewSound.mp3 jackal@192.168.55.100:~/colcon_ws/src/jackal_audio/sounds/Scan_Sound.mp3
```

---

## 4️⃣ 카메라 해상도, FPS — Bringup

**파일**: `~/ros2_ws/jackal-ORCA/src/jackal_orca_bringup/launch/bringup_all.launch.py`

### 핵심 변수

```python
CAM_PROFILE = "424x240x15"   # WIDTHxHEIGHTxFPS
```

| 값 | 의미 | 부담 |
|---|---|---|
| `"424x240x15"` (현재) | 매우 가벼움, USB 대역폭 적게 | ✅ 두 카메라 OK |
| `"640x480x15"` | 표준 | ⚠️ USB 대역폭 빠듯 |
| `"640x480x30"` | 부드러움 | ❌ 두 카메라 동시 어려움 |
| `"848x480x15"` | 고해상도 | ❌ USB 부족 |

### 변경 방법
```bash
nano ~/ros2_ws/jackal-ORCA/src/jackal_orca_bringup/launch/bringup_all.launch.py

# CAM_PROFILE = "424x240x15"
# →
# CAM_PROFILE = "640x480x15"
```

> ⚠️ 해상도 바꾸면 `depth_detector`의 `min_area_px`도 비례해서 조정 필요. 픽셀 수가 늘어나니까.
> 
> 예: 424×240 → 640×480 (픽셀 ~2.3배) → `min_area_px: 200 → 460`

### 변경 후 재시작
```bash
sudo pkill -9 -f realsense && sleep 5
source ~/ros2_ws/jackal-ORCA/install/setup.bash
ros2 launch jackal_orca_bringup bringup_all.launch.py
```

---

## 5️⃣ LiDAR 동작 모드 — Livox config

**파일**: `~/ros2_ws/jackal-ORCA/tutorials/livox_tutorial/config_backup/MID360_config.json`

### 주요 설정

```json
{
  "lidar_summary_info" : {
    "lidar_type" : 8
  },
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": {
      "cmd_data_ip": "192.168.1.5",       ← Jetson IP (eno1)
      "cmd_data_port": 56101,
      ...
    }
  }
}
```

### 자주 만지는 것

| 항목 | 의미 | 변경 |
|---|---|---|
| `cmd_data_ip` | Jetson의 eno1 IP | 네트워크 바꿀 때 |
| `pcl_data_type` | 점군 데이터 타입 (1: cartesian, 2: spherical) | 보통 안 만짐 |
| `pattern_mode` | 0=비반복, 1=반복 스캔 | SLAM은 0 권장 (기본) |
| `imu_data_en` | IMU 활성화 (true/false) | true 유지 (SLAM용) |

> 💡 LiDAR 자체 설정은 거의 안 만져도 됨. 네트워크나 SLAM 알고리즘 바꿀 때만.

### Topic / Publishing 설정
**파일**: `~/ros2_ws/jackal-ORCA/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py`

```python
xfer_format = 0     # 0: pointcloud2 (현재), 1: livox custom, 2: pcl pointcloud
multi_topic = 0     # 0: 모든 LiDAR이 같은 토픽 / 1: 각각 토픽
data_src = 0        # 0: lidar에서 받기 / 1: rosbag 등에서 재생
publish_freq = 10.0 # publish 주기 (Hz)
output_type = 0     # 0: 표준
frame_id = 'livox_frame'  # TF 프레임 이름
```

**`publish_freq`** 변경하면 모든 다운스트림 노드 부하 영향. 5Hz로 줄이면 절반 부하.

---

## 🔁 모든 파라미터 변경의 공통 워크플로우

```bash
# 1. 파일 수정 (nano 또는 sed)
nano ~/ros2_ws/jackal-ORCA/src/jackal_orca_perception/launch/lidar_detector.launch.py

# 2. 해당 노드 종료
sudo pkill -9 -f lidar_obstacle      # 또는 depth_object, realsense 등

# 3. 잠시 대기
sleep 2

# 4. 재시작
source ~/ros2_ws/jackal-ORCA/install/setup.bash
ros2 launch jackal_orca_perception lidar_detector.launch.py
```

> 💡 **`--symlink-install` 덕분에 launch / 스크립트 변경은 `colcon build` 불필요**. 노드만 재시작하면 즉시 반영.
> 
> 예외: C++ 패키지나 message 정의 변경은 빌드 필요.

---

## 🎚️ 실행 시점 파라미터 오버라이드 (재시작 없이)

이미 떠있는 노드의 파라미터를 동적으로 변경:

```bash
# 현재 값 확인
ros2 param get /lidar_obstacle_detector dbscan_eps_m

# 변경
ros2 param set /lidar_obstacle_detector dbscan_eps_m 0.5

# 즉시 반영 확인
ros2 param get /lidar_obstacle_detector dbscan_eps_m
```

> ⚠️ 노드가 파라미터 변경 콜백을 등록해야 동작. 본 프로젝트 노드는 시작 시점에만 읽으므로 `ros2 param set`은 별 의미 없음 (재시작 권장).

---

## 📊 동작 모니터링

### 부하 확인
```bash
top -bn 1 | head -10
```

### 토픽 주기 확인
```bash
source ~/ros2_ws/jackal-ORCA/install/setup.bash

timeout 5 ros2 topic hz /perception/camera1/annotated_image
timeout 5 ros2 topic hz /perception/lidar/clusters_markers
timeout 5 ros2 topic hz /livox/lidar
```

기대값:
- 카메라 annotated: ~15 Hz
- LiDAR markers: ~10 Hz

### 검출 통계 확인
LiDAR detector 터미널에서 2초마다 자동 출력:
```
[stats] in= 20064  filtered= 3326  clusters=6
```

- `in`: 입력 점 수 (0이면 LiDAR 안 옴)
- `filtered`: 필터/voxel 후 점 수 (너무 적으면 range/height 조건이 너무 엄격함)
- `clusters`: 최종 클러스터 개수

---

## 🎯 시나리오별 빠른 레시피

### "큰 물체 위주로 검출하고 싶다"
```bash
# 카메라
sed -i "s/'min_area_px': 200/'min_area_px': 500/" \
  ~/ros2_ws/jackal-ORCA/src/jackal_orca_perception/launch/depth_detector.launch.py

# LiDAR
sed -i "s/'min_cluster_points': 30/'min_cluster_points': 80/" \
  ~/ros2_ws/jackal-ORCA/src/jackal_orca_perception/launch/lidar_detector.launch.py

sudo pkill -9 -f "depth_object|lidar_obstacle" && sleep 2
# 재시작
```

### "작은 물체도 다 잡고 싶다 (실험용)"
```bash
sed -i "s/'min_area_px': 200/'min_area_px': 80/" .../depth_detector.launch.py
sed -i "s/'dbscan_min_samples': 10/'dbscan_min_samples': 5/" .../lidar_detector.launch.py
sed -i "s/'min_cluster_points': 30/'min_cluster_points': 10/" .../lidar_detector.launch.py
```

### "실내 좁은 공간이라 5m면 충분"
```bash
sed -i "s/'max_distance_m': 5.0/'max_distance_m': 3.0/" .../depth_detector.launch.py
sed -i "s/'range_max_m': 10.0/'range_max_m': 5.0/" .../lidar_detector.launch.py
```

### "CPU 부하 줄이기"
```bash
sed -i "s/'voxel_size_m': 0.05/'voxel_size_m': 0.08/" .../lidar_detector.launch.py
sed -i "s/'range_max_m': 10.0/'range_max_m': 5.0/" .../lidar_detector.launch.py
# + bringup의 publish_freq를 10 → 5로 줄여도 효과적
```

### "사운드 더 크게/작게"
```bash
# Mini PC에서
ssh jackal@192.168.55.100
sed -i "s/'volume': 16384/'volume': 24576/" \
  ~/colcon_ws/src/jackal_audio/launch/audio_player.launch.py
```

또는 JBL 본체 볼륨 버튼 사용.

---

## 📁 모든 파라미터 파일 위치 요약

```
~/ros2_ws/jackal-ORCA/
├── src/jackal_orca_bringup/launch/
│   └── bringup_all.launch.py              ← 카메라 해상도, USB stagger
├── src/jackal_orca_perception/launch/
│   ├── depth_detector.launch.py           ← Phase 1.1 검출 파라미터
│   └── lidar_detector.launch.py           ← Phase 1.2 클러스터링 파라미터
├── src/jackal_orca_perception/scripts/
│   ├── depth_object_detector.py           ← 알고리즘 (필요 시 직접 수정)
│   └── lidar_obstacle_detector.py         ← 알고리즘 (marker lifetime 등)
├── src/livox_ros_driver2/launch_ROS2/
│   └── msg_MID360_launch.py               ← LiDAR publish 주기, frame_id
└── tutorials/livox_tutorial/config_backup/
    └── MID360_config.json                 ← LiDAR 하드웨어 설정

# Mini PC (jackal@192.168.55.100)
~/colcon_ws/src/jackal_audio/launch/
└── audio_player.launch.py                 ← 사운드 볼륨, timeout
```

---

## ❓ 도움 요청

파라미터 조정 후 예상과 다르게 동작하면:
1. 재시작 했는지 확인 (`sudo pkill -9 -f ...`)
2. 현재 값 검증: `ros2 param get /노드_이름 파라미터_이름`
3. launch 파일이 실제로 수정됐는지: `grep` 으로 값 확인
4. 다른 launch가 같은 노드를 띄우고 있지 않은지 (`ps aux | grep`)
