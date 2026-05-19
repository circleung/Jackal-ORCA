# Jackal-ORCA 실행 가이드

처음 사용하시는 분을 위한 단계별 실행 가이드입니다.

---

## 🎯 무엇을 하나

- **카메라 2대**로 깊이 기반 객체 검출 (앞/뒤)
- **LiDAR**로 360° 3D 클러스터 검출
- **카메라 영상**은 노트북 브라우저로
- **LiDAR 3D**는 Foxglove Studio로
- **JBL 스피커**로 탐지 사운드 피드백

## 📋 사전 점검

### 1. 하드웨어 연결 확인
| 항목 | 어디 |
|---|---|
| RealSense 카메라 2대 | Jetson USB 3.0 포트 |
| Livox Mid-360 LiDAR | Jetson eno1 (이더넷) |
| Jetson ↔ Jackal Mini PC | USB-C 케이블 |
| JBL Go 4 | 전원 켜고 Jackal Mini PC와 Bluetooth 페어링 상태 |

### 2. 네트워크 확인 (Jetson에서)
```bash
# LiDAR 통신
ping -c 2 192.168.1.182

# Mini PC 통신
ssh jackal@192.168.55.100 "echo OK"   # 비번 없이 OK 출력돼야 함

# 노트북 (Tailscale) 통신
ip addr show tailscale0 | grep inet     # 100.75.100.71 보여야 함
```

세 가지 다 OK여야 다음 진행.

### 3. USB autosuspend 끄기 (재부팅됐다면 매번)
```bash
sudo bash -c "echo -1 > /sys/module/usbcore/parameters/autosuspend"
cat /sys/module/usbcore/parameters/autosuspend   # -1 확인
```

---

## 🚀 실행 — 5개 터미널

> 각 터미널마다 SSH로 Jetson에 새로 접속하거나, `tmux`/`screen`으로 분리해서 실행합니다.

### 🟢 터미널 1 — Bringup (센서 드라이버)

```bash
cd ~/ros2_ws/jackal-ORCA
source install/setup.bash
ros2 launch jackal_orca_bringup bringup_all.launch.py
```

**30~60초 대기**. 다음 메시지 보이면 OK:
- ✅ `RealSense Node Is Up!` (두 번 — camera1, camera2)
- ✅ `livox/lidar publish use PointCloud2 format`
- ❌ `Frames didn't arrive` ← 이 메시지가 보이면 카메라 문제 (USB autosuspend 다시 확인)

### 🟢 터미널 2 — Depth Detector + 사운드 자동 시작

```bash
source ~/ros2_ws/jackal-ORCA/install/setup.bash
ros2 launch jackal_orca_perception depth_detector.launch.py
```

자동으로 실행되는 것:
- camera1, camera2 객체 검출 노드
- `/perception/depth_active` heartbeat (1Hz)
- SSH로 Mini PC 접속 → `audio_player_node.py` 실행
- **JBL에서 사운드 무한 재생** 🎵

### 🟢 터미널 3 — LiDAR Detector

```bash
source ~/ros2_ws/jackal-ORCA/install/setup.bash
ros2 launch jackal_orca_perception lidar_detector.launch.py
```

2초마다 통계 출력:
```
[stats] in= 20064  filtered= 3326  clusters=6
```

`in`이 0이면 LiDAR 데이터 안 옴 → bringup 확인.

### 🟢 터미널 4 — Web Video Server (카메라 시각화)

```bash
source ~/ros2_ws/jackal-ORCA/install/setup.bash
ros2 run web_video_server web_video_server
```

`Waiting For connections on 0.0.0.0:8080` 메시지 확인.

### 🟢 터미널 5 — Foxglove Bridge (LiDAR 시각화)

```bash
source ~/ros2_ws/jackal-ORCA/install/setup.bash
ros2 run foxglove_bridge foxglove_bridge
```

`WebSocket server listening on 0.0.0.0:8765` 확인.

---

## 💻 노트북에서 접속

### A. 카메라 영상 — 브라우저

브라우저(Chrome/Edge 추천) 탭 2개:

**탭 1 — 전방 카메라**
```
http://100.75.100.71:8080/stream?topic=/perception/camera1/annotated_image
```

**탭 2 — 후방 카메라**
```
http://100.75.100.71:8080/stream?topic=/perception/camera2/annotated_image
```

보여야 할 것:
- 카메라 영상 + 거리별 색깔 박스 (🟢 ≥1.5m / 🟠 0.8~1.5m / 🔴 <0.8m)
- 좌상단: `camera1: N obj | closest: X.XXm`

### B. LiDAR 3D — Foxglove Studio

#### 1. Foxglove 열기
- **웹**: https://app.foxglove.dev (Google 계정 로그인)
- **데스크톱 앱**: https://foxglove.dev/download (더 안정적)

#### 2. 연결
1. **Open connection** 클릭
2. **Foxglove WebSocket** 선택
3. URL 입력:
   ```
   ws://100.75.100.71:8765
   ```
4. **Open** 클릭

#### 3. 토픽 활성화 (처음 한 번만)
1. 좌측 사이드바에서 **Panel** 탭 클릭
2. 우측 상단에 3D 패널이 없으면: **+ 패널 추가** → **3D**
3. 3D 패널 클릭 → 좌측에 토픽 목록 표시
4. 다음 토픽들 옆 **눈 아이콘 🙈 → 👁** 으로 클릭해서 켜기:
   - `/livox/lidar` (원본 점군)
   - `/perception/lidar/clusters_markers` (클러스터 박스)
5. 화면에 점군 + 색깔 박스 표시되면 성공!

#### 4. 시점 조작
- **마우스 드래그**: 회전
- **마우스 휠**: 줌
- **Shift + 드래그**: 이동
- **R 키**: 시점 초기화

#### 5. 점이 안 보이면
- `/livox/lidar` 옆 **▶** 클릭해서 펼치기 → **Point size** 를 `3.0` 으로 변경
- **Frame** 섹션에서 **Display frame** = `livox_frame` 확인

---

## ✅ 동작 확인 체크리스트

| 항목 | 어디서 확인 | 정상이면 |
|---|---|---|
| 카메라 영상 | 브라우저 탭 1, 2 | 영상 흐름 + 박스 |
| LiDAR 점군 | Foxglove 3D 패널 | 360° 점들 회전 |
| LiDAR 클러스터 | Foxglove 3D 패널 | 색깔 박스 + 거리 라벨 |
| 사운드 | JBL 스피커 | 사운드 무한 재생 ♪ |
| CPU 부하 | `top -bn 1 | head -8` | ~40% 정도 |

---

## 🛑 종료 방법

### 정상 종료
각 터미널에서 **Ctrl + C** (역순 권장: 5 → 4 → 3 → 2 → 1)

### 강제 종료 (한 번에)
```bash
sudo pkill -9 -f "ros2|realsense|livox|depth_object|lidar_obstacle|foxglove|web_video"
ssh jackal@192.168.55.100 "pkill -9 mpg123; pkill -9 audio_player" 2>/dev/null
sleep 2
```

---

## 🆘 자주 보는 문제

### 카메라가 안 뜬다 (`Frames didn't arrive`)
```bash
# USB autosuspend 다시 끄기
sudo bash -c "echo -1 > /sys/module/usbcore/parameters/autosuspend"

# 모든 노드 죽이고 재시작
sudo pkill -9 -f realsense
sleep 5
# 터미널 1부터 다시 실행
```

### LiDAR 데이터 안 옴 (`LiDAR 메시지 미수신`)
```bash
# 1. ping 테스트
ping -c 3 192.168.1.182

# 2. 안 되면 nmcli 다시 활성화
sudo nmcli connection up livox-lidar
```

### Foxglove 연결 안 됨
- Tailscale 연결 확인 (노트북 + Jetson 양쪽)
- 노트북 브라우저에서 `ws://100.75.100.71:8765` 직접 입력 확인
- 데스크톱 앱이 웹보다 안정적

### Foxglove 화면이 비어있음
- 좌측 패널의 토픽 옆 **눈 아이콘**이 켜져있는지 (👁) 확인
- Display frame이 `livox_frame` 또는 `base_link` 로 설정됐는지

### JBL에서 사운드 안 남
```bash
# Mini PC에 SSH 접속
ssh jackal@192.168.55.100

# 사운드 출력 장치 확인 (JBL이 기본인지)
pactl info | grep "Default Sink"
pactl list short sinks

# JBL 아니면 변경 (예시)
pactl set-default-sink bluez_sink.XX_XX_XX.a2dp_sink
```

### CPU 부하 너무 높음
다음 순서로 노드를 끄면서 부하 확인:
1. terminal 4 (web_video_server) 끄기
2. terminal 5 (foxglove_bridge) 끄기
3. terminal 3 (lidar_detector) 끄기
4. terminal 2 (depth_detector) 끄기

---

## 📁 폴더 구조 참고

```
~/ros2_ws/jackal-ORCA/
├── src/
│   ├── jackal_orca_bringup/
│   │   └── launch/bringup_all.launch.py     ← 센서 드라이버 시작
│   ├── jackal_orca_perception/
│   │   ├── scripts/
│   │   │   ├── depth_object_detector.py    ← Phase 1.1 카메라 검출
│   │   │   └── lidar_obstacle_detector.py  ← Phase 1.2 LiDAR 검출
│   │   └── launch/
│   │       ├── depth_detector.launch.py    ← 카메라 검출 + 사운드 통합
│   │       └── lidar_detector.launch.py
│   └── livox_ros_driver2/
└── install/  (colcon build 결과)

# Mini PC 쪽 (jackal@192.168.55.100)
~/colcon_ws/src/jackal_audio/
├── scripts/audio_player_node.py    ← 사운드 재생 노드
├── launch/audio_player.launch.py
└── sounds/Scan_Sound.mp3           ← 재생할 MP3
```

---

## 🚀 한 번에 실행하는 스크립트 (편의 기능)

```bash
cat > ~/run_all.sh << 'EOF'
#!/bin/bash
WS=~/ros2_ws/jackal-ORCA
SRC="source $WS/install/setup.bash"

gnome-terminal --tab --title="bringup" -- bash -c "$SRC && ros2 launch jackal_orca_bringup bringup_all.launch.py; exec bash" &
sleep 45

gnome-terminal --tab --title="depth+audio" -- bash -c "$SRC && ros2 launch jackal_orca_perception depth_detector.launch.py; exec bash" &
gnome-terminal --tab --title="lidar" -- bash -c "$SRC && ros2 launch jackal_orca_perception lidar_detector.launch.py; exec bash" &
gnome-terminal --tab --title="web_video" -- bash -c "$SRC && ros2 run web_video_server web_video_server; exec bash" &
gnome-terminal --tab --title="foxglove" -- bash -c "$SRC && ros2 run foxglove_bridge foxglove_bridge; exec bash" &

echo "5개 탭 시작됨. bringup 안정화 1분 기다리세요."
EOF
chmod +x ~/run_all.sh
```

사용:
```bash
~/run_all.sh
```

---

## 📞 도움 받기

- **파라미터 조정**: `PARAMETERS.md` 참고
- **GitHub**: https://github.com/circleung/Jackal-ORCA
- **branch**: `sensor-fusion`
