# Claude.md — Jackal UGV 자율 스캔 프로젝트

> 이 문서는 프로젝트 전반의 컨텍스트, 아키텍처, 팀 업무 분담, 인터페이스 명세를
> 한 곳에 정리한 참조 문서입니다. 새로 합류하는 사람이나 LLM에게 컨텍스트로
> 제공할 수 있도록 작성되었습니다.

---

## 0. 핸드오프 — 2026-05-27 작업 이전 (Jetson → Jackal mini PC)

### 0.1 이번 세션에서 결정된 사항
- **reactive_explorer 동작 방식 변경**: §8의 FSM(FORWARD/TURNING/BACKING) 이산 전이 모델
  은 폐기. LiDAR clearance 기반 **연속 곡선 회피**로 구현. 즉:
  - `angular_vel = k * tanh((left_clearance - right_clearance) / scale)` (연속)
  - `linear_vel = forward_speed * clamp(front_clearance / safe_dist)` (연속 감속)
  - 제자리 회전(linear=0)은 진짜 막다른 골목 fallback만.
  - 이유: 청소기식 "정지→제자리 회전→재출발" 패턴은 좁은 통로/모서리 진동 및 벽 충돌
    유발. LiDAR가 있으므로 부드럽게 휘어 돌아가는 게 안전.
- **컴퓨트 역할 분리 확정**:
  - **Jackal mini PC**: Livox 드라이버, FAST-LIO2, reactive_explorer, mission_node,
    safety_layer, Jackal bringup (`/cmd_vel` 소비자)
  - **Jetson**: depth 카메라 드라이버, YOLO/AprilTag 검출 (GPU 필요한 추론만)
  - LiDAR 이더넷 케이블은 **Jackal mini PC**에 연결됨 (이번에 이동).

### 0.2 환경 차이 — 주의 사항
| 항목 | Jetson (이전 작업 장소) | Jackal mini PC (현 작업 장소) |
|---|---|---|
| 아키텍처 | ARM64 (tegra) | x86_64 |
| OS | Ubuntu 22.04 | Ubuntu 24.04 (kernel 6.17) |
| ROS | Humble | **Jazzy** ← CLAUDE.md §3 기존 명세와 다름 |
| 워크스페이스 | `~/ros2_ws/` | `~/colcon_ws/` |

- **CLAUDE.md §3은 "Humble + 22.04 고정"으로 명시되어 있었으나, 실제 mini PC는 Jazzy.**
  FAST-LIO2 빌드/실행이 Jazzy에서 검증되지 않음. 빌드 실패 시 Jazzy 호환 fork 또는
  Docker 컨테이너 검토 필요.
- 빌드 파일(`build/`, `install/`)은 아키텍처가 달라 이전 안 됨. mini PC에서 재빌드.

### 0.3 Jackal mini PC 기존 상태
- `~/colcon_ws/src/` 에 이미 존재: `clearpath_common`, `clearpath_robot`,
  `clearpath_ros2_socketcan_interface`, `jackal_audio`, `micro_ros_setup`,
  `roas2_bringup`, `uros`
- `~/.bashrc` 에서 `/opt/ros/jazzy/setup.bash` + `~/colcon_ws/install/setup.bash` source
- rviz alias: `ros2 launch roas2_bringup view_robot.launch.py namespace:=j100_0915 ...`
  → **Jackal이 `j100_0915` namespace 사용 중일 가능성 매우 높음**
  → `/cmd_vel`이 아니라 `/j100_0915/cmd_vel` 일 수 있음. bringup 띄운 후
    `ros2 topic list | grep cmd_vel` 로 실제 이름 확인 필수.

### 0.4 이전된 파일
- `src/jackal_mine_detection/` → `~/colcon_ws/src/jackal_mine_detection/`
- `jackal-ORCA/src/FAST_LIO/` → `~/colcon_ws/src/FAST_LIO/` (Jazzy 빌드 검증 필요)
- `claude.md` → `~/colcon_ws/claude.md` (본 문서)

### 0.5 안 옮긴 것
- `build/`, `install/`, `log/` — 아키텍처 다름
- `bags/` (8.1 GB) — 필요시 별도 전송
- `.claude/projects/.../memory/` — 새 호스트에서 Claude 메모리는 새로 쌓임

### 0.6 다음 작업 (2026-05-27 밤 세션 진행 상황 반영)
- ✅ **작업 1: `/cmd_vel` 응답 확인** — 완료 (2026-05-27 저녁).
- ✅ **작업 2: `reactive_explorer` MVP 구현 + 라이브 LiDAR 검증** — 완료 (2026-05-27 밤).
  실모터 모드로 들어올린 상태에서 연속 곡선 회피 동작 확인. 좌측 좁아질 때 우회전,
  정면 가까울 때 감속 모두 의도대로 동작. CustomMsg 입력 모드로 마이그레이션됨 (§0.8 참조).
- ✅ **작업 2a: Livox 드라이버 설치/빌드/검증** — 완료. `livox_ros_driver2`가 MID360
  IP `192.168.1.182`로 통신, `/livox/lidar`(CustomMsg) + `/livox/imu` 발행 확인.
- ✅ **작업 2b: FAST-LIO2 Jazzy 빌드** — 완료. CMakeLists.txt를 C++14 → C++17로 패치하니
  Jazzy rclcpp 매크로(`std::is_convertible_v` 사용)와 호환 OK. `/Odometry`, `/cloud_registered`,
  `/path` 발행 확인. 정지 상태에서 position ≈ 원점, 자연스러운 드리프트.
- ⏭ **작업 3: `safety_layer` 구현** — 미시작. `/cmd_vel_raw` → 속도 상한 clamp + LiDAR
  정면 근접 시 강제 0 → `/j100_0915/cmd_vel` 발행. reactive_explorer와 독립.
- ⏭ **작업 4: AprilTag + YOLO 인식 파이프라인 셋업** — 다음 세션 1순위. **Jetson에서**
  실행 (mini PC 아님). depth 카메라(RealSense) RGB → `apriltag_ros`(tag 검출) +
  YOLO(객체 인식)를 한 노드 그래프로 같이 돌림. mini PC 측에서는 결과(`/apriltag_*/detections`,
  YOLO detection 토픽)만 ROS_DOMAIN_ID로 받아서 `tag_mapper`(§7.1 F9)에서 map frame
  변환. 사용자 확정(2026-05-27 밤): "AprilTag도 YOLO와 함께 객체 인식 형태로 Jetson에서
  진행". 즉 §0.1의 컴퓨트 분리 원칙 그대로 유지.
- ⏭ **작업 5: `mission_node` FSM 구현 + 클러스터링 단계 추가** — §0.9 새 설계 참조.

### 0.7 mini PC에서 새 Claude 세션 시작하면 할 일
```bash
cd ~/colcon_ws
# claude.md 자동 인식됨. §0 전체(특히 §0.6 / §0.8 / §0.9) 읽고 진행.
# 2026-05-27 밤 세션까지 작업 1, 2, 2a, 2b 완료.
# 다음 진입점: apriltag_ros 셋업 (작업 4) 또는 mission_node 설계 (작업 5).

# ── 환경 전제 (세션 시작마다 확인) ─────────────────────────────────
# enp86s0의 IPv4가 사라졌으면 재할당 (NetworkManager 재시작 시 사라질 수 있음):
ip -4 addr show enp86s0 | grep inet || sudo ip addr add 192.168.1.5/24 dev enp86s0
# enp86s0은 NM unmanaged로 설정해두긴 했지만 부팅 후 검증 권장:
nmcli device status | grep enp86s0    # 'unmanaged'여야 함
ping -c 2 192.168.1.182                # MID360 응답 확인 (~1-2ms ttl=255)

# ── 풀 스택 띄우기 (검증된 순서) ───────────────────────────────────
# 1) Livox 드라이버 (CustomMsg 모드 — §0.8 결정)
ros2 launch livox_ros_driver2 msg_MID360_launch.py
# 토픽 확인: /livox/lidar (livox_ros_driver2/CustomMsg, 10Hz), /livox/imu (sensor_msgs/Imu)

# 2) FAST-LIO2 (rviz는 SSH라 꺼둠)
ros2 launch fast_lio mapping.launch.py rviz:=false
# 토픽 확인: /Odometry, /cloud_registered, /path

# 3) reactive_explorer (들어올린 상태에서 실모터 모드)
ros2 run jackal_mine_detection reactive_explorer_node \
  --ros-args -r __ns:=/j100_0915 -p cmd_vel_topic:=/j100_0915/cmd_vel
# 디버그: ros2 topic echo /j100_0915/reactive_explorer/state std_msgs/msg/String

# ── 다음 단계 (작업 4) ─────────────────────────────────────────────
# 4) apriltag_ros + YOLO 파이프라인은 **Jetson**에서 셋업 (§0.6 작업 4 참조).
#    mini PC에서는 ROS_DOMAIN_ID 맞춰서 detection 토픽 구독 + tag_mapper 구현.
#    Jetson 세션은 별도 SSH/Claude로 작업.
#
#    mini PC에서 직접 할 일: tag_mapper 노드 (custom_msgs/TagPoseArray 정의 +
#    `/apriltag_*/detections` → map frame 변환 + EMA 평활화 + `/tags_in_map` 발행).
#    §7.1 F9 참조.
```

### 0.8 2026-05-27 저녁 세션에서 확인된 부수 사실 (메모)
- **PS 패드(`DualSense Wireless Controller`) → ROS 입력 경로가 깨진 상태**.
  Bluetooth 페어링은 살아있고 `/dev/input/js*`도 잡혀있는데, `/j100_0915/joy_teleop/joy`
  에 메시지가 거의 안 들어옴. 사용자가 deadman+스틱을 눌러도 모터 응답 없음을
  확인. 자율 주행에는 무관하지만, **비상시 manual override를 쓰려면 joy_node가
  실제로 어느 `/dev/input/jsX`를 잡고 있는지 확인 필요** (joy_node의 `dev`
  파라미터 점검, `evtest`/`jstest`로 raw 입력 검증).
- **`emergency_stop` 토픽은 event-driven Bool**(latched 아님, 주기 발행 아님).
  세션 초기 `data: true` 한 번 잡힌 게 e-stop 살아있는 것처럼 보였으나, 이후
  외부 `/j100_0915/cmd_vel` 발행에 모터가 정상 응답한 것으로 **false positive**
  확인됨. 다음에 같은 신호 보더라도 단정하지 말고 motors/feedback과 cmd_vel_out
  으로 교차 검증할 것.
- **twist_mux 우선순위**: external/external_cmd=1, interactive_marker=8, joy=10,
  rc=12 (모두 timeout 0.5 s). joy/rc 활성 시 외부 cmd_vel은 자동 override됨
  = 1차 안전망.
- **TwistStamped의 frame_id**: 본 프로젝트에서 `base_link` 사용
  (reactive_explorer_node.py 기본값).
- **(2026-05-27 밤 추가) Livox 드라이버 메시지 포맷 결정**: `xfer_format=1` (CustomMsg)
  로 확정. 이유는 FAST-LIO2 `lidar_type=1`이 `livox_ros_driver2/CustomMsg`를 요구하기
  때문. PointCloud2(`xfer_format=0`)도 가능하지만 그 경우 FAST-LIO2를 `lidar_type=4`
  (generic)로 낮춰야 하며 ring/intensity 활용 못 함. reactive_explorer는 이번 세션에
  CustomMsg 입력으로 마이그레이션 완료 (`np.fromiter(p.x for p in msg.points)` 패턴).
  다른 노드를 추가할 때도 CustomMsg를 기준으로 작성할 것. 변환 필요 시 별도
  `livox_to_pointcloud2` 노드 도입 검토.
- **(2026-05-27 밤 추가) FAST_LIO Jazzy 빌드 패치**: `~/colcon_ws/src/FAST_LIO/CMakeLists.txt`
  에서 `-std=c++14` / `CMAKE_CXX_STANDARD 14` → **C++17**로 모두 교체해야 빌드됨.
  Jazzy의 `RCLCPP_INFO` 매크로가 `std::is_convertible_v` (C++17 변수 템플릿)를 사용.
  업스트림(hku-mars/FAST_LIO ROS2 브랜치)은 아직 C++14로 고정돼 있어서 fork/patch 유지 필요.
- **(2026-05-27 밤 추가) enp86s0 NetworkManager 우회**: MID360 연결 IP `192.168.1.5/24`를
  `sudo ip addr add`로 줘도 NetworkManager가 enp86s0을 관리하면 곧 제거됨. 부팅 직후
  `nmcli device set enp86s0 managed no` + `ip addr add`로 안정화. 부팅 시 자동 적용
  하려면 netplan 또는 systemd-networkd 설정으로 영구화 권장.
- **(2026-05-27 밤 추가) `mine_cluster_node` 이미 존재**:
  `jackal_mine_detection/jackal_mine_detection/`에 `mine_cluster_node` 빌드 산출물 발견.
  사용자가 §0.9에서 요청한 "tag 클러스터링 → 중심 이동" 기능과 중복/연계 가능성 있음.
  다음 세션에서 코드 리뷰하여 재사용 여부 결정할 것.

### 0.9 자율 미션 흐름 — 사용자 확정 (2026-05-27 밤)
사용자와 합의된 미션 전체 흐름. §7.2의 기존 mission_node FSM을 확장해야 하는
새 요구사항이 포함됨.

**컴퓨트 분배** (§0.1 원칙 재확인):
- **Jackal mini PC**: Livox 드라이버, FAST-LIO2(SLAM), reactive_explorer, mission_node,
  tag_mapper, clustering_node, safety_layer, Jackal bringup
- **Jetson**: RealSense 드라이버, `apriltag_ros`(tag 검출), YOLO(객체 인식). 모두 같은
  RealSense RGB 스트림 위에서 동시 실행. 검출 결과만 ROS 토픽으로 mini PC에 전송.

**전체 흐름**:
1. SCANNING — mini PC: reactive_explorer 자율 주행 + FAST-LIO2 SLAM. Jetson:
   apriltag_ros + YOLO가 RealSense RGB로 tag/객체 검출 동시 진행.
2. TAG_FOUND — 새 AprilTag 검출 시 reactive_explorer 일시 중지
3. APPROACHING_TAG — 카메라가 tag 정면을 바라보는 적정 거리까지 정밀 접근/도킹
4. → SCANNING 복귀 (이 사이클 반복)
5. SCAN_DONE — 영역 스캔 종료 조건 충족 시 (조건은 §0.10 미정 — 시간/박스 커버리지/신규 검출 부재 등)
6. CLUSTERING — 누적된 모든 tag 좌표를 공간적으로 클러스터링 (서로 다른 tag id들이
   가까이 모인 클러스터를 찾음, 사용자 확정: "같은 id의 다중 검출이 아니라 다른 id들의
   공간 군집")
7. APPROACH_CENTROID — 가장 큰 클러스터의 중심으로 이동
8. DONE

**추가 산출물 필요**:
- `clustering_node` (또는 기존 `mine_cluster_node` 재사용) — DBSCAN/k-means로
  `/tags_in_map` 누적 좌표 → 최대 클러스터 중심 발행
- `mission_node` FSM 확장 — `SCAN_DONE → CLUSTERING → APPROACH_CENTROID` 상태 추가
- 영역 스캔 완료 조건 정의 (미결정, 작업 5 진행 시 확정)
- AprilTag 정밀 접근 제어기 (§7.2 C6, 미시작)

**§7.2의 기존 FSM과의 관계**: 기존 FSM은 `TAG_FOUND → APPROACHING_TAG → DONE`(단일 tag)
구조였음. §0.9에서는 도킹 후 SCANNING으로 복귀 + 모든 tag 누적 후 클러스터링 단계가
추가됨. 다음 세션에서 §7.2 정식 갱신할 것.

### 0.10 2026-06-04 세션 — 아키텍처 큰 전환 + 인프라 정비

**a) 제어팀 인터페이스 합의 (Nav2 스타일 도입)**
원웅님(제어팀)이 제안한 토픽 인터페이스를 perception 팀이 수용. **§1·§3·§11의 "Nav2 미사용"
결정이 사실상 뒤집힘**. 새 흐름:

```
Goal Manager → /goal_pose (PoseStamped)
             ↓
Global Planner (가 2D /map 사용) → /path (nav_msgs/Path)
             ↓
Local Planner → /cmd_vel
             ↓
Jackal twist_mux external slot
```

YOLO 노드(Jetson) → `/tag_detected (std_msgs/Bool)` → Goal Manager 구독.

**미해결 짚어둘 점** (제어팀과 추가 합의 필요):
- `/cmd_vel`을 제어팀은 `Twist`로 표기했으나 Jackal Jazzy bringup은 `TwistStamped` (§5.1 ⚠).
  Local Planner 출력단에 Twist→TwistStamped+namespace 어댑터 필요. 누가 만들 건지 미정.
- `/tag_detected (Bool)`만으로는 Goal Manager가 어디로 갈지 모름. 좌표 전달용
  토픽이 별도 필요 — 본 절(b)의 TF 기반 접근으로 해결.
- `reactive_explorer` 운명: Local Planner inner loop 흡수 / fallback / 폐기 중 미정.
- `mission_node` / `mine_cluster_node` 운명: Goal Manager에 흡수인지 상위 레이어 유지인지 미정.

**b) Cross-distro 통신 이슈와 TF 기반 tag 좌표 합의**
**중요한 발견**: `apriltag_msgs/AprilTagDetection` 메시지가 Humble→Jazzy에서 구조 변경됨.
- Humble: `pose` 필드(3D PoseWithCovarianceStamped) 포함
- Jazzy: `pose` 필드 **삭제**. 대신 `centre`, `corners[4]` (2D 픽셀) + `homography[9]`만.

이 때문에:
- 기존 `mine_recorder_node.py` (`det.pose.header` 접근)는 Jazzy에서 `AttributeError`로
  실행 불가. `sim_apriltag_detector_node.py`도 마찬가지.
- **Jetson(Humble) ↔ mini PC(Jazzy) 사이 `AprilTagDetectionArray` 직접 통신 불가** —
  메시지 type hash가 달라 DDS가 라우팅 안 함. cross-distro 발견은 RealSense `/camera/camera/*`
  토픽으로 검증됨 (그건 잘 보임).

**해결책 — TF 기반 (사용자 확정 2026-06-04)**:
- Jetson `apriltag_ros`(또는 그에 준하는 노드)가 검출 시 **각 tag마다 TF frame 발행**.
  명명 규약: `tag<family>:<id>` (apriltag_ros 표준) 또는 `tag_<id>` (단순화). 부모 frame은
  `camera_<front|back>_color_optical_frame` 또는 `camera_color_optical_frame`.
- mini PC `mine_recorder_node` 리라이트: `/detections`(혹은 동등 토픽)를 구독해서 어떤 tag id가
  지금 보이는지만 얻고, **TF lookup으로 `map → tag_<id>` 변환을 받아** `/mine_positions`에 누적.
- TF는 distro 무관 표준이라 cross-distro OK.
- 추가 이점: `/tag_detected (Bool)`을 별도로 만들지 않아도, `/mine_positions`가 비어있지
  않으면 = 검출됨으로 Goal Manager 측이 판단 가능. 또는 어댑터 노드 1줄로 변환 가능.

**c) Octomap_server 통합 (SLAM 측 의무 추가)**
Global Planner가 2D `/map` 입력 필요 → `ros-jazzy-octomap-server` apt 설치 + 새 launch
(`jackal_mine_detection/launch/sensor/octomap.launch.py`). `/cloud_registered`(3D, FAST-LIO2)
→ projected_map(2D) → `/map`으로 remap. 동시에 `map ↔ odom` static_transform_publisher 포함
(FAST-LIO2가 `map→odom` TF를 발행하지 않고 `odom→base_link`만 발행하기 때문).

부수 설치/패치:
- `ros-jazzy-pcl-ros` 2.6.2 → 2.6.4 업그레이드 (`libpcl_ros_tf.so` 누락 → octomap_server 로드
  실패였음).
- `ros-jazzy-apriltag-msgs` 설치 (mine_recorder import에 필요).

**d) FAST-LIO2 `/path` remap**
제어팀 `/path`(Local Planner 입력)와 충돌 방지 위해 FAST-LIO2의 SLAM 궤적 토픽을
`/fastlio/path`로 remap. `src/FAST_LIO/launch/mapping.launch.py`에 `remappings=[('/path','/fastlio/path')]`
추가. 재빌드 필요.

**e) Foxglove 시각화 활성화**
Jackal bringup이 `foxglove_bridge`를 포트 8765에 자동 실행 중. Tailscale로 `ws://100.72.78.94:8765`
접속해서 3D 패널에서 `/map` + `/cloud_registered` + TF 확인 완료. RViz 대신 표준 시각화 경로.

**f) 검증 완료된 것**
- octomap_server 정지 상태에서 `/map` 10Hz 발행, frame=map, FOV 안쪽 grid 채워짐 ✓
- `fake_mine_publisher → mine_cluster_node(dbscan)` 파이프라인 — 입력 (2,1),(2.2,1.1),(2.1,0.8)
  + 외톨이 (5,-1) → 출력 centroid (2.1, 0.967, 0) 정확 ✓
- §0.9 클러스터링 알고리즘 동작 확인 ✓

**g) 다음 진입점**
- ✅ mini PC: `mine_recorder_node` TF 리라이트 + 풀 체인(sim_apriltag → mine_recorder →
  mine_cluster) 검증 완료 (2026-06-04 오후, §0.10h 참조)
- mini PC: FAST-LIO2 재시작해서 `/path` remap 적용 (사용자 Terminal 2 Ctrl+C → 재실행)
- mini PC: 자칼 움직이며 `/map` 확장 검증
- Jetson(별도 세션): `apriltag_ros` 띄워서 검출 시 TF 발행 동작 확인. **frame 명명 규약:
  `tag36h11:<id>`, 부모는 camera optical frame** (mini PC mine_recorder의 `tag_frame_pattern`
  기본값과 일치해야 함). `apriltag_headless_test`는 test 스크립트로 publish 없음 — 폐기 또는
  정식 구현으로 교체

**h) 풀 체인 검증 + DDS 스냅샷 불일치 해결 (2026-06-04 오후 세션)**
- **apt 스냅샷 불일치 이슈**: `ros-jazzy-apriltag-msgs`(4월 빌드)가 fastcdr 2.2.7 심볼을
  요구하는데 설치된 fastcdr은 2.2.5(1월 빌드) → `/detections` 발행 시
  `undefined symbol: _ZN8eprosima7fastcdr3Cdr9serializeEj` 크래시.
  fastcdr만 2.2.7로 올리면 이번엔 1월 빌드 fastrtps 2.14.5와 런타임 비호환
  (`BadParamException: This member is not been selected` — 모든 노드 생성 실패).
  **해결**: DDS 레이어 7개 패키지를 모두 현행 스냅샷으로 통일 — fastcdr 2.2.7,
  fastrtps 2.14.6, rmw-fastrtps-cpp/shared-cpp(4월), rosidl-typesupport-fastrtps-c/cpp(4월),
  rosidl-dynamic-typesupport-fastrtps(3월).
  ⚠ 교훈: **이 시스템은 1월/4월 ROS 스냅샷 혼재 상태 (559개 업그레이드 보류 중).
  ros-jazzy 패키지를 새로 설치하면 의존 패키지끼리 빌드 스냅샷이 어긋나 심볼/런타임
  에러가 날 수 있음. 증상이 같으면 관련 패키지군을 같은 스냅샷으로 묶어 업그레이드할 것.**
- **풀 체인 검증 성공** (로봇 정지, 가상 TF `map→sim_base` (3.0,0.5,0) yaw 90°):
  sim_apriltag_detector(TF broadcast + /detections) → mine_recorder(TF lookup) →
  mine_cluster. tag 0/1/2 검출(범위 밖 tag 3 제외 정확), `/mine_positions` 좌표
  시뮬값과 일치, `/mine_cluster_center` = (3.0, 1.45, 0.8) centroid 정확, CSV 기록 정상.
  → **§0.10b TF 기반 인터페이스의 mini PC 측 구현은 완료. 남은 건 Jetson 측 TF 발행.**
- 부수: 홈 디렉토리 정리 (~2.6GB — vscode-server/claude 구버전, ROAS_BACKUP.zip, 옛 빌드 로그)

### 0.11 2026-06-04 저녁 — Jetson 인식 파이프라인 핸드오프 수신 + mini PC 정합 작업

**a) Jetson 측 완료 사항** (Jetson 세션 핸드오프, `~/ros2_ws/jackal-ORCA/src/jackal_orca_perception`):
- 패키지를 apriltag+YOLO 전용으로 정리. slam.launch/odom_watch/gazebo 삭제.
  **placeholder static TF 제거 — Jetson은 TF를 일절 발행하지 않음.**
- 카메라 이름 camera_front/camera_back 분리 (optical frame §4와 일치). "rear"→"back" 통일.
- 토픽 계약(§5.1) 일치: `/camera_{front,back}/color/image_raw`, `/apriltag_{front,back}/detections`.
- `custom_msgs/TagCandidate.msg` 신설 (Jetson 빌드 통과).
- `tag_yolo_detector_node`: camera_info(fx,cx) + 장착 yaw(front=0, back=π)로 bearing 계산
  → `/yolo/tag_candidate` 발행 (3프레임마다 추론, conf≥0.30). camera_info 수신 전 발행 안 함.
- `tag_recorder_node`는 디버그 백업으로 강등 — 정본 기록은 mini PC tag_mapper.
- Jetson 실행: `ros2 launch jackal_orca_perception apriltag_pipeline.launch.py enable_yolo:=true`

**b) ⚠ §0.10b TF 합의와의 충돌 — 미해결**
- Jetson이 TF를 발행하지 않으므로, §0.10h에서 검증한 `mine_recorder`의 TF lookup 방식은
  **실기에서는 동작 불가** (sim 전용으로 강등). 새 계약: tag pose는 `/apriltag_*/detections`로.
- 그러나 §0.10b에서 **Humble↔Jazzy `AprilTagDetectionArray` 직접 통신 불가** 확인됨.
  Jazzy 정의 확인 완료(2026-06-04 저녁): family/id/hamming/goodness/decision_margin/
  centre/corners[4]/homography[9] — **pose 없음**.
- **해결 후보**: Jetson 측 정의와 비교 후 (handoff 4번), 불일치 시 mini PC에서 Humble 호환
  `apriltag_msgs`를 워크스페이스 오버레이로 소스 빌드 → 정의 일치시켜 라우팅 복구.
  (mini PC에서 apriltag_msgs를 쓰는 건 자체 노드들뿐이라 오버레이 부작용 없음)

**c) mini PC 측 완료**
- ✅ `custom_msgs` 패키지 신설 + `TagCandidate.msg` 추가, 빌드·`ros2 interface show` 검증 OK.
- ✅ DDS 환경 확인: mini PC는 ROS_DOMAIN_ID/LOCALHOST_ONLY/RMW 전부 미설정(기본값
  0/0/fastrtps). Jetson도 동일 기본값이어야 함.

**d) mini PC 측 TODO (Jetson 핸드오프 우선순위 순) — 진행 현황**
1. ✅ `mission_node` 시각 서보잉 — **구현·시뮬 검증 완료** (§0.11e).
   back 카메라 검출 시 **후진으로 접근** (사용자 확정 2026-06-04).
2. `tag_mapper`(F9) — **계획 변경으로 폐기**: apriltag_msgs 정의 비교 결과(아래 4)
   **양 distro 모두 pose 필드 없음** → detections 기반 3D 변환 자체가 불가.
   대신 **도킹 위치 기록 방식**으로 확정 (사용자 확정 2026-06-04): 도착 판정 순간
   로봇의 map 위치 + 진행방향 dock_offset_m(0.5)을 태그 좌표로 기록. mission_node에
   내장됨. 캘리브·시간동기 불요. 오차 ~0.5m는 클러스터링 반경(1m) 안.
3. ✅ chrony 설치 완료 (서버/클라이언트 설정은 미적용 — 도킹 방식 채택으로 필수성은
   낮아짐. 로그 시각 정합용으로 후순위 진행 권장).
4. ✅ apriltag_msgs 정의 검증 완료: **Jetson(Humble)과 mini PC(Jazzy) 정의가 완전 동일**
   (둘 다 family/id/hamming/goodness/decision_margin/centre/corners/homography, pose 없음).
   → §0.10b의 "Humble에 pose 있음 / type hash 불일치로 라우팅 불가" 결론은 **정정 필요**.
   정의가 같으므로 cross-distro 라우팅이 될 가능성 높음 — 실연동 시 재검증.
5. ✅ DDS 환경: 양쪽 기본값 (mini PC 확인 완료, Jetson도 기본값이어야 함).

**e) mission_node 구현·검증 (2026-06-04 저녁)**
- `jackal_mine_detection/mission_node.py` 신규. FSM: IDLE→SCANNING→APPROACHING_TAG
  →(도킹 기록)→COOLDOWN→SCANNING. 20 Hz 서보잉(front: bearing→0 전진 / back:
  bearing→π **후진**), 도착 판정 = 해당 카메라 detections `arrival_frames`(8) 연속 수신.
  이탈: candidate 끊김(2s)/접근 타임아웃(60s). COOLDOWN(15s) 동안 candidate 무시
  (YOLO candidate에 id가 없어 같은 태그 재트리거 방지). 추가 억제: 최근 2s detections
  id가 전부 기록완료면 트리거 무시. 기록: `/mine_positions`(PoseArray) +
  `/tmp/mission_tag_positions.csv`. 상태: `/mission/state`(std_msgs/String, 10 Hz —
  §5.2 MissionState 커스텀 메시지 대신 String 채택, custom_msgs 양 호스트 동일성 유지 목적).
- `reactive_explorer`에 §5.3 게이팅 추가: `/mission/state` 구독, `active_states`
  (기본 SCANNING·COOLDOWN) 밖이면 cmd_vel 발행 중지. 미수신 시(단독 실행) 항상 발행.
- 시뮬 검증 (가상 TF, cmd는 `/mission_test/cmd_vel`로 격리):
  front 서보잉 lin/ang 계산값 일치, back 후진 lin<0 일치, front 도킹 기록 (3.0,1.0) ✓,
  back 도킹 기록 (3.0,0.0) ✓ (로봇 (3.0,0.5) yaw90° 기준), FSM 전이 전부 정상.
- ⚠ explorer 게이팅은 LiDAR 입력이 필요해 라이브 미검증 — 다음 실기 기동 시 확인.
- ⚠ 테스트 시 교훈: `ros2 run`을 background로 띄우고 wrapper PID를 kill하면 자식
  노드가 살아남음. 정리는 `pkill -TERM -P <wrapper_pid>` 후 wrapper kill. pgrep -f
  cmdline 패턴 매칭은 자기 셸을 죽일 수 있어 금지.

### 0.12 2026-06-04 — 제어 로직 인수 + 미합의 사항 해소 (하이브리드 A* 스택 완성)

**배경**: 사용자가 제어 로직 팀 몫까지 직접 구현하기로 결정. §0.10a의 미합의
사항 4건을 우리 스택 기준으로 확정.

**a) 미합의 사항 해소 결정** (사용자 확정 2026-06-04)
| 항목 | 결정 |
|---|---|
| Twist↔TwistStamped 어댑터 | **불요** — 우리가 발행자이므로 처음부터 TwistStamped 발행 (safety_layer 출력) |
| `/tag_detected` 좌표 전달 | 도킹 위치 기록 방식으로 이미 해결 (§0.11d-2) |
| Global Planner (`/path`) | **하이브리드 채택**: 신규 `global_planner_node`(A* on `/map`) → `/path`, reactive_explorer 가 carrot 추종 (회피 로직 유지). 합의 토픽 계약(/goal_pose→/path→/cmd_vel) 준수 |
| reactive_explorer / mission_node 운명 | 유지 — reactive 는 local planner 역할 흡수, mission 은 상위 레이어 |

**b) 구현 내역**
- **`global_planner_node.py` 신규**: `/goal_pose`+`/map`+TF → A* → `/path`.
  장애물 chebyshev 팽창(robot_radius 0.27 = reactive swept_half 와 일치),
  unknown 셀은 비용 3배로 통과 허용(frontier goal 대응), start/goal 막힘 시
  최근접 free 셀 스냅, Bresenham LOS 단축(WP 간격 ≤2m), 2s 주기 리플랜,
  실패 시 **빈 Path 발행** = 실패 신호.
- **`reactive_explorer` carrot 추종 추가**: `/path` 신선(10s)하면 lookahead
  1m 내 최원점을 조향 목표로, 도달 판정은 path 끝점 기준(planner 가 goal 을
  스냅했을 수 있음). path 없음/빈 path/stale → 기존 goal-bias 직선 추종 폴백
  → **플래너가 죽어도 주행 지속**. 새 goal 수신 시 구 path 폐기.
- **`mine_goal_sender` 전면 리라이트**: Nav2 action + 복도(y=0) 하드코딩 폐기.
  `/finish_exploration` 트리거 → `/mine_cluster_center`에서 로봇 방향으로
  approach_offset(0.8m) 떨어진 지점을 `/goal_pose` 발행, yaw 는 중심 응시.
  `/goal_reached` 수신 → `/final_goal_reached` 발행. 90s 재전송 ×3회 후 포기.
- **레거시 삭제** (git 히스토리에 보존): waypoint_follower_node,
  exploration_manager_node + Nav2/Gazebo 시대 launch 13개 (stage1~4*,
  hw_frontier, waypoint_explorer, mine_pipeline, sensor/nav2) + nav2_params.yaml.
  params.yaml 정리 (global_planner / 신규 mine_goal_sender 섹션 추가).
- **autonomy.launch.py 갱신**: global_planner (`use_planner:=true` 기본) +
  mine_goal_sender 추가. 전체 체인:
  `frontier → /goal_pose → global_planner → /path → reactive(carrot+회피)
  → /cmd_vel_raw → safety_layer → /j100_0915/cmd_vel`

**c) 검증 (격리 도메인, 18/18 통과)**
A* 단위(벽+갭 우회·unknown 통과·완전차단 None·start 스냅), carrot 선택 3종,
ROS 통합(/path 발행·빈 path), mine_goal_sender 트리거→goal→reached 체인.
테스트 스크립트: 단일 프로세스 executor 방식 (`/tmp/test_control_stack.py`).

**d) ⚠ 교훈 — 라이브 DDS 오염**
시뮬 테스트를 기본 도메인에서 돌렸더니 **실기 스택(octomap /map 10Hz, 실제
TF)이 섞여 들어와 가짜 실패** + 반대로 테스트의 가짜 /map·/goal_pose 가
라이브 reactive_explorer 에 주입됨 (safety_layer paused=True 라 실주행 없었음).
**앞으로 시뮬 테스트는 반드시 `ROS_DOMAIN_ID=77
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 로 격리할 것.**

**e) 다음 진입점**
- 실기에서 떠 있는 reactive_explorer/mission_node 등은 **구버전 코드** —
  재시작해야 carrot 추종 반영됨: `ros2 launch jackal_mine_detection
  autonomy.launch.py` (octomap, FAST-LIO 는 그대로 두면 됨)
- 실기 라이브 검증: /map 위 A* path 생성 → carrot 추종 → frontier 사이클
- 라이브 노드 중복 정리: `map_to_odom_static` 가 9개 떠 있음 (octomap launch
  재실행 잔재로 추정) — 재부팅 또는 정리 후 1개만 유지
- explorer 게이팅 라이브 검증 (§0.11e 잔여), 자칼 주행 중 /map 확장 검증 (§0.10g)

### 0.13 2026-06-05 — 라이브 체인 검증 + 세션 제어 스크립트 (`jackal_ctl.sh`)

**a) 라이브 검증 완료** (§0.12e 진입점 소화):
- 부팅 후 enp86s0 IP 소실 재발 → netplan 영구화 권장 (§0.8 그대로)
- 풀 스택 기동: frontier → `/goal_pose` → global_planner → `/path`(frame=map) →
  reactive carrot 수신 → `/cmd_vel_raw` → safety_layer 모두 라이브 확인 ✓
- explorer `/mission/state` 게이팅 라이브 확인 ✓ (§0.11e 해소)
- `/fastlio/path` remap 라이브 적용 확인 ✓
- pause/resume 라이브 확인: `/pause false` → 1s 내 cmd 통과, `true` → 즉시 0 ✓
- 잔여: 실주행 carrot 추종 / 주행 중 /map 확장 (§0.10g) — 아직 pause 상태로 대기

**b) ⚠ Foxglove 로봇-맵 정렬 문제 원인 규명**:
`/tf`(FAST-LIO odom→base_link)와 `/j100_0915/tf`(휠 오도메트리 odom→base_link)가
**같은 frame 이름**을 발행. Foxglove 는 모든 TF 토픽을 합치므로 로봇 포즈가 휠
오도메트리로 덮어써져 `/map`(FAST-LIO 좌표계)과 어긋남. 로봇이 안 움직였을 땐
두 값이 우연히 일치해 안 보이던 문제.
**해결**: 전용 `foxglove_bridge`(노드명 `foxglove_bridge_map`, **port 8766**)를
`/j100_0915/tf` 제외 whitelist 로 운용. 자율주행 시각화는 `ws://100.72.78.94:8766`,
플랫폼 진단은 기존 8765. Foxglove 3D 패널: Display frame=`map`, Follow mode=`Fixed`.

**c) `scripts/jackal_ctl.sh` 신설** (go/pause/resume/status):
- `go [--hold]` — 새 세션: 이전 SLAM/맵/탐사/플래너 전부 정리 후 재기동 (SLAM 원점
  = 현재 로봇 위치 → 정렬 보장). `--hold` 면 무장 정지 상태로 준비만.
- `pause` / `resume` — `/pause` Bool 발행 (세션 유지). `status` — 프로세스/상태 요약.
- 원칙: safety_layer 비재시작(pause 경로 상시 유지), livox 드라이버 불간섭,
  8766 브리지 자동 기동.
- 구 `jackal_newrun.sh` 는 호환 래퍼로 강등. **구버전 버그**: global_planner /
  mine_goal_sender 를 죽이기만 하고 재시작 안 했음 → launch 고아 노드가 우연히
  체인을 메꿔주고 있었음. jackal_ctl 에서 수정.
- 주의: 스크립트에 `set -u` 금지 (ROS setup.bash 가 unbound variable 에러).

**d) 안전 기능 추가 (safety_layer v2, 2026-06-05 오후)**:
- **끼임(stuck) 자동 정지**: 속도 명령(|lin|≥0.05 또는 |ang|≥0.10)이 4s 지속되는데
  /Odometry(FAST-LIO) 이동<6cm & 회전<8.6° → 자동 pause 래치. 격리 도메인 검증:
  정지 odom→3s 래치 ✓, 이동 odom→오탐 0 ✓. 파라미터 stuck_*, 끄려면 stuck_enable:=false.
- **미션 완료 정지**: /final_goal_reached 수신 → 자동 pause (클러스터 중심 도착 후 그 자리 종료).
- frontier_explorer: /pause 구독 추가 — pause 중 탐사 타임아웃 동결
  (이전엔 pause 중에도 900s 카운트다운되어 조기 /finish_exploration 발행되는 버그).

**e) 미션 풀 체인 통합 완료 (2026-06-05 오후)**:
- `mine_cluster_node`가 스택에 빠져 있던 것 발견 → jackal_ctl go + autonomy.launch.py에
  추가 (cluster_radius=1.0, min_cluster_size=2, radius_count).
- 전체 체인: SCANNING(frontier→planner→reactive) → /yolo/tag_candidate(Jetson) →
  mission 서보잉·도킹기록 → /mine_positions → mine_cluster → /mine_cluster_center
  → (탐사종료) /finish_exploration → mine_goal_sender → /goal_pose(중심 0.8m 앞)
  → /goal_reached → /final_goal_reached → safety 자동 정지 🏁
- 후반부(클러스터→최종goal→완료정지) 격리 도메인 E2E 검증: 중심 (3.03,0.97) 정확
  (외톨이 제외), goal=중심 0.8m 앞 (2.27,0.72), 완료 정지 래치 ✓.
- **Jetson 연동 라이브 확인**: /apriltag_{front,back}/detections, /yolo/tag_candidate,
  /camera_* 모두 mini PC에서 보임 — §0.11b의 cross-distro 라우팅 우려 해소.

**f) Foxglove 대역폭 (WiFi 끊김 해결)**:
- /map 풀레이트 ≈17 Mbps가 WiFi 포화 → /path 등 작은 토픽 굶음.
- `topic_tools throttle`로 /viz/map(1Hz)·/viz/cloud(2Hz) 생성, 8766 브리지 whitelist를
  viz 토픽으로 교체 (/map·/cloud_registered 원본은 일부러 미노출).
  **Foxglove 3D 패널은 /viz/map, /viz/cloud 사용할 것.** 재기동: `jackal_ctl.sh viz`.

**g) ⚠⚠ pkill -f self-kill 함정 (이번 세션 3회 발생)**:
`pkill -f <패턴>`은 패턴이 **자기 셸/래퍼의 cmdline에 포함되면 자신을 죽임**.
이전 세션 background launch들이 죽은 미스터리 = 구 jackal_newrun.sh의 pkill이
Claude background 래퍼(cmdline에 launch 명령 텍스트 포함)를 잡은 것.
규칙: ① 스크립트 실행 커맨드라인에 노드 경로 문자열 넣지 말 것 (py_compile 인자도 위험)
② 인터랙티브 pkill은 브래킷 패턴(`pkill -f "[s]afety_layer_node"`) ③ jackal_ctl.sh 자체는 안전.

**h') 미션 사운드 효과 (`sound_player_node`, 2026-06-05 저녁)**:
USB에서 복사한 mp3 4종 (`~/colcon_ws/sounds/`) — SCANNING=scifi_beeps(루프),
태그발견(→APPROACHING_TAG)=lock_on, 도킹기록(→COOLDOWN)=scanner, 끼임(stuck 증가)=pullup_alarm.
/mission/state·/safety/state 구독, mpg123 → 기본 sink(JBL). pause 중 루프 자동 정지.
jackal_ctl go + autonomy.launch.py 에 포함. 격리 도메인 검증 완료.

**i) 보수성 완화 + 복도 SLAM 튜닝 + 박스 모델 (2026-06-05 저녁, B-런 준비)**:
- **주행 보수성 완화** (사용자: "갈 수 있는 길을 못 간다"): swept_half 0.27→0.235
  (실측 폭 0.47/2), escape 0.45/0.80→0.35/0.60, safe_dist 0.7→0.55, side_safe
  0.4→0.30, safety hard_stop 0.25→0.20, planner radius 0.27→0.20 (팽창 3→2셀).
- **복도(퇴화 환경) FAST-LIO 튜닝** (mid360.yaml, 선행 사례 기반): point_filter 2→1,
  max_iter 4→8, voxel 0.3→0.2, det_range 100→40, cube 1000→500. CPU 23% 검증.
  근본 한계는 퇴화 인지 LIO(D²-LIO 등) 필요 — 현 스택에선 잔특징 보존이 최선.
- **로봇 직육면체 모델** (사용자 실측 0.50H×0.47W×0.55L, 여유 포함):
  라이다 광학중심 높이 0.39m (스캔 바닥 최저점 -0.39 교차 확인). z 슬라이스 =
  [바닥+5cm, 라이다+0.11] → ≤5cm 턱은 통과, 로봇 상단보다 높은 개구부 통과 허용.
  reactive·safety 는 jackal_ctl 이 z_min=-0.34/z_max=0.11 주입, octomap 은
  occupancy_min/max_z 동일 기준. **Mid-360 하향 FOV -7° → 근거리 바닥 사각지대**:
  낮은 장애물은 1.5~4m 에서 octomap 이 기억 → 플래너 우회가 담당, 끼임 감지가 최후 안전망.
- ⚠ **pub_pause DDS 레이스 수정**: `-w 1` + 5s 발행 (짧은 pub 는 매칭 전 종료되어 유실).
- ⚠ stuck 오탐 완화: 6s/5.7° (스키드스티어 제자리 회전은 명령의 ~1/10 속도).

**j) "막히면 후진" 자동 복구 + 라이다 전후 위치 실측 (2026-06-05 밤)**:
- **배경**: B-런에서 mission 후진 접근 중 구석에 끼어 stuck 2회. APPROACHING_TAG
  중엔 reactive(ESCAPE 보유)가 게이트로 꺼져 mission 이 벽에 밀어붙임.
- **mission blocked-abort**: /safety/state 의 front/back 거리 파싱, 접근 방향이
  1.5s 연속 막히면(front<0.40 / back<0.80) 접근 포기 → COOLDOWN.
- **safety stuck 자동 복구**: stuck 감지 시 즉시 래치 대신 LiDAR 여유 방향으로
  0.1 m/s × 2s 탈출 기동 (여유 0.05m 미만 시 조기 중단). 60s window 에 2회까지,
  초과 시 래치 + **/pause true 전파** (frontier 타이머·사운드 동기 — 기존엔 내부
  래치라 다른 노드가 모르는 갭 있었음). 격리 검증: 복구→복구→래치 시퀀스 ✓.
- **라이다 전후 위치 실측**: 후단-0.40m (전장 0.55 → 전단 0.15). front_extent
  0.10→0.15, rear_extent 0.45(실측 0.40+여유 0.05) → 차단 임계 차체 기준
  전방 0.20m/후방 0.25m — 후방이 더 보수적 (저고도 후방은 차체 가림 사각).
- go 는 safety 를 재시작하지 않으므로 코드 변경 시 safety 수동 재시작 필요 (완료).

**h) 블루투스 스피커 자동 연결 (JBL Go 4, E8:26:CF:82:BE:88)**:
trust 설정 + systemd user 서비스 `bt-speaker.service`(linger 활성화로 부팅 시 기동).
`scripts/bt_speaker_connect.sh`가 15s 주기로 연결 감시 — 스피커를 늦게 켜도 자동 연결.
로그: `journalctl --user -u bt-speaker.service`.

---

## 1. 프로젝트 개요

### 목표
Jackal UGV가 **장애물을 자율 회피하며 주행해 3D SLAM 맵을 생성**하고,
동시에 환경에 설치된 **AprilTag를 검출하여 맵 좌표계에 등록**한다.
Nav2는 사용하지 않으며, **반응형 회피 주행**(기본 전진, 정면이 막히면
회전해서 우회) 방식. 영역 커버리지를 명시적으로 추적하지 않고, 주행
경로의 부산물로 자연스럽게 맵이 확장된다.

### 핵심 의사결정
- **3D SLAM**: FAST-LIO2 (Livox Mid-360 비반복 스캔에 최적화)
- **자율 주행**: 자체 구현, 반응형 회피 (Nav2 미사용 — 시간 제약)
- **AprilTag 검출**: depth 카메라 2대 (전/후방), 검출 전용
- **장애물 회피**: LiDAR 단독 (depth 카메라는 회피에 사용 안 함)
- **모든 계산은 Jackal mini PC에서 수행**

### 팀 구성
- **센서 퓨전 팀**: 인식·위치추정 담당 (SLAM, 캘리브, AprilTag pose 변환)
- **제어 로직 팀**: 주행·미션 담당 (자율 주행, 미션 FSM, 안전 로직)

---

## 2. 하드웨어 구성

| 장치 | 모델 | 역할 | 위치 |
|---|---|---|---|
| UGV 플랫폼 | Clearpath Jackal | 베이스 이동체 | — |
| 컴퓨트 | Jackal 내장 mini PC | SLAM + 제어 + 미션 모두 수행 | Jackal 내부 |
| LiDAR | Livox Mid-360 | 3D SLAM, 장애물 회피 | 상단 중앙 |
| IMU | Mid-360 내장 BMI088 | SLAM용, 200 Hz | LiDAR 내부 |
| Depth 카메라 1 | (예: RealSense D435) | AprilTag 검출만 | 전방 |
| Depth 카메라 2 | (예: RealSense D435) | AprilTag 검출만 | 후방 |

**주의**: depth 카메라는 RGB 영상만 AprilTag 검출에 사용. depth 채널은
사용하지 않음 (다만 향후 정밀 도킹 시 활용 여지 있음).

---

## 3. 소프트웨어 스택

| 컴포넌트 | 선택 | 출처 |
|---|---|---|
| OS | Ubuntu 22.04 LTS | — |
| 미들웨어 | ROS 2 Humble | docs.ros.org |
| Livox 드라이버 | livox_ros_driver2 | github.com/Livox-SDK |
| 3D SLAM | FAST-LIO2 | github.com/hku-mars/FAST_LIO |
| AprilTag | apriltag_ros | github.com/christianrauch/apriltag_ros |
| 카메라 | (모델별 ROS 2 드라이버) | — |
| 자율 주행 | 자체 구현 (reactive_explorer.py) | 본 프로젝트 |
| 미션 로직 | 자체 구현 (Python FSM) | 본 프로젝트 |

### 외부 패키지로 해결, 직접 구현 안 함
- SLAM 알고리즘
- LiDAR-IMU 정합
- AprilTag 검출
- 카메라 캘리브레이션

### 직접 구현 필요
- `tag_mapper`: AprilTag pose → map frame 변환 노드
- `reactive_explorer`: 반응형 회피 주행 노드 (전진 + 회피)
- `mission_node`: 미션 FSM (단순 Python)

---

## 4. 좌표계 (TF Tree)

REP-105 준수.

```
map
 └── odom                              [FAST-LIO2 publish]
      └── base_link                    [Jackal 본체]
           ├── livox_frame             [static, URDF]
           │    └── livox_imu_frame    [static, 공장값]
           ├── camera_front_link       [static, URDF]
           │    └── camera_front_color_optical_frame
           └── camera_back_link        [static, URDF]
                └── camera_back_color_optical_frame
```

| Transform | Publisher | 종류 |
|---|---|---|
| map → odom | FAST-LIO2 | dynamic |
| odom → base_link | FAST-LIO2 | dynamic |
| base_link → livox_frame | URDF | static |
| livox_frame → livox_imu_frame | URDF (공장 캘리값) | static |
| base_link → camera_*_link | URDF (캘리값) | static |
| camera_*_link → camera_*_color_optical_frame | 카메라 드라이버 | static |

**중요**: 모든 AprilTag detection은 `camera_*_color_optical_frame`에서 나옴.
`tag_mapper` 노드가 이를 `map` frame으로 변환.

---

## 5. ROS 통합 인터페이스 명세 (팀 간 계약)

### 5.1 토픽 목록

| Topic | Type | Rate | Publisher | Subscriber | Frame |
|---|---|---|---|---|---|
| `/livox/lidar` | sensor_msgs/PointCloud2 | 10 Hz | livox driver | FAST-LIO2, reactive_explorer | livox_frame |
| `/livox/imu` | sensor_msgs/Imu | 200 Hz | livox driver | FAST-LIO2 | livox_imu_frame |
| `/camera_front/color/image_raw` | sensor_msgs/Image | 30 Hz | cam driver | apriltag_front | camera_front_color_optical_frame |
| `/camera_front/color/camera_info` | sensor_msgs/CameraInfo | 30 Hz | cam driver | apriltag_front | — |
| `/camera_back/color/image_raw` | sensor_msgs/Image | 30 Hz | cam driver | apriltag_back | camera_back_color_optical_frame |
| `/camera_back/color/camera_info` | sensor_msgs/CameraInfo | 30 Hz | cam driver | apriltag_back | — |
| `/apriltag_front/detections` | apriltag_msgs/AprilTagDetectionArray | 30 Hz | apriltag_ros | tag_mapper | camera_front_color_optical_frame |
| `/apriltag_back/detections` | apriltag_msgs/AprilTagDetectionArray | 30 Hz | apriltag_ros | tag_mapper | camera_back_color_optical_frame |
| `/Odometry` | nav_msgs/Odometry | 100+ Hz | FAST-LIO2 | reactive_explorer, mission | odom → base_link |
| `/cloud_registered` | sensor_msgs/PointCloud2 | 10 Hz | FAST-LIO2 | octomap_server, (시각화) | map |
| `/map` | nav_msgs/OccupancyGrid | 10 Hz | octomap_server | frontier_explorer, global_planner | map |
| ~~`/tags_in_map`~~ | ~~custom_msgs/TagPoseArray~~ | — | ~~tag_mapper~~ | **폐기** (§0.11d-2 도킹 기록 방식) | — |
| `/goal_pose` | geometry_msgs/PoseStamped | 비주기 | frontier_explorer / mine_goal_sender | global_planner, reactive_explorer | map |
| `/path` | nav_msgs/Path | ~0.5 Hz | global_planner | reactive_explorer (carrot) — 빈 Path=계획 실패 | map |
| `/goal_reached` | std_msgs/Bool | 이벤트 | reactive_explorer | frontier_explorer, global_planner, mine_goal_sender | — |
| `/finish_exploration` | std_msgs/Bool | 이벤트 | frontier_explorer | mine_goal_sender | — |
| `/final_goal_reached` | std_msgs/Bool | 이벤트 | mine_goal_sender | (모니터링) | — |
| `/cmd_vel_raw` | geometry_msgs/TwistStamped | 10–20 Hz | reactive_explorer / mission_node | safety_layer | base_link |
| `/cmd_vel` ⚠ | **geometry_msgs/TwistStamped** | 20 Hz | **safety_layer** | twist_mux `external` slot → Jackal base | base_link |
| `/pause` | std_msgs/Bool | 이벤트 | (수동) | safety_layer — 긴급 정지/재개 | — |
| `/explorer/state` | std_msgs/String | 1 Hz | reactive_explorer | (디버깅) | — |
| `/planner/state` | std_msgs/String | 1 Hz | global_planner | (디버깅) | — |
| `/mission/state` | std_msgs/String ⚠§0.11e | 10 Hz | mission_node | reactive_explorer 게이팅, frontier 동결 | — |

> **⚠ Jazzy + 최신 Clearpath bringup 주의사항 (2026-05-27 확인)**
> - `/cmd_vel`의 실제 메시지 타입은 `geometry_msgs/TwistStamped` (Humble 시대 `Twist`에서 변경됨).
> - 본 표는 추상 토픽명. 실제 Jackal에서는 모두 namespace `j100_0915`가 붙음
>   (예: `/cmd_vel` → `/j100_0915/cmd_vel`).
> - 외부 cmd_vel 발행자는 `/j100_0915/cmd_vel`로 발행 → `twist_mux`의 `external` 슬롯
>   (priority 1, timeout 0.5 s)으로 들어감. joy(10) / rc(12) 우선순위가 높아 자동 override됨 = 1차 안전망.
> - `/cmd_vel` 응답 검증 완료 (2026-05-27, 들어올린 상태):
>   전진 0.15 m/s 5s → 양 바퀴 +3.97/+3.96 m, 좌·우 회전 3s × 2 → 반대 부호 균등 변화.

### 5.2 커스텀 메시지

```
# custom_msgs/msg/TagPose.msg
std_msgs/Header header                # frame_id = "map"
int32 tag_id
geometry_msgs/PoseWithCovariance pose
float64 detection_confidence          # 0.0 ~ 1.0
builtin_interfaces/Time last_seen
string source_camera                  # "front" or "back"
```

```
# custom_msgs/msg/TagPoseArray.msg
std_msgs/Header header
custom_msgs/TagPose[] tags
```

```
# custom_msgs/msg/MissionState.msg
std_msgs/Header header
uint8 IDLE=0
uint8 SCANNING=1                      # reactive_explorer 활성
uint8 TAG_FOUND=2
uint8 APPROACHING_TAG=3
uint8 DONE=4
uint8 ERROR=5
uint8 state
int32 current_target_tag_id
string status_message
```

### 5.3 `/cmd_vel` 발행 중재 규칙
- `mission_node`가 IDLE 또는 SCANNING이면 `reactive_explorer`가 발행
- `mission_node`가 APPROACHING_TAG 등 다른 상태면 `mission_node`가 발행
- **두 노드가 동시에 발행하지 않도록 둘 중 하나가 publisher를 enable/disable**

권장 구현: `reactive_explorer`가 `/mission/state` 구독해서 SCANNING이 아니면
스스로 STOPPED로 전환하고 publish 안 함.

---

## 6. 캘리브레이션 절차

순서가 중요. 뒤 단계가 앞 단계 결과를 사용함.

### Step 1. 카메라 intrinsic (전/후방 각각)
- 도구: `ros2 run camera_calibration cameracalibrator ...`
- 체커보드 사용
- 산출물: `camera_front_info.yaml`, `camera_back_info.yaml`

### Step 2. LiDAR ↔ IMU extrinsic
- Mid-360 공장 캘리값 사용 (시간 없으면 검증 스킵)
- 검증 도구: LI-Init (FAST-LIO2 부속)
- 산출물: `lidar_imu_extrinsic.yaml`

### Step 3. LiDAR ↔ 카메라 extrinsic (전/후방 각각)
- 도구: `livox_camera_calib` (HKU-MARS)
- 환경: 직선 엣지 많은 실내
- 시간 부족 시: 줄자 측정 → URDF에 직접 입력 (정확도 5cm 이내면 미션 OK)
- 산출물: 4x4 변환 행렬 → URDF의 `base_link → camera_*_link`에 반영

### Step 4. URDF 통합
```bash
ros2 run tf2_tools view_frames
```
TF tree 정상 출력 확인. RViz에서 두 카메라 영상과 PointCloud가 정합되는지
시각 검증.

---

## 7. 팀 업무 분담

### 7.1 센서 퓨전 팀 — 인식·위치추정

| # | 업무 | 산출물 | 우선순위 |
|---|---|---|---|
| F1 | Livox 드라이버 설정, IMU/PointCloud 발행 검증 | bag 파일 | P0 |
| F2 | 전/후방 카메라 드라이버 설정 | launch | P0 |
| F3 | 카메라 intrinsic 캘리브 (각 1회) | yaml | P0 |
| F4 | LiDAR-IMU extrinsic 검증 (LI-Init) | yaml | P1 |
| F5 | LiDAR-카메라 extrinsic (전/후방 각각) | 4x4 행렬 | P0 |
| F6 | Jackal URDF에 센서 마운트 추가 | xacro | P0 |
| F7 | FAST-LIO2 통합 및 파라미터 튜닝 | config | P0 |
| F8 | apriltag_ros 노드 셋업 (전/후방 분리) | launch | P0 |
| F9 | **tag_mapper 노드 구현** | ROS 패키지 | P0 |
| F10 | 통합 launch (`sensor_stack.launch.py`) | launch | P1 |
| F11 | RViz 시각화 설정 | rviz config | P2 |

**tag_mapper 노드 핵심 로직**:
1. `/apriltag_front/detections`, `/apriltag_back/detections` 둘 다 구독
2. 각 detection에 대해 TF로 `map` frame 변환
3. 같은 tag_id 다중 검출 시 EMA 평활화
4. 마지막 검출 시각 기록
5. `/tags_in_map`으로 통합 발행 (10 Hz)

### 7.2 제어 로직 팀 — 주행·미션

| # | 업무 | 산출물 | 우선순위 |
|---|---|---|---|
| C1 | Jackal bringup, `/cmd_vel` 수동 조작 검증 | 검증 보고 | P0 |
| C2 | Mock 노드 (가짜 odom/tags 발행) | 패키지 | P0 |
| C3 | Gazebo 시뮬레이션 환경 구축 | launch | P0 |
| C4 | **reactive_explorer.py** 구현 및 튜닝 | 노드 | P0 |
| C5 | 영역 경계 파라미터 (선택, 일정 박스 내에서만 주행) | yaml | P2 |
| C6 | AprilTag 접근 제어기 (정밀 접근/도킹) | 노드 | P0 |
| C7 | **mission_node** (FSM 구현) | 패키지 | P0 |
| C8 | 안전 레이어 (e-stop, 속도 상한, LiDAR 근접 정지) | 노드 | P0 |
| C9 | 통합 launch (`control_stack.launch.py`) | launch | P1 |

**reactive_explorer 핵심**: 본 프로젝트 `reactive_explorer.py` 참조 (8장).

**mission_node FSM**:
```
IDLE 
  → 시작 신호 → SCANNING (reactive_explorer 활성)
SCANNING 
  → /tags_in_map에 목표 tag 검출 → TAG_FOUND
TAG_FOUND 
  → reactive_explorer 중지 → APPROACHING_TAG
APPROACHING_TAG 
  → 태그 도달 → DONE (또는 다음 태그로)
```

### 7.3 두 팀 공동 책임
- 본 문서 5장 인터페이스 명세 변경 시 git PR로 양 팀 동의 필요
- 좌표계 명명 규칙 변경 금지
- Mid-360 PTP 시간 동기화 검증 (mini PC와 동기)
- 매주 금요일 30분 합동 동기화 미팅
- 통합 테스트 시 디버깅 책임 매트릭스 합의

---

## 8. 자율 주행 동작: 반응형 회피 모드

### 8.1 전체 동작
- 기본 상태는 `FORWARD` — 일정 속도로 직진
- LiDAR 정면 섹터(±FOV) 내에 `obstacle_distance` 이내 점이 있으면 → `TURNING`
- `TURNING` 중에는 좌/우 섹터 중 더 멀리 트인 방향 선택해서 그쪽으로 회전
- 정면이 다시 비면 `FORWARD` 복귀
- `TURNING`이 일정 시간 이상 지속되면 → `BACKING` (살짝 후진 후 다시 회전 시도)
- 일정 시간 동안 거의 안 움직이면 → `STUCK` (안전 정지 + 알람)

### 8.2 상태 전이도
```
FORWARD ──정면 막힘──→ TURNING ──정면 비면──→ FORWARD
                         │
                   타임아웃(예: 5s)
                         ↓
                      BACKING ──→ TURNING ──→ ...

FORWARD ──장시간 정체──→ STUCK (수동 개입 대기)
```

### 8.3 핵심 파라미터

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `forward_speed` | 0.3 m/s | 직진 속도 |
| `turn_speed` | 0.4 rad/s | 회전 각속도 |
| `obstacle_distance` | 0.7 m | 정면 회피 트리거 거리 |
| `front_sector_deg` | 60 | 정면 감지 섹터(±half) |
| `side_sector_deg` | 90 | 좌/우 거리 비교 섹터 |
| `clear_distance` | 1.0 m | `TURNING → FORWARD` 복귀 거리 |
| `turn_timeout` | 5 s | 회전이 길어지면 BACKING 진입 |
| `stuck_window` | 5 s | 정체 판정 시간 창 |
| `stuck_min_travel` | 0.15 m | 그 시간 동안 최소 이동 |
| `use_bounds` | false | 사각형 경계 활성 (선택) |
| `bound_*` | — | 경계 박스 좌표 (`map` frame) |

### 8.4 FAST-LIO2 친화 설정
- 각속도 0.5 rad/s 이하 유지 (motion distortion 회피)
- 선속도 0.5 m/s 이하 권장
- 위 기본값은 이 범위 안에 있음

### 8.5 LiDAR 입력 처리
- `/livox/lidar` (`livox_frame`)을 직접 사용. base_link 변환은 정적 TF로 처리됨.
- 매 스캔에서 z축 ±0.2m 정도만 사용해 천장/바닥 노이즈 제거(파라미터화).
- xy 평면에 투영 후 정면/좌/우 섹터별 최근접 거리 계산.

---

## 9. 프로젝트 파일 구조

```
project_root/
├── claude.md                          # 본 문서
├── docs/
│   ├── interface_spec.md              # 5장 상세 버전
│   └── calibration_log.md             # 캘리브 기록
├── urdf/
│   └── jackal_custom.urdf.xacro       # 센서 마운트 포함
├── config/
│   ├── camera_front_info.yaml
│   ├── camera_back_info.yaml
│   ├── fastlio_mid360.yaml
│   ├── apriltag_config.yaml
│   └── explorer_params.yaml
├── launch/
│   ├── sensor_stack.launch.py         # 퓨전팀 통합
│   ├── control_stack.launch.py        # 제어팀 통합
│   └── full_stack.launch.py           # 전체 통합
├── custom_msgs/                       # ROS 2 패키지
│   └── msg/
│       ├── TagPose.msg
│       ├── TagPoseArray.msg
│       └── MissionState.msg
├── tag_mapper/                        # 퓨전팀 노드
│   └── tag_mapper_node.py
├── reactive_explorer/                 # 제어팀 노드
│   └── reactive_explorer.py          # 본 프로젝트 핵심
├── mission_node/                      # 제어팀 노드
│   └── mission_node.py
└── safety/                            # 제어팀 노드
    └── safety_layer.py
```

---

## 10. 주요 명령어 모음

### 빌드
```bash
cd ~/ws && colcon build --symlink-install
source install/setup.bash
```

### 단계별 실행 (디버깅용)
```bash
# 1. LiDAR만
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 2. SLAM 추가
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml

# 3. 카메라 + AprilTag
ros2 launch <pkg> cameras.launch.py
ros2 launch <pkg> apriltag.launch.py

# 4. tag_mapper
ros2 run tag_mapper tag_mapper_node

# 5. 자율 주행
python3 reactive_explorer.py
```

### 통합 실행
```bash
ros2 launch <pkg> full_stack.launch.py
```

### 모니터링
```bash
# 토픽 발행 빈도 확인
ros2 topic hz /livox/lidar
ros2 topic hz /Odometry
ros2 topic hz /tags_in_map

# TF tree 시각화
ros2 run tf2_tools view_frames

# 상태 모니터링
ros2 topic echo /explorer/state
ros2 topic echo /mission/state

# bag 녹화 (항상 권장)
ros2 bag record /livox/lidar /livox/imu /camera_front/color/image_raw \
                /camera_back/color/image_raw /Odometry /tags_in_map \
                /cmd_vel /tf /tf_static
```

---

## 11. 마일스톤

| 주차 | 목표 | 검증 기준 |
|---|---|---|
| W1 | 환경 세팅, 인터페이스 명세 확정, mock 노드 | 두 팀 mock으로 병행 개발 시작 |
| W2 | FAST-LIO2 단독 동작, AprilTag 검출 동작, reactive_explorer 시뮬 동작 | 각 노드 단독 검증 |
| W3 | 캘리브 완료, tag_mapper 동작, 실기 회피 주행 | end-to-end 부분 통합 |
| W4 | 전체 통합, 영역 스캔 + 태그 등록 | 도메인 1회 완주 |
| W5 | 안전 로직, 미션 시퀀스, 필드 테스트 | 시연 가능 수준 |

---

## 12. 알려진 함정 / 자주 막히는 곳

### 빌드/환경
- **Livox-SDK2와 livox_ros_driver2 버전 매칭** 필수. 둘 다 최신 main 권장.
- **PCL / Eigen 버전 충돌**로 FAST-LIO2 빌드 실패 흔함 → Docker 컨테이너 권장.
- ROS 2 Humble + Ubuntu 22.04 조합 고정. 다른 조합은 지원 불가 사항.

### Mid-360
- **네트워크 IP 설정**이 99% 함정. config 파일과 PC IP 대역 일치 필수.
- PTP 시간 동기화 안 하면 SLAM에서 점진적 발산. mini PC 시간 매번 확인.

### FAST-LIO2
- 처음에는 **공식 샘플 bag**으로 먼저 검증. 자기 데이터 바로 넣지 말 것.
- 맵이 튀면 IMU extrinsic부터 의심. `livox_imu_frame` 회전 방향 확인.
- 각속도 너무 빠르면 발산. `turn_speed` 키울 때 0.5 rad/s 이하 유지.

### 자율 주행
- **첫 테스트는 무조건 로봇 들어올리고**. 풀스피드로 벽 박는 사고 반복됨.
- `forward_speed`는 0.15부터 시작해서 점진적으로 올림.
- 측면 벽이 매끄러우면 LiDAR가 못 잡고 그대로 박을 수 있음 → 모서리에 마커.
- 반응형 회피만으로는 좁은 통로/막다른 골목에서 진동(좌→우→좌) 발생 가능.
  필요 시 BACKING 후 더 큰 각도 회전으로 탈출.

### AprilTag
- **태그 한 변 크기**가 config와 실제 인쇄물에서 다르면 거리가 다 틀어짐.
- 태그가 살짝 휘어 있으면 pose가 점프함. 평평한 판에 부착.
- 카메라 노출 자동조정이 너무 느리면 빠른 주행 시 검출 실패. 노출 고정 권장.

### 통신
- `/cmd_vel` 두 노드 동시 발행 시 로봇이 떨림. 5.3절 중재 규칙 준수.
- bag 녹화 시 카메라 raw image는 용량 큼. compressed 토픽으로 녹화 권장.

---

## 13. 변경 이력

| 날짜 | 변경 내용 | 작성자 |
|---|---|---|
| 2026-05-27 | 초기 작성 (Claude와 협업) | — |
| | depth 카메라 전/후방 듀얼로 확정 | — |
| | Nav2 미사용, 곡선 vacuum 방식 확정 | — |
| | tag_mapper, mission_node 구조 합의 | — |
| 2026-05-27 | vacuum 커버리지 모델 제거, 반응형 회피(FORWARD/TURNING/BACKING/STUCK)로 변경. 노드명 `vacuum_explorer_curved` → `reactive_explorer`. `/vacuum/*` 토픽 → `/explorer/*`. 영역 커버리지 추적 삭제. | — |
| 2026-05-27 | FSM 이산 회피 → **LiDAR clearance 기반 연속 곡선 회피**로 변경 (§0.1 참조). 컴퓨트 역할 분리 확정: SLAM/제어=Jackal mini PC, YOLO/AprilTag=Jetson. LiDAR 케이블 mini PC로 이동. 작업 장소 Jetson(Humble/ARM64) → Jackal mini PC(Jazzy/x86_64) 이전. | — |
| 2026-05-27 | mini PC에서 §5.1 검증. `/cmd_vel` 타입을 `Twist` → **`TwistStamped`**로 정정 (Jazzy + 최신 Clearpath). namespace `j100_0915` 확정, 외부 발행은 `twist_mux` `external` 슬롯으로 라우팅. 들어올린 상태에서 cmd_vel 응답 정상 확인. | — |
| 2026-05-27 저녁 | `reactive_explorer_node.py` 작성·빌드 통과 (jackal_mine_detection 패키지에 추가). 알고리즘은 §0.1의 연속 곡선 회피(tanh + clearance-based linear). LiDAR 드라이버 미설치(`livox_ros_driver2` 패키지 없음)로 알고리즘 검증은 보류. 부수 발견: PS 패드 ROS 입력 경로 깨진 상태, `emergency_stop`의 초기 true는 false positive였음 (§0.8 메모). 다음 세션 진입점은 §0.7. | — |
| 2026-05-27 밤 | Livox 드라이버 + FAST-LIO2 + reactive_explorer 풀 스택 검증 완료. enp86s0에 192.168.1.5/24 할당(NM unmanaged), MID360(192.168.1.182)와 통신 확인. FAST_LIO CMakeLists C++14→C++17 패치로 Jazzy 빌드 성공. Livox 드라이버 `xfer_format=1`(CustomMsg)로 확정 — reactive_explorer를 CustomMsg 입력으로 마이그레이션. 실모터 모드 검증(들어올린 상태)에서 연속 곡선 회피 정상 동작. 사용자 확정: 미션 흐름에 SCAN_DONE → CLUSTERING → APPROACH_CENTROID 단계 추가(§0.9). AprilTag 검출은 Jetson에서 YOLO와 함께 RealSense RGB로 진행(§0.1 컴퓨트 분리 재확인). 다음 진입점: Jetson 측 apriltag_ros + YOLO 셋업 / mini PC 측 tag_mapper 구현(작업 4). | — |
| 2026-06-04 | **§1·§3의 "Nav2 미사용" 결정 사실상 폐기** — 제어팀 인터페이스 합의(§0.10a). octomap_server 통합 + FAST-LIO2 `/path` remap(§0.10c,d). Cross-distro 발견: `apriltag_msgs/AprilTagDetection`의 `pose` 필드가 Humble→Jazzy에서 삭제 — Jetson↔mini PC 직접 통신 불가 확인(§0.10b). **TF 기반 tag 좌표 인터페이스로 합의** — mine_recorder, sim_apriltag 리라이트 필요. `mine_cluster_node` DBSCAN 검증 완료. Foxglove 시각화 경로 확립(§0.10e). | — |
| 2026-06-04 오후 | mine_recorder/sim_apriltag **TF 리라이트 풀 체인 검증 완료** (§0.10h). DDS 스냅샷 불일치(fastcdr/fastrtps 1월↔4월 혼재) 발견 → DDS 7개 패키지 현행 스냅샷으로 통일. 다음 진입점: Jetson 측 apriltag TF 발행 (frame 규약 `tag36h11:<id>`). | — |
| 2026-06-04 저녁 | Jetson 인식 파이프라인 핸드오프 수신(§0.11). **Jetson TF 미발행으로 전환** — §0.10h TF lookup 방식은 sim 전용 강등, 실기 tag pose는 detections 경유(cross-distro 이슈 미해결, §0.11b). mini PC에 `custom_msgs/TagCandidate` 신설·빌드·검증. DDS env 기본값 확인. chrony 미설치 확인. | — |
| 2026-06-04 저녁2 | **apriltag_msgs 정의 동일 확인** (양 distro 모두 pose 없음 — §0.10b 결론 정정). 태그 좌표는 **도킹 위치 기록 방식** 확정, tag_mapper(F9) 폐기(§0.11d-2). back 카메라 접근=**후진** 확정. `mission_node` 구현+시뮬 검증 완료, `reactive_explorer` §5.3 게이팅 추가(§0.11e). chrony 설치. | — |

---

## 14. 참고 자료

- ROS 2 Humble: https://docs.ros.org/en/humble/
- FAST-LIO2: https://github.com/hku-mars/FAST_LIO
- Livox ROS 2 Driver: https://github.com/Livox-SDK/livox_ros_driver2
- livox_camera_calib: https://github.com/hku-mars/livox_camera_calib
- apriltag_ros: https://github.com/christianrauch/apriltag_ros
- Clearpath Jackal: https://docs.clearpathrobotics.com/docs/robots/indoor_robots/jackal/
- REP-105 좌표계 규약: https://www.ros.org/reps/rep-0105.html
