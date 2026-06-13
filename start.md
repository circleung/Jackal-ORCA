# Jackal 자율탐사 미션 — 시작 가이드 (start.md)

> **미션**: 2D SLAM으로 미지 공간을 frontier 자율탐사하며 벽 AprilTag를 map 좌표에 누적 →
> 탐사 종료 후 태그 hotspot으로 이동. (Nav2 미사용, A*+Pure Pursuit 직접 구현)
> 자세한 구조는 `src/tag_hotspot_nav/ARCHITECTURE.md` 참고.

---

## 0단계 — 부팅 직후 점검

### (a) Clearpath 플랫폼 기동  ⚠ 수동 (systemd 아님)
모터·휠오돔·TF·조이스틱의 기반. **이게 먼저 떠 있어야** 모든 게 동작.
```bash
ros2 launch roas2_bringup platform.launch.py
```
확인:
```bash
ros2 node list | grep j100_0915      # 노드들 보이면 OK
```

### (b) 블루투스 — 자동 연결 (확인만)
부팅 시 systemd user 서비스가 **자동 연결** 시도 (15초마다). **기기를 켜기만** 하면 됨.
- 🔊 **JBL Go4 스피커**: 전원 켜면 자동 연결
- 🎮 **DualSense 패드**: PS버튼으로 켜면 자동 연결

확인:
```bash
systemctl --user status bt-speaker.service bt-dualsense.service   # active(running) 이면 OK
bluetoothctl info E8:26:CF:82:BE:88 | grep Connected   # 스피커
bluetoothctl info 50:EE:32:F7:62:28 | grep Connected   # 패드
```
> **패드가 처음이거나 연결 안 되면** (Bonded:no): 페어링 모드(PS+Create 5초, 라이트바 2번씩 깜빡) 후
> `~/colcon_ws/scripts/bt_dualsense_connect.sh` 1회 실행 → 이후엔 자동.
> 무선 안 되면 USB-C 직결이 백업.

---

## 1단계 — 탐사 스택 기동

**새 터미널**을 엽니다 (`.bashrc`가 ROS·워크스페이스·alias·FASTDDS 설정을 자동 적용).

```bash
start_explore                # 기본 (cliff 보호 on)
# 또는
start_explore --no-cliff     # 젯슨 cliff 오탐 회피 모드 (계단 보호 없음 → 주행 감시 필수)
```

`start_explore`가 자동으로 하는 것:
1. 플랫폼 떠있는지 확인
2. 옛 스택/좀비 정리 (플랫폼은 안 건드림)
3. `slam_2d` 기동 → `/scan` 수신 대기 → **slam_toolbox active 자동 확인/activate**
4. `perception` + `explore` 기동
5. 노드 등록 대기(13초) 후 목록 출력

`✅ 기동 완료` 메시지가 뜨면 다음 단계로.

---

## 2단계 — 탐사 실행

기동한 터미널(또는 source된 새 터미널)에서:

```bash
reset      # 처음부터 매핑 (맵·상태·태그·keep-out 전부 초기화) — '구독자 5개' 확인
go         # 탐사 시작 (로봇 움직임 — 출발방향 감시!)
pause      # 정지
go         # 재개 (resume 명령은 없음, go가 곧 재개)
save       # 맵(.pgm/.yaml) + posegraph + (있으면)정리맵 저장
```

### 동작
- 로봇이 frontier(미탐사 경계)를 찾아 자율주행하며 맵을 채움
- 동시에 젯슨 카메라가 벽 AprilTag 탐지 → map 좌표 누적 (`/tags_in_map`, `tag_observations.json`)
- **종료는 자동**: 갈 곳 없으면(`no_frontier_limit` 연속) 스스로 정지 + "탐사 완료!" 로그
- **저장은 수동**: 완료(또는 원하는 시점)에 `save` 입력

### 진행 로그 보기
```bash
tail -f /tmp/explore.log     # frontier 목표/도달/막힘
tail -f /tmp/slam_2d.log     # slam
```

---

## 3단계 — 결과 확인 (시각화)

별도 터미널에서 띄우고 **Foxglove**로 봅니다.
Foxglove 접속: `ws://<로봇IP>:8765` (플랫폼, 항상 켜짐) 또는 `:8766` (탐사 스택).

```bash
view_map           # 최근 저장 맵 → /map_saved  (또는 view_map <맵이름>)
view_tags          # 누적 태그 → /tag_markers (빨간 구 + #번호)
view_map --list    # 저장된 맵 목록
```

Foxglove **3D 패널**에 추가:
- `/map_saved` (OccupancyGrid) — 맵
- `/tag_markers` (MarkerArray) — 태그 위치
- `/slam_toolbox/graph_visualization` — (탐사 중) 라이브 포즈그래프

> ⚠ **태그와 맵이 안 맞으면**: ① 태그(`tag_observations.json`)와 맵이 **다른 세션** 것일 수 있음
> (각 SLAM 세션은 원점이 달라 좌표가 안 맞음 → 같은 세션 것끼리 봐야 함).
> ② 카메라 extrinsic이 placeholder라 **태그 좌표에 계통 오차** 있음 (정밀도는 실측 보정 필요).

---

## 종료

```bash
stop_explore       # 탐사 스택만 정지 (플랫폼 roas2_bringup 은 유지)
```
플랫폼까지 끄려면 platform.launch.py 터미널에서 Ctrl+C (보통 유지).

---

## 명령어 요약

| 명령 | 역할 |
|------|------|
| `start_explore [--no-cliff]` | 탐사 스택 일괄 기동 |
| `stop_explore` | 탐사 스택만 정지 (플랫폼 유지) |
| `reset` | 매핑 처음부터 (전부 초기화) |
| `go` | 시작 / 재개 |
| `pause` | 정지 |
| `save` | 맵 + posegraph 저장 |
| `view_map [이름]` | 저장 맵을 `/map_saved`로 발행 |
| `view_tags` | 누적 태그를 `/tag_markers`로 발행 |

저장 위치: 맵 `src/tag_hotspot_nav/maps/`, 태그 `tag_observations.json`

---

## 트러블슈팅

| 증상 | 원인 / 해결 |
|------|------------|
| **탐사를 안 함** (로봇 정지, 로그 조용) | slam_toolbox가 `inactive`에 멈춤 → `ros2 lifecycle get /slam_toolbox` 확인 → `ros2 lifecycle set /slam_toolbox activate`. (start_explore는 자동 가드 포함) |
| **`reset` 했는데 일부 노드 초기화 안 됨** | 디스커버리 타이밍 — "구독자 5개" 안 뜨면 한 번 더 `reset` |
| **같은 곳에서 막혀 안 움직임** | 전방 장애물(`/obstacle_block` true)에 정지. `pause` → 패드로 방향 틀거나 장애물 치우고 → `go` |
| **터미널 화면 깨짐** | `stty sane; clear`. ⚠ **`reset` 치지 말 것**(탐사 리셋임!). 터미널 리셋은 `/usr/bin/reset` |
| **`RTPS_TRANSPORT_SHM` 빨간 줄** | DDS 노이즈, 무해. `.bashrc`의 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`로 새 터미널/재기동 시 사라짐 |
| **`command not found` (go/start_explore 등)** | alias 없는 터미널 → 새 터미널 열거나 `source ~/.bashrc` |
| **계단 cliff 오탐으로 탐사 블록** | 젯슨 depth 오탐 → 임시로 `start_explore --no-cliff` (계단 보호 없음, 감시 필수) |

> ⚠ **절대 금지**: `pgrep/pkill -f`에 노드명을 평문으로 쓰면 자기 셸/플랫폼을 죽일 수 있음.
> 플랫폼(roas2_bringup)은 모든 것의 기반이라 절대 죽이지 말 것.

---

## 표준 시작 순서 (요약)

```bash
# 1. (플랫폼 터미널) — 보통 부팅 후 한 번
ros2 launch roas2_bringup platform.launch.py

# 2. (새 터미널) 스피커/패드 켜고
start_explore --no-cliff
reset
go

# 3. (또 다른 터미널) 보기
view_map
view_tags
#  → Foxglove ws://<IP>:8765 에서 /map_saved + /tag_markers

# 4. 끝나면
save
stop_explore
```
