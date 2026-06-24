# YOLO AprilTag 인식 발행 토픽 정리

> 노드: `jackal_orca_perception/scripts/tag_yolo_detector_node.py`
> RealSense 앞/뒤 카메라 → YOLO로 AprilTag 탐지 → 방향(bearing) + confidence 발행

---

## 0. 컴퓨트 분담 (중요)

이 노드는 **Jetson**에서 돌고, 결과를 받아 쓰는 `mission_node`는 **Jackal mini PC**에서 돈다.
Jetson과 mini PC는 **물리적으로 분리된 별개 머신**이며, 토픽은 네트워크(DDS) 너머로 흐른다.

| 머신 | 역할 | 관련 노드 |
|------|------|-----------|
| **Jetson** (ARM64, Humble) | GPU 추론만 — 검출 결과를 토픽으로 **전송만** | `tag_yolo_detector_node`, `apriltag_ros`, RealSense 드라이버 |
| **Jackal mini PC** (x86_64, Jazzy) | SLAM + 제어 + 미션 | `mission_node`(FSM), `reactive_explorer`, `FAST-LIO2`, `safety_layer` |

```
Jetson                                  Jackal mini PC
  tag_yolo_detector_node ──/yolo/tag_candidate──▶ mission_node (시각 서보잉)
         (YOLO 추론)         (DDS 네트워크)        TAG_FOUND → APPROACHING_TAG
```

> `mission_node`는 **Jetson이 아니라 Jackal mini PC** 쪽이다. (근거: `claude.md` §0.1, §0.9)
> Jetson은 TF를 전혀 쓰지 않는다 — bearing은 camera_info intrinsic + 장착 yaw만으로 계산.

---

## 1. 핵심 토픽: `/yolo/tag_candidate`

서보잉 피드백 스트림. mini PC `mission_node`가 받아 `TAG_FOUND → APPROACHING_TAG` 시각 서보잉 수행.

- **메시지 타입:** `custom_msgs/TagCandidate`
- **정의 파일:** `src/custom_msgs/msg/TagCandidate.msg`
- **발행 조건:** `max_conf >= candidate_threshold(0.30)` **그리고** camera_info 수신 완료
- **발행 코드:** `tag_yolo_detector_node.py:213-220`

| 필드 | 타입 | 담긴 정보 | 비고 |
|------|------|-----------|------|
| `header.stamp` | time | **원본 이미지 stamp 그대로** 유지 | 지연 보정용 |
| `header.frame_id` | string | 항상 `"base_link"` | |
| `source_camera` | string | `"front"` / `"back"` | 어느 카메라가 탐지했나 |
| `bearing_rad` | float64 | **태그 방향각** (base_link yaw 기준, [-π, π) 정규화) | 서보잉 핵심 값 |
| `range_m` | float64 | 추정 거리 [m] — **현재 항상 -1.0** | RGB-only, depth 미도입 |
| `confidence` | float64 | YOLO 최대 confidence (0.0 ~ 1.0) | |

### bearing 계산 (`tag_yolo_detector_node.py:206-211`)

Jetson은 TF 없이 camera_info intrinsic + 장착 yaw 파라미터만 사용:

```python
u = (x1 + x2) / 2.0                          # bbox 중심 가로 픽셀
bearing = mount_yaw + atan2(cx - u, fx)      # cx, fx ← camera_info
bearing = atan2(sin(bearing), cos(bearing))  # [-pi, pi) 정규화
```

- `front` 카메라 `mount_yaw = 0.0`, `back` 카메라 `mount_yaw = π` (`:64-65`)
- 영상 오른쪽일수록 로봇 기준 오른쪽(−yaw)
- ⚠️ **camera_info 수신 전에는 발행 안 함** — intrinsic 없이 bearing 추정 금지 (`:200-204`)

---

## 2. 부가 발행 토픽 (디버그 / 이벤트)

| 토픽 | 타입 | 담긴 정보 | 코드 |
|------|------|-----------|------|
| `/tag_confidence` | `std_msgs/Float32` | 매 프레임 YOLO 최대 confidence (없으면 0.0) | `:188` |
| `/tag_detected` | `std_msgs/Bool` | `max_conf >= 0.55(confidence_threshold)`일 때 `True` 이벤트 | `:191-192` |
| `/yolo/debug_image_front` | `sensor_msgs/Image` | bbox 오버레이 영상 (녹색=확정 / 노랑=후보 / 회색=미달) | `:117, 184` |
| `/yolo/debug_image_back` | `sensor_msgs/Image` | 동일 (back). **구독자 있을 때만** 그리고 발행(평시 부하 0) | `:118, 184` |

---

## 2.5. QoS 설정

### `/yolo/tag_candidate` 발행 QoS — 명시 설정 없음 → ROS 2 **기본값**

발행 코드 (`tag_yolo_detector_node.py:112`):

```python
self.pub_candidate = self.create_publisher(TagCandidate, '/yolo/tag_candidate', 10)
```

세 번째 인자가 `QoSProfile`이 아니라 **정수 `10`** → history depth만 지정, 나머지는 기본 프로파일:

| 항목 | 값 |
|------|-----|
| **Reliability** | `RELIABLE` |
| **Durability** | `VOLATILE` |
| **History** | `KEEP_LAST` |
| **Depth** | `10` |

이 노드의 다른 발행(`/tag_confidence`, `/tag_detected`, debug image)도 전부 동일하게
depth만 지정 → **RELIABLE / VOLATILE** 기본값.

### 프로젝트 전반 QoS 패턴

- 노드들은 대부분 기본 QoS(depth 10, **RELIABLE / VOLATILE**)를 사용한다. RealSense
  센서 스트림을 직접 구독할 때만 `qos_profile_sensor_data`(= **BEST_EFFORT**, KEEP_LAST(5))를
  쓰는 게 안전하다 (센서 스트림이 보통 BEST_EFFORT로 발행되기 때문).
- `claude.md`에 토픽별 명시적 QoS 규약은 문서화돼 있지 않음 (rate만 `~10 Hz`로 기재).

### 구독 측(mini PC `mission_node`) 주의사항

> ⚠️ `mission_node`가 사는 mini PC 워크스페이스(`~/colcon_ws/`)는 **이 Jetson 머신에 없음** →
> 구독 측 실제 QoS 코드는 여기서 확인 불가. 단, 발행 측이 RELIABLE이므로 호환 규칙은 명확.

- `/yolo/tag_candidate`는 **RELIABLE** 발행 → mini PC 구독자도 **RELIABLE**(또는 호환 설정)으로 받아야 함.
- DDS QoS 매칭 규칙:
  - RELIABLE 발행 ↔ RELIABLE 구독 → ✅ 정상
  - RELIABLE 발행 ↔ BEST_EFFORT 구독 → ✅ 매칭됨 (지금은 받힘)
  - **BEST_EFFORT 발행 ↔ RELIABLE 구독 → ❌ 매칭 실패 (메시지 안 들어옴)**
- **권장:** 서보잉 피드백 스트림은 양쪽 모두 **RELIABLE / depth 10** 유지가 가장 안전·일관적.

---

## 3. 두 단계 임계값(threshold) 구조

| 파라미터 | 기본값 | 역할 |
|----------|--------|------|
| `candidate_threshold` | 0.30 | `/yolo/tag_candidate` 발행 — 서보잉 피드백은 낮은 신뢰도도 흘림 |
| `confidence_threshold` | 0.55 | `/tag_detected` 확정 이벤트 |

---

## 4. 구독 토픽 (입력)

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/camera_front/color/image_raw` | `sensor_msgs/Image` | front 추론 입력 |
| `/camera_back/color/image_raw` | `sensor_msgs/Image` | back 추론 입력 |
| `/camera_front/color/camera_info` | `sensor_msgs/CameraInfo` | front intrinsic (fx, cx) — bearing 계산용 |
| `/camera_back/color/camera_info` | `sensor_msgs/CameraInfo` | back intrinsic (fx, cx) |

---

## 5. (참고) 별도 노드 — `apriltag_ros` 정식 경로

`tag_recorder_node.py`는 YOLO가 아니라 **`apriltag_ros` 정식 탐지**(`/apriltag_front|back/detections`)를 구독한다.
YOLO 경로와 혼동하지 말 것.

| 토픽 | 타입 | 담긴 정보 |
|------|------|-----------|
| `/recorded_tag_positions` | `geometry_msgs/PoseArray` | 누적된 태그의 map 좌표 |
| `/tag_saved_event` | `std_msgs/UInt32` | 저장된 tag_id |

---

## 요약

YOLO AprilTag 인식의 핵심 출력은 **`/yolo/tag_candidate`** 이며,
**"어느 카메라가(`source_camera`) / 어느 방향(`bearing_rad`)에서 / 얼마의 신뢰도(`confidence`)로"**
태그를 봤는지를 담아 mini PC `mission_node`의 시각 서보잉 피드백으로 흘려보낸다.
거리(`range_m`)는 아직 미지원(-1.0). 이 토픽을 받는 `mission_node`는 **Jetson이 아니라 Jackal mini PC**에 있다.
