# map 프레임에 누적된 태그 1개의 추정 위치 (tag_collector 발행)
# apriltag corners → solvePnP → camera optical pose → tf2 로 map 변환 → id별 누적
std_msgs/Header header                # frame_id = "map"
int32 tag_id
geometry_msgs/PoseWithCovariance pose # map 기준 추정 pose (covariance: 누적 분산, 미사용 시 0)
float64 detection_confidence          # 0.0 ~ 1.0 (누적 신뢰도)
builtin_interfaces/Time last_seen     # 마지막 관측 시각
string source_camera                  # "front" | "back" (마지막 관측 카메라)
int32 observation_count               # 누적에 사용된 관측 수
