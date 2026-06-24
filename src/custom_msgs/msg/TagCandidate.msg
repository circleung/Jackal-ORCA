# YOLO가 검출한 태그 후보 — Jetson → mini PC (mission_node 시각 서보잉용)
# claude.md §5 인터페이스 계약 참조.
#
# bearing_rad 는 base_link 기준 방위각 (전방 0, 좌측 +, REP-103).
# Jetson이 camera_info intrinsic(fx, cx)과 카메라 장착 yaw(front=0, back=pi)로
# 계산해서 보냄. mini PC는 TF 없이 이 값으로 바로 회전 제어 가능.
std_msgs/Header header        # stamp = 원본 이미지 stamp, frame_id = "base_link"
string source_camera          # "front" | "back"
float64 bearing_rad           # 태그 방향 (base_link yaw 기준)
float64 range_m               # 추정 거리 [m], 미지원 시 -1.0 (현재 RGB-only라 -1.0)
float64 confidence            # YOLO confidence 0.0 ~ 1.0
