std_msgs/Header header   # stamp = 원본 이미지 stamp, frame_id = "base_link"
string source_camera     # "front" | "back"
float64 bearing_rad      # base_link 기준 방위각 (전방 0, 좌 +, REP-103)
float64 range_m          # -1.0 (RGB-only, depth 도입 시 채움)
float64 confidence       # YOLO 0.0~1.0
