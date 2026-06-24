#!/usr/bin/env python3
import cv2
import numpy as np
from pupil_apriltags import Detector

img = cv2.imread('/home/minseok/jackal-j100-lidar-setup/tags/tag36h11_id0.png', cv2.IMREAD_GRAYSCALE)
if img is None:
    raise RuntimeError("tag image not found")

# 너무 작아서 검출이 안 되므로 nearest-neighbor로 크게 키움
scale = 40
img_big = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)

# 바깥 흰 여백 추가
canvas = np.full((img_big.shape[0] + 200, img_big.shape[1] + 200), 255, dtype=np.uint8)
canvas[100:100 + img_big.shape[0], 100:100 + img_big.shape[1]] = img_big

detector = Detector(
    families='tag36h11',
    nthreads=2,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

detections = detector.detect(canvas, estimate_tag_pose=False)
print("detections:", len(detections))

vis = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)

for det in detections:
    tag_id = int(det.tag_id)
    corners = det.corners.astype(int)
    center = tuple(det.center.astype(int))

    for i in range(4):
        p1 = tuple(corners[i])
        p2 = tuple(corners[(i + 1) % 4])
        cv2.line(vis, p1, p2, (0, 255, 0), 2)

    cv2.circle(vis, center, 4, (0, 0, 255), -1)
    cv2.putText(
        vis,
        f'ID {tag_id}',
        (corners[0][0], corners[0][1] - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2
    )

cv2.imshow("AprilTag File Test", vis)
cv2.waitKey(0)
cv2.destroyAllWindows()
