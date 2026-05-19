#!/usr/bin/env python3
"""
depth_object_detector — Phase 1.1 (v2: 바닥 제외 + 객체 분리 개선)

변경점:
- 화면 하단 X% 영역은 마스크에서 제거 (바닥 추정)
- MORPH_CLOSE 제거 (객체들이 하나로 합쳐지는 거 방지)
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DepthObjectDetector(Node):
    def __init__(self):
        super().__init__('depth_object_detector')

        self.declare_parameter('camera_name', 'camera1')
        self.declare_parameter('min_distance_m', 0.3)
        self.declare_parameter('max_distance_m', 5.0)
        self.declare_parameter('min_area_px', 200)
        self.declare_parameter('warn_dist_m', 1.5)
        self.declare_parameter('danger_dist_m', 0.8)
        self.declare_parameter('morph_kernel', 3)
        self.declare_parameter('floor_crop_ratio', 0.55)

        camera = self.get_parameter('camera_name').value
        self.min_mm = int(self.get_parameter('min_distance_m').value * 1000)
        self.max_mm = int(self.get_parameter('max_distance_m').value * 1000)
        self.min_area = int(self.get_parameter('min_area_px').value)
        self.warn_dist = float(self.get_parameter('warn_dist_m').value)
        self.danger_dist = float(self.get_parameter('danger_dist_m').value)
        self.floor_crop = float(self.get_parameter('floor_crop_ratio').value)
        ks = int(self.get_parameter('morph_kernel').value)

        self.camera_name = camera
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ks, ks))
        self.bridge = CvBridge()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        color_topic = f'/camera/{camera}/color/image_raw'
        depth_topic = f'/camera/{camera}/depth/image_rect_raw'
        out_topic = f'/perception/{camera}/annotated_image'

        color_sub = message_filters.Subscriber(self, Image, color_topic, qos_profile=sensor_qos)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic, qos_profile=sensor_qos)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.callback)

        self.pub = self.create_publisher(Image, out_topic, 10)

        self.get_logger().info(
            f'\n[depth_object_detector / {camera}]\n'
            f'  range: {self.min_mm/1000:.1f} - {self.max_mm/1000:.1f} m\n'
            f'  min area: {self.min_area} px\n'
            f'  morph kernel: {ks}\n'
            f'  floor crop ratio: {self.floor_crop}  (하단 {int((1-self.floor_crop)*100)}% 영역 무시)'
        )

    def callback(self, color_msg, depth_msg):
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

            # 1. 거리 마스크
            mask = ((depth >= self.min_mm) & (depth <= self.max_mm)).astype(np.uint8) * 255

            # 2. 바닥 영역 제거 (하단 일정 비율 마스크 0으로)
            h = mask.shape[0]
            bottom_y = int(h * self.floor_crop)
            mask[bottom_y:, :] = 0

            # 3. 모폴로지 OPEN만 (노이즈 제거) — CLOSE 제거 (객체 분리 유지)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

            # color 해상도에 맞추기
            if mask.shape[:2] != color.shape[:2]:
                mask = cv2.resize(mask, (color.shape[1], color.shape[0]),
                                  interpolation=cv2.INTER_NEAREST)
                depth = cv2.resize(depth, (color.shape[1], color.shape[0]),
                                   interpolation=cv2.INTER_NEAREST)

            # 4. 연결 요소 분석
            num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

            annotated = color.copy()
            # 바닥 마스크된 영역은 시각적으로 어둡게 표시 (참고용)
            ch = color.shape[0]
            floor_y = int(ch * self.floor_crop)
            cv2.rectangle(annotated, (0, floor_y), (color.shape[1], ch),
                          (50, 50, 50), 1)
            cv2.putText(annotated, 'floor zone (ignored)', (5, floor_y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)

            obj_count = 0
            closest_dist = float('inf')

            for i in range(1, num):
                area = stats[i, cv2.CC_STAT_AREA]
                if area < self.min_area:
                    continue

                x = stats[i, cv2.CC_STAT_LEFT]
                y = stats[i, cv2.CC_STAT_TOP]
                w = stats[i, cv2.CC_STAT_WIDTH]
                hh = stats[i, cv2.CC_STAT_HEIGHT]

                valid_depths = depth[(labels == i) &
                                     (depth >= self.min_mm) &
                                     (depth <= self.max_mm)]
                if valid_depths.size == 0:
                    continue

                dist_m = float(np.median(valid_depths)) / 1000.0
                closest_dist = min(closest_dist, dist_m)
                obj_count += 1

                if dist_m < self.danger_dist:
                    bbox_color = (0, 0, 255)
                elif dist_m < self.warn_dist:
                    bbox_color = (0, 128, 255)
                else:
                    bbox_color = (0, 255, 0)

                cv2.rectangle(annotated, (x, y), (x + w, y + hh), bbox_color, 2)

                label = f'{dist_m:.2f}m'
                (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(annotated, (x, y - th - baseline - 6),
                              (x + tw + 6, y), bbox_color, -1)
                cv2.putText(annotated, label, (x + 3, y - baseline - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            summary = f'{self.camera_name}: {obj_count} obj'
            if obj_count > 0 and closest_dist != float('inf'):
                summary += f' | closest: {closest_dist:.2f}m'
            cv2.rectangle(annotated, (5, 5), (380, 35), (0, 0, 0), -1)
            cv2.putText(annotated, summary, (10, 27),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = color_msg.header
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'callback error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DepthObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
