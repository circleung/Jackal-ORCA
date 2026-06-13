#!/usr/bin/env python3
"""view_tags.py — 누적 태그 위치를 맵 위에 마커로 발행 (Foxglove/RViz 용).

tag_observations.json 의 태그별 estimate_median(map 좌표)을 읽어
/tag_markers (visualization_msgs/MarkerArray) 로 1Hz 발행한다.
각 태그 = 구(SPHERE) + "#id" 텍스트 라벨. frame_id='map'.

사용:
  python3 ~/colcon_ws/scripts/view_tags.py            # 기본 JSON
  python3 ~/colcon_ws/scripts/view_tags.py <json경로>
Foxglove/RViz 에서  /map_saved(맵) + /tag_markers(태그)  같이 보면 '태그 찍힌 맵'.
종료: Ctrl+C
"""
import json
import os
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

DEFAULT_JSON = os.path.expanduser('~/colcon_ws/tag_observations.json')


class TagMarkers(Node):
    def __init__(self, path):
        super().__init__('view_tags')
        self.path = path
        self.pub = self.create_publisher(MarkerArray, '/tag_markers', 1)
        self.frame = 'map'
        self._load()
        self.create_timer(1.0, self._publish)   # 주기 발행 (늦게 붙는 뷰어도 받게)
        self.get_logger().info(
            f'{len(self.tags)}개 태그 → /tag_markers (frame={self.frame})')

    def _load(self):
        try:
            d = json.load(open(self.path))
        except Exception as e:
            self.get_logger().error(f'JSON 로드 실패 {self.path}: {e}')
            self.tags = {}
            return
        self.frame = d.get('map_frame', 'map')
        self.tags = d.get('tags', {})
        for tid, t in self.tags.items():
            xyz = t.get('estimate_median') or [0, 0, 0]
            self.get_logger().info(
                f'  #{tid}: map=({xyz[0]:.2f},{xyz[1]:.2f},{xyz[2]:.2f}) '
                f'관측 {t.get("count", "?")}회')

    def _publish(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for tid, t in self.tags.items():
            xyz = t.get('estimate_median') or [0, 0, 0]
            x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
            i = int(tid)

            sph = Marker()
            sph.header.frame_id = self.frame
            sph.header.stamp = now
            sph.ns = 'tags'
            sph.id = i
            sph.type = Marker.SPHERE
            sph.action = Marker.ADD
            sph.pose.position = Point(x=x, y=y, z=z)
            sph.pose.orientation.w = 1.0
            sph.scale.x = sph.scale.y = sph.scale.z = 0.3
            sph.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.9)
            arr.markers.append(sph)

            txt = Marker()
            txt.header.frame_id = self.frame
            txt.header.stamp = now
            txt.ns = 'tag_labels'
            txt.id = i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position = Point(x=x, y=y, z=z + 0.35)
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.35
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = f'#{tid}'
            arr.markers.append(txt)
        self.pub.publish(arr)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_JSON
    rclpy.init()
    node = TagMarkers(path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
