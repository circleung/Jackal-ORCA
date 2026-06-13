"""
clustering.py — 누적 태그를 DBSCAN 으로 군집화해 hotspot 중심을 내는 노드 (3단계).

입력은 tag_collector 가 내는 /tags_in_map(TagPoseArray):
  tag_id 하나당 이미 median 으로 정리된 map 좌표 한 점이 들어온다.
따라서 여기 DBSCAN 은 "한 태그의 원시관측을 뭉치는 것"이 아니라
"여러 태그의 (x,y) 위치를 밀도 기반으로 묶어 태그가 밀집한 hotspot 을 찾는 것"이다.
(ARCHITECTURE Phase 3: 태그가 밀집된 hotspot 중심으로 Phase 4 가 접근)

알고리즘:
  1. /tags_in_map 최신 스냅샷 유지 (tag_id → x,y,count)  ← 2Hz 갱신
  2. cluster_period 마다 (x,y) 점들에 DBSCAN(eps, min_samples) — sklearn 없어 numpy 직접구현
  3. 클러스터별 centroid = 멤버 태그 위치 평균 (weight_by_count 면 observation_count 가중)
  4. 멤버 많은 순(=밀집·신뢰 높은 순)으로 정렬해 /hotspots(PoseArray) 발행
     + /hotspot_markers(MarkerArray, 구+텍스트) 디버그 시각화

주의:
  - 우리 네비는 2D → (x,y)만 군집. z 는 무시.
  - min_samples 는 자기 자신 포함(sklearn 관례). min=2 → 이웃 1개 이상 필요, 단독 태그는 noise.
  - PoseArray 는 크기순 정렬이라 Phase 4 가 poses[0] 부터 접근하면 가장 밀집한 hotspot 우선.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose, Quaternion
from visualization_msgs.msg import Marker, MarkerArray

from custom_msgs.msg import TagPoseArray


def dbscan(points, eps, min_samples):
    """numpy 전용 DBSCAN. points (N,2) → labels (N,) (-1 = noise).

    태그 수가 수십 개 수준이라 O(N^2) 거리행렬로 충분(grid_utils 와 같은 직접구현 방침).
    min_samples 는 자기 자신 포함 카운트(sklearn 관례).
    """
    n = len(points)
    labels = np.full(n, -1, dtype=np.int64)
    if n == 0:
        return labels

    # 쌍거리 행렬 → eps 이내 이웃(자기 포함)
    diff = points[:, None, :] - points[None, :, :]
    dist = np.sqrt((diff ** 2).sum(axis=2))
    neighbors = [np.where(row <= eps)[0] for row in dist]

    visited = np.zeros(n, dtype=bool)
    cluster_id = 0
    for i in range(n):
        if visited[i]:
            continue
        visited[i] = True
        if len(neighbors[i]) < min_samples:
            continue  # core 아님 → 일단 noise (나중에 다른 클러스터의 경계로 흡수될 수 있음)
        # 새 클러스터 시드 확장
        labels[i] = cluster_id
        seeds = list(neighbors[i])
        k = 0
        while k < len(seeds):
            j = seeds[k]
            k += 1
            if not visited[j]:
                visited[j] = True
                if len(neighbors[j]) >= min_samples:   # j 도 core → 확장
                    seeds.extend(neighbors[j])
            if labels[j] == -1:                        # noise/미할당 → 경계점으로 흡수
                labels[j] = cluster_id
        cluster_id += 1
    return labels


class ClusteringNode(Node):

    def __init__(self):
        super().__init__('clustering')

        self.declare_parameter('eps', 2.0)              # [m] 같은 hotspot 으로 묶을 태그 간 거리
        self.declare_parameter('min_samples', 2)        # hotspot 성립 최소 태그 수(자기 포함)
        self.declare_parameter('cluster_period', 3.0)   # [s] 주기적 재계산
        self.declare_parameter('weight_by_count', True) # centroid 를 observation_count 가중평균
        self.declare_parameter('map_frame', 'map')

        self.eps = float(self.get_parameter('eps').value)
        self.min_samples = int(self.get_parameter('min_samples').value)
        self.weight_by_count = bool(self.get_parameter('weight_by_count').value)
        self.map_frame = self.get_parameter('map_frame').value
        period = float(self.get_parameter('cluster_period').value)

        # tag_id → (x, y, count)  — 최신 스냅샷만 유지
        self.snapshot = {}

        self.create_subscription(TagPoseArray, '/tags_in_map', self.tags_cb, 10)

        self.hotspots_pub = self.create_publisher(PoseArray, '/hotspots', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/hotspot_markers', 10)

        self.create_timer(period, self.cluster)

        self.get_logger().info(
            f'clustering 시작 — eps={self.eps}m, min_samples={self.min_samples}, '
            f'period={period}s, weight_by_count={self.weight_by_count}')

    def tags_cb(self, msg: TagPoseArray):
        # 최신 상태로 통째 갱신(태그 삭제/리셋도 자연 반영)
        snap = {}
        for tp in msg.tags:
            p = tp.pose.pose.position
            snap[tp.tag_id] = (p.x, p.y, max(1, tp.observation_count))
        self.snapshot = snap

    def cluster(self):
        snap = self.snapshot
        if not snap:
            return
        tag_ids = list(snap.keys())
        pts = np.array([[snap[t][0], snap[t][1]] for t in tag_ids], dtype=np.float64)
        counts = np.array([snap[t][2] for t in tag_ids], dtype=np.float64)

        labels = dbscan(pts, self.eps, self.min_samples)

        # 클러스터별 centroid 집계
        hotspots = []  # (centroid_x, centroid_y, n_tags, member_tag_ids)
        for cid in sorted(set(labels) - {-1}):
            mask = labels == cid
            members = pts[mask]
            if self.weight_by_count:
                w = counts[mask]
                centroid = np.average(members, axis=0, weights=w)
            else:
                centroid = members.mean(axis=0)
            ids = [tag_ids[k] for k in np.where(mask)[0]]
            hotspots.append((centroid[0], centroid[1], int(mask.sum()), ids))

        # 멤버 많은 순(밀집·신뢰 높은 순) → Phase 4 가 poses[0] 부터 접근
        hotspots.sort(key=lambda h: h[2], reverse=True)

        self.publish_hotspots(hotspots)
        self.publish_markers(hotspots)

        n_noise = int((labels == -1).sum())
        self.get_logger().info(
            f'태그 {len(tag_ids)}개 → hotspot {len(hotspots)}개 (noise {n_noise}개) '
            + ' '.join(f'#{i}({h[2]}태그@{h[0]:.2f},{h[1]:.2f})'
                       for i, h in enumerate(hotspots)),
            throttle_duration_sec=5.0)

    def publish_hotspots(self, hotspots):
        arr = PoseArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = self.map_frame
        for cx, cy, _n, _ids in hotspots:
            pose = Pose()
            pose.position.x = float(cx)
            pose.position.y = float(cy)
            pose.orientation = Quaternion(w=1.0)   # 자세 미사용(2D)
            arr.poses.append(pose)
        self.hotspots_pub.publish(arr)

    def publish_markers(self, hotspots):
        ma = MarkerArray()
        # 이전 마커 잔상 제거
        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.ns = 'hotspots'
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        for i, (cx, cy, n, _ids) in enumerate(hotspots):
            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = stamp
            sphere.ns = 'hotspots'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(cx)
            sphere.pose.position.y = float(cy)
            sphere.pose.orientation = Quaternion(w=1.0)
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.3
            sphere.color.r = 1.0
            sphere.color.g = 0.4
            sphere.color.a = 0.9
            ma.markers.append(sphere)

            label = Marker()
            label.header.frame_id = self.map_frame
            label.header.stamp = stamp
            label.ns = 'hotspots'
            label.id = 1000 + i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(cx)
            label.pose.position.y = float(cy)
            label.pose.position.z = 0.4
            label.pose.orientation = Quaternion(w=1.0)
            label.scale.z = 0.25
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'#{i}: {n} tags'
            ma.markers.append(label)
        self.markers_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = ClusteringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
