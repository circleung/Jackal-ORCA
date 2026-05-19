#!/usr/bin/env python3
"""
lidar_obstacle_detector — Phase 1.2

Livox Mid-360 점군 → 필터링 → 다운샘플링 → DBSCAN 클러스터링
출력:
  - /perception/lidar/clusters_filtered (PointCloud2) : 필터된 점군
  - /perception/lidar/clusters_markers (MarkerArray)  : 클러스터별 3D bbox + 거리 라벨
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from sklearn.cluster import DBSCAN


class LidarObstacleDetector(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector')

        self.declare_parameter('range_min_m', 0.3)
        self.declare_parameter('range_max_m', 10.0)
        self.declare_parameter('height_min_m', 0.1)
        self.declare_parameter('height_max_m', 2.0)
        self.declare_parameter('voxel_size_m', 0.05)
        self.declare_parameter('dbscan_eps_m', 0.30)
        self.declare_parameter('dbscan_min_samples', 10)
        self.declare_parameter('min_cluster_points', 30)
        self.declare_parameter('input_topic', '/livox/lidar')

        self.range_min = float(self.get_parameter('range_min_m').value)
        self.range_max = float(self.get_parameter('range_max_m').value)
        self.height_min = float(self.get_parameter('height_min_m').value)
        self.height_max = float(self.get_parameter('height_max_m').value)
        self.voxel_size = float(self.get_parameter('voxel_size_m').value)
        self.dbscan_eps = float(self.get_parameter('dbscan_eps_m').value)
        self.dbscan_min = int(self.get_parameter('dbscan_min_samples').value)
        self.min_cluster_pts = int(self.get_parameter('min_cluster_points').value)
        input_topic = self.get_parameter('input_topic').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.callback, sensor_qos)

        self.pub_cloud = self.create_publisher(
            PointCloud2, '/perception/lidar/clusters_filtered', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/perception/lidar/clusters_markers', 10)

        self._frame_count = 0
        self._last_n_in = 0
        self._last_n_filtered = 0
        self._last_n_clusters = 0
        self.create_timer(2.0, self.stats_log)

        self.get_logger().info(
            f'\n[lidar_obstacle_detector]\n'
            f'  input:  {input_topic}\n'
            f'  range:  {self.range_min:.1f} ~ {self.range_max:.1f} m\n'
            f'  height: {self.height_min:.1f} ~ {self.height_max:.1f} m\n'
            f'  voxel:  {self.voxel_size*100:.0f} cm\n'
            f'  DBSCAN: eps={self.dbscan_eps} m, min={self.dbscan_min}'
        )

    def stats_log(self):
        if self._frame_count == 0:
            self.get_logger().warn('LiDAR 메시지 미수신 (구독 대기 중)')
            return
        self.get_logger().info(
            f'[stats] in={self._last_n_in:6d}  '
            f'filtered={self._last_n_filtered:5d}  '
            f'clusters={self._last_n_clusters}'
        )

    def callback(self, msg: PointCloud2):
        self._frame_count += 1

        points_iter = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts_raw = np.array(list(points_iter))
        if pts_raw.size == 0:
            return
        pts = np.column_stack([pts_raw['x'], pts_raw['y'], pts_raw['z']]) \
            if pts_raw.dtype.names else pts_raw.reshape(-1, 3)
        pts = pts.astype(np.float32)
        n_in = pts.shape[0]

        d_xy = np.hypot(pts[:, 0], pts[:, 1])
        mask = (d_xy >= self.range_min) & (d_xy <= self.range_max)
        mask &= (pts[:, 2] >= self.height_min) & (pts[:, 2] <= self.height_max)
        pts_f = pts[mask]

        if pts_f.shape[0] < self.dbscan_min:
            self._last_n_in = n_in
            self._last_n_filtered = pts_f.shape[0]
            self._last_n_clusters = 0
            return

        keys = np.floor(pts_f / self.voxel_size).astype(np.int32)
        _, idx = np.unique(keys, axis=0, return_index=True)
        pts_v = pts_f[idx]

        db = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min, n_jobs=-1).fit(pts_v)
        labels = db.labels_
        unique_labels = [l for l in set(labels) if l != -1]

        self._publish_filtered_cloud(msg.header, pts_v)
        self._publish_markers(msg.header, pts_v, labels, unique_labels)

        self._last_n_in = n_in
        self._last_n_filtered = pts_v.shape[0]
        self._last_n_clusters = len(unique_labels)

    def _publish_filtered_cloud(self, header, pts):
        cloud = pc2.create_cloud_xyz32(header, pts.tolist())
        self.pub_cloud.publish(cloud)

    def _publish_markers(self, header, pts, labels, unique_labels):
        marker_array = MarkerArray()
        palette = [
            (0.95, 0.30, 0.30), (0.30, 0.85, 0.40), (0.30, 0.55, 0.95),
            (0.95, 0.65, 0.20), (0.75, 0.40, 0.90), (0.20, 0.85, 0.85),
        ]

        del_marker = Marker()
        del_marker.header = header
        del_marker.action = Marker.DELETEALL
        marker_array.markers.append(del_marker)

        for i, label in enumerate(unique_labels):
            cluster = pts[labels == label]
            if cluster.shape[0] < self.min_cluster_pts:
                continue

            mn = cluster.min(axis=0)
            mx = cluster.max(axis=0)
            center = (mn + mx) / 2.0
            size = mx - mn
            distance = float(np.hypot(center[0], center[1]))

            color = palette[i % len(palette)]

            m = Marker()
            m.header = header
            m.ns = 'cluster_bbox'
            m.id = int(label)
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(center[0])
            m.pose.position.y = float(center[1])
            m.pose.position.z = float(center[2])
            m.pose.orientation.w = 1.0
            m.scale.x = max(float(size[0]), 0.05)
            m.scale.y = max(float(size[1]), 0.05)
            m.scale.z = max(float(size[2]), 0.05)
            m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.35)
            m.lifetime.sec = 0
            m.lifetime.nanosec = 200_000_000
            marker_array.markers.append(m)

            t = Marker()
            t.header = header
            t.ns = 'cluster_label'
            t.id = int(label)
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(center[0])
            t.pose.position.y = float(center[1])
            t.pose.position.z = float(mx[2] + 0.15)
            t.pose.orientation.w = 1.0
            t.scale.z = 0.20
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text = f'{distance:.2f}m'
            t.lifetime.sec = 0
            t.lifetime.nanosec = 200_000_000
            marker_array.markers.append(t)

        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
