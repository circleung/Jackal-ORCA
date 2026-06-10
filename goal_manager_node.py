#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
goal_manager_node.py  (최적화판: 잡음 지도 강건화)

기존 대비 변경점:
  1) 지도 디노이즈      - 점유 격자에서 연결 크기가 작은 덩어리(소금-후추 잡음)를
                         자유공간으로 되돌린 뒤 프론티어를 계산.
  2) 프론티어 검증 강화 - 미탐색 이웃이 unknown_neighbor_min 개 이상 + 벽/장애물에서
                         clearance_cells 칸 이상 떨어진 셀만 프론티어로 인정.
  3) 정보 이득 기반 선정 - '가장 가까운'이 아니라 '넓으면서 가까운'(size/(dist+1)) 선택.
  4) 목표 hysteresis    - 도달/timeout 전까지 목표 유지(잡음으로 매 프레임 흔들리지 않게).

[토픽]
  구독 /map(OccupancyGrid), /tag_detected(Bool), /tag_pose(PoseStamped)
  발행 /goal_pose(PoseStamped)
  TF   map -> base_link
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import tf2_ros
from tf2_ros import TransformException


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class GoalManagerNode(Node):

    def __init__(self):
        super().__init__('goal_manager_node')

        # ---------------- 파라미터 ----------------
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('tag_detected_topic', '/tag_detected')
        self.declare_parameter('tag_pose_topic', '/tag_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')

        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('goal_tolerance', 0.35)
        self.declare_parameter('goal_timeout', 30.0)
        self.declare_parameter('planning_period', 1.5)
        self.declare_parameter('blacklist_radius', 0.6)

        # --- 잡음 강건화 파라미터 ---
        self.declare_parameter('min_occupied_blob', 4)     # 점유 덩어리가 이보다 작으면 잡음 처리
        self.declare_parameter('unknown_neighbor_min', 3)  # 미탐색 이웃이 이 개수 이상이어야 프론티어
        self.declare_parameter('clearance_cells', 2)       # 벽/장애물에서 떨어뜨릴 칸 수
        self.declare_parameter('min_frontier_size', 8)     # 프론티어 군집 최소 셀 수(상향)

        g = lambda n: self.get_parameter(n).value
        self.map_topic = g('map_topic')
        self.goal_topic = g('goal_topic')
        self.tag_detected_topic = g('tag_detected_topic')
        self.tag_pose_topic = g('tag_pose_topic')
        self.map_frame = g('map_frame')
        self.robot_base_frame = g('robot_base_frame')
        self.occ_thresh = int(g('occupied_threshold'))
        self.goal_tolerance = float(g('goal_tolerance'))
        self.goal_timeout = float(g('goal_timeout'))
        self.planning_period = float(g('planning_period'))
        self.blacklist_radius = float(g('blacklist_radius'))
        self.min_occupied_blob = int(g('min_occupied_blob'))
        self.unknown_neighbor_min = int(g('unknown_neighbor_min'))
        self.clearance_cells = int(g('clearance_cells'))
        self.min_frontier_size = int(g('min_frontier_size'))

        # ---------------- 상태 ----------------
        self.map_msg = None
        self.state = 'EXPLORING'
        self.current_goal = None
        self.goal_start_time = None
        self.blacklist = []
        self.tag_pose = None
        self._prev_tag = False
        self._tag_goal_published = False

        # ---------------- TF ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- 통신 ----------------
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.history = QoSHistoryPolicy.KEEP_LAST

        self.create_subscription(OccupancyGrid, self.map_topic, self.map_cb, map_qos)
        self.create_subscription(Bool, self.tag_detected_topic, self.tag_cb, 10)
        self.create_subscription(PoseStamped, self.tag_pose_topic, self.tag_pose_cb, 10)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)

        self.timer = self.create_timer(self.planning_period, self.on_timer)
        self.get_logger().info("goal_manager(최적화판) 시작 — 잡음 강건 프론티어 탐색")

    # ======================================================================
    #  콜백
    # ======================================================================
    def map_cb(self, msg):
        self.map_msg = msg

    def tag_cb(self, msg):
        if msg.data and not self._prev_tag and self.state != 'TAG_APPROACH':
            self.get_logger().info("태그 감지! 탐색 중단 -> 태그 접근 모드")
            self.state = 'TAG_APPROACH'
            self._tag_goal_published = False
        self._prev_tag = msg.data

    def tag_pose_cb(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, msg.header.frame_id, rclpy.time.Time())
                p = PoseStamped()
                p.header.frame_id = self.map_frame
                p.pose.position.x = msg.pose.position.x + tf.transform.translation.x
                p.pose.position.y = msg.pose.position.y + tf.transform.translation.y
                p.pose.orientation = msg.pose.orientation
                self.tag_pose = p
                return
            except TransformException as e:
                self.get_logger().warn(f"tag_pose TF 변환 실패: {e}")
                return
        self.tag_pose = msg

    # ======================================================================
    #  상태 머신
    # ======================================================================
    def on_timer(self):
        if self.state == 'DONE':
            return
        if self.map_msg is None:
            self.get_logger().warn("아직 /map 미수신", throttle_duration_sec=5.0)
            return

        if self.state == 'TAG_APPROACH':
            self.handle_tag_approach()
            return

        robot = self.get_robot_xy()
        if robot is None:
            self.get_logger().warn("로봇 위치(TF) 미확보", throttle_duration_sec=5.0)
            return

        # 현재 목표 도달/지연 판정 (hysteresis)
        if self.current_goal is not None:
            d = math.dist(robot, self.current_goal)
            if d < self.goal_tolerance:
                self.get_logger().info("목표 도달 -> 다음 프론티어")
                self.current_goal = None
            elif self._elapsed(self.goal_start_time) > self.goal_timeout:
                self.get_logger().warn("목표 지연 -> 블랙리스트 후 재선정")
                self.blacklist.append(self.current_goal)
                self.current_goal = None
            else:
                return

        frontiers = self.find_frontiers()
        best = self.select_frontier(frontiers, robot)
        if best is None:
            self.get_logger().info("유효 프론티어 없음 -> 탐색 완료(DONE)")
            self.state = 'DONE'
            return

        self.current_goal = best
        self.goal_start_time = self.get_clock().now()
        self.publish_goal(best[0], best[1], robot)
        self.get_logger().info(f"새 목표: ({best[0]:.2f}, {best[1]:.2f})")

    def handle_tag_approach(self):
        if self._tag_goal_published:
            return
        if self.tag_pose is not None:
            self.goal_pub.publish(self.tag_pose)
            self._tag_goal_published = True
            self.get_logger().info("태그 좌표로 목표 발행(접근)")
        else:
            robot = self.get_robot_xy()
            if robot is not None:
                self.publish_goal(robot[0], robot[1], robot)
                self._tag_goal_published = True
                self.get_logger().warn("/tag_pose 없음 -> 현재 위치 목표로 정지")

    # ======================================================================
    #  프론티어 (잡음 강건)
    # ======================================================================
    def find_frontiers(self):
        m = self.map_msg
        w, h = m.info.width, m.info.height
        if w == 0 or h == 0:
            return []
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y

        grid = np.array(m.data, dtype=np.int16).reshape(h, w)
        grid = self._denoise_occupied(grid)        # (1) 점유 잡음 제거

        free = (grid >= 0) & (grid < self.occ_thresh)
        unknown = (grid < 0)
        occ = (grid >= self.occ_thresh)

        # (2) 미탐색 이웃 개수(8방향) — 단일 노이즈성 경계 배제
        unk = unknown.astype(np.uint8)
        cnt = np.zeros((h, w), dtype=np.uint8)
        cnt[1:, :]    += unk[:-1, :]
        cnt[:-1, :]   += unk[1:, :]
        cnt[:, 1:]    += unk[:, :-1]
        cnt[:, :-1]   += unk[:, 1:]
        cnt[1:, 1:]   += unk[:-1, :-1]
        cnt[:-1, :-1] += unk[1:, 1:]
        cnt[1:, :-1]  += unk[:-1, 1:]
        cnt[:-1, 1:]  += unk[1:, :-1]

        # (3) 점유 dilation(clearance) — 벽/장애물 근처 프론티어 제외
        occ_dil = occ.copy()
        for _ in range(max(self.clearance_cells, 0)):
            d = occ_dil.copy()
            d[1:, :]  |= occ_dil[:-1, :]
            d[:-1, :] |= occ_dil[1:, :]
            d[:, 1:]  |= occ_dil[:, :-1]
            d[:, :-1] |= occ_dil[:, 1:]
            occ_dil = d

        frontier = free & (cnt >= self.unknown_neighbor_min) & (~occ_dil)

        # (4) 8-연결 군집화 + 최소 크기 필터
        visited = np.zeros_like(frontier, dtype=bool)
        clusters = []
        ys, xs = np.where(frontier)
        for sy, sx in zip(ys, xs):
            if visited[sy, sx]:
                continue
            q = deque([(sy, sx)])
            visited[sy, sx] = True
            cells = []
            while q:
                cy, cx = q.popleft()
                cells.append((cy, cx))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dy == 0 and dx == 0:
                            continue
                        ny, nx = cy + dy, cx + dx
                        if 0 <= ny < h and 0 <= nx < w \
                                and frontier[ny, nx] and not visited[ny, nx]:
                            visited[ny, nx] = True
                            q.append((ny, nx))
            if len(cells) >= self.min_frontier_size:
                mr = sum(c[0] for c in cells) / len(cells)
                mc = sum(c[1] for c in cells) / len(cells)
                wx = ox + (mc + 0.5) * res
                wy = oy + (mr + 0.5) * res
                clusters.append((wx, wy, len(cells)))
        return clusters

    def _denoise_occupied(self, grid):
        """연결 크기가 min_occupied_blob 미만인 점유 덩어리를 자유공간(0)으로 되돌림."""
        occ = (grid >= self.occ_thresh)
        h, w = occ.shape
        visited = np.zeros_like(occ, dtype=bool)
        cleaned = grid.copy()
        ys, xs = np.where(occ)
        for sy, sx in zip(ys, xs):
            if visited[sy, sx]:
                continue
            q = deque([(sy, sx)])
            visited[sy, sx] = True
            comp = []
            while q:
                cy, cx = q.popleft()
                comp.append((cy, cx))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dy == 0 and dx == 0:
                            continue
                        ny, nx = cy + dy, cx + dx
                        if 0 <= ny < h and 0 <= nx < w \
                                and occ[ny, nx] and not visited[ny, nx]:
                            visited[ny, nx] = True
                            q.append((ny, nx))
            if len(comp) < self.min_occupied_blob:
                for (cy, cx) in comp:
                    cleaned[cy, cx] = 0     # 잡음 -> 자유공간 취급
        return cleaned

    def select_frontier(self, frontiers, robot):
        """정보 이득 기반: utility = size / (distance + 1) 가 가장 큰 프론티어."""
        best, best_u = None, -1.0
        for (wx, wy, size) in frontiers:
            if self._blacklisted(wx, wy):
                continue
            d = math.dist(robot, (wx, wy))
            if d < self.goal_tolerance:        # 이미 그 자리
                continue
            u = size / (d + 1.0)
            if u > best_u:
                best_u, best = u, (wx, wy)
        return best

    # ======================================================================
    #  유틸
    # ======================================================================
    def get_robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_base_frame, rclpy.time.Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException:
            return None

    def publish_goal(self, x, y, robot):
        yaw = math.atan2(y - robot[1], x - robot[0]) if robot else 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.goal_pub.publish(msg)

    def _blacklisted(self, x, y):
        return any(math.dist((x, y), b) < self.blacklist_radius for b in self.blacklist)

    def _elapsed(self, start_time):
        if start_time is None:
            return 0.0
        return (self.get_clock().now() - start_time).nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = GoalManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
