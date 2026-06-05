#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
goal_manager_node.py

자율주행 파이프라인의 ①번 두뇌: "어디로 갈지" 결정하는 노드.

  - 평소(EXPLORING) : /map 의 프론티어(탐색된 빈공간과 미탐색 영역의 경계)를
                      찾아 가장 가까운 곳을 /goal_pose 로 발행 → 미지 영역 탐사.
  - 태그 발견(TAG_APPROACH) : 탐색을 멈추고 태그 좌표를 목표로 전환.
                              /tag_pose 가 있으면 접근, 없으면 현재 위치를 찍어 정지.

[토픽 인터페이스]
  구독 : /map          (nav_msgs/OccupancyGrid)   - 프론티어 계산용 지도
        /tag_detected  (std_msgs/Bool)            - YOLO 노드의 태그 감지 신호
        /tag_pose      (geometry_msgs/PoseStamped)- (선택) 태그의 map 좌표
  발행 : /goal_pose    (geometry_msgs/PoseStamped)- Global Planner 입력
  TF   : map -> base_link (로봇 현재 위치 추정)

모든 토픽/프레임 이름은 ROS 파라미터로 바꿀 수 있다.
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
    """평면(2D) 회전각(yaw)을 쿼터니언 (x, y, z, w) 로 변환."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class GoalManagerNode(Node):

    def __init__(self):
        super().__init__('goal_manager_node')

        # ---------------- 파라미터 선언 ----------------
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('tag_detected_topic', '/tag_detected')
        self.declare_parameter('tag_pose_topic', '/tag_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')

        self.declare_parameter('occupied_threshold', 65)   # 이 값 이상이면 '벽(점유)' 으로 간주
        self.declare_parameter('min_frontier_size', 5)      # 이보다 작은 프론티어 군집은 무시(셀 수)
        self.declare_parameter('goal_tolerance', 0.35)      # 목표 도달 판정 거리(m)
        self.declare_parameter('goal_timeout', 30.0)        # 한 목표에 매달리는 최대 시간(s)
        self.declare_parameter('planning_period', 1.5)      # 상태머신 실행 주기(s)
        self.declare_parameter('blacklist_radius', 0.6)     # 블랙리스트로 거를 반경(m)

        # ---------------- 파라미터 읽기 ----------------
        g = lambda n: self.get_parameter(n).value
        self.map_topic = g('map_topic')
        self.goal_topic = g('goal_topic')
        self.tag_detected_topic = g('tag_detected_topic')
        self.tag_pose_topic = g('tag_pose_topic')
        self.map_frame = g('map_frame')
        self.robot_base_frame = g('robot_base_frame')
        self.occ_thresh = int(g('occupied_threshold'))
        self.min_frontier_size = int(g('min_frontier_size'))
        self.goal_tolerance = float(g('goal_tolerance'))
        self.goal_timeout = float(g('goal_timeout'))
        self.planning_period = float(g('planning_period'))
        self.blacklist_radius = float(g('blacklist_radius'))

        # ---------------- 내부 상태 ----------------
        self.map_msg = None              # 최신 OccupancyGrid
        self.state = 'EXPLORING'         # 'EXPLORING' | 'TAG_APPROACH' | 'DONE'
        self.current_goal = None         # (x, y) - 현재 추적 중인 목표
        self.goal_start_time = None      # 목표 발행 시각
        self.blacklist = []              # 도달 실패한 목표 좌표 [(x, y), ...]
        self.tag_pose = None             # map 프레임 태그 좌표(PoseStamped)
        self._prev_tag = False           # /tag_detected 의 이전 값(상승엣지 검출용)
        self._tag_goal_published = False # 태그 목표를 이미 발행했는지

        # ---------------- TF ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- 구독/발행 ----------------
        # /map 은 보통 latched(TRANSIENT_LOCAL) 로 발행되므로 QoS 를 맞춰야 받을 수 있다.
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.history = QoSHistoryPolicy.KEEP_LAST

        self.create_subscription(OccupancyGrid, self.map_topic, self.map_cb, map_qos)
        self.create_subscription(Bool, self.tag_detected_topic, self.tag_cb, 10)
        self.create_subscription(PoseStamped, self.tag_pose_topic, self.tag_pose_cb, 10)

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)

        self.timer = self.create_timer(self.planning_period, self.on_timer)

        self.get_logger().info(
            f"goal_manager 시작 | map='{self.map_topic}' goal='{self.goal_topic}' "
            f"tag='{self.tag_detected_topic}'")

    # ======================================================================
    #  콜백
    # ======================================================================
    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg

    def tag_cb(self, msg: Bool):
        # 상승엣지(False -> True)에서만 태그 접근 모드로 전환(한 번만 트리거).
        if msg.data and not self._prev_tag and self.state != 'TAG_APPROACH':
            self.get_logger().info("태그 감지! 탐색 중단 -> 태그 접근 모드로 전환")
            self.state = 'TAG_APPROACH'
            self._tag_goal_published = False
        self._prev_tag = msg.data

    def tag_pose_cb(self, msg: PoseStamped):
        # 태그 좌표가 map 프레임이 아니면 TF 로 변환 시도.
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, msg.header.frame_id, rclpy.time.Time())
                dx = tf.transform.translation.x
                dy = tf.transform.translation.y
                # 평면 가정: 회전 무시 + 평행이동만 적용(간단화). 필요시 do_transform_pose 사용.
                p = PoseStamped()
                p.header.frame_id = self.map_frame
                p.pose.position.x = msg.pose.position.x + dx
                p.pose.position.y = msg.pose.position.y + dy
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
            self.get_logger().warn("아직 /map 을 받지 못함", throttle_duration_sec=5.0)
            return

        if self.state == 'TAG_APPROACH':
            self.handle_tag_approach()
            return

        # ----- EXPLORING -----
        robot = self.get_robot_xy()
        if robot is None:
            self.get_logger().warn(
                f"로봇 위치(TF {self.map_frame}->{self.robot_base_frame})를 못 구함",
                throttle_duration_sec=5.0)
            return

        # 현재 목표가 있으면 도달/지연 여부부터 판단
        if self.current_goal is not None:
            d = math.dist(robot, self.current_goal)
            elapsed = self._elapsed(self.goal_start_time)
            if d < self.goal_tolerance:
                self.get_logger().info("목표 도달 -> 다음 프론티어 탐색")
                self.current_goal = None
            elif elapsed > self.goal_timeout:
                self.get_logger().warn("목표 도달 지연 -> 블랙리스트 등록 후 재선정")
                self.blacklist.append(self.current_goal)
                self.current_goal = None
            else:
                return  # 아직 가는 중

        # 새 목표 선정
        frontiers = self.find_frontiers()
        best = self.select_frontier(frontiers, robot)
        if best is None:
            self.get_logger().info("도달 가능한 프론티어 없음 -> 탐색 완료(DONE)")
            self.state = 'DONE'
            return

        self.current_goal = best
        self.goal_start_time = self.get_clock().now()
        self.publish_goal(best[0], best[1], robot)
        self.get_logger().info(
            f"새 탐색 목표 발행: ({best[0]:.2f}, {best[1]:.2f})")

    def handle_tag_approach(self):
        if self._tag_goal_published:
            return
        if self.tag_pose is not None:
            self.goal_pub.publish(self.tag_pose)
            self._tag_goal_published = True
            self.get_logger().info("태그 좌표로 목표 발행(접근)")
        else:
            # 좌표가 없으면 현재 위치를 목표로 찍어 정지(접근은 불가).
            robot = self.get_robot_xy()
            if robot is not None:
                self.publish_goal(robot[0], robot[1], robot)
                self._tag_goal_published = True
                self.get_logger().warn(
                    "/tag_pose 없음 -> 접근 불가, 현재 위치 목표로 정지")

    # ======================================================================
    #  프론티어 탐색
    # ======================================================================
    def find_frontiers(self):
        """현재 지도에서 프론티어 군집을 찾아 [(wx, wy, size), ...] 반환."""
        m = self.map_msg
        w, h = m.info.width, m.info.height
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y

        grid = np.array(m.data, dtype=np.int16).reshape(h, w)
        free = (grid >= 0) & (grid < self.occ_thresh)   # 자유 공간
        unknown = (grid < 0)                              # 미탐색(-1)

        # 자유 공간 셀 중 4방향 이웃에 미탐색이 하나라도 있으면 프론티어.
        nb_unknown = np.zeros_like(unknown)
        nb_unknown[1:, :]  |= unknown[:-1, :]
        nb_unknown[:-1, :] |= unknown[1:, :]
        nb_unknown[:, 1:]  |= unknown[:, :-1]
        nb_unknown[:, :-1] |= unknown[:, 1:]
        frontier = free & nb_unknown

        # 8방향 연결 군집화(BFS)
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
                wx = ox + (mc + 0.5) * res   # 셀 -> map 좌표 (origin 회전=0 가정)
                wy = oy + (mr + 0.5) * res
                clusters.append((wx, wy, len(cells)))
        return clusters

    def select_frontier(self, frontiers, robot):
        """블랙리스트를 제외하고 로봇에서 가장 가까운 프론티어를 선택."""
        best, best_d = None, float('inf')
        for (wx, wy, _size) in frontiers:
            if self._blacklisted(wx, wy):
                continue
            d = math.dist(robot, (wx, wy))
            if d < best_d:
                best_d, best = d, (wx, wy)
        return best

    # ======================================================================
    #  유틸
    # ======================================================================
    def get_robot_xy(self):
        """TF 로 로봇의 map 프레임 (x, y) 를 구한다. 실패 시 None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_base_frame, rclpy.time.Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException:
            return None

    def publish_goal(self, x, y, robot):
        """목표 PoseStamped 발행. 진행 방향(robot->goal)을 바라보도록 yaw 설정."""
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
