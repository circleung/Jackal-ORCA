#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pure_pursuit_node.py  (개선판: perception 장애물 인지형 Pure Pursuit)

기존 Pure Pursuit는 /path 를 그대로 추종만 해서 동적 장애물에 무방비였다.
이 개선판은 perception 노드가 내보내는 LiDAR 클러스터(장애물 marker)를 받아
  - 전방 통로(corridor) 안 장애물이 warn 거리 안 → 비례 감속
  - danger 거리 안 → 정지 후 회피 방향으로 살짝 회전
  - 경로 끝 도달 / 태그 감지 → 정지
하도록 했다. 임계값은 perception 파라미터 문서의 warn/danger 개념을 재사용.

※ 기존 pure_pursuit_node.py 를 대체하는 드롭인 개선판이다. 기존 파일은 백업해 둘 것.

[구독]
  /path                                nav_msgs/Path
  /perception/lidar/clusters_markers   visualization_msgs/MarkerArray
  /tag_detected                        std_msgs/Bool
[발행]
  /j100_0915/cmd_vel                   geometry_msgs/Twist
[TF]
  <path frame> -> base_link , <marker frame> -> base_link
"""

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException


def quat_to_yaw(q):
    """쿼터니언 -> 평면 yaw."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit_node')

        # ---------------- 파라미터 ----------------
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('clusters_topic', '/perception/lidar/clusters_markers')
        self.declare_parameter('tag_detected_topic', '/tag_detected')
        self.declare_parameter('cmd_vel_topic', '/j100_0915/cmd_vel')
        self.declare_parameter('robot_base_frame', 'base_link')

        self.declare_parameter('max_linear_speed', 0.5)     # m/s
        self.declare_parameter('max_angular_speed', 1.2)    # rad/s
        self.declare_parameter('lookahead_distance', 0.6)   # m
        self.declare_parameter('goal_tolerance', 0.25)      # m, 경로 끝 도달 판정
        self.declare_parameter('control_frequency', 20.0)   # Hz
        self.declare_parameter('max_linear_accel', 0.6)     # m/s^2 (부드러운 가감속)

        # 장애물 회피 (perception 문서의 warn/danger 의미 재사용)
        self.declare_parameter('warn_dist_m', 1.5)          # 이내면 감속 시작
        self.declare_parameter('danger_dist_m', 0.8)        # 이내면 정지 + 회피
        self.declare_parameter('corridor_half_width', 0.45) # 전방 통로 반폭(m)
        self.declare_parameter('enable_reactive_turn', True)
        self.declare_parameter('reactive_turn_speed', 0.6)  # rad/s

        g = lambda n: self.get_parameter(n).value
        self.path_topic = g('path_topic')
        self.clusters_topic = g('clusters_topic')
        self.tag_topic = g('tag_detected_topic')
        self.cmd_topic = g('cmd_vel_topic')
        self.base_frame = g('robot_base_frame')
        self.v_max = float(g('max_linear_speed'))
        self.w_max = float(g('max_angular_speed'))
        self.Ld = float(g('lookahead_distance'))
        self.goal_tol = float(g('goal_tolerance'))
        self.freq = float(g('control_frequency'))
        self.a_max = float(g('max_linear_accel'))
        self.warn = float(g('warn_dist_m'))
        self.danger = float(g('danger_dist_m'))
        self.corridor = float(g('corridor_half_width'))
        self.reactive = bool(g('enable_reactive_turn'))
        self.turn_speed = float(g('reactive_turn_speed'))

        # ---------------- 상태 ----------------
        self.path = None              # [(x, y), ...]  (path frame)
        self.path_frame = 'map'
        self.obstacles = []           # [(frame_id, x, y), ...]  (원본 프레임)
        self.tag_stop = False
        self.prev_v = 0.0
        self._tf_cache = {}           # 루프 1회분 프레임별 TF 캐시

        # ---------------- TF ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- 통신 ----------------
        self.create_subscription(Path, self.path_topic, self.path_cb, 10)
        self.create_subscription(MarkerArray, self.clusters_topic, self.clusters_cb, 10)
        self.create_subscription(Bool, self.tag_topic, self.tag_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.dt = 1.0 / self.freq
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("pure_pursuit(개선판) 시작 — 장애물 인지 주행")

    # ======================================================================
    #  콜백
    # ======================================================================
    def path_cb(self, msg: Path):
        if msg.header.frame_id:
            self.path_frame = msg.header.frame_id
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def clusters_cb(self, msg: MarkerArray):
        obs = []
        for m in msg.markers:
            if m.action != Marker.ADD:
                continue
            if m.type == Marker.TEXT_VIEW_FACING:   # 라벨 마커는 제외
                continue
            obs.append((m.header.frame_id, m.pose.position.x, m.pose.position.y))
        self.obstacles = obs

    def tag_cb(self, msg: Bool):
        if msg.data and not self.tag_stop:
            self.get_logger().info("태그 감지 — 정지")
        self.tag_stop = bool(msg.data)

    # ======================================================================
    #  제어 루프
    # ======================================================================
    def control_loop(self):
        self._tf_cache = {}   # 매 루프 캐시 초기화

        # 태그 정지 최우선
        if self.tag_stop or not self.path:
            self.publish(0.0, 0.0)
            return

        path_tf = self.get_tf(self.path_frame)
        if path_tf is None:
            self.get_logger().warn(
                f"TF {self.base_frame}<-{self.path_frame} 없음 — 정지",
                throttle_duration_sec=5.0)
            self.publish(0.0, 0.0)
            return

        # 경로를 로봇(base) 좌표계로 변환
        pts = [self.apply_tf(path_tf, x, y) for (x, y) in self.path]

        # 경로 끝 도달?
        gx, gy = pts[-1]
        if math.hypot(gx, gy) < self.goal_tol:
            self.publish(0.0, 0.0)
            self.get_logger().info("경로 끝 도달 — 정지", throttle_duration_sec=3.0)
            return

        # Lookahead 점: 전방(x>0)으로 Ld 이상 떨어진 첫 점
        target = None
        for (px, py) in pts:
            if px <= 0.0:
                continue
            if math.hypot(px, py) >= self.Ld:
                target = (px, py)
                break
        if target is None:
            target = (gx, gy)

        lx, ly = target
        Ld = max(math.hypot(lx, ly), 1e-3)
        curvature = 2.0 * ly / (Ld * Ld)          # Pure Pursuit 곡률

        # 장애물 응답: 속도 계수 + 회피 회전
        speed_factor, turn_bias = self.obstacle_response()

        v_target = self.v_max * speed_factor
        # 조향: 정지 직전에도 최소 회전 능력 유지
        w = curvature * max(v_target, 0.15) + turn_bias
        w = max(-self.w_max, min(self.w_max, w))
        v = self.accel_limit(v_target)
        self.publish(v, w)

    def obstacle_response(self):
        """전방 통로 안 가장 가까운 장애물로 (속도계수, 회피각속도) 산출."""
        nearest_d = None
        nearest_side = 0.0
        for (frame, ox, oy) in self.obstacles:
            b = self.to_base(frame, ox, oy)
            if b is None:
                continue
            bx, by = b
            if bx <= 0.0:                 # 전방만
                continue
            if abs(by) > self.corridor:   # 통로 밖 무시
                continue
            d = math.hypot(bx, by)
            if d > self.warn:             # 관심 거리 밖
                continue
            if nearest_d is None or d < nearest_d:
                nearest_d = d
                nearest_side = by

        if nearest_d is None:
            return 1.0, 0.0
        if nearest_d < self.danger:
            # 정지 + 장애물 반대쪽으로 회전 (오른쪽 장애물이면 좌회전)
            turn = 0.0
            if self.reactive:
                turn = self.turn_speed if nearest_side <= 0.0 else -self.turn_speed
            return 0.0, turn
        # warn~danger 사이 선형 감속
        factor = (nearest_d - self.danger) / max(self.warn - self.danger, 1e-3)
        return max(0.0, min(1.0, factor)), 0.0

    # ======================================================================
    #  TF / 유틸
    # ======================================================================
    def get_tf(self, src_frame):
        """base_frame <- src_frame 변환을 (tx, ty, yaw) 로 반환(루프 내 캐시)."""
        if src_frame in self._tf_cache:
            return self._tf_cache[src_frame]
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, src_frame, rclpy.time.Time())
            res = (t.transform.translation.x,
                   t.transform.translation.y,
                   quat_to_yaw(t.transform.rotation))
        except TransformException:
            res = None
        self._tf_cache[src_frame] = res
        return res

    @staticmethod
    def apply_tf(tf, x, y):
        tx, ty, yaw = tf
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * x - s * y + tx, s * x + c * y + ty)

    def to_base(self, frame, x, y):
        if frame == self.base_frame or frame == '':
            return (x, y)
        tf = self.get_tf(frame)
        if tf is None:
            return None
        return self.apply_tf(tf, x, y)

    def accel_limit(self, v_target):
        """선형 속도 변화량 제한 → 급가감속 방지."""
        dv = v_target - self.prev_v
        max_dv = self.a_max * self.dt
        if dv > max_dv:
            v = self.prev_v + max_dv
        elif dv < -max_dv:
            v = self.prev_v - max_dv
        else:
            v = v_target
        self.prev_v = v
        return v

    def publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)
        if v == 0.0:
            self.prev_v = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.0, 0.0)   # 종료 시 정지 명령
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
