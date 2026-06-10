#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pure_pursuit_node.py  (최적화판: 장애물 인지 + 잡음 클러스터 강건화)

추가 최적화:
  1) 위험정지 디바운스 - 단발성 노이즈 클러스터로 급정지하지 않도록, danger 조건이
                        obstacle_confirm_frames 프레임 연속될 때만 정지.
  2) 속도계수 평활화   - 깜빡이는 클러스터로 속도가 출렁이지 않게 저역통과(EMA) 적용.
  3) 마커 크기 필터    - marker scale 면적이 min_obstacle_area 미만이면 잡음으로 무시.
  4) 가감속 제한       - 급출발/급정지 억제(기존 유지).

[구독] /path(Path), /perception/lidar/clusters_markers(MarkerArray), /tag_detected(Bool)
[발행] /j100_0915/cmd_vel(Twist)
[TF]   <path frame> -> base_link , <marker frame> -> base_link
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

        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('lookahead_distance', 0.6)
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('max_linear_accel', 0.6)

        # 장애물 회피
        self.declare_parameter('warn_dist_m', 1.5)
        self.declare_parameter('danger_dist_m', 0.8)
        self.declare_parameter('corridor_half_width', 0.45)
        self.declare_parameter('enable_reactive_turn', True)
        self.declare_parameter('reactive_turn_speed', 0.6)

        # --- 잡음 강건화 ---
        self.declare_parameter('obstacle_confirm_frames', 3)  # danger 정지 디바운스
        self.declare_parameter('factor_smoothing', 0.4)       # 속도계수 EMA 계수(0~1)
        self.declare_parameter('min_obstacle_area', 0.0)      # marker 면적 하한(m^2), 0=off

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
        self.confirm_frames = int(g('obstacle_confirm_frames'))
        self.alpha = float(g('factor_smoothing'))
        self.min_obstacle_area = float(g('min_obstacle_area'))

        # ---------------- 상태 ----------------
        self.path = None
        self.path_frame = 'map'
        self.obstacles = []           # [(frame, x, y, area), ...]
        self.tag_stop = False
        self.prev_v = 0.0
        self.danger_count = 0
        self.factor_filt = 1.0
        self._tf_cache = {}

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
        self.get_logger().info("pure_pursuit(최적화판) 시작 — 장애물 인지 + 잡음 강건")

    # ======================================================================
    #  콜백
    # ======================================================================
    def path_cb(self, msg):
        if msg.header.frame_id:
            self.path_frame = msg.header.frame_id
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def clusters_cb(self, msg):
        obs = []
        for m in msg.markers:
            if m.action != Marker.ADD:
                continue
            if m.type == Marker.TEXT_VIEW_FACING:
                continue
            area = abs(m.scale.x * m.scale.y)
            if self.min_obstacle_area > 0.0 and area < self.min_obstacle_area:
                continue
            obs.append((m.header.frame_id, m.pose.position.x, m.pose.position.y, area))
        self.obstacles = obs

    def tag_cb(self, msg):
        if msg.data and not self.tag_stop:
            self.get_logger().info("태그 감지 — 정지")
        self.tag_stop = bool(msg.data)

    # ======================================================================
    #  제어 루프
    # ======================================================================
    def control_loop(self):
        self._tf_cache = {}

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

        pts = [self.apply_tf(path_tf, x, y) for (x, y) in self.path]

        gx, gy = pts[-1]
        if math.hypot(gx, gy) < self.goal_tol:
            self.publish(0.0, 0.0)
            self.get_logger().info("경로 끝 도달 — 정지", throttle_duration_sec=3.0)
            return

        # Lookahead
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
        curvature = 2.0 * ly / (Ld * Ld)

        # 장애물 응답 (디바운스 + 평활화)
        nearest_d, side = self.nearest_obstacle()
        if nearest_d is None:
            raw_factor, danger = 1.0, False
        elif nearest_d < self.danger:
            raw_factor, danger = 0.0, True
        elif nearest_d < self.warn:
            raw_factor = (nearest_d - self.danger) / max(self.warn - self.danger, 1e-3)
            raw_factor, danger = max(0.0, min(1.0, raw_factor)), False
        else:
            raw_factor, danger = 1.0, False

        self.danger_count = self.danger_count + 1 if danger else 0
        stop = self.danger_count >= self.confirm_frames
        # 속도계수 EMA
        self.factor_filt = self.alpha * raw_factor + (1.0 - self.alpha) * self.factor_filt

        if stop:
            v_target = 0.0
            turn_bias = 0.0
            if self.reactive:
                turn_bias = self.turn_speed if side <= 0.0 else -self.turn_speed
        else:
            v_target = self.v_max * self.factor_filt
            turn_bias = 0.0

        w = curvature * max(v_target, 0.15) + turn_bias
        w = max(-self.w_max, min(self.w_max, w))
        v = self.accel_limit(v_target)
        self.publish(v, w)

    def nearest_obstacle(self):
        """전방 통로 안 최근접 장애물의 (거리, 측면부호) 반환. 없으면 (None, 0)."""
        nearest_d = None
        nearest_side = 0.0
        for (frame, ox, oy, _area) in self.obstacles:
            b = self.to_base(frame, ox, oy)
            if b is None:
                continue
            bx, by = b
            if bx <= 0.0:
                continue
            if abs(by) > self.corridor:
                continue
            d = math.hypot(bx, by)
            if d > self.warn:
                continue
            if nearest_d is None or d < nearest_d:
                nearest_d = d
                nearest_side = by
        return nearest_d, nearest_side

    # ======================================================================
    #  TF / 유틸
    # ======================================================================
    def get_tf(self, src_frame):
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
        node.publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
