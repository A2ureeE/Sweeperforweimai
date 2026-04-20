#!/usr/bin/env python3
"""
Local planner node.

已修复的关键缺陷：
  1. 停滞检测误报 — 旧版每 200ms 检查位移 > 0.15m，在 v=0.1m/s 时每帧只移动
     0.02m，会持续触发"停滞"跳路！改用累积行程计数器，只有 6s 内累积位移
     < stagnation_min_dist_total 才判为真正停滞。
  2. progress 重复发布 — coverage_node 也发布 /planner/progress，造成两个节点
     互相覆盖。planner_node 改为发布 /planner/path_progress（供日志区分）。
  3. 路径进度跳变 — 搜索窗口包含 U 形弯时，最近点可能是弯道反向点导致回退。
     改为先用前向滑动窗口找最近点，再强制 new_idx >= progress_idx。

Subscribes:
  /coverage/path               nav_msgs/Path
  /odom                        nav_msgs/Odometry
  /perception/obstacle_points  geometry_msgs/PolygonStamped
  /behavior/mode               std_msgs/String

Publishes:
  /reference_path              nav_msgs/Path
  /planner/path_progress       std_msgs/Float32  (0-1, 不再与 coverage_node 冲突)
"""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PolygonStamped
from std_msgs.msg import String, Float32
import tf_transformations as tft


def yaw_from_quat(q):
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.declare_parameters('', [
            ('lookahead_dist',            12.0),
            ('obstacle_inflate',           0.6),
            ('obstacle_memory_s',          8.0),
            ('gate_align_dist',            4.0),
            ('detour_shift_m',             1.0),
            ('stagnation_time_s',          6.0),
            # 停滞判定：6s 内累积行程必须 > 此值才认为有在移动
            ('stagnation_min_dist_total',  0.5),
        ])
        g = self.get_parameter
        self.lookahead    = g('lookahead_dist').value
        self.inflate      = g('obstacle_inflate').value
        self.obs_mem_s    = g('obstacle_memory_s').value
        self.gate_align   = g('gate_align_dist').value
        self.detour_shift = g('detour_shift_m').value
        self.stag_time    = g('stagnation_time_s').value
        self.stag_min_d   = g('stagnation_min_dist_total').value

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_path = self.create_subscription(
            Path, '/coverage/path', self.cb_cov, 5)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.cb_odom, sensor_qos)
        self.sub_obs  = self.create_subscription(
            PolygonStamped, '/perception/obstacle_points', self.cb_obs, sensor_qos)
        self.sub_mode = self.create_subscription(
            String, '/behavior/mode', self.cb_mode, 5)

        self.pub_ref   = self.create_publisher(Path,    '/reference_path',        5)
        # 改名避免与 coverage_node 冲突
        self.pub_prog  = self.create_publisher(Float32, '/planner/path_progress', 5)

        self.cov_pts: list  = []
        self.robot          = None       # (x, y, yaw)
        self.mode           = 'COVERAGE'
        self.progress_idx   = 0
        self.obs_memory: dict = {}       # (x, y) → expiry time

        # 停滞检测：累积行程
        self._stag_accum_d  = 0.0       # 当前周期累积位移
        self._stag_start_t  = time.time()
        self._last_stag_pos = None

        self.create_timer(0.2, self.tick)
        self.get_logger().info('planner_node ready')

    # ── 回调 ────────────────────────────────────────────────────────────
    def cb_cov(self, msg: Path):
        self.cov_pts       = [(p.pose.position.x, p.pose.position.y)
                              for p in msg.poses]
        self.progress_idx  = 0
        self._stag_accum_d = 0.0
        self._stag_start_t = time.time()
        self._last_stag_pos = None
        self.get_logger().info(f'Coverage path received: {len(self.cov_pts)} pts')

    def cb_odom(self, msg: Odometry):
        self.robot = (msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      yaw_from_quat(msg.pose.pose.orientation))

    def cb_obs(self, msg: PolygonStamped):
        now = time.time()
        for p in msg.polygon.points:
            k = (round(float(p.x), 1), round(float(p.y), 1))
            self.obs_memory[k] = now + self.obs_mem_s
        expired = [k for k, t in self.obs_memory.items() if t < now]
        for k in expired:
            del self.obs_memory[k]

    def cb_mode(self, msg: String):
        self.mode = msg.data.split('_')[0] if msg.data.startswith('EDGE_FOLLOW') else msg.data

    # ── 主循环 ──────────────────────────────────────────────────────────
    def tick(self):
        if not self.cov_pts or self.robot is None:
            return

        rx, ry, ryaw = self.robot
        n   = len(self.cov_pts)
        pts = np.array(self.cov_pts)

        # ── 前向滑动窗口找最近点（防止 U 形弯回退）──────────────────
        window_end = min(n - 1, self.progress_idx + 80)
        sub  = pts[self.progress_idx: window_end + 1]
        d2   = np.sum((sub - np.array([rx, ry])) ** 2, axis=1)
        new_idx = self.progress_idx + int(np.argmin(d2))
        # 严格单调递增
        self.progress_idx = max(self.progress_idx, new_idx)

        # ── 停滞检测（累积行程版）────────────────────────────────────
        now = time.time()
        if self._last_stag_pos is not None:
            step = math.hypot(rx - self._last_stag_pos[0],
                              ry - self._last_stag_pos[1])
            self._stag_accum_d += step
        self._last_stag_pos = (rx, ry)

        elapsed = now - self._stag_start_t
        if elapsed >= self.stag_time:
            if (self._stag_accum_d < self.stag_min_d
                    and self.mode in ('COVERAGE', 'STATIC_DETOUR')):
                jump = min(n - 1, self.progress_idx + 50)
                self.get_logger().warn(
                    f'停滞检测：{elapsed:.1f}s 内仅移动 {self._stag_accum_d:.2f}m '
                    f'— 跳跃 idx {self.progress_idx}→{jump}')
                self.progress_idx = jump
            # 重置计数器
            self._stag_accum_d = 0.0
            self._stag_start_t = now

        prog = float(self.progress_idx) / max(1, n - 1)
        self.pub_prog.publish(Float32(data=prog))

        # ── 截取参考窗口 ─────────────────────────────────────────────
        end_idx  = self.progress_idx
        dist_acc = 0.0
        while end_idx + 1 < n:
            dx = pts[end_idx + 1][0] - pts[end_idx][0]
            dy = pts[end_idx + 1][1] - pts[end_idx][1]
            dist_acc += math.hypot(dx, dy)
            end_idx  += 1
            if dist_acc >= self.lookahead:
                break

        slice_pts = [tuple(p) for p in pts[self.progress_idx: end_idx + 1]]
        if len(slice_pts) < 2:
            return

        # ── 障碍物侧向绕行（COVERAGE / STATIC_DETOUR）───────────────
        if self.mode in ('COVERAGE', 'STATIC_DETOUR'):
            slice_pts = self._apply_detour(slice_pts)

        self._publish_path(slice_pts)

    # ── 工具函数 ─────────────────────────────────────────────────────────
    def _apply_detour(self, pts: list) -> list:
        """将路径点从障碍物处横向推开。"""
        if not self.obs_memory:
            return pts
        obstacles = list(self.obs_memory.keys())
        out = []
        for x, y in pts:
            sx, sy = 0.0, 0.0
            for ox, oy in obstacles:
                dist = math.hypot(x - ox, y - oy)
                if dist < self.inflate + 0.1 and dist > 1e-4:
                    rep = (self.inflate + 0.1 - dist) / (self.inflate + 0.1)
                    nx  = (x - ox) / dist
                    ny  = (y - oy) / dist
                    sx += rep * nx * self.detour_shift
                    sy += rep * ny * self.detour_shift
            out.append((x + sx, y + sy))
        return out

    def _publish_path(self, pts: list):
        msg = Path()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for i, (x, y) in enumerate(pts):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            if i + 1 < len(pts):
                yaw = math.atan2(pts[i+1][1] - y, pts[i+1][0] - x)
            elif i > 0:
                yaw = math.atan2(y - pts[i-1][1], x - pts[i-1][0])
            else:
                yaw = 0.0
            q = tft.quaternion_from_euler(0, 0, yaw)
            ps.pose.orientation.x = q[0]; ps.pose.orientation.y = q[1]
            ps.pose.orientation.z = q[2]; ps.pose.orientation.w = q[3]
            msg.poses.append(ps)
        self.pub_ref.publish(msg)


def main():
    rclpy.init()
    node = PlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
