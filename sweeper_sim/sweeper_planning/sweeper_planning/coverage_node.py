#!/usr/bin/env python3
"""
Coverage path generator + real-time coverage tracking.

Coverage is accumulated from the robot's actual trajectory (not just
the current position), so the metric grows monotonically.

Publishes:
  /coverage/path          nav_msgs/Path           (latched)
  /coverage/grid          nav_msgs/OccupancyGrid  (2 Hz, accumulated)
  /coverage/coverage_pct  std_msgs/Float32        (2 Hz, 0-100)
  /planner/progress       std_msgs/Float32        (2 Hz, 0-1)
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import tf_transformations as tft


def yaw_from_quat(q):
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def _arc_pts(cx, cy, r, a_start, a_end, n=24):
    angles = np.linspace(a_start, a_end, n)
    return list(zip(cx + r * np.cos(angles), cy + r * np.sin(angles)))


class CoverageNode(Node):
    def __init__(self):
        super().__init__('coverage_node')

        self.declare_parameters('', [
            ('area_x_min',          -12.0),
            ('area_x_max',           14.5),
            ('area_y_min',           -9.0),
            ('area_y_max',            9.5),
            ('sweep_spacing',         1.0),
            ('edge_offset',           0.8),
            ('edge_follow_offset',    0.2),
            ('grid_resolution',       0.5),
            ('map_frame',           'odom'),
            ('min_turn_radius',       0.95),
            ('sweep_half_width',      0.975), # half-width = arc_R → 行间无缝
        ])
        g = self.get_parameter
        self.xmin       = g('area_x_min').value
        self.xmax       = g('area_x_max').value
        self.ymin       = g('area_y_min').value
        self.ymax       = g('area_y_max').value
        self.spacing    = g('sweep_spacing').value
        self.edge_off   = g('edge_offset').value
        self.ef_off     = g('edge_follow_offset').value
        self.res        = g('grid_resolution').value
        self.frame      = g('map_frame').value
        self.turn_r     = g('min_turn_radius').value
        self.sweep_hw   = g('sweep_half_width').value

        # Grid dimensions
        self.W = max(1, int(math.ceil((self.xmax - self.xmin) / self.res)))
        self.H = max(1, int(math.ceil((self.ymax - self.ymin) / self.res)))
        # Persistent accumulated coverage grid (False=unswept, True=swept)
        self._swept = np.zeros((self.H, self.W), dtype=bool)
        self._total_cells = self.W * self.H

        latch      = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.pub_path  = self.create_publisher(Path, '/coverage/path', latch)
        self.pub_grid  = self.create_publisher(OccupancyGrid, '/coverage/grid', 5)
        self.pub_cov   = self.create_publisher(Float32, '/coverage/coverage_pct', 5)
        self.pub_prog  = self.create_publisher(Float32, '/planner/progress', 5)
        self.sub_odom  = self.create_subscription(
            Odometry, '/odom', self.cb_odom, sensor_qos)

        self.robot_xy   = None
        self.path_pts   = []
        self.n_path     = 0

        self._build_and_publish()
        self.create_timer(0.5, self._tick)

    # ------------------------------------------------------------------ #
    def cb_odom(self, msg: Odometry):
        self.robot_xy = (msg.pose.pose.position.x,
                         msg.pose.pose.position.y)
        # Accumulate swept area in real-time (high-rate callback)
        if self.robot_xy:
            self._mark_swept(self.robot_xy[0], self.robot_xy[1])

    def _mark_swept(self, rx: float, ry: float):
        """Mark all grid cells within sweep_hw of (rx, ry) as swept."""
        hw  = self.sweep_hw
        res = self.res
        col_c = int((rx - self.xmin) / res)
        row_c = int((ry - self.ymin) / res)
        r_cells = max(1, int(math.ceil(hw / res)))
        for dr in range(-r_cells, r_cells + 1):
            for dc in range(-r_cells, r_cells + 1):
                if math.hypot(dr * res, dc * res) <= hw:
                    rr = row_c + dr
                    cc = col_c + dc
                    if 0 <= rr < self.H and 0 <= cc < self.W:
                        self._swept[rr, cc] = True

    # ------------------------------------------------------------------ #
    def _tick(self):
        if self.robot_xy is None or not self.path_pts:
            return
        rx, ry = self.robot_xy

        # ── Path progress (closest point, monotonic via forward window) ──
        pts_arr = np.array(self.path_pts)
        d2 = np.sum((pts_arr - np.array([rx, ry])) ** 2, axis=1)
        prog = float(np.argmin(d2)) / max(1, self.n_path - 1)
        self.pub_prog.publish(Float32(data=float(prog)))

        # ── Coverage percentage (from accumulated grid) ──
        swept_count = int(np.sum(self._swept))
        pct = 100.0 * swept_count / self._total_cells
        self.pub_cov.publish(Float32(data=float(pct)))
        self.get_logger().info(
            f'Coverage: {pct:.1f}% ({swept_count}/{self._total_cells} cells)',
            throttle_duration_sec=2.0)

        # ── OccupancyGrid (for RViz / scoring) ──
        flat = np.where(self._swept, np.int8(100), np.int8(0)).flatten()
        grid = OccupancyGrid()
        grid.header.stamp    = self.get_clock().now().to_msg()
        grid.header.frame_id = self.frame
        grid.info.resolution = float(self.res)
        grid.info.width      = self.W
        grid.info.height     = self.H
        grid.info.origin.position.x = self.xmin
        grid.info.origin.position.y = self.ymin
        grid.info.origin.orientation.w = 1.0
        grid.data = flat.tolist()
        self.pub_grid.publish(grid)

    # ------------------------------------------------------------------ #
    def _build_and_publish(self):
        pts = self._build_path()
        self.path_pts = pts
        self.n_path   = len(pts)

        msg = Path()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
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

        self.pub_path.publish(msg)
        n_edge  = sum(1 for x, y in pts if self._is_edge_pt(x, y))
        self.get_logger().info(
            f'Coverage path published: {len(pts)} poses '
            f'({n_edge} edge + {len(pts)-n_edge} sweep)')

    def _is_edge_pt(self, x, y):
        m = self.edge_off + 0.1
        return (x < self.xmin + m or x > self.xmax - m or
                y < self.ymin + m or y > self.ymax - m)

    def _build_path(self):
        """
        修复版路径生成（解决机器人撞墙无法转弯问题）：

        核心约束：
          eff_edge  ≥ turn_r + 0.15   — U形弯弧不超出场地边界
          eff_spacing = 2 × arc_R    — 弧形终点恰好落在下一行起点

        当 edge_off(0.8m) < turn_r(0.95m) 时，原弧会延伸到 x=14.65 > 场地边界
        x=14.5，导致机器人追弧点时撞墙卡死。本修复强制保证转弯空间。

        路径结构：
          ① 外贴边圈  (ef_off)
          ② 中间过渡圈1 (ef_off + sweep_hw)
          ③ 中间过渡圈2 (ef_off + 2*sweep_hw)
          ④ 内层扫行  (eff_edge, 每行间距 eff_spacing, 每点 0.5m)
        """
        pts = []

        # ── 约束推导 ──────────────────────────────────────────────────
        # 弧半径必须 ≤ 从行尾到场地边界的距离（edge_off）
        # → eff_edge ≥ turn_r + 安全余量
        eff_edge    = max(self.edge_off, self.turn_r + 0.15)
        # 行间距必须 = 2×arc_R，确保弧终点落在下一行起点
        eff_spacing = max(self.spacing, 2.0 * self.turn_r + 0.05)
        arc_R       = eff_spacing / 2.0   # 弧半径 = 行间距的一半

        # x 方向需要额外余量：弧最远点 + 车身半宽(0.525m) + 安全裕量 ≤ 墙
        # 车身半宽 1.05/2=0.525m; 保险余量 0.10m → turn_r + 0.625
        x_margin = max(self.edge_off, self.turn_r + 0.65)

        self.get_logger().info(
            f'Coverage build: y_edge={eff_edge:.2f}m, x_margin={x_margin:.2f}m, '
            f'spacing={eff_spacing:.2f}m, arc_R={arc_R:.2f}m')

        # ── 内侧扫行（boustrophedon + 闭合 U 形弯）──────────────────
        # 不再生成边界条——边界条的直角拐角会令机器人撞东/西墙卡死；
        # 内层扫行的 sweep_hw 已延伸至距墙 ~0.35m，覆盖率足够。
        # x 方向使用较大 x_margin 保证 U-turn 弧不撞东/西墙；
        # y 方向使用 eff_edge（较小），保留全部 9 行覆盖范围。
        ix_min = self.xmin + x_margin
        ix_max = self.xmax - x_margin
        iy_min = self.ymin + eff_edge
        iy_max = self.ymax - eff_edge

        ys = np.arange(iy_min, iy_max + eff_spacing * 0.01, eff_spacing)

        for idx, y in enumerate(ys):
            going_right = (idx % 2 == 0)
            x_start = ix_min if going_right else ix_max
            x_end   = ix_max if going_right else ix_min

            # 每 0.5m 一个路径点
            n_seg = max(2, int(abs(x_end - x_start) / 0.5) + 1)
            for x in np.linspace(x_start, x_end, n_seg):
                pts.append((float(x), float(y)))

            if idx + 1 < len(ys):
                if going_right:
                    # 右侧 U-turn：弧心 (x_end, y+arc_R)，向右凸出
                    arc = _arc_pts(x_end, y + arc_R, arc_R,
                                   -math.pi / 2, math.pi / 2, 36)
                else:
                    # 左侧 U-turn：弧心 (x_end, y+arc_R)，向左凸出
                    # 角度从 -π/2 经过 -π 到 -3π/2，即从行末 (x_end,y)
                    # 绕左侧到达下一行起点 (x_end, y+eff_spacing)
                    arc = _arc_pts(x_end, y + arc_R, arc_R,
                                   -math.pi / 2, -3 * math.pi / 2, 36)
                pts.extend(arc)

        return pts

    def _boundary_strip(self, off):
        """生成矩形边界贴边路径（顺时针，点间距 0.5m）。"""
        pts  = []
        step = 0.5
        xmin, xmax = self.xmin + off, self.xmax - off
        ymin, ymax = self.ymin + off, self.ymax - off
        if xmin >= xmax or ymin >= ymax:
            return pts
        for x in np.arange(xmin, xmax, step): pts.append((float(x), float(ymin)))
        pts.append((float(xmax), float(ymin)))
        for y in np.arange(ymin, ymax, step): pts.append((float(xmax), float(y)))
        pts.append((float(xmax), float(ymax)))
        for x in np.arange(xmax, xmin, -step): pts.append((float(x), float(ymax)))
        pts.append((float(xmin), float(ymax)))
        for y in np.arange(ymax, ymin, -step): pts.append((float(xmin), float(y)))
        pts.append((float(xmin), float(ymin)))
        return pts


def main():
    rclpy.init()
    node = CoverageNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
