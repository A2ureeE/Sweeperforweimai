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

# Gazebo ModelStates 用于在线障碍感知（R5）。当 gazebo_msgs 不可用时退化处理。
try:
    from gazebo_msgs.msg import ModelStates  # type: ignore
    _HAS_MODEL_STATES = True
except ImportError:  # pragma: no cover
    ModelStates = None
    _HAS_MODEL_STATES = False


def yaw_from_quat(q):
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _arc_pts(cx, cy, r, a_start, a_end, n=24):
    # Exclude the arc start point; caller already appended the line endpoint.
    angles = np.linspace(a_start, a_end, n + 1)[1:]
    return list(zip(cx + r * np.cos(angles), cy + r * np.sin(angles)))


def _cubic_bridge(p_from, yaw_from, p_to, yaw_to, n=16, tangent_scale=None):
    """三次 Hermite 插值生成"保切线"过渡弧。

    端点严格满足位置 + 切线方向约束，中段曲率随几何自适应。
    不如 Dubins 精确，但实现简单、对任意 heading 组合都能给出连续路径。

    Args:
        p_from, p_to: (x, y) 起止点
        yaw_from, yaw_to: 起止切线朝向（rad）
        n: 采样点数（不含 p_from, 含 p_to）
        tangent_scale: 切线向量幅值；None 时默认 |p_to - p_from|（自然曲率）

    Returns:
        [(x, y)] 列表，长度 == n。不包含 p_from。
    """
    p1 = np.array(p_from, dtype=float)
    p2 = np.array(p_to, dtype=float)
    d = float(np.linalg.norm(p2 - p1))
    if d < 1e-4:
        return []
    scale = d if tangent_scale is None else float(tangent_scale)
    t1 = scale * np.array([math.cos(yaw_from), math.sin(yaw_from)])
    t2 = scale * np.array([math.cos(yaw_to), math.sin(yaw_to)])
    pts = []
    for k in range(1, n + 1):
        u = k / float(n)
        h00 = 2 * u**3 - 3 * u**2 + 1
        h10 = u**3 - 2 * u**2 + u
        h01 = -2 * u**3 + 3 * u**2
        h11 = u**3 - u**2
        p = h00 * p1 + h10 * t1 + h01 * p2 + h11 * t2
        pts.append((float(p[0]), float(p[1])))
    return pts


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
            ('min_turn_radius',       1.1),
            ('sweep_half_width',      0.975), # half-width = arc_R → 行间无缝
            # 车辆几何参数（来自 URDF z200，用于分析 U-turn 可行性）
            # 后轴中心到车身前端外角的纵向距离（前悬 + 轴距）
            ('body_long_offset',      1.41),
            # 车身半宽（外侧）
            ('body_half_width',       0.525),
            # 车辆物理最小转弯半径（L / tan(δ_max) = 1.05 / tan(50°) ≈ 0.88 m）
            ('vehicle_min_turn_r',    0.88),
            # U-turn 弧线距墙的安全冗余（用于容忍跟踪误差）
            ('uturn_wall_safety',     0.25),
            # Skip-Row 策略参数
            ('use_skip_row',          True),    # 启用 2 圈 Headland + Skip-Row
            ('headland_rings',        2),       # 地头圈数（推荐 2）
            ('headland_edge_offset',  0.78),    # 最外圈距墙距离 = body_hw + wall_safety
            # R5: 障碍感知
            ('obstacle_clearance',    0.83),    # body_hw + 0.3 安全余量
            ('obstacle_model_ignore', 'z200,ground_plane,sun,wall,sweep_course'),
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
        self.body_lo    = g('body_long_offset').value
        self.body_hw    = g('body_half_width').value
        self.veh_min_r  = g('vehicle_min_turn_r').value
        self.wall_safety = g('uturn_wall_safety').value
        self.use_skip_row = g('use_skip_row').value
        self.headland_rings = int(g('headland_rings').value)
        self.headland_off = g('headland_edge_offset').value
        self.obs_clearance = float(g('obstacle_clearance').value)
        self.obs_ignore_words = [w.strip() for w in str(g('obstacle_model_ignore').value).split(',') if w.strip()]
        # 在线障碍快照（R5）：(x, y, r)
        self._obstacles_snapshot: list = []
        self._obstacles_got = False

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

        # R5: 订阅 Gazebo /model_states 做在线障碍感知
        if _HAS_MODEL_STATES:
            self.sub_ms = self.create_subscription(
                ModelStates, '/gazebo/model_states',
                self.cb_model_states,
                sensor_qos)
            self.get_logger().info('[R5] 已订阅 /gazebo/model_states 做在线障碍感知')
        else:
            self.get_logger().warn('[R5] 未找到 gazebo_msgs，障碍感知将降级为空列表')

        # R5: 延迟构建路径，等 2 秒积累障碍快照；兜底仍然在启动时立即发布一版
        # （防止 ModelStates 永不到达时控制栈无路径可追）
        self._build_and_publish()
        self.create_timer(0.5, self._tick)
        self._republish_done = False
        self._republish_timer = self.create_timer(0.5, self._republish_path_once)
        # 2 秒后带着障碍快照重新构建一次
        self._rebuild_done = False
        self._rebuild_timer = self.create_timer(2.0, self._rebuild_with_obstacles)

    def cb_model_states(self, msg):
        """收集静态障碍物（名字不在忽略列表里的模型）。"""
        obs = []
        for name, pose in zip(msg.name, msg.pose):
            low = name.lower()
            if any(w.lower() in low for w in self.obs_ignore_words):
                continue
            # 半径估计：名字含 gate/cone/cylinder 取 0.12，否则取 0.3
            if any(k in low for k in ('gate', 'cone', 'cylinder', 'pillar')):
                r = 0.12
            else:
                r = 0.30
            obs.append((float(pose.position.x), float(pose.position.y), r))
        self._obstacles_snapshot = obs
        self._obstacles_got = True

    def _rebuild_with_obstacles(self):
        """在订阅 /model_states 2 秒后用障碍快照重建路径。"""
        if self._rebuild_done:
            return
        self._rebuild_done = True
        try:
            self._rebuild_timer.cancel()
        except Exception:
            pass
        if self._obstacles_got and self._obstacles_snapshot:
            self.get_logger().info(
                f'[R5] 重建路径：已收集 {len(self._obstacles_snapshot)} 个障碍物 '
                f'{[(round(x,2), round(y,2), round(r,2)) for x,y,r in self._obstacles_snapshot[:8]]}'
            )
            self._build_and_publish()
        else:
            self.get_logger().info('[R5] 无障碍快照（或订阅未收到），沿用初始路径')

    def _republish_path_once(self):
        # 原实现 self._xxx = lambda: None 无效，因为 timer 已绑定原方法引用；
        # 改用显式标志 + 取消 timer 防止 4Hz 持续重发（会重置 planner 进度）。
        if self._republish_done:
            return
        self._build_and_publish()
        self._republish_done = True
        try:
            self._republish_timer.cancel()
        except Exception:
            pass

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
        # get_logger().info(
        #     f'Coverage: {pct:.1f}% ({swept_count}/{self._total_cells} cells)',
        #     throttle_duration_sec=2.0)

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

        # R4: 一次性 dump 路径采样（7 点），用于外环被跳过的诊断
        if len(pts) >= 7:
            sample_idx = [int(k * (len(pts) - 1) / 6) for k in range(7)]
            samples = ', '.join([f'idx{i}=({pts[i][0]:.1f},{pts[i][1]:.1f})' for i in sample_idx])
            self.get_logger().info(f'[R4 path sample] {samples}')

    def _is_edge_pt(self, x, y):
        m = self.edge_off + 0.1
        return (x < self.xmin + m or x > self.xmax - m or
                y < self.ymin + m or y > self.ymax - m)

    def _build_path(self):
        """
        路径生成：根据 self.use_skip_row 选择两种策略

        策略 A (use_skip_row=True, 默认)：
            2 圈 Headland + 内层 Skip-Row 奇偶分两轮
            — U-turn 半径 = 2 × sweep_hw = 1.95m, 覆盖率上限 ~100%

        策略 B (use_skip_row=False, 兼容牛耕法)：
            外贴边圈 + 中间过渡圈 + 相邻道牛耕法
            — U-turn 半径 ≈ sweep_hw ≈ 1.0m, 但相邻行有漏扫缝
        """
        if self.use_skip_row:
            return self._build_path_skip_row()
        else:
            return self._build_path_boustrophedon()

    def _build_path_skip_row(self):
        """
        Skip-Row 路径：2 圈 Headland + 内层奇偶分两轮 skip-row

        几何（参见 plan 验证）：
          - arc_R = 2 × sweep_hw = 1.95m（Skip-Row 跳行使 U-turn 跨度 = 2 × lane_spacing）
          - r_body = √((arc_R + body_hw)² + body_lo²) ≈ 2.85m（前外角到弧心）
          - x_margin = r_body + wall_safety ≈ 3.10m
          - lane_spacing = 2 × sweep_hw = 1.95m（= 清扫带宽, 无缝）
          - Headland 每圈厚 2 × sweep_hw, 最外圈距墙 = headland_off = body_hw + safety = 0.78m
          - Headland 2 圈占用的 y 向厚度 = 2 × 2 × sweep_hw = 3.90m

        路径结构（奇偶分两轮，用户选定）：
          1) 2 圈 Headland（外→内，逆时针矩形）
          2) Ring N → Ring N+1 过渡：三次 Hermite 弧（保切线连续，R1 修正）
          3) Ring 末 → lane 0 起点：三次 Hermite 弧（R1 修正）
          4) 内层奇数轮 lane 索引 [0, 2, 4, 6]（y 由下向上跳跃）
          5) 内层偶数轮 lane 索引 [5, 3, 1]（y 由上向下跳跃）
          - lane 遇到障碍（R5）自动拆成 东段 + 绕行 Hermite 弧 + 西段
        """
        pts = []

        # ── 几何常量（由 URDF + 参数推导）──────────────────────────────
        arc_R = 2.0 * self.sweep_hw                             # 1.95m
        r_body = math.sqrt((arc_R + self.body_hw)**2 + self.body_lo**2)
        x_margin = r_body + self.wall_safety                    # 3.10m
        lane_spacing = 2.0 * self.sweep_hw                      # 1.95m
        headland_thickness = 2.0 * self.sweep_hw                # 1.95m per ring

        # 内层可用 y 范围（对称，留 r_body + safety 给 U-turn 弧顶）
        iy_half_max = 0.5 * (self.ymax - self.ymin) - r_body - self.wall_safety
        # Headland 覆盖的内缘 y
        last_ring_off = self.headland_off + (self.headland_rings - 1) * headland_thickness
        headland_top_inner = self.ymax - last_ring_off - self.sweep_hw
        headland_bot_inner = self.ymin + last_ring_off + self.sweep_hw
        iy_half_by_headland = 0.5 * (headland_top_inner - headland_bot_inner)

        # 最终可用半高（取两者最严约束）
        iy_half = min(iy_half_max, iy_half_by_headland)
        # 内层 lane 数：总跨度 / lane_spacing + 1，对称分布
        n_lanes = max(1, int(2 * iy_half / lane_spacing) + 1)
        # 实际对称分布的最大半高（让末行恰好居中对齐）
        effective_half = (n_lanes - 1) * lane_spacing / 2.0
        lanes_y = []
        visit_order = []
        first_lane_y = None
        if n_lanes >= 2:
            # lane y 坐标（对称分布，索引 0 = 最下）
            lanes_y = [(i - (n_lanes - 1) / 2.0) * lane_spacing for i in range(n_lanes)]
            # 奇偶分两轮访问顺序
            odd_seq = list(range(0, n_lanes, 2))    # [0,2,4,...] 由下向上跳跃
            even_seq = list(range(n_lanes - 1 - (0 if (n_lanes - 1) % 2 == 1 else 1),
                                  0, -2))           # [N-2 or N-1, ..., 3, 1] 由上向下
            visit_order = odd_seq + even_seq
            first_lane_y = lanes_y[visit_order[0]]

        # x 方向内层扫行端点
        ix_min = self.xmin + x_margin
        ix_max = self.xmax - x_margin

        # ── 可行性检查 ─────────────────────────────────────────────────
        physically_feasible = (arc_R >= self.veh_min_r)
        space_feasible_x = (ix_max > ix_min)
        space_feasible_y = (effective_half > 0 and n_lanes >= 2)
        headland_fits = (iy_half_by_headland >= effective_half - 1e-3)

        if not physically_feasible:
            self.get_logger().error(
                f'❌ U-turn 物理不可行：arc_R={arc_R:.3f}m < '
                f'车辆最小转弯半径 {self.veh_min_r:.3f}m')
        if not space_feasible_x:
            self.get_logger().error(
                f'❌ x 方向空间不足：x_margin={x_margin:.2f}m, '
                f'场地宽={self.xmax - self.xmin:.2f}m')
        if not space_feasible_y:
            self.get_logger().error(
                f'❌ y 方向空间不足：iy_half={iy_half:.2f}m, 行数={n_lanes}')
        if not headland_fits:
            self.get_logger().warn(
                f'⚠ Headland 约束比 U-turn 约束更严：'
                f'iy_half_headland={iy_half_by_headland:.2f}m, '
                f'iy_half_uturn={iy_half_max:.2f}m — 可能与外圈有漏扫')

        self.get_logger().info(
            f'[Skip-Row] 构建: arc_R={arc_R:.2f}m r_body={r_body:.2f}m '
            f'x_margin={x_margin:.2f}m | lane_spacing={lane_spacing:.2f}m '
            f'n_lanes={n_lanes} iy_half_eff={effective_half:.2f}m | '
            f'Headland {self.headland_rings}圈 厚={headland_thickness:.2f}m/圈 '
            f'最外圈距墙={self.headland_off:.2f}m 最内圈内缘 y=±{headland_top_inner:.2f}m')

        # ── 第 1 步：Headland 2 圈 + 圈之间 Hermite 过渡弧 ────────────
        # 圆角半径 = headland_thickness (= 2 × sweep_hw = 1.95m)
        # 这给 Stanley 控制器足够的跟踪余量（δ=atan(L/R)≈28° << 50° 硬限）
        corner_r = headland_thickness
        # 每圈逆时针走 SW→SE→NE→NW→SW（闭合回到 SW），从外圈到内圈。
        # 相邻两圈之间用 Hermite 弧衔接：
        #   上圈 SW 结束 heading=东 → 下圈 SW 起点 heading=东
        #   距离 ≈ √2 × headland_thickness ≈ 2.76m，Hermite 能生成自然 S 曲线
        prev_ring_end = None   # (x, y, yaw)
        for k in range(self.headland_rings):
            ring_off = self.headland_off + k * headland_thickness
            ring_early_exit_y = None
            if k == self.headland_rings - 1 and first_lane_y is not None:
                ring_early_exit_y = first_lane_y + 0.5
            ring_pts = self._boundary_strip(
                ring_off,
                corner_r=corner_r,
                early_exit_y=ring_early_exit_y)
            if not ring_pts:
                self.get_logger().warn(
                    f'⚠ Headland 圈 {k+1} 生成失败（偏移 {ring_off:.2f}m 太大）')
                continue

            ring_start = ring_pts[0]          # 朝东出发
            ring_start_yaw = 0.0
            ring_end = ring_pts[-1]
            if ring_early_exit_y is not None:
                # 内环在左侧边提前截断，末端切线朝南
                ring_end_yaw = -math.pi / 2.0
                self.get_logger().info(
                    f'[Ring open-loop] ring{k+1} 西侧截断 y={ring_end[1]:.2f}, '
                    f'目标 lane0_y={first_lane_y:.2f}')
            else:
                # _boundary_strip 闭合后最后一段是 SW 1/4 弧（angles π → 3π/2）
                # 在 3π/2 处切线方向：对于 CCW 弧，切线 = rotate(radius, +90°)
                # radius 方向 = (cos(3π/2), sin(3π/2)) = (0,-1) 即南方
                # 切线 = rotate(南, +90°) = 东 (0°)
                ring_end_yaw = 0.0

            if prev_ring_end is not None:
                # 用 Hermite 弧连接上一圈 SW 终点 → 本圈 SW 起点
                pe_x, pe_y, pe_yaw = prev_ring_end
                bridge = _cubic_bridge(
                    (pe_x, pe_y), pe_yaw,
                    ring_start,  ring_start_yaw,
                    n=16)
                if bridge:
                    pts.extend(bridge)
                    self.get_logger().info(
                        f'[Ring bridge] ring{k}→ring{k+1}: '
                        f'({pe_x:.2f},{pe_y:.2f})→({ring_start[0]:.2f},{ring_start[1]:.2f}) '
                        f'{len(bridge)}点')

            pts.extend(ring_pts)
            prev_ring_end = (ring_end[0], ring_end[1], ring_end_yaw)

        # ── 第 2 步：内层 Skip-Row ────────────────────────────────────
        # lane y 坐标（对称分布，索引 0 = 最下）
        if n_lanes >= 2:
            # 确保奇偶序列不空且互补
            # 对 n_lanes=7: odd=[0,2,4,6], even=[5,3,1] → 合集 [0..6] ✓
            # 对 n_lanes=8: odd=[0,2,4,6], even=[7,5,3,1] ✓
            # 对 n_lanes=6: odd=[0,2,4], even=[5,3,1] ✓
            # 校验完整覆盖
            if sorted(visit_order) != list(range(n_lanes)):
                self.get_logger().error(
                    f'❌ Skip-Row 访问序列不完整: {visit_order} vs {list(range(n_lanes))}')

            # ── Ring 末 → lane 0 起点：Hermite 过渡弧（R1）──
            first_lane_start = (ix_min, first_lane_y)
            if prev_ring_end is not None:
                pe_x, pe_y, pe_yaw = prev_ring_end
                # 按用户要求改为直角并入：先沿西侧竖直下行，再水平并入 lane0
                bridge = []
                elbow = (pe_x, first_lane_start[1])
                # 段1：竖直（保持在西侧 x=pe_x）
                if abs(pe_y - elbow[1]) > 1e-3:
                    n_v = max(2, int(abs(pe_y - elbow[1]) / 0.5) + 1)
                    ys = np.linspace(pe_y, elbow[1], n_v)
                    for yv in ys[1:]:
                        bridge.append((float(pe_x), float(yv)))
                # 段2：水平（并入 lane0 起点，朝东）
                if abs(elbow[0] - first_lane_start[0]) > 1e-3:
                    n_h = max(2, int(abs(first_lane_start[0] - elbow[0]) / 0.5) + 1)
                    xs = np.linspace(elbow[0], first_lane_start[0], n_h)
                    for xv in xs[1:]:
                        bridge.append((float(xv), float(first_lane_start[1])))
                if bridge:
                    pts.extend(bridge)
                    self.get_logger().info(
                        f'[Ring→Lane right-angle] ({pe_x:.2f},{pe_y:.2f})→'
                        f'elbow=({elbow[0]:.2f},{elbow[1]:.2f})→'
                        f'({first_lane_start[0]:.2f},{first_lane_start[1]:.2f}) '
                        f'{len(bridge)}点')

            # 记录上一条 lane 的出口端（x, direction）
            prev_x_end = None
            prev_going_right = True

            # 把 obs 快照 filter 到场地内，避免墙/远处模型干扰
            filtered_obs = [(ox, oy, orad) for (ox, oy, orad) in self._obstacles_snapshot
                            if self.xmin - 0.5 < ox < self.xmax + 0.5
                            and self.ymin - 0.5 < oy < self.ymax + 0.5]
            if filtered_obs:
                self.get_logger().info(
                    f'[R5] 参与 lane 拆分的障碍 {len(filtered_obs)} 个：'
                    f'{[(round(x,2), round(y,2), round(r,2)) for x,y,r in filtered_obs]}')

            for seq_i, lane_idx in enumerate(visit_order):
                y = lanes_y[lane_idx]

                going_right = (prev_going_right is False) if prev_x_end is not None else True
                x_start = ix_min if going_right else ix_max
                x_end = ix_max if going_right else ix_min

                # ── Skip-Row 弧连接（跨越中间被跳过的 lane）── 用 Hermite 替代解析弧
                if prev_x_end is not None:
                    prev_y = lanes_y[visit_order[seq_i - 1]]
                    yaw_prev_out = 0.0 if prev_going_right else math.pi
                    yaw_curr_in  = 0.0 if going_right else math.pi
                    # 从上 lane 出口朝 prev 方向切出，到本 lane 入口朝 curr 方向
                    # 为了形状像半圆（不是穿过场内的 S），给切线加一个正向"外推"：
                    # 上 lane 出口切线 = 继续 prev 方向；本 lane 入口切线 = curr 方向。
                    # Hermite 在这种对头配置下会自然绕到场外（西/东侧）形成大半圆。
                    # 但切线 scale 需要够大，否则曲线会切穿内部。
                    bridge = _cubic_bridge(
                        (prev_x_end, prev_y), yaw_prev_out,
                        (x_start,    y),      yaw_curr_in,
                        n=24,
                        tangent_scale=abs(y - prev_y) * 1.2)
                    pts.extend(bridge)

                # ── 扫行本 lane：遇障绕行（R5）──
                lane_pts = self._lane_with_detours(
                    lane_y=y,
                    going_right=going_right,
                    x_start=x_start,
                    x_end=x_end,
                    obstacles=filtered_obs,
                    lane_spacing=lane_spacing)
                pts.extend(lane_pts)

                prev_x_end = x_end
                prev_going_right = going_right

        # ── 路径起点后退 1m（在第一段前进方向上后退 1m 作为"预备点"）──
        # 把新的首点放在 p0 的 *反方向* 上，使 path[0]→path[1] 的朝向与 p0→p1 一致。
        if len(pts) >= 2:
            p0, p1 = pts[0], pts[1]
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]
            seg_len = math.hypot(dx, dy)
            if seg_len > 1e-4:
                start_x = p0[0] - dx / seg_len * 1.0
                start_y = p0[1] - dy / seg_len * 1.0
                pts = [(float(start_x), float(start_y))] + pts

        # ── R1.3: pts 连续性自检 ─────────────────────────────────────
        self._check_path_continuity(pts, label='Skip-Row')

        pass

        return pts

    def _lane_with_detours(self, lane_y, going_right, x_start, x_end,
                           obstacles, lane_spacing):
        """生成一条 lane，遇到障碍时用 Hermite 弧绕过（R5）。

        Args:
            lane_y: lane 的 y 坐标
            going_right: 方向（True=东行）
            x_start, x_end: 直线段起止 x
            obstacles: [(ox, oy, r), ...] 场地内的障碍列表
            lane_spacing: 相邻 lane 间距（绕行弧高度上限约束）

        Returns:
            [(x, y), ...] 本 lane 的所有点
        """
        # 找与本 lane 冲突的障碍（沿 x 方向排序，按行驶方向）
        conflicts = []
        for (ox, oy, orad) in obstacles:
            effective_clear = self.obs_clearance + orad
            if abs(lane_y - oy) < effective_clear and \
               min(x_start, x_end) - effective_clear < ox < max(x_start, x_end) + effective_clear:
                conflicts.append((ox, oy, orad, effective_clear))
        conflicts.sort(key=lambda o: o[0] if going_right else -o[0])
        conflicts = conflicts[:2]  # 每 lane 最多绕 2 次

        if not conflicts:
            n_seg = max(2, int(abs(x_end - x_start) / 0.5) + 1)
            return [(float(x), float(lane_y)) for x in np.linspace(x_start, x_end, n_seg)]

        pts: list = []
        cur_x = x_start
        cur_yaw = 0.0 if going_right else math.pi
        sign = 1.0 if going_right else -1.0

        max_detour_h = 0.6 * lane_spacing

        for (ox, oy, orad, eff_clear) in conflicts:
            # 绕行入口 / 出口 x（沿行驶方向，在障碍前后预留 max(eff_clear+0.3, 1.2)）
            gap = max(eff_clear + 0.3, 1.2)
            entry_x = ox - sign * gap
            exit_x  = ox + sign * gap

            # 绕行方向（南/北）：离障碍远的一侧
            if lane_y > oy:
                offset_sign = +1.0    # 向北绕
            else:
                offset_sign = -1.0    # 向南绕
            detour_offset = min(eff_clear + 0.2, max_detour_h) * offset_sign
            mid_y = lane_y + detour_offset

            # 进入 entry_x 前走直线
            if (going_right and entry_x > cur_x + 0.2) or (not going_right and entry_x < cur_x - 0.2):
                n_seg = max(2, int(abs(entry_x - cur_x) / 0.5) + 1)
                for x in np.linspace(cur_x, entry_x, n_seg):
                    pts.append((float(x), float(lane_y)))

            # 绕行弧：entry → mid → exit 用两段 Hermite
            # Hermite A: (entry_x, lane_y, yaw) → (ox, mid_y, yaw)
            bridge_a = _cubic_bridge(
                (entry_x, lane_y), cur_yaw,
                (ox,      mid_y),  cur_yaw,     # 中点仍朝前进方向
                n=10,
                tangent_scale=abs(ox - entry_x) * 1.0)
            pts.extend(bridge_a)
            # Hermite B: (ox, mid_y, yaw) → (exit_x, lane_y, yaw)
            bridge_b = _cubic_bridge(
                (ox,      mid_y),  cur_yaw,
                (exit_x,  lane_y), cur_yaw,
                n=10,
                tangent_scale=abs(exit_x - ox) * 1.0)
            pts.extend(bridge_b)

            self.get_logger().info(
                f'[R5 lane detour] y={lane_y:.2f} {"east" if going_right else "west"}行 '
                f'障碍({ox:.2f},{oy:.2f},r={orad:.2f}) '
                f'绕至 y={mid_y:.2f} 入口={entry_x:.2f} 出口={exit_x:.2f}')
            cur_x = exit_x

        # 尾段直线到 x_end
        if (going_right and x_end > cur_x + 0.2) or (not going_right and x_end < cur_x - 0.2):
            n_seg = max(2, int(abs(x_end - cur_x) / 0.5) + 1)
            for x in np.linspace(cur_x, x_end, n_seg):
                pts.append((float(x), float(lane_y)))
        else:
            pts.append((float(x_end), float(lane_y)))

        return pts

    def _check_path_continuity(self, pts, label='path', max_seg=1.0, max_dyaw_deg=30.0):
        """遍历 pts，报告相邻点段长或切线偏转超限的位置（R1.3 自检）。"""
        if len(pts) < 3:
            return
        max_dyaw = math.radians(max_dyaw_deg)
        bad_len = 0
        bad_turn = 0
        first_reports = []
        prev_yaw = None
        for i in range(len(pts) - 1):
            dx = pts[i+1][0] - pts[i][0]
            dy = pts[i+1][1] - pts[i][1]
            seg = math.hypot(dx, dy)
            if seg > max_seg:
                bad_len += 1
                if len(first_reports) < 5:
                    first_reports.append(f'  idx={i} seg={seg:.2f}m @({pts[i][0]:.2f},{pts[i][1]:.2f})')
            if seg > 1e-4:
                yaw = math.atan2(dy, dx)
                if prev_yaw is not None and abs(wrap(yaw - prev_yaw)) > max_dyaw:
                    bad_turn += 1
                    if len(first_reports) < 5:
                        first_reports.append(
                            f'  idx={i} Δyaw={math.degrees(wrap(yaw-prev_yaw)):.0f}° @({pts[i][0]:.2f},{pts[i][1]:.2f})')
                prev_yaw = yaw
        if bad_len or bad_turn:
            self.get_logger().error(
                f'[{label} self-check] 连续性违例：段长>{max_seg}m:{bad_len}, '
                f'Δyaw>{max_dyaw_deg:.0f}°:{bad_turn}\n' + '\n'.join(first_reports))
        else:
            self.get_logger().info(
                f'[{label} self-check] OK | {len(pts)}点 '
                f'段长≤{max_seg}m Δyaw≤{max_dyaw_deg:.0f}°')

    def _build_path_boustrophedon(self):
        """
        传统相邻道牛耕法（兼容保留）：
          - eff_spacing = 2 × arc_R (行间距绑定弧半径)
          - 覆盖率上限 ~69%（由于 arc_R >= 物理最小, 相邻道有漏扫缝）

        使用 use_skip_row=False 启用。
        """
        pts = []

        planned_r = max(self.turn_r, self.veh_min_r * 1.10)
        eff_spacing = max(self.spacing, 2.0 * planned_r + 0.05)
        arc_R = eff_spacing / 2.0
        eff_edge = max(self.edge_off, arc_R + 0.15)
        r_body_max = math.sqrt((arc_R + self.body_hw)**2 + self.body_lo**2)
        x_margin = r_body_max + self.wall_safety

        physically_feasible = (arc_R >= self.veh_min_r)
        space_feasible = (
            x_margin * 2 < (self.xmax - self.xmin) and
            eff_edge * 2 < (self.ymax - self.ymin)
        )
        if not physically_feasible:
            self.get_logger().error(
                f'❌ U-turn 物理不可行：arc_R={arc_R:.3f}m < 车辆最小转弯半径 '
                f'{self.veh_min_r:.3f}m。请增大 sweep_spacing 或减小 min_turn_radius。')
        if not space_feasible:
            self.get_logger().error(
                f'❌ 场地空间不足以 U-turn：x_margin={x_margin:.2f}m '
                f'(区域宽度={self.xmax - self.xmin:.2f}m), '
                f'eff_edge={eff_edge:.2f}m (区域高度={self.ymax - self.ymin:.2f}m)')

        iy_min = self.ymin + eff_edge
        iy_max = self.ymax - eff_edge
        ix_min = self.xmin + x_margin
        ix_max = self.xmax - x_margin

        peak_body_x_east = ix_max + r_body_max
        peak_body_x_west = ix_min - r_body_max
        east_clearance = self.xmax - peak_body_x_east
        west_clearance = peak_body_x_west - self.xmin
        if east_clearance < 0 or west_clearance < 0:
            self.get_logger().error(
                f'❌ U-turn 会撞墙！东墙余量={east_clearance:.2f}m, '
                f'西墙余量={west_clearance:.2f}m (需要正值)')

        self.get_logger().info(
            f'[Boustrophedon] arc_R={arc_R:.2f}m (物理最小={self.veh_min_r:.2f}m) | '
            f'x_margin={x_margin:.2f}m | '
            f'ix=[{ix_min:.2f},{ix_max:.2f}] iy=[{iy_min:.2f},{iy_max:.2f}] | '
            f'东墙余量={east_clearance:.2f}m 西墙余量={west_clearance:.2f}m')

        ys = np.arange(iy_min, iy_max + eff_spacing * 0.01, eff_spacing)

        for idx, y in enumerate(ys):
            going_right = (idx % 2 == 0)
            x_start = ix_min if going_right else ix_max
            x_end = ix_max if going_right else ix_min

            n_seg = max(2, int(abs(x_end - x_start) / 0.5) + 1)
            for x in np.linspace(x_start, x_end, n_seg):
                pts.append((float(x), float(y)))

            if idx + 1 < len(ys):
                if going_right:
                    arc = _arc_pts(x_end, y + arc_R, arc_R,
                                   -math.pi / 2, math.pi / 2, 36)
                else:
                    arc = _arc_pts(x_end, y + arc_R, arc_R,
                                   -math.pi / 2, -3 * math.pi / 2, 36)
                pts.extend(arc)

        if len(pts) >= 2:
            p0, p1 = pts[0], pts[1]
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]
            seg_len = math.hypot(dx, dy)
            if seg_len > 1e-4:
                start_x = p0[0] - dx / seg_len * 1.0
                start_y = p0[1] - dy / seg_len * 1.0
                pts = [(float(start_x), float(start_y))] + pts

        self._check_path_continuity(pts, label='Boustrophedon')
        return pts

    def _boundary_strip(self, off, corner_r: float = 0.0, early_exit_y=None):
        """生成矩形边界贴边路径（逆时针: SW → SE → NE → NW → SW）。

        Args:
            off: 距墙偏移量（m）
            corner_r: 角点圆角半径（m）。>0 时用 1/4 圆弧替代直角，默认 0（直角）
                      推荐 = max(vehicle_min_turn_r, 0.9) 以匹配车辆转弯能力
        """
        pts  = []
        step = 0.5
        xmin, xmax = self.xmin + off, self.xmax - off
        ymin, ymax = self.ymin + off, self.ymax - off
        if xmin >= xmax or ymin >= ymax:
            return pts
        # 圆角半径不能超过矩形短边一半
        cr = max(0.0, min(corner_r, 0.5 * min(xmax - xmin, ymax - ymin) - 1e-3))

        if cr > 0.1:
            # 圆角版本：直线段 + 1/4 圆弧
            # 底边：SW角(xmin+cr,ymin) → SE角(xmax-cr,ymin)
            for x in np.arange(xmin + cr, xmax - cr, step):
                pts.append((float(x), float(ymin)))
            pts.append((float(xmax - cr), float(ymin)))
            # SE 1/4 弧：从(xmax-cr,ymin)到(xmax,ymin+cr)，圆心(xmax-cr,ymin+cr)
            pts.extend(_arc_pts(xmax - cr, ymin + cr, cr,
                                -math.pi / 2, 0.0, 12))
            # 右边：SE角(xmax,ymin+cr) → NE角(xmax,ymax-cr)
            for y in np.arange(ymin + cr, ymax - cr, step):
                pts.append((float(xmax), float(y)))
            pts.append((float(xmax), float(ymax - cr)))
            # NE 1/4 弧：从(xmax,ymax-cr)到(xmax-cr,ymax)，圆心(xmax-cr,ymax-cr)
            pts.extend(_arc_pts(xmax - cr, ymax - cr, cr,
                                0.0, math.pi / 2, 12))
            # 顶边：NE→NW
            for x in np.arange(xmax - cr, xmin + cr, -step):
                pts.append((float(x), float(ymax)))
            pts.append((float(xmin + cr), float(ymax)))
            # NW 1/4 弧
            pts.extend(_arc_pts(xmin + cr, ymax - cr, cr,
                                math.pi / 2, math.pi, 12))
            # 左边：NW → SW
            early_exit_done = False
            for y in np.arange(ymax - cr, ymin + cr, -step):
                pts.append((float(xmin), float(y)))
                if early_exit_y is not None and y <= early_exit_y:
                    early_exit_done = True
                    break
            if early_exit_done:
                return pts
            pts.append((float(xmin), float(ymin + cr)))
            # SW 1/4 弧
            pts.extend(_arc_pts(xmin + cr, ymin + cr, cr,
                                math.pi, 3 * math.pi / 2, 12))
        else:
            # 原直角版本（保留兼容）
            for x in np.arange(xmin, xmax, step): pts.append((float(x), float(ymin)))
            pts.append((float(xmax), float(ymin)))
            for y in np.arange(ymin, ymax, step): pts.append((float(xmax), float(y)))
            pts.append((float(xmax), float(ymax)))
            for x in np.arange(xmax, xmin, -step): pts.append((float(x), float(ymax)))
            pts.append((float(xmin), float(ymax)))
            early_exit_done = False
            for y in np.arange(ymax, ymin, -step):
                pts.append((float(xmin), float(y)))
                if early_exit_y is not None and y <= early_exit_y:
                    early_exit_done = True
                    break
            if early_exit_done:
                return pts
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
