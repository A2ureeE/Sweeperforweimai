#!/usr/bin/env python3
"""
Local planner node — 优化版

优化内容（对应评分维度）：
  ① NARROW_GATE（15分）：订阅 /perception/gate_pose，当检测到门时，
    自动生成对准门中心的 5 点直线穿越路径（从当前位置→门前1m→门中心→门后2m），
    替换当前参考路径，确保机器人精确穿越限宽门。
  ② DYNAMIC_AVOID（25分）：计算动态障碍物对机器人的横向威胁方向，
    将参考路径整体横向偏移 detour_shift_m，实现真实轨迹级别的动态绕行。
  ③ 停滞检测（辅助避障）：累积行程版，避免缓慢移动时误报。

Subscribes:
  /coverage/path               nav_msgs/Path
  /odom                        nav_msgs/Odometry
  /perception/obstacle_points  geometry_msgs/PolygonStamped
  /perception/dynamic_obstacles geometry_msgs/PoseArray
  /perception/gate_pose        geometry_msgs/PoseStamped  (新增)
  /behavior/mode               std_msgs/String

Publishes:
  /reference_path              nav_msgs/Path
  /planner/path_progress       std_msgs/Float32
"""
import math
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PolygonStamped, PoseArray
from std_msgs.msg import String, Float32
import tf_transformations as tft
import yaml


def yaw_from_quat(q):
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.declare_parameters('', [
            ('area_x_min',              -12.0),
            ('area_x_max',               14.5),
            ('area_y_min',               -9.0),
            ('area_y_max',                9.5),
            ('wall_filter_margin',        0.5),
            ('map_config_file',          ''),
            ('lookahead_dist',            8.0),
            ('obstacle_inflate',           0.6),
            ('obstacle_detour_range',      0.7),
            ('obstacle_memory_s',          8.0),
            ('gate_align_dist',            4.0),
            ('detour_shift_m',             1.0),
            ('enable_group_detour',      False),
            ('obstacle_group_cluster_dist', 2.0),
            ('obstacle_group_margin_m',    0.8),
            ('obstacle_group_min_count',   3),
            ('stagnation_time_s',          6.0),
            ('stagnation_min_dist_total',  0.5),
            # 避障回切迟滞：考虑车身长度，避免车尾尚未通过障碍就回切原路径
            ('rejoin_tail_clearance_m',    1.5),
            ('rejoin_min_hold_s',          0.8),
            # 穿门路径参数
            ('gate_approach_dist',         1.2),  # 门前等待点距门中心的距离
            ('gate_exit_dist',             2.5),  # 门后目标点距门中心的距离
            ('gate_pose_timeout_s',        5.0),
            ('gate_engage_dist',           6.0),
            ('gate_pass_radius',           2.0),
            # 仅当门位于 coverage 路径附近时，才允许在 COVERAGE 模式下强制穿门
            ('force_gate_near_path_only',  True),
            ('gate_path_max_dist',         1.5),
            # 穿门候选打分：ConvergePath 权重最高
            ('gate_converge_path_weight',  10.0),
            ('gate_robot_dist_weight',     1.0),
            ('gate1_x',                    float('nan')),
            ('gate1_y',                    float('nan')),
            ('gate1_heading',              0.0),
            ('gate2_x',                    float('nan')),
            ('gate2_y',                    float('nan')),
            ('gate2_heading',              0.0),
            # 动态绕行：预测时间窗（秒）
            ('dyn_predict_s',              1.5),
            ('vehicle_width_m',            1.05),
            ('bridge_detour_margin_m',     0.18),
            ('bridge_detour_max_shift_m',  1.05),
        ])
        g = self.get_parameter
        self.area_x_min     = float(g('area_x_min').value)
        self.area_x_max     = float(g('area_x_max').value)
        self.area_y_min     = float(g('area_y_min').value)
        self.area_y_max     = float(g('area_y_max').value)
        self.wall_filter_margin = float(g('wall_filter_margin').value)
        self.map_config_file = str(g('map_config_file').value)
        self.lookahead       = g('lookahead_dist').value
        self.inflate         = g('obstacle_inflate').value
        self.detour_range    = g('obstacle_detour_range').value
        self.obs_mem_s       = g('obstacle_memory_s').value
        self.gate_align      = g('gate_align_dist').value
        self.detour_shift    = g('detour_shift_m').value
        self.enable_group_detour = bool(g('enable_group_detour').value)
        self.obs_group_cluster_dist = float(g('obstacle_group_cluster_dist').value)
        self.obs_group_margin = float(g('obstacle_group_margin_m').value)
        self.obs_group_min_count = int(g('obstacle_group_min_count').value)
        self.stag_time       = g('stagnation_time_s').value
        self.stag_min_d      = g('stagnation_min_dist_total').value
        self.rejoin_tail_clearance = float(g('rejoin_tail_clearance_m').value)
        self.rejoin_min_hold_s = float(g('rejoin_min_hold_s').value)
        self.gate_approach   = g('gate_approach_dist').value
        self.gate_exit       = g('gate_exit_dist').value
        self.gate_pose_timeout_s = float(g('gate_pose_timeout_s').value)
        self.gate_engage_dist = float(g('gate_engage_dist').value)
        self.gate_pass_radius = float(g('gate_pass_radius').value)
        self.force_gate_near_path_only = bool(g('force_gate_near_path_only').value)
        self.gate_path_max_dist = float(g('gate_path_max_dist').value)
        self.gate_converge_path_weight = float(g('gate_converge_path_weight').value)
        self.gate_robot_dist_weight = float(g('gate_robot_dist_weight').value)
        self.dyn_predict_s   = g('dyn_predict_s').value
        self.vehicle_width = float(g('vehicle_width_m').value)
        self.bridge_detour_margin = float(g('bridge_detour_margin_m').value)
        self.bridge_detour_max_shift = float(g('bridge_detour_max_shift_m').value)

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        # coverage/path 是 transient_local 发布；这里同样用 transient_local 订阅，
        # 避免 planner 晚于 coverage 启动时拿不到路径（n=0）。
        path_latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_path      = self.create_subscription(
            Path,            '/coverage/path',                  self.cb_cov,       path_latch_qos)
        self.sub_odom      = self.create_subscription(
            Odometry,        '/odom',                           self.cb_odom,      sensor_qos)
        self.sub_obs       = self.create_subscription(
            PolygonStamped,  '/perception/obstacle_points',     self.cb_obs,       sensor_qos)
        self.sub_dyn       = self.create_subscription(
            PoseArray,       '/perception/dynamic_obstacles',   self.cb_dyn,       sensor_qos)
        self.sub_gate_pose = self.create_subscription(
            PoseStamped,     '/perception/gate_pose',           self.cb_gate_pose, 5)
        self.sub_mode      = self.create_subscription(
            String,          '/behavior/mode',                  self.cb_mode,      5)

        self.pub_ref   = self.create_publisher(Path,    '/reference_path',        5)
        self.pub_prog  = self.create_publisher(Float32, '/planner/path_progress', 5)
        self.pub_detour_end = self.create_publisher(String, '/planner/detour_cleared', 5)

        self.cov_pts: list   = []
        self.robot           = None       # (x, y, yaw)
        self.mode            = 'COVERAGE'
        self.progress_idx    = 0
        self.obs_memory: dict = {}

        # 动态障碍物（用于横向偏移）
        self.dyn_obstacles: list = []   # [(wx, wy, vx, vy, speed), ...]

        # 门位姿（来自 perception）
        self.gate_pose       = None     # (cx, cy, yaw) 或 None
        self._gate_pose_t    = 0.0      # 门位姿时间戳
        self._has_area_bounds = True
        self.known_gates = self._build_known_gates_from_params(g)
        self._load_map_config()
        self._active_gate_id = None
        self._in_gate_path = False
        self._gate_rejoin_path: list = []
        self._gate_rejoin_target_idx = 0

        # 停滞检测
        self._stag_accum_d   = 0.0
        self._stag_start_t   = time.time()
        self._last_stag_pos  = None

        # 避障后回归原路径
        self._prev_mode             = 'COVERAGE'
        self._detour_start_idx      = None
        self._returning_from_detour = False
        self._detour_clear_pending  = False
        self._detour_exit_t         = 0.0

        # R4: 启动时间戳，用于初始强制 progress_idx=0
        # controller_node 延迟 10s 启动（launch 里 delayed_controller period=10），
        # 在此之前车辆不会移动。warmup 必须覆盖这段时间，否则 stagnation
        # 检测会在 6s 时把 progress 跳到 +20，跳过整个外环。
        self._t_start = time.time()
        self._warmup_sec = 12.0  # 覆盖 controller 延迟 10s + 2s 缓冲
        self._warmup_log_done = False
        # R4: progress 跳变日志（只记录启动后 20 秒内的变化）
        self._last_logged_progress = -1

        self.create_timer(0.2, self.tick)
        self.create_timer(5.0, self._log_status)
        self.get_logger().info('planner_node ready (gate + dynamic detour enabled)')

    # ── 回调 ────────────────────────────────────────────────────────────
    def cb_cov(self, msg: Path):
        self.cov_pts       = [(p.pose.position.x, p.pose.position.y)
                              for p in msg.poses]
        if (not self._has_area_bounds) and self.cov_pts:
            xs = [p[0] for p in self.cov_pts]
            ys = [p[1] for p in self.cov_pts]
            self.area_x_min = min(xs) - 0.5
            self.area_x_max = max(xs) + 0.5
            self.area_y_min = min(ys) - 0.5
            self.area_y_max = max(ys) + 0.5
            self._has_area_bounds = True
            self.get_logger().warn(
                f'未从地图文件读取到边界，使用路径推断边界: '
                f'x=[{self.area_x_min:.1f},{self.area_x_max:.1f}] '
                f'y=[{self.area_y_min:.1f},{self.area_y_max:.1f}]')
        self.progress_idx  = 0
        self._stag_accum_d = 0.0
        self._stag_start_t = time.time()
        self._last_stag_pos = None
        self._refresh_gate_path_distances()
        # 门只需要穿过一次：不要在路径刷新时重置 passed 状态。
        # 否则 coverage_node 重发 /coverage/path 后会再次要求穿门。
        if self._active_gate_id is not None:
            if all(g['id'] != self._active_gate_id or g['passed'] for g in self.known_gates):
                self._active_gate_id = None
        # get_logger().info(f'Coverage path received: {len(self.cov_pts)} pts')

    def cb_odom(self, msg: Odometry):
        self.robot = (msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      yaw_from_quat(msg.pose.pose.orientation))

    def cb_obs(self, msg: PolygonStamped):
        now = time.time()
        for p in msg.polygon.points:
            if self._is_wall_point(float(p.x), float(p.y)):
                continue
            k = (round(float(p.x), 1), round(float(p.y), 1))
            self.obs_memory[k] = now + self.obs_mem_s
        expired = [k for k, t in self.obs_memory.items() if t < now]
        for k in expired:
            del self.obs_memory[k]

    def cb_dyn(self, msg: PoseArray):
        out = []
        for p in msg.poses:
            yaw = tft.euler_from_quaternion(
                [p.orientation.x, p.orientation.y,
                 p.orientation.z, p.orientation.w])[2]
            sp = float(p.position.z)
            out.append((float(p.position.x), float(p.position.y),
                        sp * math.cos(yaw), sp * math.sin(yaw), sp))
        self.dyn_obstacles = out

    def cb_gate_pose(self, msg: PoseStamped):
        self.gate_pose  = (msg.pose.position.x,
                           msg.pose.position.y,
                           yaw_from_quat(msg.pose.orientation))
        self._gate_pose_t = time.time()

    def cb_mode(self, msg: String):
        raw = msg.data
        self.mode = raw.split('_')[0] if raw.startswith('EDGE_FOLLOW') else raw

    # ── 主循环 ──────────────────────────────────────────────────────────
    def tick(self):
        if self.robot is None:
            return

        rx, ry, ryaw = self.robot
        now = time.time()

        # ── 门位姿过期清理 ──
        if self.gate_pose and now - self._gate_pose_t > self.gate_pose_timeout_s:
            self.gate_pose = None

        self._update_gate_state(rx, ry)
        gate_target = self._select_gate_target(rx, ry)
        if gate_target is not None:
            gcx, gcy, gyaw, src = gate_target
            path = self._build_gate_path(rx, ry, (gcx, gcy, gyaw), src)
            if path:
                self._in_gate_path = True
                self._publish_path(path)
                return
        if self._in_gate_path:
            self._in_gate_path = False
            self._detour_clear_pending = False
            self._detour_start_idx = None
            self._returning_from_detour = False
            self._stag_accum_d = 0.0
            self._stag_start_t = now
            self._last_stag_pos = None
            if self.cov_pts:
                n_cov = len(self.cov_pts)
                rejoin_idx = self._find_nearest_cov_idx(
                    rx, ry, span_back=n_cov, span_fwd=n_cov)
                safe_rejoin_idx = self._advance_to_safe_rejoin_idx(
                    rejoin_idx,
                    clear_threshold=self.rejoin_tail_clearance,
                    search_ahead=80,
                    min_advance=12,
                )
                if safe_rejoin_idx != rejoin_idx:
                    self.get_logger().info(
                        f'穿门回归前推: idx {rejoin_idx}→{safe_rejoin_idx} '
                        f'(clear>={self.rejoin_tail_clearance:.2f}m)')
                rejoin_idx = safe_rejoin_idx
                self.progress_idx = rejoin_idx
                self._gate_rejoin_target_idx = rejoin_idx
                bridge = self._build_rejoin_bridge(rx, ry, ryaw, rejoin_idx)
                self._gate_rejoin_path = bridge
                self.get_logger().info(
                    f'穿门结束，生成回归路径 bridge={len(bridge)}pts → '
                    f'cov idx={rejoin_idx}')

        if self._gate_rejoin_path:
            d_to_target = 999.0
            if self.cov_pts and self._gate_rejoin_target_idx < len(self.cov_pts):
                tx, ty = self.cov_pts[self._gate_rejoin_target_idx]
                d_to_target = math.hypot(rx - tx, ry - ty)
            if d_to_target < 1.5:
                self._gate_rejoin_path = []
                self.get_logger().info(
                    f'回归路径跟踪完成, dist={d_to_target:.2f}m')
            else:
                self._publish_path(self._gate_rejoin_path)
                return

        if not self.cov_pts:
            return

        n   = len(self.cov_pts)
        pts = np.array(self.cov_pts)

        # ── 避障模式切换检测：保存/恢复路径位置 ──────────────────────
        detour_modes = ('STATIC_DETOUR', 'DYNAMIC_AVOID')
        entering_detour = (self.mode in detour_modes
                           and self._prev_mode not in detour_modes)
        leaving_detour  = (self.mode not in detour_modes
                           and self._prev_mode in detour_modes)

        if entering_detour and self._detour_start_idx is None:
            self._detour_start_idx = self.progress_idx
            self._detour_clear_pending = False
            self.get_logger().info(
                f'进入避障模式，保存路径位置 idx={self.progress_idx}')
        if self.mode in detour_modes:
            self._detour_start_idx = self.progress_idx

        if leaving_detour and self._detour_start_idx is not None:
            # 进入“待清障”阶段：不要立刻回切原路径，要等车尾也越过障碍。
            # 否则车头刚过杆子就回切，尾部会扫到杆子。
            self._detour_clear_pending = True
            self._detour_exit_t = now
            self.get_logger().info(
                f'避障结束，进入尾部清障等待: idx={self.progress_idx}')

        # 尾部清障判定：只有满足“时间 + 距离”双条件才发送 cleared。
        if self._detour_clear_pending:
            self._detour_start_idx = self.progress_idx
            min_obs_d = self._nearest_obstacle_distance(rx, ry)
            hold_s = now - self._detour_exit_t
            if hold_s >= self.rejoin_min_hold_s and min_obs_d >= self.rejoin_tail_clearance:
                rejoin_idx = self._find_nearest_cov_idx(rx, ry, span_back=30, span_fwd=120)
                safe_rejoin_idx = self._advance_to_safe_rejoin_idx(
                    rejoin_idx,
                    clear_threshold=self.rejoin_tail_clearance,
                    search_ahead=60,
                    min_advance=6,
                )
                if safe_rejoin_idx != rejoin_idx:
                    self.get_logger().info(
                        f'避障回切前推: idx {rejoin_idx}→{safe_rejoin_idx} '
                        f'(clear>={self.rejoin_tail_clearance:.2f}m)')
                rejoin_idx = safe_rejoin_idx
                self.get_logger().info(
                    f'避障回切放行: hold={hold_s:.1f}s, min_obs={min_obs_d:.2f}m '
                    f'>= {self.rejoin_tail_clearance:.2f}m, rejoin_idx={rejoin_idx}')
                self._detour_clear_pending = False
                self.progress_idx = rejoin_idx
                self._detour_start_idx = rejoin_idx
                self._returning_from_detour = True
                # 通知 behavior_node 和 controller：避障结束，可以恢复路径跟踪
                self.pub_detour_end.publish(String(data='cleared'))
            else:
                self.get_logger().info(
                    f'避障回切等待: hold={hold_s:.1f}s, min_obs={min_obs_d:.2f}m',
                    throttle_duration_sec=1.0)

        self._prev_mode = self.mode

        # ── R4: 启动 warmup，前 N 秒强制 progress_idx=0 ─────────────────
        warmup_elapsed = now - self._t_start
        in_warmup = warmup_elapsed < self._warmup_sec
        if in_warmup:
            self.progress_idx = 0
            self._stag_accum_d = 0.0
            self._stag_start_t = now
            if not self._warmup_log_done:
                self.get_logger().info(
                    f'[R4 warmup] 前 {self._warmup_sec:.1f}s 强制 progress_idx=0, '
                    f'pos=({rx:.1f},{ry:.1f}), 路径前 3 点='
                    f'{[(round(float(pts[i][0]),2), round(float(pts[i][1]),2)) for i in range(min(3, n))]}')
                self._warmup_log_done = True

        # ── 前向滑动窗口推进 ────────────────────────────────────────────
        old_idx = self.progress_idx
        if in_warmup:
            # warmup 期间不推进 progress，直接发布头部参考窗口
            nearest_dist = 0.0  # 占位
        elif self.progress_idx == 0 and not self._returning_from_detour:
            # 刚结束 warmup，从 idx=0 开始逐步推进
            window_end = min(n - 1, self.progress_idx + 10)
            sub = pts[self.progress_idx: window_end + 1]
            d2 = np.sum((sub - np.array([rx, ry])) ** 2, axis=1)
            nearest_in_window = int(np.argmin(d2))
            nearest_dist = math.sqrt(float(d2[nearest_in_window]))
            # 只有距离 < 3m 才用 argmin 推进；否则每 tick 推 1 点（等车开到附近）
            if nearest_dist < 3.0:
                self.progress_idx = max(self.progress_idx, self.progress_idx + nearest_in_window)
            else:
                self.progress_idx = min(n - 1, self.progress_idx + 1)
            self.get_logger().debug(f'[DEBUG] 分支1: idx 0→{self.progress_idx}, dist={nearest_dist:.1f}m, pos=({rx:.1f},{ry:.1f})')
        elif self._returning_from_detour:
            window_end = min(n - 1, self.progress_idx + 50)
            sub  = pts[self.progress_idx: window_end + 1]
            d2   = np.sum((sub - np.array([rx, ry])) ** 2, axis=1)
            nearest_in_window = int(np.argmin(d2))
            new_idx = self.progress_idx + nearest_in_window
            nearest_dist = math.sqrt(d2[nearest_in_window])
            self.progress_idx = max(self.progress_idx, new_idx)
            if nearest_dist < 1.5:
                self._returning_from_detour = False
                self.get_logger().info(
                    f'已回到原路径, dist={nearest_dist:.2f}m')
        else:
            # 小窗口 argmin：限制为 +3 点，确保不跳过 U-turn 弧桥。
            # tick=0.2s, v_max≈0.6m/s → 每 tick 行进 ≈0.12m；
            # 路径点间距 ~0.25-0.5m，+3 已覆盖所有合理推进。
            MAX_STEP = 3
            window_end = min(n - 1, self.progress_idx + MAX_STEP)
            sub  = pts[self.progress_idx: window_end + 1]
            d2   = np.sum((sub - np.array([rx, ry])) ** 2, axis=1)
            argmin_rel = int(np.argmin(d2))
            nearest_dist = math.sqrt(float(d2[argmin_rel]))
            if nearest_dist < 3.0:
                new_idx = self.progress_idx + argmin_rel
            else:
                new_idx = min(n - 1, self.progress_idx + 1)
            old_progress = self.progress_idx
            self.progress_idx = max(self.progress_idx, new_idx)

        # R4 诊断：启动后 20 秒内每次 progress_idx 变化都打印一行
        if warmup_elapsed < 20.0 and self.progress_idx != self._last_logged_progress:
            self.get_logger().info(
                f'[progress_jump] {old_idx}→{self.progress_idx} '
                f'dist={nearest_dist:.2f}m t={warmup_elapsed:.2f}s '
                f'pos=({rx:.1f},{ry:.1f})')
            self._last_logged_progress = self.progress_idx

        # ── 本地重同步：progress 点离车过远时，按邻域最近点修正 ────────────
        # 目的：避免 progress 与车辆位置脱钩导致角点“反方向打舵”。
        if n > 0:
            px, py = pts[self.progress_idx]
            d_prog = math.hypot(rx - float(px), ry - float(py))
            if d_prog > 4.0:
                # 前向搜索上界：用路径折返检测（不跨越 U-turn）
                hi_fwd = self.progress_idx
                _rs_x0 = float(pts[self.progress_idx][0])
                _rs_y0 = float(pts[self.progress_idx][1])
                _rs_dist = 0.0
                _rs_max_chord = 0.0
                _rs_max_arc = 0.0
                while hi_fwd + 1 < n and hi_fwd < self.progress_idx + 80:
                    _dx_r = float(pts[hi_fwd + 1][0] - pts[hi_fwd][0])
                    _dy_r = float(pts[hi_fwd + 1][1] - pts[hi_fwd][1])
                    _rs_dist += math.hypot(_dx_r, _dy_r)
                    hi_fwd += 1
                    _rs_chord = math.hypot(float(pts[hi_fwd][0]) - _rs_x0,
                                           float(pts[hi_fwd][1]) - _rs_y0)
                    if _rs_chord > _rs_max_chord:
                        _rs_max_chord = _rs_chord
                        _rs_max_arc = _rs_dist
                    if (_rs_max_chord > 2.0
                            and _rs_chord < _rs_max_chord * 0.5
                            and _rs_dist > _rs_max_arc + 1.0):
                        break
                sub_fwd = pts[self.progress_idx:hi_fwd + 1]
                d2_fwd = np.sum((sub_fwd - np.array([rx, ry])) ** 2, axis=1)
                rel_fwd = int(np.argmin(d2_fwd))
                fwd_dist = math.sqrt(float(d2_fwd[rel_fwd]))
                new_idx = self.progress_idx + rel_fwd

                if fwd_dist > 4.0:
                    lo_bwd = max(0, self.progress_idx - 20)
                    sub_bwd = pts[lo_bwd:self.progress_idx]
                    if len(sub_bwd) > 0:
                        d2_bwd = np.sum((sub_bwd - np.array([rx, ry])) ** 2, axis=1)
                        rel_bwd = int(np.argmin(d2_bwd))
                        bwd_dist = math.sqrt(float(d2_bwd[rel_bwd]))
                        if bwd_dist < fwd_dist:
                            new_idx = lo_bwd + rel_bwd
                            self.get_logger().warn(
                                f'[progress_resync] backward fallback '
                                f'idx {self.progress_idx}→{new_idx} '
                                f'd_fwd={fwd_dist:.1f}m d_bwd={bwd_dist:.1f}m '
                                f'pos=({rx:.1f},{ry:.1f})')

                new_idx = max(self.progress_idx - 20, new_idx)

                self.get_logger().warn(
                    f'[progress_resync] idx {self.progress_idx}→{new_idx} '
                    f'd={d_prog:.1f}m pos=({rx:.1f},{ry:.1f})')
                self.progress_idx = new_idx

        # ── 停滞检测 ────────────────────────────────────────────────
        if self._last_stag_pos is not None:
            step = math.hypot(rx - self._last_stag_pos[0],
                              ry - self._last_stag_pos[1])
            self._stag_accum_d += step
        self._last_stag_pos = (rx, ry)

        elapsed = now - self._stag_start_t
        if elapsed >= self.stag_time:
            if (self._stag_accum_d < self.stag_min_d
                    and self.mode in ('COVERAGE', 'STATIC_DETOUR')):
                jump = min(n - 1, self.progress_idx + 20)
                self.get_logger().warn(
                    f'停滞 {elapsed:.1f}s / {self._stag_accum_d:.2f}m — '
                    f'跳至 idx {self.progress_idx}→{jump}')
                self.progress_idx = jump
            self._stag_accum_d = 0.0
            self._stag_start_t = now

        prog = float(self.progress_idx) / max(1, n - 1)
        self.pub_prog.publish(Float32(data=prog))

        # ── 截取参考窗口 ─────────────────────────────────────────────
        # 距离约束 + 路径折返检测：
        #   1) 沿路径累积距离达到 lookahead 时截断
        #   2) 当路径上某点离起点的直线距离开始"回缩"（折返），说明
        #      路径正在 U-turn 折回，继续暴露会让 controller 跳线。
        #      折返条件：当前点到起点距离 < 已见最远距离 × 0.5 且已走超
        #      过最远距离的一半弧长（避免在 bridge 起步微小抖动误判）。
        start_x = float(pts[self.progress_idx][0])
        start_y = float(pts[self.progress_idx][1])
        end_idx  = self.progress_idx
        dist_acc = 0.0
        max_chord = 0.0
        max_chord_arc = 0.0
        while end_idx + 1 < n:
            dx = pts[end_idx + 1][0] - pts[end_idx][0]
            dy = pts[end_idx + 1][1] - pts[end_idx][1]
            seg_len = math.hypot(dx, dy)
            dist_acc += seg_len
            end_idx  += 1
            chord = math.hypot(float(pts[end_idx][0]) - start_x,
                               float(pts[end_idx][1]) - start_y)
            if chord > max_chord:
                max_chord = chord
                max_chord_arc = dist_acc
            if dist_acc >= self.lookahead:
                break
            if (max_chord > 2.0
                    and chord < max_chord * 0.5
                    and dist_acc > max_chord_arc + 1.0):
                break

        slice_pts = [tuple(p) for p in pts[self.progress_idx: end_idx + 1]]
        if len(slice_pts) < 2:
            # 路径走完——从头循环（确保覆盖全场地）
            self.progress_idx = 0
            self._stag_accum_d = 0.0
            self._stag_start_t = time.time()
            self.get_logger().info('路径遍历完成，从头循环覆盖...')
            return

        # ── 静态障碍物侧向推开 ──────────────────────────────────────
        if self.mode in ('COVERAGE', 'STATIC_DETOUR'):
            used_structured_detour = False
            if self.enable_group_detour:
                group_detour = self._plan_obstacle_group_detour(slice_pts)
                if group_detour is not None:
                    slice_pts, group_n, group_shift = group_detour
                    used_structured_detour = True
                    self.get_logger().info(
                        f'[GROUP_DETOUR] obstacles={group_n} shift={group_shift:.2f}m '
                        f'progress_idx={self.progress_idx}',
                        throttle_duration_sec=0.5)
            pre_detour = slice_pts[:]
            bridge_detour = self._plan_bridge_detour(slice_pts, rx, ry, ryaw)
            if bridge_detour is not None:
                slice_pts, hit_obs, hit_clear, min_clear = bridge_detour
                used_structured_detour = True
                self.get_logger().info(
                    f'[BRIDGE_DETOUR] obstacle=({hit_obs[0]:.1f},{hit_obs[1]:.1f}) '
                    f'nearest={hit_clear:.2f}m min_clear={min_clear:.2f}m '
                    f'progress_idx={self.progress_idx}',
                    throttle_duration_sec=0.5)
            if not used_structured_detour:
                slice_pts = self._apply_detour(slice_pts)
            max_shift = 0.0
            for i in range(len(slice_pts)):
                sh = math.hypot(slice_pts[i][0] - pre_detour[i][0],
                                slice_pts[i][1] - pre_detour[i][1])
                if sh > max_shift:
                    max_shift = sh
            if max_shift > 0.05:
                self.get_logger().info(
                    f'[DETOUR_DIAG] progress_idx={self.progress_idx} end_idx={end_idx} '
                    f'窗口{len(slice_pts)}点 最大偏移={max_shift:.2f}m '
                    f'障碍数={len(self.obs_memory)} '
                    f'pos=({rx:.1f},{ry:.1f}) 模式={self.mode}',
                    throttle_duration_sec=0.5)

        # ── 动态障碍物横向整体偏移 ───────────────────────────────────
        if self.mode == 'DYNAMIC_AVOID':
            slice_pts = self._apply_dynamic_detour(slice_pts, rx, ry, ryaw)

        self._publish_path(slice_pts)

    # ── 5 秒状态日志 ─────────────────────────────────────────────────────
    def _log_status(self):
        if self.robot is None:
            return
        rx, ry, ryaw = self.robot
        n = len(self.cov_pts)
        pct = self.progress_idx / max(1, n - 1) * 100 if n > 1 else 0
        # 计算到当前 progress 点的距离，辅助判断路径跟踪精度
        if n > 0 and self.progress_idx < n:
            pp = self.cov_pts[self.progress_idx]
            d_to_prog = math.hypot(rx - pp[0], ry - pp[1])
        else:
            d_to_prog = -1.0
        self.get_logger().info(
            f'[规划] 模式={self.mode} | '
            f'路径进度={self.progress_idx}/{n} ({pct:.1f}%) | '
            f'静态障碍记忆={len(self.obs_memory)}个 '
            f'动态障碍={len(self.dyn_obstacles)}个 | '
            f'窄门={"检测到" if self.gate_pose else "无"} '
            f'剩余门={len([g for g in self.known_gates if not g["passed"]])} | '
            f'停滞累积={self._stag_accum_d:.2f}m | '
            f'位置=({rx:.1f},{ry:.1f}) 航向={math.degrees(ryaw):.0f}° '
            f'到prog点={d_to_prog:.1f}m')

    # ── 穿门路径生成 ─────────────────────────────────────────────────────
    def _build_gate_path_points(self, rx, ry, gate_pose):
        """
        生成 5 点穿门路径（不记录日志），用于候选评估与碰撞检查。
        """
        gcx, gcy, gate_yaw = gate_pose
        gate_to_robot = math.atan2(gcy - ry, gcx - rx)
        if abs(wrap(gate_yaw - gate_to_robot)) > math.pi * 0.5:
            gate_yaw = wrap(gate_yaw + math.pi)
        cos_y = math.cos(gate_yaw)
        sin_y = math.sin(gate_yaw)

        # 5个路径点
        approach_d = self.gate_approach
        exit_d     = self.gate_exit
        pre_x = gcx - cos_y * approach_d
        pre_y = gcy - sin_y * approach_d
        post_x = gcx + cos_y * exit_d
        post_y = gcy + sin_y * exit_d
        pts = [
            (rx, ry),                                         # 0: 当前位置
            ((rx + pre_x) * 0.5, (ry + pre_y) * 0.5),        # 1: 门前引导
            (gcx, gcy),                                       # 2: 门中心
            ((gcx + post_x) * 0.5, (gcy + post_y) * 0.5),    # 3: 门后半程
            (post_x, post_y),                                # 4: 完全穿越
        ]
        # 检查路径有效性（避免退行）
        for i in range(len(pts) - 1):
            dx = pts[i+1][0] - pts[i][0]
            dy = pts[i+1][1] - pts[i][1]
            if math.hypot(dx, dy) < 0.05:
                return None
        return pts, gate_yaw

    def _build_gate_path(self, rx, ry, gate_pose, source: str = 'perception') -> list:
        """
        生成 5 点穿门路径：
          当前位置 → 门前等待点 → 门中心 → 门后出口点 → 更远目标
        以门朝向为基准，强制穿过门中心。
        """
        out = self._build_gate_path_points(rx, ry, gate_pose)
        if out is None:
            return None
        pts, gate_yaw = out
        gcx, gcy, _ = gate_pose
        self.get_logger().info(
            f'穿门路径[{source}]: 门中心({gcx:.1f},{gcy:.1f}), 门朝向={math.degrees(gate_yaw):.0f}°',
            throttle_duration_sec=1.0)
        return pts

    def _update_gate_state(self, rx: float, ry: float):
        for gate in self.known_gates:
            if gate['passed']:
                continue
            if math.hypot(rx - gate['x'], ry - gate['y']) <= self.gate_pass_radius:
                gate['passed'] = True
                if self._active_gate_id == gate['id']:
                    self._active_gate_id = None
                self.get_logger().info(f'Gate {gate["id"]} 已穿过')

    def _distance_to_path_range(self, px: float, py: float, start_idx: int, end_idx: int) -> float:
        if len(self.cov_pts) < 2:
            return float('inf')
        start_idx = max(0, start_idx)
        end_idx = min(len(self.cov_pts) - 1, end_idx)
        if end_idx - start_idx < 1:
            return float('inf')
        best = float('inf')
        for i in range(start_idx, end_idx):
            ax, ay = self.cov_pts[i]
            bx, by = self.cov_pts[i + 1]
            vx = bx - ax
            vy = by - ay
            seg2 = vx * vx + vy * vy
            if seg2 <= 1e-9:
                qx, qy = ax, ay
            else:
                t = ((px - ax) * vx + (py - ay) * vy) / seg2
                t = max(0.0, min(1.0, t))
                qx = ax + t * vx
                qy = ay + t * vy
            d = math.hypot(px - qx, py - qy)
            if d < best:
                best = d
        return best

    def _distance_to_cov_path(self, px: float, py: float) -> float:
        return self._distance_to_path_range(px, py, 0, len(self.cov_pts) - 1)

    def _distance_to_converge_path(self, px: float, py: float) -> float:
        if not self.cov_pts:
            return float('inf')
        n = len(self.cov_pts)
        # ConvergePath: 以当前 progress 为中心的局部跟踪段，而非全局 coverage。
        lo = max(0, self.progress_idx - 30)
        hi = min(n - 1, self.progress_idx + 140)
        return self._distance_to_path_range(px, py, lo, hi)

    def _refresh_gate_path_distances(self):
        if not self.known_gates:
            return
        for gate in self.known_gates:
            gate['path_dist'] = self._distance_to_cov_path(gate['x'], gate['y'])
            if math.isfinite(gate['path_dist']):
                self.get_logger().info(
                    f'Gate {gate["id"]} 到 coverage 路径最近距离: '
                    f'{gate["path_dist"]:.2f}m')

    def _is_gate_path_collision_free(self, rx: float, ry: float, gate_pose) -> bool:
        out = self._build_gate_path_points(rx, ry, gate_pose)
        if out is None:
            return False
        pts, _ = out
        if not self.obs_memory and not self.dyn_obstacles:
            return True

        static_safe = max(0.7, self.inflate + 0.5 * self.vehicle_width)
        dynamic_safe = static_safe + 0.3
        samples = []
        for i in range(len(pts) - 1):
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            seg = math.hypot(bx - ax, by - ay)
            n = max(2, int(seg / 0.25) + 1)
            for k in range(n):
                t = k / (n - 1)
                sx = ax + (bx - ax) * t
                sy = ay + (by - ay) * t
                # 近似未来时刻，用于动态障碍预测
                p = (i + t) / max(1.0, float(len(pts) - 1))
                samples.append((sx, sy, p * self.dyn_predict_s))

        for sx, sy, pred_t in samples:
            for ox, oy in self.obs_memory.keys():
                if math.hypot(sx - ox, sy - oy) < static_safe:
                    return False
            for wx, wy, vx, vy, _ in self.dyn_obstacles:
                px = wx + vx * pred_t
                py = wy + vy * pred_t
                if math.hypot(sx - px, sy - py) < dynamic_safe:
                    return False
        return True

    def _select_gate_target(self, rx: float, ry: float):
        # 优先级：
        # 1) ConvergePath（门必须贴近 coverage path，且权重最高）
        # 2) 触发避障模式时禁止穿门，优先避障
        # 3) 门路径必须通过碰撞检查
        if self.mode in ('STATIC_DETOUR', 'DYNAMIC_AVOID', 'STOP'):
            return None

        pending_gates = [g for g in self.known_gates if not g['passed']]
        gate_candidates = []
        for gate in pending_gates:
            robot_dist = math.hypot(rx - gate['x'], ry - gate['y'])
            if robot_dist > self.gate_engage_dist:
                continue
            path_dist = self._distance_to_converge_path(gate['x'], gate['y'])
            global_path_dist = gate.get('path_dist', float('inf'))
            if self.force_gate_near_path_only and path_dist > self.gate_path_max_dist:
                self.get_logger().info(
                    f'跳过强制穿门: gate={gate["id"]} '
                    f'距ConvergePath{path_dist:.2f}m (全局{global_path_dist:.2f}m) '
                    f'> 阈值{self.gate_path_max_dist:.2f}m',
                    throttle_duration_sec=2.0)
                continue
            if not self._is_gate_path_collision_free(
                    rx, ry, (gate['x'], gate['y'], gate['yaw'])):
                self.get_logger().info(
                    f'跳过强制穿门: gate={gate["id"]} 路径碰撞风险',
                    throttle_duration_sec=2.0)
                continue
            # ConvergePath 权重最高：优先选择更贴近 coverage path 的门
            score = (self.gate_converge_path_weight * path_dist
                     + self.gate_robot_dist_weight * robot_dist)
            gate_candidates.append((score, gate, robot_dist, path_dist))

        best_gate = None
        if gate_candidates:
            gate_candidates.sort(key=lambda x: x[0])
            best_gate = gate_candidates[0][1]

        if self.mode == 'NARROW_GATE':
            if self.gate_pose is not None:
                pgx, pgy, pyaw = self.gate_pose
                p_path_dist = self._distance_to_converge_path(pgx, pgy)
                if (not self.force_gate_near_path_only
                        or p_path_dist <= self.gate_path_max_dist):
                    if self._is_gate_path_collision_free(rx, ry, (pgx, pgy, pyaw)):
                        return (pgx, pgy, pyaw, 'perception')
            if best_gate is not None:
                self._active_gate_id = best_gate['id']
                return (best_gate['x'], best_gate['y'], best_gate['yaw'], 'map_fallback')
            return None

        if best_gate is not None:
            self._active_gate_id = best_gate['id']
            self.get_logger().info(
                f'强制穿门: gate={best_gate["id"]} dist='
                f'{math.hypot(rx - best_gate["x"], ry - best_gate["y"]):.2f}m',
                throttle_duration_sec=1.0)
            return (best_gate['x'], best_gate['y'], best_gate['yaw'], 'map_forced')

        return None

    # ── 动态障碍物横向偏移 ──────────────────────────────────────────────
    def _apply_dynamic_detour(self, pts: list, rx, ry, ryaw) -> list:
        """
        计算动态障碍物对路径的横向威胁，将整条路径向安全侧偏移。
        安全侧 = 障碍物运动方向的垂直侧，取离障碍物预测位置更远的方向。
        """
        if not self.dyn_obstacles:
            return pts

        # 选最近、最威胁的动态障碍
        best_threat  = 0.0
        best_shift_y = 0.0   # 在机器人坐标系的左(+)/右(-) 偏移

        for wx, wy, vx, vy, sp in self.dyn_obstacles:
            dist = math.hypot(wx - rx, wy - ry)
            if dist > 4.0:
                continue

            # 障碍预测位置
            px = wx + vx * self.dyn_predict_s
            py = wy + vy * self.dyn_predict_s

            # 把预测位置转换到机器人局部坐标
            dx = px - rx; dy = py - ry
            bearing = math.atan2(dy, dx) - ryaw
            local_y = math.hypot(dx, dy) * math.sin(bearing)

            # 威胁度 = 1/dist（越近威胁越大）
            threat = 1.0 / max(0.3, dist)
            if threat > best_threat:
                best_threat = threat
                # 避让方向：障碍在左→向右偏；在右→向左偏
                best_shift_y = -math.copysign(self.detour_shift, local_y)

        if best_threat < 0.1:
            return pts

        # 将路径整体在机器人局部坐标左右方向偏移
        cos_y = math.cos(ryaw)
        sin_y = math.sin(ryaw)
        # 左方向单位向量（机器人坐标 +y = 左）
        left_x = -sin_y
        left_y  =  cos_y

        shifted = []
        for x, y in pts:
            shifted.append((
                x + left_x * best_shift_y,
                y + left_y * best_shift_y,
            ))
        return shifted

    # ── 静态障碍物排斥偏移 ──────────────────────────────────────────────
    def _is_wall_point(self, wx: float, wy: float) -> bool:
        if not self._has_area_bounds:
            return False
        m = self.wall_filter_margin
        return (
            abs(wx - self.area_x_min) <= m or
            abs(wx - self.area_x_max) <= m or
            abs(wy - self.area_y_min) <= m or
            abs(wy - self.area_y_max) <= m
        )

    @staticmethod
    def _to_float(v, default=float('nan')):
        try:
            return float(v)
        except (TypeError, ValueError):
            return default

    def _build_known_gates_from_params(self, get_param):
        out = []
        x1 = self._to_float(get_param('gate1_x').value)
        y1 = self._to_float(get_param('gate1_y').value)
        h1 = self._to_float(get_param('gate1_heading').value, 0.0)
        if math.isfinite(x1) and math.isfinite(y1):
            out.append({'id': 1, 'x': x1, 'y': y1, 'yaw': h1, 'passed': False})
        x2 = self._to_float(get_param('gate2_x').value)
        y2 = self._to_float(get_param('gate2_y').value)
        h2 = self._to_float(get_param('gate2_heading').value, 0.0)
        if math.isfinite(x2) and math.isfinite(y2):
            out.append({'id': 2, 'x': x2, 'y': y2, 'yaw': h2, 'passed': False})
        return out

    def _load_map_config(self):
        if not self.map_config_file:
            self._has_area_bounds = all(math.isfinite(v) for v in (
                self.area_x_min, self.area_x_max, self.area_y_min, self.area_y_max))
            return
        if not os.path.isfile(self.map_config_file):
            self.get_logger().warn(f'map_config_file 不存在: {self.map_config_file}')
            return
        try:
            with open(self.map_config_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().warn(f'读取地图配置失败: {e}')
            return

        m = data.get('map', {})
        area = m.get('area', {})
        ax_min = self._to_float(area.get('x_min'), self.area_x_min)
        ax_max = self._to_float(area.get('x_max'), self.area_x_max)
        ay_min = self._to_float(area.get('y_min'), self.area_y_min)
        ay_max = self._to_float(area.get('y_max'), self.area_y_max)
        if all(math.isfinite(v) for v in (ax_min, ax_max, ay_min, ay_max)):
            self.area_x_min, self.area_x_max = ax_min, ax_max
            self.area_y_min, self.area_y_max = ay_min, ay_max
            self._has_area_bounds = True

        gates = m.get('gates', [])
        parsed_gates = []
        for idx, gate in enumerate(gates, start=1):
            center = gate.get('center', {})
            gx = self._to_float(center.get('x'))
            gy = self._to_float(center.get('y'))
            gh = self._to_float(gate.get('heading'), 0.0)
            gid = int(gate.get('id', idx))
            if math.isfinite(gx) and math.isfinite(gy):
                parsed_gates.append({
                    'id': gid, 'x': gx, 'y': gy, 'yaw': gh,
                    'passed': False, 'path_dist': float('inf')
                })
        if parsed_gates:
            self.known_gates = parsed_gates
        self.get_logger().info(
            f'地图配置已加载: bounds=({self.area_x_min:.1f},{self.area_x_max:.1f},'
            f'{self.area_y_min:.1f},{self.area_y_max:.1f}) gates={len(self.known_gates)}')

    def _plan_obstacle_group_detour(self, pts: list):
        if len(pts) < 6 or len(self.obs_memory) < self.obs_group_min_count:
            return None

        near_obs = []
        near_thresh = self.detour_range + 0.6
        for ox, oy in self.obs_memory.keys():
            best = min(math.hypot(px - ox, py - oy) for px, py in pts)
            if best < near_thresh:
                near_obs.append((ox, oy))

        if len(near_obs) < self.obs_group_min_count:
            return None

        clusters = []
        remaining = set(range(len(near_obs)))
        while remaining:
            seed = remaining.pop()
            queue = [seed]
            cluster = [seed]
            while queue:
                i = queue.pop()
                ix, iy = near_obs[i]
                to_add = []
                for j in remaining:
                    jx, jy = near_obs[j]
                    if math.hypot(ix - jx, iy - jy) <= self.obs_group_cluster_dist:
                        to_add.append(j)
                for j in to_add:
                    remaining.remove(j)
                    queue.append(j)
                    cluster.append(j)
            clusters.append(cluster)

        cluster_idx = max(clusters, key=len)
        if len(cluster_idx) < self.obs_group_min_count:
            return None
        cluster = [near_obs[i] for i in cluster_idx]

        cx = sum(p[0] for p in cluster) / len(cluster)
        cy = sum(p[1] for p in cluster) / len(cluster)
        group_r = max(math.hypot(ox - cx, oy - cy) for ox, oy in cluster) + self.obs_group_margin

        hit_idx = [i for i, (px, py) in enumerate(pts)
                   if math.hypot(px - cx, py - cy) <= group_r + self.detour_range * 0.5]
        if len(hit_idx) < 2:
            return None

        start_i = max(1, min(hit_idx) - 1)
        end_i = min(len(pts) - 2, max(hit_idx) + 1)
        if end_i - start_i < 2:
            return None

        ax, ay = pts[start_i - 1]
        bx, by = pts[end_i + 1]
        tx = bx - ax
        ty = by - ay
        tnorm = math.hypot(tx, ty)
        if tnorm < 1e-4:
            return None
        tx /= tnorm
        ty /= tnorm
        nx = -ty
        ny = tx

        mid_i = (start_i + end_i) // 2
        mx, my = pts[mid_i]
        detour_amp = max(self.detour_shift * 1.2, group_r)
        detour_amp = min(
            detour_amp,
            max(self.detour_shift, self.bridge_detour_max_shift),
        )
        plus_d = math.hypot(mx + nx * detour_amp - cx, my + ny * detour_amp - cy)
        minus_d = math.hypot(mx - nx * detour_amp - cx, my - ny * detour_amp - cy)
        sign = 1.0 if plus_d >= minus_d else -1.0

        out = pts[:]
        span = max(1, end_i - start_i)
        for i in range(start_i, end_i + 1):
            t = float(i - start_i) / float(span)
            profile = math.sin(math.pi * t)
            offset = detour_amp * profile * sign
            out[i] = (pts[i][0] + nx * offset, pts[i][1] + ny * offset)
        return out, len(cluster), detour_amp

    @staticmethod
    def _robot_frame(wx: float, wy: float, rx: float, ry: float, ryaw: float):
        dx = wx - rx
        dy = wy - ry
        return (
            dx * math.cos(ryaw) + dy * math.sin(ryaw),
            -dx * math.sin(ryaw) + dy * math.cos(ryaw),
        )

    def _segment_min_clearance(self, pts: list, start_i: int, end_i: int) -> float:
        if not self.obs_memory:
            return 999.0
        best = 999.0
        obstacles = list(self.obs_memory.keys())
        for i in range(start_i, end_i + 1):
            px, py = pts[i]
            for ox, oy in obstacles:
                d = math.hypot(px - ox, py - oy)
                if d < best:
                    best = d
        return best

    def _plan_bridge_detour(self, pts: list, rx: float, ry: float, ryaw: float):
        """围绕正前方阻塞物生成平滑桥接路径，替代简单点级排斥。"""
        if len(pts) < 6 or not self.obs_memory:
            return None

        candidate_thresh = max(self.inflate + 0.25, self.detour_range + 0.10)
        candidates = []
        for ox, oy in self.obs_memory.keys():
            lx, ly = self._robot_frame(ox, oy, rx, ry, ryaw)
            if lx < -0.4 or abs(ly) > 3.5:
                continue
            best_i = 0
            best_d = float('inf')
            for i, (px, py) in enumerate(pts):
                d = math.hypot(px - ox, py - oy)
                if d < best_d:
                    best_d = d
                    best_i = i
            if best_d <= candidate_thresh:
                candidates.append((best_i, best_d, ox, oy))

        if not candidates:
            return None

        candidates.sort(key=lambda item: (item[0], item[1]))
        hit_i, hit_d, ox, oy = candidates[0]
        start_i = max(1, hit_i - 4)
        end_i = min(len(pts) - 2, hit_i + 8)
        if end_i - start_i < 3:
            return None

        ax, ay = pts[start_i - 1]
        bx, by = pts[end_i + 1]
        tx = bx - ax
        ty = by - ay
        tnorm = math.hypot(tx, ty)
        if tnorm < 1e-4:
            return None
        tx /= tnorm
        ty /= tnorm
        nx = -ty
        ny = tx

        px, py = pts[hit_i]
        lateral_err = (ox - px) * nx + (oy - py) * ny
        min_safe_clear = max(
            self.inflate + 0.10,
            self.vehicle_width * 0.5 + self.bridge_detour_margin,
        )
        desired_clear = max(min_safe_clear + 0.20, self.detour_shift * 1.05)
        detour_amp = max(desired_clear, abs(lateral_err) + min_safe_clear + 0.18)
        detour_amp = min(
            detour_amp,
            max(min_safe_clear + 0.12, self.bridge_detour_max_shift),
        )

        current_clear = self._segment_min_clearance(pts, start_i, end_i)
        best_choice = None
        best_score = -float('inf')

        for sign in (1.0, -1.0):
            out = pts[:]
            span = float(max(1, end_i - start_i))
            for i in range(start_i, end_i + 1):
                t = float(i - start_i) / span
                profile = math.sin(math.pi * t)
                offset = sign * detour_amp * profile
                out[i] = (pts[i][0] + nx * offset, pts[i][1] + ny * offset)

            min_clear = self._segment_min_clearance(out, start_i, end_i)
            hit_clear = min(
                math.hypot(out[i][0] - ox, out[i][1] - oy)
                for i in range(start_i, end_i + 1)
            )
            away_bonus = -sign * math.copysign(1.0, lateral_err) if abs(lateral_err) > 1e-3 else 0.0
            score = min_clear + 0.35 * hit_clear + 0.15 * away_bonus
            if score > best_score:
                best_score = score
                best_choice = (out, hit_clear, min_clear)

        if best_choice is None:
            return None

        out, hit_clear, min_clear = best_choice
        if min_clear < min_safe_clear or hit_clear < min_safe_clear:
            self.get_logger().warn(
                f'[BRIDGE_DETOUR_REJECT] obstacle=({ox:.1f},{oy:.1f}) '
                f'min_clear={min_clear:.2f}m hit_clear={hit_clear:.2f}m '
                f'< safe={min_safe_clear:.2f}m',
                throttle_duration_sec=0.5)
            return None
        if min_clear < current_clear + 0.05 and hit_clear < desired_clear:
            return None
        return out, (ox, oy), hit_clear, min_clear

    def _apply_detour(self, pts: list) -> list:
        if not self.obs_memory:
            return pts
        obstacles = list(self.obs_memory.keys())
        out = []
        effective_range = max(self.inflate + 0.1, self.detour_range)
        max_shift = max(self.detour_shift, self.bridge_detour_max_shift)
        for x, y in pts:
            sx, sy = 0.0, 0.0
            for ox, oy in obstacles:
                dist = math.hypot(x - ox, y - oy)
                if dist < effective_range and dist > 1e-4:
                    rep = (effective_range - dist) / effective_range
                    nx  = (x - ox) / dist
                    ny  = (y - oy) / dist
                    sx += rep * nx * self.detour_shift
                    sy += rep * ny * self.detour_shift
            shift_norm = math.hypot(sx, sy)
            if shift_norm > max_shift and shift_norm > 1e-6:
                scale = max_shift / shift_norm
                sx *= scale
                sy *= scale
            out.append((x + sx, y + sy))
        return out

    def _nearest_obstacle_distance(self, rx: float, ry: float) -> float:
        """估算机器人到当前障碍集的最近距离（世界坐标）。"""
        best = 999.0
        for ox, oy in self.obs_memory.keys():
            d = math.hypot(rx - ox, ry - oy)
            if d < best:
                best = d
        for wx, wy, *_ in self.dyn_obstacles:
            d = math.hypot(rx - wx, ry - wy)
            if d < best:
                best = d
        return best

    def _find_nearest_cov_idx(self, rx: float, ry: float, span_back: int = 20, span_fwd: int = 80) -> int:
        if not self.cov_pts:
            return self.progress_idx
        n = len(self.cov_pts)
        lo = max(0, self.progress_idx - max(1, span_back))
        hi = min(n - 1, self.progress_idx + max(1, span_fwd))
        best_i = self.progress_idx
        best_d = float('inf')
        for i in range(lo, hi + 1):
            px, py = self.cov_pts[i]
            d = math.hypot(rx - px, ry - py)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def _advance_to_safe_rejoin_idx(self, start_idx: int,
                                    clear_threshold: float,
                                    search_ahead: int = 60,
                                    min_advance: int = 0) -> int:
        if not self.cov_pts or not self.obs_memory:
            if not self.cov_pts:
                return start_idx
            n = len(self.cov_pts)
            return max(0, min(start_idx + max(0, min_advance), n - 1))
        n = len(self.cov_pts)
        start_idx = max(0, min(start_idx, n - 1))
        start_scan_idx = min(n - 1, start_idx + max(0, min_advance))
        best_idx = start_scan_idx
        best_clear = -1.0
        for idx in range(start_scan_idx, min(n, start_idx + max(1, search_ahead) + 1)):
            end_idx = min(n - 1, idx + 10)
            seg_clear = self._segment_min_clearance(self.cov_pts, idx, end_idx)
            if seg_clear > best_clear:
                best_clear = seg_clear
                best_idx = idx
            if seg_clear >= clear_threshold:
                return idx
        return best_idx

    def _build_rejoin_bridge(self, rx: float, ry: float, ryaw: float,
                              target_idx: int) -> list:
        """从当前位置生成平滑路径回到 coverage path 上的 target_idx 点。

        路径结构: 当前位置 → 中间插值点 → 目标点 → 目标后延续几个 coverage 点
        这样 controller 可以平滑过渡而不突然跳转。
        """
        if not self.cov_pts or target_idx >= len(self.cov_pts):
            return [(rx, ry)]
        tx, ty = self.cov_pts[target_idx]
        dist = math.hypot(rx - tx, ry - ty)
        pts = [(rx, ry)]
        if dist > 1.0:
            steps = max(2, int(dist / 0.5))
            for i in range(1, steps + 1):
                t = i / float(steps)
                pts.append((rx + (tx - rx) * t, ry + (ty - ry) * t))
        else:
            pts.append((tx, ty))
        n = len(self.cov_pts)
        tail_count = min(20, n - target_idx - 1)
        for i in range(1, tail_count + 1):
            ci = target_idx + i
            if ci < n:
                pts.append(self.cov_pts[ci])
        return pts

    # ── 发布路径 ─────────────────────────────────────────────────────────
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
