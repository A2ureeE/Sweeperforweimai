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
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PolygonStamped, PoseArray
from std_msgs.msg import String, Float32
import tf_transformations as tft


def yaw_from_quat(q):
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.declare_parameters('', [
            ('lookahead_dist',            8.0),
            ('obstacle_inflate',           0.6),
            ('obstacle_memory_s',          8.0),
            ('gate_align_dist',            4.0),
            ('detour_shift_m',             1.0),
            ('stagnation_time_s',          6.0),
            ('stagnation_min_dist_total',  0.5),
            # 避障回切迟滞：考虑车身长度，避免车尾尚未通过障碍就回切原路径
            ('rejoin_tail_clearance_m',    2.1),
            ('rejoin_min_hold_s',          0.8),
            # 穿门路径参数
            ('gate_approach_dist',         1.2),  # 门前等待点距门中心的距离
            ('gate_exit_dist',             2.5),  # 门后目标点距门中心的距离
            # 动态绕行：预测时间窗（秒）
            ('dyn_predict_s',              1.5),
        ])
        g = self.get_parameter
        self.lookahead       = g('lookahead_dist').value
        self.inflate         = g('obstacle_inflate').value
        self.obs_mem_s       = g('obstacle_memory_s').value
        self.gate_align      = g('gate_align_dist').value
        self.detour_shift    = g('detour_shift_m').value
        self.stag_time       = g('stagnation_time_s').value
        self.stag_min_d      = g('stagnation_min_dist_total').value
        self.rejoin_tail_clearance = float(g('rejoin_tail_clearance_m').value)
        self.rejoin_min_hold_s = float(g('rejoin_min_hold_s').value)
        self.gate_approach   = g('gate_approach_dist').value
        self.gate_exit       = g('gate_exit_dist').value
        self.dyn_predict_s   = g('dyn_predict_s').value

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
        self._gate_pose_t    = 0.0      # 门位姿时间戳（过期 3s 清除）

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
        self.progress_idx  = 0
        self._stag_accum_d = 0.0
        self._stag_start_t = time.time()
        self._last_stag_pos = None
        # get_logger().info(f'Coverage path received: {len(self.cov_pts)} pts')

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

        # ── 门位姿过期清理（3s）──
        if self.gate_pose and now - self._gate_pose_t > 3.0:
            self.gate_pose = None

        # ── NARROW_GATE：生成穿门专用路径 ────────────────────────────
        if self.mode == 'NARROW_GATE' and self.gate_pose is not None:
            path = self._build_gate_path(rx, ry, ryaw)
            if path:
                self._publish_path(path)
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

        if leaving_detour and self._detour_start_idx is not None:
            # 进入“待清障”阶段：不要立刻回切原路径，要等车尾也越过障碍。
            # 否则车头刚过杆子就回切，尾部会扫到杆子。
            self._detour_clear_pending = True
            self._detour_exit_t = now
            self.get_logger().info(
                f'避障结束，进入尾部清障等待: idx={self.progress_idx}')

        # 尾部清障判定：只有满足“时间 + 距离”双条件才发送 cleared。
        if self._detour_clear_pending:
            min_obs_d = self._nearest_obstacle_distance(rx, ry)
            hold_s = now - self._detour_exit_t
            if hold_s >= self.rejoin_min_hold_s and min_obs_d >= self.rejoin_tail_clearance:
                self.get_logger().info(
                    f'避障回切放行: hold={hold_s:.1f}s, min_obs={min_obs_d:.2f}m '
                    f'>= {self.rejoin_tail_clearance:.2f}m')
                self._detour_clear_pending = False
                self._detour_start_idx = None
                self._returning_from_detour = False
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
            pre_detour = slice_pts[:]
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
            f'窄门={"检测到" if self.gate_pose else "无"} | '
            f'停滞累积={self._stag_accum_d:.2f}m | '
            f'位置=({rx:.1f},{ry:.1f}) 航向={math.degrees(ryaw):.0f}° '
            f'到prog点={d_to_prog:.1f}m')

    # ── 穿门路径生成 ─────────────────────────────────────────────────────
    def _build_gate_path(self, rx, ry, ryaw) -> list:
        """
        生成 5 点穿门路径：
          当前位置 → 门前等待点 → 门中心 → 门后出口点 → 更远目标
        以门的进入方向（ryaw）为基准生成直线路径。
        """
        gcx, gcy, g_yaw = self.gate_pose

        # 用机器人当前朝向作为进门方向（更实用）
        cos_y = math.cos(ryaw)
        sin_y = math.sin(ryaw)

        # 5个路径点
        approach_d = self.gate_approach
        exit_d     = self.gate_exit
        pts = [
            (rx, ry),                                         # 0: 当前位置
            (gcx - cos_y * approach_d * 0.5,
             gcy - sin_y * approach_d * 0.5),                # 1: 门前半程
            (gcx, gcy),                                       # 2: 门中心
            (gcx + cos_y * exit_d * 0.5,
             gcy + sin_y * exit_d * 0.5),                    # 3: 门后半程
            (gcx + cos_y * exit_d,
             gcy + sin_y * exit_d),                          # 4: 完全穿越
        ]
        # 检查路径有效性（避免退行）
        for i in range(len(pts) - 1):
            dx = pts[i+1][0] - pts[i][0]
            dy = pts[i+1][1] - pts[i][1]
            if math.hypot(dx, dy) < 0.05:
                return None
        self.get_logger().info(
            f'穿门路径: 门中心({gcx:.1f},{gcy:.1f}), 进门方向={math.degrees(ryaw):.0f}°',
            throttle_duration_sec=1.0)
        return pts

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
    def _apply_detour(self, pts: list) -> list:
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
