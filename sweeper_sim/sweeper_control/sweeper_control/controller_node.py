#!/usr/bin/env python3
"""
Stanley 混合路径跟踪控制器 — Z200 清扫车

已修复的关键缺陷：
  1. edge_follow PD 使用 obstacle_points（不含墙面），实际上无墙面距离数据。
     改为直接订阅 /scan，从侧向激光测距中获取壁面距离。
  2. stuck 检测盲区 — 机器人若以小圆绕行，每帧移动距离均 > stuck_d 但永远
     不前进。改为统计 3s 窗口内的净位移（终点-起点距离），而非累积路程。
  3. EDGE_FOLLOW_left / EDGE_FOLLOW_right 模式字符串支持。
  4. omega clamping 与 v_min 的不一致性（旧版先用 v_sched 计算 kin_max，
     再提升 v 到 v_min，导致 omega 超过 kin 约束）— 现在 v_min 提升后重算。

Subscribes:
  /reference_path              nav_msgs/Path
  /odom                        nav_msgs/Odometry
  /behavior/mode               std_msgs/String
  /behavior/speed_limit        std_msgs/Float32
  /scan                        sensor_msgs/LaserScan  (新增，用于壁面跟踪)

Publishes:
  /cmd_vel                     geometry_msgs/Twist
"""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String, Float32
import tf_transformations as tft


def yaw_from_quat(q):
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.declare_parameters('', [
            ('area_x_min',              -12.0),
            ('area_x_max',               14.5),
            ('area_y_min',               -9.0),
            ('area_y_max',                9.5),
            ('wall_filter_margin',        0.3),
            ('control_rate_hz',         10.0),
            ('wheel_base',               1.05),
            ('wheel_separation',         1.048),  # Gazebo URDF 中左右轮间距，用于 diff_drive
            ('max_steer_angle',          0.8727),   # ~50°
            ('max_angular_rate',         0.9),
            ('target_speed',             0.8),
            ('min_tracking_speed',       0.25),
            # Stanley 增益
            ('stanley_k',               0.8),
            ('stanley_heading_gain',     1.0),
            # 速度调度
            ('curvature_speed_gain',     0.6),
            # Stuck 检测（净位移）
            ('stuck_window_s',           3.0),    # 统计窗口
            ('stuck_net_thresh',         0.12),   # 窗口内净位移低于此 = stuck
            ('recovery_forward_speed',   0.35),
            ('recovery_hold_s',          2.5),
            ('recovery_reverse_speed_ratio', 0.70),
            ('recovery_forward_speed_ratio', 0.45),
            ('recovery_turn_rate',       0.55),
            ('recovery_reverse_phase_ratio', 0.70),
            # 壁面跟踪 PD（现在使用 /scan 数据）
            ('wall_dist_ref',            0.35),
            ('kp_wall',                  0.6),
            ('kd_wall',                  0.12),
            # 侧向扫描角度范围（激光帧，度）
            ('side_scan_angle_min_deg',  70.0),
            ('side_scan_angle_max_deg', 110.0),
            # 前方反应式避障
            ('obstacle_slow_dist',       0.8),
            ('obstacle_steer_dist',      0.5),
            ('obstacle_stop_dist',       0.3),
            ('front_scan_half_angle_deg', 30.0),
            ('avoidance_omega_gain',     0.5),
        ])
        g = self.get_parameter
        self.area_x_min = g('area_x_min').value
        self.area_x_max = g('area_x_max').value
        self.area_y_min = g('area_y_min').value
        self.area_y_max = g('area_y_max').value
        self.wall_filter_margin = g('wall_filter_margin').value
        self.L          = g('wheel_base').value
        self.w_sep     = g('wheel_separation').value
        self.delta_max  = g('max_steer_angle').value
        self.w_max      = g('max_angular_rate').value
        self.v_target   = g('target_speed').value
        self.v_min      = g('min_tracking_speed').value
        self.stanley_k  = g('stanley_k').value
        self.stanley_hg = g('stanley_heading_gain').value
        self.curv_gain  = g('curvature_speed_gain').value
        self.stuck_win  = g('stuck_window_s').value
        self.stuck_thr  = g('stuck_net_thresh').value
        self.rec_spd    = g('recovery_forward_speed').value
        self.rec_hold   = g('recovery_hold_s').value
        self.rec_rev_ratio = g('recovery_reverse_speed_ratio').value
        self.rec_fwd_ratio = g('recovery_forward_speed_ratio').value
        self.rec_turn_rate = g('recovery_turn_rate').value
        self.rec_phase_ratio = g('recovery_reverse_phase_ratio').value
        self.wall_ref   = g('wall_dist_ref').value
        self.kp_wall    = g('kp_wall').value
        self.kd_wall    = g('kd_wall').value
        self.side_a_min = math.radians(g('side_scan_angle_min_deg').value)
        self.side_a_max = math.radians(g('side_scan_angle_max_deg').value)
        self.obs_slow_d    = g('obstacle_slow_dist').value
        self.obs_steer_d   = g('obstacle_steer_dist').value
        self.obs_stop_d    = g('obstacle_stop_dist').value
        self.front_half    = math.radians(g('front_scan_half_angle_deg').value)
        self.avoid_gain    = g('avoidance_omega_gain').value
        self.dt         = 1.0 / g('control_rate_hz').value

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_ref   = self.create_subscription(Path,      '/reference_path',       self.cb_ref,   5)
        self.sub_odom  = self.create_subscription(Odometry,  '/odom',                 self.cb_odom,  sensor_qos)
        self.sub_mode  = self.create_subscription(String,    '/behavior/mode',        self.cb_mode,  5)
        self.sub_speed = self.create_subscription(Float32,   '/behavior/speed_limit', self.cb_speed, 5)
        self.sub_scan  = self.create_subscription(LaserScan, '/scan',                 self.cb_scan,  sensor_qos)

        # 订阅 Gazebo /clock：用 Gazebo sim time 驱动 30s 启动延迟
        # Gazebo /clock 使用 BEST_EFFORT QoS
        clock_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_clock = self.create_subscription(Clock, '/clock', self.cb_clock, clock_qos)
        self._sim_elapsed_s = 0.0   # Gazebo sim time 秒数
        self._delay_done    = False  # 延迟是否已完成

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 5)
        self.pub_recovery_status = self.create_publisher(String, '/controller/recovery_status', 5)

        # 恢复指令订阅（来自 behavior_node，避障结束后回归原路径）
        self.sub_recovery = self.create_subscription(
            String, '/behavior/recovery_cmd', self.cb_recovery, 5)

        # 状态
        self.ref: list          = []
        self.ref_idx_hint       = 0
        self.robot              = None    # (x, y, yaw, v)
        self.mode               = 'COVERAGE'
        self.wall_side          = 'right' # 'left' 或 'right'
        self.speed_limit        = self.v_target
        self.scan_ranges        = []
        self.scan_angle_min     = 0.0
        self.scan_angle_inc     = 0.0

        # Stuck 检测：滑动窗口存储最近 stuck_win 秒的位置
        self._pos_history: list = []   # [(t, x, y), ...]

        # U-turn 状态机
        self._in_uturn = False          # 当前是否处于 PP(U-turn) 模式
        self._uturn_yaw_start = 0.0     # 进入 U-turn 时的初始航向
        self._uturn_cum_rotation = 0.0  # U-turn 期间累计航向旋转量（rad）
        self._uturn_logged = False      # 本次进入/退出是否已打印日志
        self._heading_history: list = []  # [(t, yaw), ...] 航向历史
        self._idx_history: list = []      # [(t, ref_idx_hint), ...] 路径索引历史

        # 恢复状态
        self._recovering  = False
        self._rec_start_t = 0.0
        self._rec_end_t   = 0.0
        self._rec_turn_dir = 1.0
        self._rec_mode = 'COVERAGE'
        self._rec_turn_pref = 1.0
        self._resume_pending = False

        # 壁面 PD 状态
        self._wall_err_prev = 0.0

        # 状态日志（每 5s 输出一次）
        self._log_v = 0.0
        self._log_w = 0.0
        self._log_delta = 0.0           # 转向角 delta (rad)
        self._log_delta_deg = 0.0       # 转向角 (度)
        self._log_omega_kin_max = 0.0   # 运动学最大角速度
        self._log_in_uturn = False       # 当前是否为 PP 模式
        self._log_omega_raw = 0.0         # omega 钳制前的原始值
        self._log_omega_clamped = False   # omega 是否被 kin 约束钳制
        self._log_v_cmd = 0.0            # 最终发布的速度指令
        self._was_in_uturn = False       # 上一帧是否为 PP 模式（用于检测进入/退出）
        self._trajectory_received = False  # 等待 RViz 绿色轨迹
        self._entry_locked       = False  # 入口点已锁定，延迟结束后不再更新
        self._log_front_d = 999.0
        self._log_avoid = 'none'
        self._saved_ref_idx = 0     # 避障时保存的路径索引，恢复时使用

        # U-turn 状态机
        self._in_uturn = False          # 当前是否处于 PP(U-turn) 模式
        self._uturn_yaw_start = 0.0     # 进入 U-turn 时的初始航向
        self._uturn_cum_rotation = 0.0 # U-turn 期间累计航向旋转量（rad）
        self._uturn_logged = False      # 本次进入/退出是否已打印日志
        self._uturn_stuck_t = 0.0       # PP 卡住开始时间（速度=0）
        self._uturn_north_t = 0.0       # PP 接近北/南墙时间
        self._uturn_reversed = False    # PP 期间是否已反转方向
        # Sticky：入弧后累计旋转监视，避免 entry/exit 振荡
        self._uturn_entry_t   = 0.0     # 进入 U-turn 的 sim time (s)
        self._uturn_prev_yaw  = 0.0     # 上一 tick 的 yaw（用于增量累计）
        self._uturn_signed_cum = 0.0   # U-turn 期间"带符号"累计旋转（可超 ±π）

        # 避障退出冷却：退出 STATIC_DETOUR/DYNAMIC_AVOID 后 2s 内禁止进入 U-turn
        self._prev_mode_ctrl      = 'COVERAGE'
        self._detour_exit_time    = 0.0
        self._uturn_detour_cooldown_s = 2.0

        self.create_timer(self.dt, self.tick)
        self.create_timer(1.0, self._log_status)
        self.get_logger().info('controller_node ready')

    # ── 回调 ────────────────────────────────────────────────────────────
    def cb_clock(self, msg: Clock):
        self._sim_elapsed_s = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9

    def cb_ref(self, msg: Path):
        self.ref = [(p.pose.position.x, p.pose.position.y,
                     yaw_from_quat(p.pose.orientation))
                    for p in msg.poses]
        self._trajectory_received = True  # RViz 绿色轨迹已到达

        # 路径到达时，如果 robot 位置已知，同步确定入口点
        # （robot 未到时，cb_odom 会处理）
        # 如果入口已锁定，不更新（保持延迟结束时的入点）
        if not self._entry_locked and self.robot is not None and self.ref:
            self._fix_entry_point()
            self._entry_locked = True
            return

        # 入口锁定后：每次 reference_path 更新都可能是内容级别的替换
        # （例如 POST_RECOVERY_REPLAN 桥接结束切回 coverage slice），
        # 老的 ref_idx_hint 在新 ref 上可能指向远处而非当前位置。
        # 若 ref[hint] 距车辆较远，则回到最近点；否则保持 monotonic 跟踪。
        if self._entry_locked and self.ref and self.robot is not None:
            rx, ry, _, _ = self.robot
            n_ref = len(self.ref)
            hint = max(0, min(self.ref_idx_hint, n_ref - 1))
            hx, hy, _ = self.ref[hint]
            dist_hint = math.hypot(rx - hx, ry - hy)
            if dist_hint > 2.0 or self.ref_idx_hint >= n_ref:
                best_i, best_d = self._nearest_ref_idx(rx, ry)
                old_hint = self.ref_idx_hint
                self.ref_idx_hint = best_i
                self.get_logger().info(
                    f'[ref_reanchor] hint {old_hint}->{best_i} '
                    f'dist_old={dist_hint:.2f}m dist_new={best_d:.2f}m '
                    f'ref_len={n_ref}')

    def cb_odom(self, msg: Odometry):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        v   = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.robot = (x, y, yaw, v)

        # robot 数据到达时，如果路径已就位，同步确定入口点
        # 如果入口已锁定，不更新（保持延迟结束时的入点）
        if not self._entry_locked and self.ref and self.ref:
            self._fix_entry_point()
            self._entry_locked = True

    def _fix_entry_point(self):
        """统一在 robot 位置已知 + 路径已就绪 时确定入口点。
        首次启动时强制从轨迹起点（索引0）出发，避免跳到路径中间。
        后续（避障恢复）才允许最近点搜索。
        """
        if not self.ref or self.robot is None:
            return
        rx, ry, _, _ = self.robot
        if self._trajectory_received:
            # 轨迹已到（RViz 绿色），从起点出发
            self.ref_idx_hint = 0
        else:
            # 轨迹未到但路径已收到：用最近点搜索（容错处理）
            best_i, _ = self._nearest_ref_idx(rx, ry)
            self.ref_idx_hint = best_i

    def _nearest_ref_idx(self, rx: float, ry: float):
        if not self.ref:
            return 0, float('inf')
        best_d = float('inf')
        best_i = 0
        for i, (px, py, _) in enumerate(self.ref):
            d = math.hypot(rx - px, ry - py)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i, best_d

    def _find_nearest_entry(self):
        """延迟结束时用最近点搜索定位入点，从当前位置自然切入路径。
        不强制 index 0，因为延迟期间的 odom 漂移可能导致
        强制 index 0 后机器人要逆向追赶路径起点，反而造成入点跳变。"""
        if not self.ref or self.robot is None:
            self.ref_idx_hint = 0
            self._entry_locked = True
            return
        rx, ry, _, _ = self.robot
        best_i, best_d = self._nearest_ref_idx(rx, ry)
        self.ref_idx_hint = best_i
        self._entry_locked = True
        self.get_logger().info(f'延迟结束，入口点={best_i}，最近距离={best_d:.2f}m')

    def cb_mode(self, msg: String):
        raw = msg.data
        if raw == 'EDGE_FOLLOW_left':
            self.mode      = 'EDGE_FOLLOW'
            self.wall_side = 'left'
        elif raw == 'EDGE_FOLLOW_right':
            self.mode      = 'EDGE_FOLLOW'
            self.wall_side = 'right'
        elif raw == 'EDGE_FOLLOW':
            self.mode = 'EDGE_FOLLOW'
        else:
            self.mode = raw

        detour_modes = ('STATIC_DETOUR', 'DYNAMIC_AVOID')
        if self._prev_mode_ctrl in detour_modes and self.mode not in detour_modes:
            self._detour_exit_time = time.time()
        self._prev_mode_ctrl = self.mode

    def cb_speed(self, msg: Float32):
        self.speed_limit = float(msg.data)

    def cb_scan(self, msg: LaserScan):
        self.scan_ranges    = list(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_inc = msg.angle_increment

    def cb_recovery(self, msg):
        if msg.data == 'resume_coverage':
            if self._recovering:
                self._resume_pending = True
                self.get_logger().info(
                    'resume_coverage deferred: recovery still active')
                return
            if self.robot is not None and self.ref:
                rx, ry, _, _ = self.robot
                self.ref_idx_hint, _ = self._nearest_ref_idx(rx, ry)
            else:
                self.ref_idx_hint = self._saved_ref_idx
            self._recovering = False
            self._rec_start_t = 0.0
            self._rec_end_t = 0.0
            self._pos_history.clear()
            self.get_logger().info(
                f'恢复 Coverage: ref_idx_hint={self.ref_idx_hint}')

    def _start_recovery(self, now_s: float, steer_dir: float, front_d: float):
        self._saved_ref_idx = self.ref_idx_hint
        if abs(steer_dir) > 1e-3:
            self._rec_turn_dir = math.copysign(1.0, steer_dir)
            self._rec_turn_pref = -self._rec_turn_dir
        else:
            self._rec_turn_dir = self._rec_turn_pref
            self._rec_turn_pref *= -1.0
        self._recovering = True
        self._rec_start_t = now_s
        self._rec_end_t = now_s + self.rec_hold
        self._rec_mode = self.mode
        self.get_logger().warn(
            f'Stuck 检测触发 — 恢复模式 mode={self.mode} '
            f'front={front_d:.2f}m turn={"left" if self._rec_turn_dir > 0 else "right"}')
        self.pub_recovery_status.publish(String(data='recovery_start'))

    def _apply_recovery_cmd(self, now_s: float, cmd: Twist, front_d: float) -> bool:
        if not self._recovering:
            return False
        if now_s >= self._rec_end_t:
            self._recovering = False
            self._rec_start_t = 0.0
            self._rec_end_t = 0.0
            self.ref_idx_hint, _ = self._nearest_ref_idx(self.robot[0], self.robot[1])
            self._pos_history.clear()
            self.pub_recovery_status.publish(String(data='recovery_done'))
            if self._resume_pending:
                self._resume_pending = False
                self.get_logger().info(
                    'Recovery done, applying deferred resume_coverage')
            return False

        elapsed = now_s - self._rec_start_t
        reverse_portion = self.rec_phase_ratio
        if self._rec_mode == 'STATIC_DETOUR':
            reverse_portion = max(reverse_portion, 0.85)

        if elapsed < self.rec_hold * reverse_portion or front_d < self.obs_slow_d:
            cmd.linear.x = float(-self.rec_spd * self.rec_rev_ratio)
            cmd.angular.z = float(self._rec_turn_dir * self.rec_turn_rate)
        else:
            cmd.linear.x = float(self.rec_spd * self.rec_fwd_ratio)
            cmd.angular.z = float(self._rec_turn_dir * self.rec_turn_rate * 0.45)
        return True

    # ── 主控制循环 ───────────────────────────────────────────────────────
    def tick(self):
        cmd = Twist()

        # ── 10s 启动延迟（基于 Gazebo /clock 的真实 sim time）────────
        if not self._delay_done:
            if self._sim_elapsed_s >= 10.0:
                self._delay_done = True
                # 延迟结束时：用最近点搜索定位入口，确保从当前位置跟踪路径
                self._find_nearest_entry()
                self.get_logger().info('10s sim 延迟结束，车辆开始行驶')
            else:
                # 延迟期间保持静止（不发任何指令）。
                # 之前此处发 linear.x=-0.6 是为了"对抗 Gazebo 推力"，但无墙诊断
                # 显示车辆不受额外推力。反向指令反而把车倒退到 (-5,-8) 导致起点漂移。
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)
                return
        if self.robot is None:
            self.pub_cmd.publish(cmd)
            return

        rx, ry, ryaw, rv = self.robot
        now_ns = self.get_clock().now().nanoseconds

        # ── Stuck 检测（净位移版）──────────────────────────────────────
        now_s = now_ns / 1e9
        self._pos_history.append((now_s, rx, ry))
        # 清除超出窗口的历史
        cutoff = now_s - self.stuck_win
        self._pos_history = [(t, x, y) for t, x, y in self._pos_history
                             if t >= cutoff]

        # ── U-turn 检测：航向历史记录 ──────────────────────────────────
        self._heading_history.append((now_s, ryaw))
        HEADING_WIN = 8.0
        cutoff_h = now_s - HEADING_WIN
        self._heading_history = [(t, y) for t, y in self._heading_history
                                if t >= cutoff_h]

        # 路径索引历史（用于检测 U-turn 边界卡住）
        self._idx_history.append((now_s, self.ref_idx_hint))
        IDX_WIN = 3.0
        cutoff_i = now_s - IDX_WIN
        self._idx_history = [(t, i) for t, i in self._idx_history if t >= cutoff_i]

        stuck = False
        if len(self._pos_history) >= 3:
            t0, x0, y0 = self._pos_history[0]
            net_disp = math.hypot(rx - x0, ry - y0)
            if (now_s - t0 >= self.stuck_win * 0.8 and
                    net_disp < self.stuck_thr and
                    self.mode in ('COVERAGE', 'STATIC_DETOUR')):
                stuck = True

        # ── 恢复逻辑（卡住后优先倒车脱困，再短暂前探重回轨迹）────────────
        if self._recovering:
            fd, _ = self._front_obstacle_info()
            if self._apply_recovery_cmd(now_s, cmd, fd):
                self.pub_cmd.publish(cmd)
                return

        if stuck and not self._recovering:
            fd, sd = self._front_obstacle_info()
            self._start_recovery(now_s, sd, fd)
            self._apply_recovery_cmd(now_s, cmd, fd)
            self.pub_cmd.publish(cmd)
            return

        # ── STOP 模式（主动避让，不再呆站）──────────────────────────────
        if self.mode == 'STOP' or self.speed_limit <= 0.0:
            front_d, steer_dir = self._front_obstacle_info()
            if front_d < self.obs_slow_d:
                cmd.linear.x  = float(-self.rec_spd * 0.5)
                cmd.angular.z = float(steer_dir * self.w_max * 0.6)
                self.get_logger().warn(
                    f'STOP模式主动避让: front={front_d:.2f}m, 后退+转向',
                    throttle_duration_sec=1.0)
            self.pub_cmd.publish(cmd)
            return

        # ── 无路径 ─────────────────────────────────────────────────────
        if not self.ref or len(self.ref) < 2:
            self.pub_cmd.publish(cmd)
            return

        # ── 等待 RViz 绿色轨迹 ─────────────────────────────────────────
        if not self._trajectory_received:
            self.get_logger().info('等待 RViz 绿色轨迹...', throttle_duration_sec=2.0)
            self.pub_cmd.publish(cmd)
            return

        # ── 控制策略选择 ────────────────────────────────────────────────
        # U-Turn 区间内用 Pure Pursuit（跟随参考路径），其他情况用 Stanley
        in_uturn = self._is_in_uturn()
        # DIAG: 追踪 in_uturn 切换，定位振荡源
        if in_uturn != self._was_in_uturn:
            self.get_logger().info(
                f'[DIAG] in_uturn {self._was_in_uturn}→{in_uturn} | '
                f'pos=({rx:.2f},{ry:.2f}) yaw={math.degrees(ryaw):.0f}° | '
                f'cum={math.degrees(self._uturn_cum_rotation):.0f}°',
                throttle_duration_sec=0.0)
        if not in_uturn and self._was_in_uturn and self._uturn_logged:
            cum = math.degrees(self._uturn_cum_rotation)
            self.get_logger().info(
                f'[U-turn] 退出 | 位置=({rx:.1f},{ry:.1f}) '
                f'| 累计旋转={cum:.0f}° | 路径idx={self.ref_idx_hint}/{len(self.ref)}',
                throttle_duration_sec=1.0)
            self._uturn_logged = False
        if in_uturn:
            self._uturn_logged = True
        self._was_in_uturn = in_uturn
        self._log_in_uturn = in_uturn
        if in_uturn:
            v, omega, delta = self._pure_pursuit_track(rx, ry, ryaw)
        elif self.mode == 'EDGE_FOLLOW':
            v, omega, delta = self._edge_follow_pd()
        else:
            v, omega, delta = self._stanley_track(rx, ry, ryaw, rv)

        # ── 反应式避障层（PP 模式跳过，避免干扰 U-turn）──────────────
        # 注意：此处直接修改 delta（前轮转向角），因为 Gazebo tricycle_drive 插件
        #       将 cmd.angular.z 解释为 steering angle，而不是角速度 ω。
        if not in_uturn:
            self._log_delta = delta
            self._log_delta_deg = math.degrees(delta)
            v = min(v, self.speed_limit)

            front_d, steer_dir = self._front_obstacle_info()
            self._log_front_d = front_d
            turning = self._has_upcoming_turn()
            eff_stop  = self.obs_stop_d  * (0.4 if turning else 1.0)
            eff_steer = self.obs_steer_d * (0.4 if turning else 1.0)
            eff_slow  = self.obs_slow_d  * (0.5 if turning else 1.0)

            emergency_stop = False
            avoid_label = 'none'
            if front_d < eff_stop:
                v = 0.0
                delta = steer_dir * self.avoid_gain
                emergency_stop = True
                avoid_label = 'STOP'
                self.get_logger().warn(
                    f'前方障碍 {front_d:.2f}m — 紧急转向',
                    throttle_duration_sec=1.0)
            elif front_d < eff_steer:
                t = (front_d - eff_stop) / max(0.01, eff_steer - eff_stop)
                v *= t
                v = max(v, self.v_min * 0.3)
                avoid_d = steer_dir * self.avoid_gain * (1.0 - t)
                delta = delta * t + avoid_d
                avoid_label = 'steer'
            elif front_d < eff_slow:
                t = (front_d - eff_steer) / max(0.01, eff_slow - eff_steer)
                v *= (0.5 + 0.5 * t)
                avoid_label = 'slow'
            self._log_avoid = avoid_label

            # 转向角物理限幅（URDF: ±0.8727 rad = ±50°）
            delta = max(-self.delta_max, min(self.delta_max, delta))
            self._log_delta = delta
            self._log_delta_deg = math.degrees(delta)

            # 角速度仅用于日志（实际不发给 Gazebo）
            omega = v * math.tan(delta) / self.L if abs(v) > 1e-3 else 0.0
            omega_kin_max = max(0.01, v) * math.tan(self.delta_max) / self.L
            self._log_omega_kin_max = omega_kin_max
            self._log_omega_raw = omega
            self._log_omega_clamped = False
        else:
            v = max(v, 0.22)
            delta = max(-self.delta_max, min(self.delta_max, delta))

            front_d, steer_dir = self._front_obstacle_info()
            self._log_front_d = front_d
            if front_d < self.obs_stop_d:
                v = 0.0
                delta = steer_dir * self.avoid_gain
                delta = max(-self.delta_max, min(self.delta_max, delta))
                self.get_logger().warn(
                    f'[U-turn] 前方障碍 {front_d:.2f}m — 紧急制动+转向',
                    throttle_duration_sec=1.0)
            elif front_d < self.obs_steer_d:
                t = (front_d - self.obs_stop_d) / max(0.01, self.obs_steer_d - self.obs_stop_d)
                v *= t
                v = max(v, 0.1)
                avoid_d = steer_dir * self.avoid_gain * (1.0 - t)
                delta = delta * t + avoid_d
                delta = max(-self.delta_max, min(self.delta_max, delta))

            self._log_delta = delta
            self._log_delta_deg = math.degrees(delta)
            omega = v * math.tan(delta) / self.L if abs(v) > 1e-3 else 0.0
            omega_kin_max = max(0.01, v) * math.tan(self.delta_max) / self.L
            self._log_omega_kin_max = omega_kin_max
            self._log_omega_raw = omega
            self._log_omega_clamped = False
            self._log_avoid = 'U-turn'

        self._log_v = v
        self._log_w = omega
        self._log_v_cmd = float(v)
        # Gazebo tricycle_drive 插件将 cmd.angular.z 直接作为前轮转向角 δ (rad)
        # 来源：gazebo_ros_tricycle_drive.cpp:227 — target_steering_angle = cmd_.angle
        # 历史 bug：此前发送 omega=v·tan(δ)/L（角速度），插件误当作 steering angle，
        #           导致实际转向角远小于期望，车辆 U-turn 时半径暴增撞墙
        cmd.linear.x  = float(v)
        cmd.angular.z = float(delta)
        self.pub_cmd.publish(cmd)

    # ── 路径弯道检测（U-turn 即将到来时抑制避障）───────────────────────
    def _has_upcoming_turn(self) -> bool:
        """检测前方路径是否有急弯（包括 Headland 90° 圆角与 Skip-Row U-turn）。

        前瞻距离必须覆盖 URDF 插件带来的"指令 0.2 m/s → 实跑 0.59 m/s"的 3x 超调，
        以及 max_wheel_torque=600Nm 下的惯性制动距离。以 0.6 m/s 全速行驶时，
        需至少 3s 预警时间才能减到安全入弯速度（<0.15 m/s），
        对应路径距离 ≈ 1.8m + 入弯前 1m 缓冲 ≈ 3m 前瞻。
        旧值：仅 2m 前瞻 → 预警触发时车已冲入弯，Stanley 惯性外滑。
        新值：5m 前瞻 → 至少 8s 预警，车辆有足够时间减速。"""
        pts = self.ref
        n   = len(pts)
        if n < 5:
            return False
        idx = max(0, min(self.ref_idx_hint, n - 1))
        for offset in [5, 10, 20, 30, 40, 50]:
            ci = min(n - 1, idx + offset)
            kappa = abs(self._estimate_curvature(ci))
            if kappa > 0.25:
                return True
        return False

    def _upcoming_turn_distance(self) -> float:
        """返回距最近急弯的路径距离（米），若前方 8m 内无急弯则返回 ∞。
        同时：若车辆已在弯中（当前 idx 或最近几个点 κ>0.25），返回 0。"""
        pts = self.ref
        n   = len(pts)
        if n < 5:
            return float('inf')
        idx = max(0, min(self.ref_idx_hint, n - 1))
        # 当前点及前后 3 点若已在弯中，立即返回 0 距离（强制降速）
        for i in range(max(0, idx - 3), min(n, idx + 4)):
            if abs(self._estimate_curvature(i)) > 0.25:
                return 0.0
        # 否则向前扫描查找最近的弯
        dist = 0.0
        for i in range(idx, min(n - 1, idx + 80)):
            if abs(self._estimate_curvature(i)) > 0.25:
                return dist
            dx = pts[i+1][0] - pts[i][0]
            dy = pts[i+1][1] - pts[i][1]
            dist += math.hypot(dx, dy)
        return float('inf')

    def _is_in_uturn(self) -> bool:
        """U-turn 状态机：进入基于路径曲率 + 距弧心距离，退出基于**带符号累计航向旋转**。

        关键：进入 U-turn 后采用 sticky 策略——退出条件仅基于"累计旋转"或"长时间卡住"，
        不依赖 ref_idx 或 curvature 的即时判断。这样避免了每 tick 在 PP/Stanley 之间振荡
        （原 bug 表现：cmd_δ 在 +50° 和 -50° 每 0.1s 交替，导致实际 R 从 0.88m 暴涨到 2.94m）。

        进入条件（同时满足）：
          ① 前方路径曲率 > 曲率阈值（意味着 U-turn 弧即将到来）
          ② 车辆距离该急弯段最近点 ≤ 2.0 m（真的靠近弧入口）
          ③ U-turn 可行性检查通过（弧心到车身前外角 < 距墙距离）

        退出条件（任一满足）：
          ① 带符号累计航向旋转绝对值 ≥ 175°（U-turn 已基本完成，这是主退出条件）
          ② 长时间卡住（速度 < 0.02 m/s 持续 2.5s）
          ③ 距南/北墙过近超过 0.5s（撞墙兜底）
          ④ 入弧后超过 30s 仍未完成（安全超时）
        """
        pts = self.ref
        n   = len(pts)
        if n < 5:
            # 路径无效时仍按现状返回（不强制退出，避免在无路径tick 里切出）
            return self._in_uturn

        if not self._heading_history or len(self._heading_history) < 3 or self.robot is None:
            return self._in_uturn

        rx, ry, ryaw, rv_local = self.robot
        now_s = self._heading_history[-1][0]

        if not self._in_uturn:
            entered = self._check_uturn_entry(rx, ry, ryaw)
            if entered:
                self._uturn_entry_t = now_s
                self._uturn_prev_yaw = ryaw
                self._uturn_signed_cum = 0.0
            return entered

        # ── 在 U-turn 中 ─────────────────────────────────────────────
        # 带符号增量累计：dyaw 用 wrap 规范化到 [-π, π]，然后累加
        # 这样 180° / -180° 不会触发 abs() 的回折问题
        dyaw = wrap(ryaw - self._uturn_prev_yaw)
        self._uturn_signed_cum += dyaw
        self._uturn_prev_yaw = ryaw
        abs_cum = abs(self._uturn_signed_cum)
        self._uturn_cum_rotation = abs_cum

        # 条件1：累计航向旋转 ≥ 175°（主退出条件）
        if abs_cum >= math.radians(175):
            self.get_logger().info(
                f'[U-turn] 完成退出 | 位置=({rx:.1f},{ry:.1f}) | '
                f'累计旋转={math.degrees(abs_cum):.0f}° | 耗时={now_s - self._uturn_entry_t:.1f}s',
                throttle_duration_sec=1.0)
            self._in_uturn = False
            self._uturn_logged = False
            self._uturn_resync_ref_idx(rx, ry, reason='normal_exit')
            return False

        # 条件2：长时间卡住（速度 < 0.02 m/s 持续 2.5s）
        if rv_local < 0.02:
            if self._uturn_stuck_t == 0.0:
                self._uturn_stuck_t = now_s
            elif now_s - self._uturn_stuck_t >= 2.5:
                self.get_logger().warn(
                    f'[U-turn] 卡住退出（可能撞墙）| 位置=({rx:.1f},{ry:.1f}) | '
                    f'累计旋转={math.degrees(abs_cum):.0f}° | 速度={rv_local:.2f}',
                    throttle_duration_sec=1.0)
                self._in_uturn = False
                self._uturn_logged = False
                self._uturn_stuck_t = 0.0
                self._uturn_resync_ref_idx(rx, ry, reason='stuck')
                return False
        else:
            self._uturn_stuck_t = 0.0

        # 条件3：距离南北墙过近（安全兜底）
        if ry > self.area_y_max - 0.4 or ry < self.area_y_min + 0.4:
            if self._uturn_north_t == 0.0:
                self._uturn_north_t = now_s
            elif now_s - self._uturn_north_t >= 0.5:
                self.get_logger().warn(
                    f'[U-turn] 南/北墙警报退出 | 位置=({rx:.1f},{ry:.1f}) | '
                    f'累计旋转={math.degrees(abs_cum):.0f}°',
                    throttle_duration_sec=1.0)
                self._in_uturn = False
                self._uturn_logged = False
                self._uturn_north_t = 0.0
                self._uturn_resync_ref_idx(rx, ry, reason='wall_alarm')
                return False
        else:
            self._uturn_north_t = 0.0

        # 条件4：U-turn 超时（sim 中正常 U-turn ≤ 12s，设 30s 为兜底）
        if now_s - self._uturn_entry_t >= 30.0:
            self.get_logger().warn(
                f'[U-turn] 超时 30s 退出 | 位置=({rx:.1f},{ry:.1f}) | '
                f'累计旋转={math.degrees(abs_cum):.0f}°',
                throttle_duration_sec=1.0)
            self._in_uturn = False
            self._uturn_logged = False
            self._uturn_resync_ref_idx(rx, ry, reason='timeout')
            return False

        # Sticky：在 U-turn 内，其他任何条件都不触发退出
        return True

    def _uturn_resync_ref_idx(self, rx, ry, reason: str = ''):
        """R3：U-turn 退出时把 ref_idx_hint 重同步到当前位置在 self.ref 上的最近点。

        U-turn 期间（尤其异常超时退出）ref_idx_hint 被冻结；若直接恢复 Stanley，
        目标点可能远在车辆身后或方向相反，导致 CTE 巨大并二次失控。
        """
        if not self.ref or len(self.ref) < 2:
            return
        best_d = float('inf')
        best_i = self.ref_idx_hint
        for i, (px, py, _) in enumerate(self.ref):
            d = (rx - px) ** 2 + (ry - py) ** 2
            if d < best_d:
                best_d = d
                best_i = i
        old = self.ref_idx_hint
        self.ref_idx_hint = best_i
        self.get_logger().info(
            f'[U-turn resync] ref_idx_hint {old}→{best_i}/{len(self.ref)} '
            f'dist={math.sqrt(best_d):.2f}m reason={reason}')

    def _check_uturn_entry(self, rx, ry, ryaw) -> bool:
        """判断是否应该进入 U-turn 模式。
        必要条件：
          ⓪ 当前不在避障模式，且避障退出冷却已过
          ① 前方路径在较近距离内有急弯（kappa > 0.4）
          ② 车辆距离该弯段入口足够近（沿路径距离 < 1.2m，R2 修正 2.0→1.2）
          ③ 弯段累计航向变化 ≥ 140°（区分真正的 U-turn 与 Headland 90° 圆角）
              累加在第一个 κ<0.15 的点处 *立即* 打断（R2 修正，防止跨多段弯误累加）
          ④ 几何可行性检查（_verify_uturn_feasibility）通过

        注：R2 修正后，_check_uturn_entry 只在单段连续急弯时触发；
        Headland 90° 圆角 + 其他弯道叠加造成的 289° 假急弯不再触发 U-turn。
        """
        if self.mode in ('STATIC_DETOUR', 'DYNAMIC_AVOID', 'STOP'):
            return False

        cooldown_elapsed = time.time() - self._detour_exit_time
        if cooldown_elapsed < self._uturn_detour_cooldown_s:
            return False

        pts = self.ref
        n   = len(pts)
        idx = max(0, min(self.ref_idx_hint, n - 1))

        # 查找前方第一个急弯点
        sharp_idx = None
        for offset in range(2, min(40, n - idx)):
            ci = idx + offset
            if abs(self._estimate_curvature(ci)) > 0.4:
                sharp_idx = ci
                break
        if sharp_idx is None:
            return False

        # 弯段入口距离（沿路径累积距离）
        arc_entry_dist = 0.0
        for i in range(idx, sharp_idx):
            if i + 1 < n:
                arc_entry_dist += math.hypot(
                    pts[i+1][0] - pts[i][0],
                    pts[i+1][1] - pts[i][1])
        if arc_entry_dist > 1.2:
            return False  # 还太远，先用 Stanley（R2: 2.0→1.2）

        # 条件③：检查后续弯段累计航向变化
        # 从 sharp_idx 开始沿路径向前扫描，累计相邻切线夹角。
        # 连续 2+ 个低曲率点才认为弯段结束（防止绕行弧边界处单点抖动误触发）。
        total_turn = 0.0
        scan_dist = 0.0
        prev_heading = None
        low_kappa_streak = 0
        for i in range(sharp_idx, min(n - 1, sharp_idx + 80)):
            hx = pts[i+1][0] - pts[i][0]
            hy = pts[i+1][1] - pts[i][1]
            seg = math.hypot(hx, hy)
            if seg < 1e-4:
                continue
            hdg = math.atan2(hy, hx)
            if prev_heading is not None:
                total_turn += abs(wrap(hdg - prev_heading))
            prev_heading = hdg
            scan_dist += seg
            if scan_dist > 8.0:
                break
            if i > sharp_idx and abs(self._estimate_curvature(i)) < 0.15:
                low_kappa_streak += 1
                if low_kappa_streak >= 2:
                    break
            else:
                low_kappa_streak = 0

        if total_turn < math.radians(140):
            # 不足 140°, 认为是 Headland 圆角或普通弯道，用 Stanley 跟踪
            return False

        # 几何可行性检查（确认 U-turn 弧半径够大，不会撞墙）
        if not self._verify_uturn_feasibility(sharp_idx):
            self.get_logger().warn(
                f'[U-turn] 可行性检查失败 — 继续用 Stanley 跟踪',
                throttle_duration_sec=2.0)
            return False

        # 进入 U-turn 模式
        self._in_uturn = True
        self._uturn_yaw_start = ryaw
        self._uturn_cum_rotation = 0.0
        self._uturn_logged = False
        self._uturn_stuck_t = 0.0
        self._uturn_north_t = 0.0
        self._uturn_reversed = False
        self.get_logger().info(
            f'[U-turn] 进入 | 位置=({rx:.1f},{ry:.1f}) 航向={math.degrees(ryaw):.0f}° '
            f'| 弯段idx={sharp_idx}/{n} 距离={arc_entry_dist:.2f}m '
            f'| 累计转角={math.degrees(total_turn):.0f}°',
            throttle_duration_sec=1.0)
        return True

    def _verify_uturn_feasibility(self, sharp_idx: int) -> bool:
        """检查 U-turn 路径是否物理可行：
          1. 估算弧段半径（由连续曲率的平均值推算）
          2. 弧心到车身前外角最大距离 + 安全余量 ≤ 距墙距离
          3. 估算的弧半径 ≥ 车辆物理最小转弯半径
        """
        pts = self.ref
        n   = len(pts)
        if sharp_idx >= n - 3 or sharp_idx < 3:
            return True  # 路径末端，不做严格检查

        # 估算弧半径：取连续 5 个点曲率平均值的倒数
        total_k = 0.0
        cnt = 0
        for i in range(max(0, sharp_idx - 2), min(n, sharp_idx + 3)):
            k = abs(self._estimate_curvature(i))
            if k > 0.1:
                total_k += k
                cnt += 1
        if cnt == 0:
            return True
        avg_k = total_k / cnt
        est_R = 1.0 / max(avg_k, 0.01)

        # 车辆物理最小转弯半径 = L / tan(delta_max)
        veh_min_r = self.L / math.tan(self.delta_max)

        # 条件1：估算弧半径 ≥ 车辆物理最小值 × 95%（留少量余量）
        if est_R < veh_min_r * 0.95:
            self.get_logger().warn(
                f'[U-turn] 估算半径 {est_R:.2f}m < 物理最小 {veh_min_r:.2f}m',
                throttle_duration_sec=2.0)
            return False

        # 条件2：弧段最远点（通常是横向极值）到墙距离足够
        # 取弧段点集中 x 或 y 绝对值最大的点
        sub = pts[max(0, sharp_idx - 5): min(n, sharp_idx + 20)]
        body_long = 1.41   # URDF: 后轴到前外角纵向
        body_hw = 0.525    # URDF: 车身半宽
        # 车辆在弧上跟踪时，车身前外角最大伸出距离
        # = sqrt((est_R + body_hw)² + body_long²) - est_R
        # 这是"外扩量"：弧心距车身前外角 - 弧半径
        body_out = math.sqrt((est_R + body_hw)**2 + body_long**2) - est_R
        for px, py, _ in sub:
            # 距四面墙的最小距离
            min_wall = min(
                self.area_x_max - px,
                px - self.area_x_min,
                self.area_y_max - py,
                py - self.area_y_min,
            )
            if min_wall < body_out - 0.05:
                self.get_logger().warn(
                    f'[U-turn] 弧段点({px:.1f},{py:.1f})距墙{min_wall:.2f}m < '
                    f'需要{body_out:.2f}m',
                    throttle_duration_sec=2.0)
                return False

        return True

    def _pure_pursuit_track(self, rx, ry, ryaw):
        """U-turn 模式：基于 reference_path 的 Pure Pursuit 跟踪。

        算法：
          1. 在参考路径上前瞻 Ld 距离找到目标点 (tx, ty)
          2. 计算车辆坐标系下目标点的横向偏移 ey
          3. Pure Pursuit 曲率公式：kappa = 2·ey / Ld²
          4. 转向角 delta = atan(L · kappa)
          5. 速度取较慢值，允许稳定过弯

        Ld 根据路径曲率自适应：
          直线段: Ld = 2.0m, 弯道: Ld = 1.2m（更紧跟踪）
        """
        pts = self.ref
        n   = len(pts)

        if n < 2 or self.robot is None:
            return 0.0, 0.0, 0.0

        # ── 1. 自适应前瞻距离 ─────────────────────────────────────────
        idx = max(0, min(self.ref_idx_hint, n - 1))
        # 当前曲率 → 前瞻距离
        cur_k = abs(self._estimate_curvature(idx))
        # 弯道越急，前瞻越短（0.7~2.0m）
        Ld = max(0.7, min(2.0, 2.0 - 2.5 * cur_k))

        # ── 2. 在路径上前瞻 Ld 距离找目标点 ──────────────────────────
        # 先把路径索引移到距离机器人最近的点（防止落后）
        best_d = float('inf')
        best_i = idx
        lo = max(0, idx - 3)
        hi = min(n - 1, idx + 6)
        for i in range(lo, hi + 1):
            d = math.hypot(rx - pts[i][0], ry - pts[i][1])
            if d < best_d:
                best_d = d
                best_i = i
        if best_i > self.ref_idx_hint:
            self.ref_idx_hint = best_i

        # 从 best_i 沿路径累积距离，找到第一个 >= Ld 的点
        acc = 0.0
        target_i = best_i
        for i in range(best_i, n - 1):
            d = math.hypot(pts[i+1][0] - pts[i][0],
                           pts[i+1][1] - pts[i][1])
            acc += d
            target_i = i + 1
            if acc >= Ld:
                break
        tx, ty, _ = pts[target_i]

        # ── 3. 车辆坐标系下目标点的横向偏移 ey ──────────────────────
        dx = tx - rx
        dy = ty - ry
        # 目标点到车的距离（实际前瞻）
        L_actual = max(0.3, math.hypot(dx, dy))
        # 车辆坐标系：x 沿航向前方，y 向左
        lx = math.cos(-ryaw) * dx - math.sin(-ryaw) * dy
        ly = math.sin(-ryaw) * dx + math.cos(-ryaw) * dy

        # 如果目标在车后方（lx < 0），朝向误差过大，用最大转向推动到前方
        if lx < 0.1:
            # 目标在侧后，尽量转向目标侧
            delta = self.delta_max if ly > 0 else -self.delta_max
        else:
            # ── 4. Pure Pursuit 曲率 ──────────────────────────────
            # kappa = 2·ey / L²
            kappa = 2.0 * ly / (L_actual * L_actual)
            # 转向角 δ = atan(L_wb · kappa)
            delta = math.atan(self.L * kappa)

        # 限幅到物理最大转向角
        delta = max(-self.delta_max, min(self.delta_max, delta))

        # ── 5. 速度 ─────────────────────────────────────────────────
        # U-turn 中用较低速度以维持稳定，但不能太低（避免 Gazebo 摩擦导致停滞）
        v = 0.25  # 掉头速度（平衡速度与控制精度）

        omega = v * math.tan(delta) / self.L
        return v, omega, delta

    # ── 5 秒状态日志 ─────────────────────────────────────────────────
    def _log_status(self):
        if self.robot is None:
            return
        rx, ry, ryaw, rv = self.robot
        turn = self._has_upcoming_turn()
        uturn_mode = 'U-turn' if self._log_in_uturn else self.mode
        omega_note = (' (钳制!)' if self._log_omega_clamped else '')
        delta_note = (' (限幅!)' if abs(self._log_delta) >= self.delta_max * 0.98 else '')
        self.get_logger().info(
            f'[控制] 模式={uturn_mode} | '
            f'v指令={self._log_v_cmd:.2f} omega={self._log_w:.2f}{omega_note}'
            f' delta={self._log_delta_deg:.1f}°{delta_note}'
            f' kin_max={self._log_omega_kin_max:.3f}'
            f' | 实际v={rv:.2f} | '
            f'omega_raw={self._log_omega_raw:.2f} | '
            f'前方={self._log_front_d:.2f}m 避障={self._log_avoid} '
            f'弯道={"是" if turn else "否"} | '
            f'路径={len(self.ref)}pt idx={self.ref_idx_hint} | '
            f'位置=({rx:.1f},{ry:.1f}) 航向={math.degrees(ryaw):.0f}°')

    # ── 前方障碍物扫描 ─────────────────────────────────────────────────
    def _front_obstacle_info(self):
        """扫描前方锥形区域，返回 (最小距离, 转向方向)。
        转向方向: +1 = 向左转 (omega>0), -1 = 向右转（朝更开阔的一侧）。

        逻辑：
          1) 优先使用“非墙体”回波，避免沿边作业时被墙干扰；
          2) 若没有非墙体，但前方墙体已经很近，也要触发反应式避障（防撞墙兜底）。
        """
        if not self.scan_ranges:
            return 999.0, 0.0
        if self.robot is None:
            return 999.0, 0.0
        rx, ry, ryaw, _ = self.robot
        min_nonwall = 999.0
        min_any = 999.0
        left_min  = 999.0
        right_min = 999.0
        for i, r in enumerate(self.scan_ranges):
            if not (0.05 < r < 10.0):
                continue
            angle = self.scan_angle_min + i * self.scan_angle_inc
            if abs(angle) > self.front_half:
                continue
            wx = rx + r * math.cos(angle + ryaw)
            wy = ry + r * math.sin(angle + ryaw)
            min_any = min(min_any, r)
            if not self._is_wall_point(wx, wy):
                min_nonwall = min(min_nonwall, r)
            if angle >= 0:
                left_min = min(left_min, r)
            else:
                right_min = min(right_min, r)
        steer_dir = 1.0 if right_min < left_min else -1.0
        if min_nonwall < 999.0:
            return min_nonwall, steer_dir
        # 兜底：前方仅检测到墙体时，近距离也触发避障，防止“撞墙仍显示 none”。
        wall_trigger_dist = max(self.obs_slow_d * 1.2, 1.0)
        if min_any < wall_trigger_dist:
            return min_any, steer_dir
        return 999.0, steer_dir

    def _is_wall_point(self, wx: float, wy: float) -> bool:
        m = self.wall_filter_margin
        return (
            abs(wx - self.area_x_min) <= m or
            abs(wx - self.area_x_max) <= m or
            abs(wy - self.area_y_min) <= m or
            abs(wy - self.area_y_max) <= m
        )

    # ── Stanley 路径跟踪 ─────────────────────────────────────────────────
    def _stanley_track(self, rx, ry, ryaw, rv):
        pts = self.ref
        n   = len(pts)
        # 防御性边界检查：确保 ref_idx_hint 在有效范围内
        if self.ref_idx_hint >= n:
            self.ref_idx_hint = n - 1
        if self.ref_idx_hint < 0:
            self.ref_idx_hint = 0

        lo  = max(0, self.ref_idx_hint - 5)
        hi  = min(n - 1, self.ref_idx_hint + 6)
        sub = pts[lo: hi + 1]

        best_d = float('inf')
        best_i = lo
        for i, (px, py, _) in enumerate(sub):
            d = math.hypot(rx - px, ry - py)
            if d < best_d:
                best_d = d
                best_i = lo + i

        if best_i > self.ref_idx_hint:
            self.ref_idx_hint = best_i

        idx = self.ref_idx_hint
        nxt = min(idx + 1, n - 1)
        # 当 idx 已到窗口末尾时 nxt == idx，atan2(0,0) 会返回 0.0（伪 east），
        # 导致 path_yaw 偏差极大。此时退化到用前一段方向 (pts[idx-1]→pts[idx])。
        if nxt > idx:
            path_yaw = math.atan2(pts[nxt][1] - pts[idx][1],
                                  pts[nxt][0] - pts[idx][0])
        elif idx > 0:
            path_yaw = math.atan2(pts[idx][1] - pts[idx-1][1],
                                  pts[idx][0] - pts[idx-1][0])
        else:
            path_yaw = ryaw  # 整个路径只有一个点，用车头方向兜底
        heading_err = wrap(path_yaw - ryaw)

        px, py, _ = pts[idx]
        dx = rx - px;  dy = ry - py
        cte = -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy

        # ── 脱线鲁棒性：严重偏离或路径附近曲率高时切 Pure Pursuit ────────
        # 原因：Stanley 在弧外侧时 path_yaw（切线方向）与实际需要转向方向会
        # 出现符号矛盾（heading_err 要求+，cte 要求-，抵消或反号）。
        # 另：当 idx 被推到 ref 窗口末尾时，path_yaw 用前一段兜底也可能偏差
        # 很大。Pure Pursuit 瞄准前方一段路径上的目标点，对脱线拓扑更鲁棒。
        local_kappa_abs = abs(self._estimate_curvature(idx))
        off_track = abs(cte) > 0.3      # 降低阈值：0.3m 起切 PP
        in_curve  = local_kappa_abs > 0.25
        at_ref_end = (idx >= n - 3)     # idx 接近窗口末尾时 Stanley 不可靠
        if off_track or in_curve or at_ref_end:
            Ld = max(1.2, 2.5 * max(0.15, rv))
            tgt_i = idx
            acc = 0.0
            while tgt_i + 1 < n:
                acc += math.hypot(pts[tgt_i+1][0] - pts[tgt_i][0],
                                  pts[tgt_i+1][1] - pts[tgt_i][1])
                tgt_i += 1
                if acc >= Ld:
                    break
            tx, ty, _ = pts[tgt_i]
            # 车辆坐标系下目标点的横向偏移 y_r
            dx_r =  (tx - rx) * math.cos(ryaw) + (ty - ry) * math.sin(ryaw)
            dy_r = -(tx - rx) * math.sin(ryaw) + (ty - ry) * math.cos(ryaw)
            L2 = max(dx_r * dx_r + dy_r * dy_r, 1e-3)
            # 标准 Pure Pursuit: delta = atan2(2L*y_r, L²)
            delta = math.atan2(2.0 * self.L * dy_r, L2)
            delta = max(-self.delta_max, min(self.delta_max, delta))
        else:
            # 抑制后向分量
            angle_to_next = math.atan2(pts[nxt][1] - ry, pts[nxt][0] - rx)
            behind = abs(wrap(angle_to_next - ryaw)) > 1.8
            if behind:
                heading_err *= 0.3
                cte         *= 0.3
            v_eps  = max(0.5, rv)
            delta  = (self.stanley_hg * heading_err -
                      math.atan2(self.stanley_k * cte, v_eps))
            delta  = max(-self.delta_max, min(self.delta_max, delta))

        kappa   = self._estimate_curvature(idx)
        # 使用 speed_limit 而不是 v_target，确保行为层限速生效
        v_sched = self.speed_limit / (1.0 + self.curv_gain * abs(kappa))
        v_sched = max(self.v_min, v_sched)

        # ── 曲率速度调度 ─────────────────────────────────────────────────
        # 急弯时（kappa 较大）降低速度，确保角速度不超过物理极限
        omega_max_diff = self.speed_limit * math.tan(self.delta_max) / self.L
        kappa_abs = abs(kappa)
        if kappa_abs > 1e-4:
            v_needed = omega_max_diff / kappa_abs
            v_sched = max(v_sched, min(self.v_min * 1.5, v_needed))

        # ── 角速度（ω）计算 ───────────────────────────────────────────────
        # URDF tricycle_drive 插件期望 ω = v × tan(δ) / L
        # 其中 L = wheel_base = 1.05m（前后轴中心距）
        omega = v_sched * math.tan(delta) / self.L

        v_sched = min(v_sched, self.speed_limit)

        return v_sched, omega, delta

    def _estimate_curvature(self, idx: int) -> float:
        """估计 idx 点处路径曲率（rad/m）。

        算法：对圆弧上等距三点 A、B、C，计算外接圆半径。
          - 用向量 BA、BC 夹角 θ 与弦长 |AC|
          - R = |AC| / (2·sin(θ/2))
          - κ = 1/R，保留 BA→BC 右手叉积符号标示左右转
        旧版 bug：用"长弦方向 - 短弦方向"，但两弦都是弧的角平分线，方向几乎一致，
                  使任何圆弧返回 κ≈0，导致 Headland 90° 圆角被当直路。
        """
        pts = self.ref
        n   = len(pts)
        if n < 5 or idx < 0 or idx >= n:
            return 0.0
        i_a = max(0, idx - 3)
        i_c = min(n - 1, idx + 3)
        i_b = idx
        if i_a == i_b or i_b == i_c:
            return 0.0
        ax, ay = pts[i_a][0], pts[i_a][1]
        bx, by = pts[i_b][0], pts[i_b][1]
        cx, cy = pts[i_c][0], pts[i_c][1]
        vax, vay = ax - bx, ay - by   # B→A
        vcx, vcy = cx - bx, cy - by   # B→C
        ac_len = math.hypot(cx - ax, cy - ay)
        ba_len = math.hypot(vax, vay)
        bc_len = math.hypot(vcx, vcy)
        if ac_len < 1e-3 or ba_len < 1e-3 or bc_len < 1e-3:
            return 0.0
        # θ = 外角（0 = 直线，π = 180° 急转）= π - ∠ABC
        cos_abc = (vax * vcx + vay * vcy) / (ba_len * bc_len)
        cos_abc = max(-1.0, min(1.0, cos_abc))
        angle_abc = math.acos(cos_abc)
        theta = math.pi - angle_abc
        if theta < 1e-4:
            return 0.0
        # R = (AC/2) / sin(θ/2)
        R = (ac_len * 0.5) / math.sin(theta * 0.5)
        kappa_mag = 1.0 / max(R, 1e-4)
        # 符号由 BA × BC 叉积决定（z 分量）：>0 左转，<0 右转
        cross = vax * vcy - vay * vcx
        return kappa_mag if cross >= 0.0 else -kappa_mag

    # ── 壁面贴边 PD + 朝向修正（直接使用激光数据）──────────────────────
    def _edge_follow_pd(self):
        """
        改进版贴边控制：同时修正侧向距离误差和机器人朝向（与墙面平行）。

        朝向估算：
          在侧向扇区前半段（±15°偏移）和后半段各取最近距离，
          两者之差 / 两段的前后间距 ≈ sin(偏斜角)，可估算机器人偏离墙方向的角度。

        控制律：
          omega = sign * (kp_wall * dist_err + kd_wall * d_dist_err
                         + k_heading * heading_err)
        """
        # 侧向距离（90° 处）
        wall_d   = self._get_side_dist(self.wall_side)
        dist_err = self.wall_ref - wall_d
        derr     = (dist_err - self._wall_err_prev) / max(1e-3, self.dt)
        self._wall_err_prev = dist_err

        # 朝向误差估计（比较前侧和后侧的壁面距离差）
        heading_err = self._estimate_wall_heading_err(self.wall_side)

        # 符号规则：
        #   wall_side='right': 墙在右; dist_err>0 表示太近→向左; heading_err>0 表示头朝墙→向右
        #   wall_side='left':  墙在左; dist_err>0 表示太近→向右; heading_err>0 表示头朝墙→向左
        sign  = 1.0 if self.wall_side == 'right' else -1.0
        omega = sign * (self.kp_wall * dist_err + self.kd_wall * derr
                        - 0.5 * heading_err)   # heading修正方向与距离修正相反

        v = max(self.v_min, self.speed_limit * 0.65)
        return v, omega, 0.0

    def _estimate_wall_heading_err(self, side: str) -> float:
        """
        估算机器人朝向与墙面的偏差角（弧度）。
        正值 = 机器人前端比后端更靠近墙（头朝墙）。
        方法：取侧向扇区内前 1/3 和后 1/3 的最近距离之差。
        """
        if not self.scan_ranges:
            return 0.0
        front_d = 5.0
        rear_d  = 5.0
        # 前段：60°~80°；后段：100°~120°（相对于机器人正前方）
        front_min = math.radians(60)
        front_max = math.radians(80)
        rear_min  = math.radians(100)
        rear_max  = math.radians(120)
        for i, r in enumerate(self.scan_ranges):
            if not (0.05 < r < 4.0):
                continue
            angle = self.scan_angle_min + i * self.scan_angle_inc
            abs_a = abs(angle)
            # 只看目标侧
            is_target = (side == 'right' and angle < 0) or (side == 'left' and angle > 0)
            if not is_target:
                continue
            if front_min <= abs_a <= front_max:
                front_d = min(front_d, r)
            elif rear_min <= abs_a <= rear_max:
                rear_d  = min(rear_d, r)
        # 差值越大表示偏角越大；归一化到 ~弧度
        diff = front_d - rear_d
        # 两段采样点间的纵向距离约为 scan_separation（robot_body_length ≈ 0.8m）
        scan_sep = 0.6   # 前后采样圆弧对应的纵向间距估计（m）
        heading_err = math.atan2(diff, scan_sep) if abs(diff) < 1.5 else 0.0
        return heading_err

    def _get_side_dist(self, side: str) -> float:
        """从激光数据获取指定侧（left/right）90°方向的最近壁面距离。"""
        if not self.scan_ranges:
            return self.wall_ref + 1.0
        best = 5.0
        for i, r in enumerate(self.scan_ranges):
            if not (0.05 < r < 4.0):
                continue
            angle = self.scan_angle_min + i * self.scan_angle_inc
            abs_a = abs(angle)
            if not (self.side_a_min <= abs_a <= self.side_a_max):
                continue
            if side == 'right' and angle < 0:
                best = min(best, r)
            elif side == 'left' and angle > 0:
                best = min(best, r)
        return best


def main():
    rclpy.init()
    node = ControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
