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

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 5)

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

        # 恢复状态
        self._recovering  = False
        self._rec_end_t   = 0.0

        # 壁面 PD 状态
        self._wall_err_prev = 0.0

        # 状态日志（每 5s 输出一次）
        self._log_v = 0.0
        self._log_w = 0.0
        self._trajectory_received = False  # 等待 RViz 绿色轨迹
        self._path_entry_fixed    = False  # 首次接收路径时强制从索引0出发
        self._log_front_d = 999.0
        self._log_avoid = 'none'
        self._saved_ref_idx = 0     # 避障时保存的路径索引，恢复时使用

        self.create_timer(self.dt, self.tick)
        self.create_timer(5.0, self._log_status)
        self.get_logger().info('controller_node ready')

    # ── 回调 ────────────────────────────────────────────────────────────
    def cb_ref(self, msg: Path):
        self.ref = [(p.pose.position.x, p.pose.position.y,
                     yaw_from_quat(p.pose.orientation))
                    for p in msg.poses]
        self._trajectory_received = True  # RViz 绿色轨迹已到达，可以发车

        if self.robot is not None and self.ref:
            # 仅在避障恢复时使用最近点搜索；首次启动时强制从轨迹起点(索引0)出发，
            # 避免最近的点跳到轨迹中间导致里程浪费。
            rx, ry, _, _ = self.robot
            if self._path_entry_fixed:
                # 已在轨迹上，进行避障恢复式最近点搜索
                best_d = float('inf')
                best_i = 0
                for i, (px, py, _) in enumerate(self.ref):
                    d = math.hypot(rx - px, ry - py)
                    if d < best_d:
                        best_d = d
                        best_i = i
                self.ref_idx_hint = best_i
            else:
                # 首次接收路径：强制从轨迹起点出发，不做最近点搜索
                self.ref_idx_hint = 0
                self._path_entry_fixed = True
        else:
            self.ref_idx_hint = 0

    def cb_odom(self, msg: Odometry):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        v   = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.robot = (x, y, yaw, v)

        # 路径早到、odom 晚到：确保路径入口固定到起点
        if self.ref and not self._path_entry_fixed:
            self.ref_idx_hint = 0
            self._path_entry_fixed = True

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

    def cb_speed(self, msg: Float32):
        self.speed_limit = float(msg.data)

    def cb_scan(self, msg: LaserScan):
        self.scan_ranges    = list(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_inc = msg.angle_increment

    def cb_recovery(self, msg):
        if msg.data == 'resume_coverage':
            self.ref_idx_hint = self._saved_ref_idx
            self._recovering = False
            self._pos_history.clear()
            self.get_logger().info(
                f'恢复 Coverage: ref_idx_hint={self._saved_ref_idx}')

    # ── 主控制循环 ───────────────────────────────────────────────────────
    def tick(self):
        cmd = Twist()
        if self.robot is None:
            self.pub_cmd.publish(cmd)
            return

        rx, ry, ryaw, rv = self.robot
        now_ns = self.get_clock().now().nanoseconds()

        # ── Stuck 检测（净位移版）──────────────────────────────────────
        now_s = now_ns / 1e9
        self._pos_history.append((now_s, rx, ry))
        # 清除超出窗口的历史
        cutoff = now_s - self.stuck_win
        self._pos_history = [(t, x, y) for t, x, y in self._pos_history
                             if t >= cutoff]

        stuck = False
        if len(self._pos_history) >= 3:
            t0, x0, y0 = self._pos_history[0]
            net_disp = math.hypot(rx - x0, ry - y0)
            if (now_s - t0 >= self.stuck_win * 0.8 and
                    net_disp < self.stuck_thr and
                    self.mode in ('COVERAGE', 'STATIC_DETOUR')):
                stuck = True

        # ── 恢复逻辑（前方有障碍时后退转向，无障碍时前进）──────────────
        if self._recovering:
            if now_s < self._rec_end_t:
                fd, sd = self._front_obstacle_info()
                if fd < self.obs_steer_d:
                    cmd.linear.x  = float(-self.rec_spd * 0.5)
                    cmd.angular.z = float(sd * 0.4)
                else:
                    cmd.linear.x  = float(self.rec_spd)
                    cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)
                return
            else:
                self._recovering = False
                self.ref_idx_hint = self._saved_ref_idx  # 回到障前保存的位置
                self._pos_history.clear()

        if stuck and not self._recovering:
            self._saved_ref_idx = self.ref_idx_hint  # 进入恢复前保存当前位置
            self.get_logger().warn('Stuck 检测触发 — 恢复模式')
            self._recovering = True
            self._rec_end_t  = now_s + self.rec_hold
            fd, sd = self._front_obstacle_info()
            if fd < self.obs_steer_d:
                cmd.linear.x  = float(-self.rec_spd * 0.5)
                cmd.angular.z = float(sd * 0.4)
            else:
                cmd.linear.x  = float(self.rec_spd)
                cmd.angular.z = 0.0
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
        # U-Turn 区间内用 Pure Pursuit，其他情况用 Stanley
        if self._is_in_uturn():
            v, omega = self._pure_pursuit_track(rx, ry, ryaw)
        elif self.mode == 'EDGE_FOLLOW':
            v, omega = self._edge_follow_pd()
        else:
            v, omega = self._stanley_track(rx, ry, ryaw, rv)

        # 应用速度限制
        v = min(v, self.speed_limit)

        # 前方障碍物反应式避障（路径弯道时抑制，避免干扰 U-turn）
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
            omega = steer_dir * self.avoid_gain
            emergency_stop = True
            avoid_label = 'STOP'
            self.get_logger().warn(
                f'前方障碍 {front_d:.2f}m — 紧急转向',
                throttle_duration_sec=1.0)
        elif front_d < eff_steer:
            t = (front_d - eff_stop) / max(0.01, eff_steer - eff_stop)
            v *= t
            v = max(v, self.v_min * 0.3)
            avoid_w = steer_dir * self.avoid_gain * (1.0 - t)
            omega = omega * t + avoid_w
            avoid_label = 'steer'
        elif front_d < eff_slow:
            t = (front_d - eff_steer) / max(0.01, eff_slow - eff_steer)
            v *= (0.5 + 0.5 * t)
            avoid_label = 'slow'
        self._log_avoid = avoid_label

        # 运动学约束（Ackermann 模型：omega = v × tan(δ) / L）
        omega_kin_max = max(0.01, v) * math.tan(self.delta_max) / self.L
        if emergency_stop:
            omega = max(-self.w_max, min(self.w_max, omega))
        else:
            if abs(omega) > 0.05:
                v = max(v, self.v_min)
            omega = max(-min(self.w_max, omega_kin_max),
                        min(min(self.w_max, omega_kin_max), omega))

        self._log_v = v
        self._log_w = omega
        # PP 模式下强制 0.2 硬上限（U-turn 时允许蹭过去）
        if self._is_in_uturn():
            v = 0.2
        cmd.linear.x  = float(v)
        cmd.angular.z = float(omega)
        self.pub_cmd.publish(cmd)

    # ── 路径弯道检测（U-turn 即将到来时抑制避障）───────────────────────
    def _has_upcoming_turn(self) -> bool:
        pts = self.ref
        if len(pts) < 5:
            return False
        idx = self.ref_idx_hint
        offsets = [3, 6, 10, 14]  # 直线段: 1.5m~7m 预警; 弧段: 0.5m~2.3m 触发 PP
        for offset in offsets:
            ci = min(len(pts) - 1, idx + offset)
            kappa = abs(self._estimate_curvature(ci))
            if kappa > 0.10:
                return True
        return False

    def _is_in_uturn(self) -> bool:
        """检测当前是否处于 U-Turn 区间内（而不仅是即将到来）。"""
        pts = self.ref
        if len(pts) < 5:
            return False
        idx = self.ref_idx_hint
        total_kappa = 0.0
        count = 0
        for delta in range(-3, 4):
            ci = min(len(pts) - 1, max(0, idx + delta))
            total_kappa += abs(self._estimate_curvature(ci))
            count += 1
        avg_kappa = total_kappa / count
        return avg_kappa > 0.3

    def _pure_pursuit_track(self, rx, ry, ryaw):
        """Pure Pursuit：朝 look-ahead 点 steering，保证走规划路径。"""
        pts = self.ref
        n   = len(pts)
        idx = self.ref_idx_hint

        L_DA = 0.6  # look-ahead 距离（U-turn 时用短距离，更精准）

        best_d  = float('inf')
        best_i  = idx
        search_lo = max(0, idx - 5)
        search_hi = min(n - 1, idx + 50)
        for i in range(search_lo, search_hi + 1):
            d = math.hypot(rx - pts[i][0], ry - pts[i][1])
            if d < best_d:
                best_d = d
                best_i = i

        la_i = best_i
        for i in range(best_i, min(n - 1, best_i + 50)):
            d = math.hypot(rx - pts[i][0], ry - pts[i][1])
            if d >= L_DA:
                la_i = i
                break

        lx, ly, _ = pts[la_i]
        dx = lx - rx
        dy = ly - ry
        alpha = wrap(math.atan2(dy, dx) - ryaw)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), L_DA)
        delta = max(-self.delta_max, min(self.delta_max, delta))

        v = 0.2  # U-turn 目标速度：0.2 m/s
        omega = v * math.tan(delta) / self.L
        return v, omega

    # ── 5 秒状态日志 ─────────────────────────────────────────────────
    def _log_status(self):
        if self.robot is None:
            return
        rx, ry, ryaw, rv = self.robot
        turn = self._has_upcoming_turn()
        self.get_logger().info(
            f'[控制] 模式={self.mode} | '
            f'指令速度={self._log_v:.2f} 指令角速度={self._log_w:.2f} '
            f'实际速度={rv:.2f} | '
            f'前方距离={self._log_front_d:.2f}m 避障状态={self._log_avoid} '
            f'弯道预判={"是" if turn else "否"} | '
            f'参考路径点数={len(self.ref)} 跟踪索引={self.ref_idx_hint} | '
            f'位置=({rx:.1f},{ry:.1f}) 航向={math.degrees(ryaw):.0f}°')

    # ── 前方障碍物扫描 ─────────────────────────────────────────────────
    def _front_obstacle_info(self):
        """扫描前方锥形区域，返回 (最小距离, 转向方向)。
        转向方向: +1 = 向左转 (omega>0), -1 = 向右转（朝更开阔的一侧）。
        近距离障碍物自动扩大检测角度以覆盖车身宽度。
        """
        if not self.scan_ranges:
            return 999.0, 0.0
        if self.robot is None:
            return 999.0, 0.0
        rx, ry, ryaw, _ = self.robot
        min_dist  = 999.0
        left_min  = 999.0
        right_min = 999.0
        for i, r in enumerate(self.scan_ranges):
            if not (0.05 < r < 10.0):
                continue
            angle = self.scan_angle_min + i * self.scan_angle_inc
            if abs(angle) > self.front_half:
                continue
            # 避障层忽略墙壁回波，墙壁由规划层和贴边控制处理
            wx = rx + r * math.cos(angle + ryaw)
            wy = ry + r * math.sin(angle + ryaw)
            if self._is_wall_point(wx, wy):
                continue
            if r < min_dist:
                min_dist = r
            if angle >= 0:
                left_min = min(left_min, r)
            else:
                right_min = min(right_min, r)
        steer_dir = 1.0 if right_min < left_min else -1.0
        return min_dist, steer_dir

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
        lo  = max(0, self.ref_idx_hint - 5)
        hi  = min(n - 1, self.ref_idx_hint + 100)
        sub = pts[lo: hi + 1]

        best_d = float('inf')
        best_i = lo
        for i, (px, py, _) in enumerate(sub):
            d = math.hypot(rx - px, ry - py)
            if d < best_d:
                best_d = d
                best_i = lo + i

        if best_d > 2.0:
            for i, (px, py, _) in enumerate(pts):
                d = math.hypot(rx - px, ry - py)
                if d < best_d:
                    best_d = d
                    best_i = i

        if best_i > self.ref_idx_hint:
            self.ref_idx_hint = best_i

        idx = self.ref_idx_hint
        nxt = min(idx + 1, n - 1)
        path_yaw = math.atan2(pts[nxt][1] - pts[idx][1],
                               pts[nxt][0] - pts[idx][0])
        heading_err = wrap(path_yaw - ryaw)

        px, py, _ = pts[idx]
        dx = rx - px;  dy = ry - py
        cte = -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy

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
        v_sched = self.v_target / (1.0 + self.curv_gain * abs(kappa))
        v_sched = max(self.v_min, v_sched)

        # ── 曲率速度调度 ─────────────────────────────────────────────────
        # 急弯时（kappa 较大）降低速度，确保角速度不超过物理极限
        omega_max_diff = self.v_target * math.tan(self.delta_max) / self.L
        kappa_abs = abs(kappa)
        if kappa_abs > 1e-4:
            v_needed = omega_max_diff / kappa_abs
            v_sched = max(v_sched, min(self.v_min * 1.5, v_needed))

        # ── 角速度（ω）计算 ───────────────────────────────────────────────
        # URDF tricycle_drive 插件期望 ω = v × tan(δ) / L
        # 其中 L = wheel_base = 1.05m（前后轴中心距）
        omega = v_sched * math.tan(delta) / self.L

        # ── 弯道预警降速（不等 PP 触发，提前降速）────────────────────────
        if self._has_upcoming_turn():
            v_sched *= 0.3   # 预警阶段主动降速 70%，提前约 3s 开始减速

        return v_sched, omega

    def _estimate_curvature(self, idx: int) -> float:
        pts = self.ref
        n   = len(pts)
        i0  = max(0, idx - 3)
        i2  = min(n - 1, idx + 3)
        if i2 <= i0:
            return 0.0
        dx = pts[i2][0] - pts[i0][0]
        dy = pts[i2][1] - pts[i0][1]
        if math.hypot(dx, dy) < 1e-3:
            return 0.0
        da  = wrap(math.atan2(dy, dx) - math.atan2(
            pts[min(n-1, idx+1)][1] - pts[max(0, idx-1)][1],
            pts[min(n-1, idx+1)][0] - pts[max(0, idx-1)][0]))
        arc = math.hypot(dx, dy)
        return da / arc

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
        return v, omega

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
