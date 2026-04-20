#!/usr/bin/env python3
"""
竞赛评分日志记录节点 — score_logger_node

记录所有评分维度的实时数据：
  ① 避障规划能力  (30分)：障碍物检测/绕行事件、碰撞判定
  ② 清扫覆盖率    (15分)：实时覆盖率百分比
  ③ 贴边清扫精度  (15分)：EDGE_FOLLOW 模式时长与壁面距离
  ④ 动态避障能力  (25分)：动态障碍物检测/回避事件
  ⑤ 限宽门通过    (15分)：NARROW_GATE 模式事件、门通过确认

输出：
  ~/.ros/sweeper_logs/run_<timestamp>/
    ├── trajectory.csv      — 10 Hz 轨迹记录 (所有指标)
    ├── events.jsonl        — 事件流 (模式切换/里程碑/告警)
    └── summary.json        — 最终评分摘要 (关机时生成)
"""
import csv
import json
import math
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, PoseArray
from std_msgs.msg import String, Float32
import tf_transformations as tft


LOG_BASE = os.path.expanduser('~/.ros/sweeper_logs')

# 评分维度权重（供摘要计算）
SCORE_WEIGHTS = {
    'obstacle_avoidance':  30,
    'coverage_rate':       15,
    'edge_follow':         15,
    'dynamic_avoidance':   25,
    'gate_passage':        15,
}


def yaw_from_quat(q):
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class ScoreLoggerNode(Node):
    def __init__(self):
        super().__init__('score_logger_node')

        # ── 创建本次运行的日志目录 ──────────────────────────────────────
        ts = int(time.time())
        self.run_dir = os.path.join(LOG_BASE, f'run_{ts}')
        os.makedirs(self.run_dir, exist_ok=True)

        self.traj_path    = os.path.join(self.run_dir, 'trajectory.csv')
        self.events_path  = os.path.join(self.run_dir, 'events.jsonl')
        self.summary_path = os.path.join(self.run_dir, 'summary.json')

        # 写 CSV 表头
        self._traj_file = open(self.traj_path, 'w', newline='')
        self._csv = csv.writer(self._traj_file)
        self._csv.writerow([
            't_wall', 't_sim',
            'x', 'y', 'yaw', 'v',              # 位姿+速度
            'mode',                              # 行为模式
            'coverage_pct',                      # 覆盖率 (%)
            'path_progress',                     # 路径进度 (0-1)
            'n_static_obs', 'n_dyn_obs',         # 障碍物数量
            'stuck', 'recovery',                 # 卡死/恢复标志
        ])
        self._events_file = open(self.events_path, 'w')

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Odometry,        '/odom',                       self._cb_odom,  sensor_qos)
        self.create_subscription(String,          '/behavior/mode',              self._cb_mode,  5)
        self.create_subscription(Float32,         '/coverage/coverage_pct',      self._cb_cov,   5)
        self.create_subscription(Float32,         '/planner/path_progress',      self._cb_prog,  5)
        self.create_subscription(PolygonStamped,  '/perception/obstacle_points', self._cb_obs,   sensor_qos)
        self.create_subscription(PoseArray,       '/perception/dynamic_obstacles', self._cb_dyn, sensor_qos)
        self.create_subscription(String,          '/mission/status',             self._cb_mission, 5)

        # ── 状态变量 ──────────────────────────────────────────────────
        self.t0          = time.time()
        self.robot       = None          # (x, y, yaw, vx, vy)
        self.mode        = 'UNKNOWN'
        self.prev_mode   = 'UNKNOWN'
        self.cov_pct     = 0.0
        self.prog        = 0.0
        self.n_static    = 0
        self.n_dyn       = 0
        self.mission     = {}

        # 事件统计
        self.mode_durations: dict = {}   # mode → 累计秒数
        self._mode_start   = time.time()
        self.mode_changes  = 0
        self.gate_events   = []
        self.obstacle_events = []
        self.dyn_events    = []
        self.stuck_events  = []
        self.cov_milestones_hit = []     # 已触发的覆盖率里程碑 (%)

        # 里程碑阈值
        self._cov_milestones = [25, 50, 75, 80, 90, 95]
        # 贴边清扫统计
        self._edge_start     = None
        self._edge_total_s   = 0.0
        # 位移卡死检测
        self._last_pos       = None
        self._last_move_t    = time.time()
        self._in_stuck       = False

        self.create_timer(0.1, self._tick)   # 10 Hz 记录
        self.get_logger().info(
            f'[ScoreLogger] 日志目录: {self.run_dir}')

    # ── 回调 ────────────────────────────────────────────────────────
    def _cb_odom(self, msg: Odometry):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        vx  = msg.twist.twist.linear.x
        vy  = msg.twist.twist.linear.y
        self.robot = (x, y, yaw, vx, vy)

    def _cb_mode(self, msg: String):
        new_mode = msg.data
        if new_mode != self.mode:
            now = time.time()
            elapsed = now - self._mode_start
            self.mode_durations[self.mode] = \
                self.mode_durations.get(self.mode, 0.0) + elapsed
            self._mode_start = now
            self.mode_changes += 1
            self._log_event('mode_change', {
                'from': self.mode, 'to': new_mode,
                'duration_s': round(elapsed, 2)
            })
            self.prev_mode = self.mode
            self.mode = new_mode

            # 限宽门检测：进入 NARROW_GATE 模式
            if new_mode == 'NARROW_GATE':
                pos = (round(self.robot[0], 1), round(self.robot[1], 1)) \
                      if self.robot else None
                self.gate_events.append({'t': round(now - self.t0, 2), 'pos': pos, 'result': 'entering'})
                self._log_event('gate_entering', {'pos': pos})
            # 离开 NARROW_GATE 模式 = 通过
            if self.prev_mode == 'NARROW_GATE' and new_mode not in ('STOP',):
                if self.gate_events and self.gate_events[-1]['result'] == 'entering':
                    self.gate_events[-1]['result'] = 'passed'
                    self._log_event('gate_passed', {})

            # 贴边清扫计时
            if new_mode == 'EDGE_FOLLOW':
                self._edge_start = now
            elif self.prev_mode == 'EDGE_FOLLOW' and self._edge_start:
                self._edge_total_s += now - self._edge_start
                self._edge_start = None

            # 动态避障事件
            if new_mode == 'DYNAMIC_AVOID':
                self.dyn_events.append({'t': round(now - self.t0, 2)})
                self._log_event('dynamic_avoid_start', {})

    def _cb_cov(self, msg: Float32):
        self.cov_pct = float(msg.data)
        # 覆盖率里程碑
        for m in self._cov_milestones:
            if self.cov_pct >= m and m not in self.cov_milestones_hit:
                self.cov_milestones_hit.append(m)
                t = round(time.time() - self.t0, 1)
                self._log_event('coverage_milestone', {'pct': m, 'elapsed_s': t})
                self.get_logger().info(f'[ScoreLogger] 覆盖率里程碑: {m}% @ {t:.1f}s')

    def _cb_prog(self, msg: Float32):
        self.prog = float(msg.data)

    def _cb_obs(self, msg: PolygonStamped):
        n = len(msg.polygon.points)
        if n != self.n_static:
            if n > self.n_static:
                self._log_event('static_obstacle_detected', {'count': n})
                self.obstacle_events.append(
                    {'t': round(time.time() - self.t0, 2), 'count': n})
            self.n_static = n

    def _cb_dyn(self, msg: PoseArray):
        self.n_dyn = len(msg.poses)

    def _cb_mission(self, msg: String):
        try:
            self.mission = json.loads(msg.data)
        except Exception:
            pass

    # ── 主记录循环 ───────────────────────────────────────────────────
    def _tick(self):
        if self.robot is None:
            return
        now  = time.time()
        rx, ry, ryaw, vx, vy = self.robot
        v    = math.hypot(vx, vy)
        t_w  = now - self.t0
        t_s  = self.get_clock().now().nanoseconds * 1e-9

        # 卡死检测
        if self._last_pos:
            moved = math.hypot(rx - self._last_pos[0], ry - self._last_pos[1])
            if moved > 0.05:
                self._last_move_t = now
                if self._in_stuck:
                    self._in_stuck = False
                    self._log_event('recovery_end', {})
            elif now - self._last_move_t > 3.5 and not self._in_stuck:
                self._in_stuck = True
                self.stuck_events.append({'t': round(t_w, 2), 'pos': (round(rx,2), round(ry,2))})
                self._log_event('stuck_detected', {'pos': (round(rx,2), round(ry,2))})
        self._last_pos = (rx, ry)

        # 写 CSV
        self._csv.writerow([
            f'{t_w:.2f}', f'{t_s:.2f}',
            f'{rx:.3f}', f'{ry:.3f}', f'{ryaw:.3f}', f'{v:.3f}',
            self.mode,
            f'{self.cov_pct:.2f}',
            f'{self.prog:.4f}',
            self.n_static, self.n_dyn,
            int(self._in_stuck), int(self.mode == 'COVERAGE' and v < 0.05),
        ])

    # ── 事件记录工具 ─────────────────────────────────────────────────
    def _log_event(self, event_type: str, data: dict):
        now = time.time()
        entry = {
            't_wall': round(now - self.t0, 2),
            't_sim':  round(self.get_clock().now().nanoseconds * 1e-9, 2),
            'event':  event_type,
            **data,
        }
        self._events_file.write(json.dumps(entry, ensure_ascii=False) + '\n')
        self._events_file.flush()

    # ── 关机时生成摘要 ───────────────────────────────────────────────
    def destroy_node(self):
        now = time.time()
        duration = now - self.t0

        # 关闭贴边计时
        if self._edge_start:
            self._edge_total_s += now - self._edge_start

        # 关闭当前模式计时
        self.mode_durations[self.mode] = \
            self.mode_durations.get(self.mode, 0.0) + (now - self._mode_start)

        # 门通过统计
        gates_passed  = sum(1 for g in self.gate_events if g.get('result') == 'passed')
        gates_entered = len(self.gate_events)

        # 避障统计
        n_static_avoid  = len(self.obstacle_events)
        n_dyn_avoid     = len(self.dyn_events)

        # 卡死统计
        n_stuck = len(self.stuck_events)

        # 评分估算（按评分细则比例）
        cov_score  = min(15.0, 15.0 * self.cov_pct / 80.0)
        edge_score = min(15.0, 15.0 * min(self._edge_total_s, 60.0) / 60.0)
        gate_score = min(15.0, 7.5 * gates_passed)
        obs_score  = min(30.0, 30.0 * (1.0 - 0.2 * n_stuck))
        dyn_score  = min(25.0, 12.5 * min(n_dyn_avoid, 2))
        total_est  = cov_score + edge_score + gate_score + obs_score + dyn_score

        summary = {
            'run_dir':         self.run_dir,
            'duration_s':      round(duration, 1),
            'trajectory_rows': self._get_csv_rows(),

            # ── 各评分维度 ──
            'scoring': {
                '避障规划能力_30分': {
                    'static_avoid_events': n_static_avoid,
                    'stuck_count':         n_stuck,
                    'stuck_details':       self.stuck_events,
                    'estimated_score':     round(obs_score, 1),
                },
                '清扫覆盖率_15分': {
                    'final_pct':           round(self.cov_pct, 2),
                    'milestones_hit':      self.cov_milestones_hit,
                    'estimated_score':     round(cov_score, 1),
                },
                '贴边清扫精度_15分': {
                    'edge_follow_s':       round(self._edge_total_s, 1),
                    'mode_duration_s':     round(self.mode_durations.get('EDGE_FOLLOW', 0), 1),
                    'estimated_score':     round(edge_score, 1),
                },
                '动态避障能力_25分': {
                    'dyn_avoid_count':     n_dyn_avoid,
                    'dyn_events':          self.dyn_events,
                    'estimated_score':     round(dyn_score, 1),
                },
                '限宽门通过_15分': {
                    'gates_entered':       gates_entered,
                    'gates_passed':        gates_passed,
                    'gate_details':        self.gate_events,
                    'estimated_score':     round(gate_score, 1),
                },
            },
            'estimated_total_score': round(total_est, 1),

            # ── 运行诊断 ──
            'diagnostics': {
                'mode_durations_s':    {k: round(v, 1) for k, v in self.mode_durations.items()},
                'mode_changes':        self.mode_changes,
                'final_coverage_pct':  round(self.cov_pct, 2),
                'mission_status':      self.mission,
            },
        }

        # 写摘要文件
        try:
            with open(self.summary_path, 'w', encoding='utf-8') as f:
                json.dump(summary, f, indent=2, ensure_ascii=False)
        except Exception as e:
            self.get_logger().error(f'摘要写入失败: {e}')

        # 关闭文件
        try:
            self._traj_file.close()
            self._events_file.close()
        except Exception:
            pass

        self.get_logger().info(
            f'\n{"="*55}\n'
            f' 评分摘要  →  {self.summary_path}\n'
            f'  时长: {duration:.0f}s   覆盖率: {self.cov_pct:.1f}%\n'
            f'  估算总分: {total_est:.1f}/100\n'
            f'    避障: {obs_score:.1f}/30   覆盖率: {cov_score:.1f}/15\n'
            f'    贴边: {edge_score:.1f}/15  动态: {dyn_score:.1f}/25\n'
            f'    限宽门: {gate_score:.1f}/15\n'
            f'{"="*55}')
        super().destroy_node()

    def _get_csv_rows(self):
        try:
            with open(self.traj_path) as f:
                return sum(1 for _ in f) - 1  # 减去表头
        except Exception:
            return 0


def main():
    rclpy.init()
    node = ScoreLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
