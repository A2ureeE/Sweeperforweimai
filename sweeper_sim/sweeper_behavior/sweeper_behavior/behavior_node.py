#!/usr/bin/env python3
"""
Behavior arbiter FSM.

States: COVERAGE | NARROW_GATE | STATIC_DETOUR | DYNAMIC_AVOID | EDGE_FOLLOW | STOP

已修复的关键缺陷：
  1. 障碍物坐标系错误 — perception 发布的是世界坐标，必须先转换为机器人相对距离
  2. EDGE_FOLLOW 模式从未触发 — _near_wall() 虽定义但从未在 tick() 中调用
  3. 门检测无迟滞 — 添加帧计数迟滞，避免频繁切换
  4. 动态障碍位置也需世界→局部转换

Publishes:
  /behavior/mode        std_msgs/String   (10 Hz)
  /behavior/speed_limit std_msgs/Float32  (10 Hz)
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PolygonStamped, PoseArray
from std_msgs.msg import String, Float32
import tf_transformations as tft


def yaw_from_quat(q):
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def world_to_robot(wx, wy, rx, ry, ryaw):
    """Convert world-frame point to robot-relative distance."""
    dx = wx - rx
    dy = wy - ry
    dist = math.hypot(dx, dy)
    # bearing in robot frame
    bearing = math.atan2(dy, dx) - ryaw
    lx =  dist * math.cos(bearing)   # forward axis
    ly =  dist * math.sin(bearing)   # left axis
    return lx, ly, dist


class BehaviorNode(Node):
    def __init__(self):
        super().__init__('behavior_node')
        self.declare_parameters('', [
            ('emergency_brake_dist',  0.35),
            ('dynamic_avoid_dist',    1.8),
            ('static_detour_dist',    1.2),
            ('narrow_gate_dist',      3.0),
            ('edge_follow_dist',      0.55),
            ('edge_follow_front_deg', 60.0),   # front cone to check for edge
            ('wall_scan_angle_deg',  30.0),
            ('normal_speed',          0.8),
            ('narrow_gate_speed',     0.4),
            ('detour_speed',          0.5),
            # Gate hysteresis: require N consecutive 'approaching' frames
            ('gate_hysteresis_frames', 3),
        ])
        g = self.get_parameter
        self.emerg_dist      = g('emergency_brake_dist').value
        self.dyn_dist        = g('dynamic_avoid_dist').value
        self.static_dist     = g('static_detour_dist').value
        self.gate_dist       = g('narrow_gate_dist').value
        self.edge_dist       = g('edge_follow_dist').value
        self.edge_front_deg  = math.radians(g('edge_follow_front_deg').value)
        self.wall_angle      = math.radians(g('wall_scan_angle_deg').value)
        self.normal_speed    = g('normal_speed').value
        self.gate_speed      = g('narrow_gate_speed').value
        self.detour_speed    = g('detour_speed').value
        self.gate_hyst       = g('gate_hysteresis_frames').value

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_odom  = self.create_subscription(
            Odometry, '/odom', self.cb_odom, sensor_qos)
        self.sub_scan  = self.create_subscription(
            LaserScan, '/scan', self.cb_scan, sensor_qos)
        self.sub_obs   = self.create_subscription(
            PolygonStamped, '/perception/obstacle_points', self.cb_obs, sensor_qos)
        self.sub_dyn   = self.create_subscription(
            PoseArray, '/perception/dynamic_obstacles', self.cb_dyn, sensor_qos)
        self.sub_gate  = self.create_subscription(
            String, '/perception/gate_event', self.cb_gate, 5)

        self.pub_mode  = self.create_publisher(String,  '/behavior/mode',        5)
        self.pub_speed = self.create_publisher(Float32, '/behavior/speed_limit', 5)

        # Robot state (world frame)
        self._robot_pos  = (0.0, 0.0)
        self.robot_yaw   = 0.0

        # Sensor data (world-frame obstacle coords from perception)
        self.obstacles_world: list = []    # [(wx, wy), ...]
        self.dyn_obstacles_world: list = [] # [(wx, wy, vx, vy, speed), ...]

        # Scan data (raw, laser frame)
        self.scan_ranges    = []
        self.scan_angle_min = 0.0
        self.scan_angle_inc = 0.0

        # Gate hysteresis counter
        self._gate_frames     = 0
        self._gate_active     = False

        self.create_timer(0.1, self.tick)
        self.get_logger().info('behavior_node ready')

    # ── 回调 ────────────────────────────────────────────────────────────
    def cb_odom(self, msg: Odometry):
        self.robot_yaw = yaw_from_quat(msg.pose.pose.orientation)
        self._robot_pos = (msg.pose.pose.position.x,
                           msg.pose.pose.position.y)

    def cb_scan(self, msg: LaserScan):
        self.scan_ranges    = list(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_inc = msg.angle_increment

    def cb_obs(self, msg: PolygonStamped):
        # World-frame coordinates from perception
        self.obstacles_world = [(float(p.x), float(p.y))
                                for p in msg.polygon.points]

    def cb_dyn(self, msg: PoseArray):
        out = []
        for p in msg.poses:
            yaw = tft.euler_from_quaternion(
                [p.orientation.x, p.orientation.y,
                 p.orientation.z, p.orientation.w])[2]
            sp = float(p.position.z)
            out.append((float(p.position.x), float(p.position.y),
                        sp * math.cos(yaw), sp * math.sin(yaw), sp))
        self.dyn_obstacles_world = out

    def cb_gate(self, msg: String):
        if msg.data == 'approaching':
            self._gate_frames = min(self._gate_frames + 1, self.gate_hyst + 2)
        else:
            self._gate_frames = max(0, self._gate_frames - 1)
        self._gate_active = (self._gate_frames >= self.gate_hyst)

    # ── 工具函数 ─────────────────────────────────────────────────────────
    def _front_clear_dist(self) -> float:
        """前方 ±30° 扇区最小测距（激光帧）。"""
        if not self.scan_ranges:
            return 999.0
        best = 999.0
        for i, r in enumerate(self.scan_ranges):
            angle = self.scan_angle_min + i * self.scan_angle_inc
            if abs(angle) <= self.wall_angle and 0.05 < r < 20.0:
                best = min(best, r)
        return best

    def _nearest_static_dist(self) -> float:
        """最近静态障碍物到机器人的真实欧氏距离（世界坐标→机器人坐标）。"""
        if not self.obstacles_world:
            return 999.0
        rx, ry = self._robot_pos
        best = 999.0
        for wx, wy in self.obstacles_world:
            d = math.hypot(wx - rx, wy - ry)
            if d < best:
                best = d
        return best

    def _nearest_dynamic_dist(self) -> float:
        """最近动态障碍物到机器人的真实距离。"""
        if not self.dyn_obstacles_world:
            return 999.0
        rx, ry = self._robot_pos
        best = 999.0
        for wx, wy, *_ in self.dyn_obstacles_world:
            d = math.hypot(wx - rx, wy - ry)
            if d < best:
                best = d
        return best

    def _near_wall(self) -> bool:
        """
        True 当侧面有墙（用于触发 EDGE_FOLLOW）。
        检查左右 80°-100° 扇区内有无 < edge_follow_dist 的测距。
        """
        if not self.scan_ranges:
            return False
        for i, r in enumerate(self.scan_ranges):
            if not (0.05 < r < self.edge_dist + 0.3):
                continue
            angle = self.scan_angle_min + i * self.scan_angle_inc
            # 左右侧：|angle| 在 60°~120° 范围
            abs_a = abs(angle)
            if math.radians(60) < abs_a < math.radians(120):
                return True
        return False

    def _nearest_wall_side_dist(self) -> tuple:
        """
        返回 (right_dist, left_dist) — 用于选择贴哪侧。
        分别取右侧 (angle ∈ -120°~-60°) 和左侧 (angle ∈ 60°~120°) 最小值。
        """
        if not self.scan_ranges:
            return 999.0, 999.0
        right_d = 999.0
        left_d  = 999.0
        for i, r in enumerate(self.scan_ranges):
            if not (0.05 < r < 5.0):
                continue
            angle = self.scan_angle_min + i * self.scan_angle_inc
            if -math.radians(120) < angle < -math.radians(60):
                right_d = min(right_d, r)
            elif math.radians(60) < angle < math.radians(120):
                left_d = min(left_d, r)
        return right_d, left_d

    # ── 主 FSM ──────────────────────────────────────────────────────────
    def tick(self):
        mode      = 'COVERAGE'
        speed_lim = self.normal_speed

        # ① 紧急制动（最高优先）
        front_dist = self._front_clear_dist()
        if front_dist < self.emerg_dist:
            self._publish('STOP', 0.0)
            return

        # ② 动态障碍物回避（正确使用世界坐标→距离）
        dyn_d = self._nearest_dynamic_dist()
        if dyn_d < self.dyn_dist:
            mode      = 'DYNAMIC_AVOID'
            speed_lim = self.detour_speed

        # ③ 限宽门通过（带迟滞）
        if mode == 'COVERAGE' and self._gate_active:
            mode      = 'NARROW_GATE'
            speed_lim = self.gate_speed

        # ④ 静态障碍物绕行（正确使用世界坐标→距离）
        if mode == 'COVERAGE':
            static_d = self._nearest_static_dist()
            if static_d < self.static_dist:
                mode      = 'STATIC_DETOUR'
                speed_lim = self.detour_speed

        # ⑤ 贴边清扫（侧向有墙时触发）
        if mode == 'COVERAGE' and self._near_wall():
            mode      = 'EDGE_FOLLOW'
            speed_lim = self.normal_speed * 0.7
            # 发布靠哪侧（供 controller 选择参考壁）
            right_d, left_d = self._nearest_wall_side_dist()
            side = 'right' if right_d < left_d else 'left'
            self.pub_mode.publish(String(data=f'EDGE_FOLLOW_{side}'))
            self.pub_speed.publish(Float32(data=float(speed_lim)))
            return

        self._publish(mode, speed_lim)

    def _publish(self, mode: str, speed: float):
        self.pub_mode.publish(String(data=mode))
        self.pub_speed.publish(Float32(data=float(speed)))


def main():
    rclpy.init()
    node = BehaviorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
