#!/usr/bin/env python3
"""
LaserScan-based perception node.

已修复的关键缺陷：
  1. 门检测假阳性 — 旧版用全局世界坐标匹配锥桶对，跨场地的两个锥桶距离恰好
     在 [0.8, 2.5]m 时也会触发。新版：只匹配机器人前方 ±45° 扇区内、且两锥桶
     连线垂直于机器人前进方向的对。
  2. 速度估计噪声严重 — 单帧位移/dt 噪声极大，改用指数移动平均（EMA）滤波。
  3. 动态障碍物 ID 漂移 — 最近邻匹配阈值从 0.5m 收紧为 0.4m，同时加入速度
     一致性检验。
  4. 墙面识别不稳定 — 增加点数下限（>15 点）与轴向延伸比例双重判据。

Publishes:
  /perception/obstacle_points    geometry_msgs/PolygonStamped  (world frame, static)
  /perception/dynamic_obstacles  geometry_msgs/PoseArray       (world frame, moving)
  /perception/gate_event         std_msgs/String               ('approaching'|'clear')
  /perception/gate_pose          geometry_msgs/PoseStamped     (gate center + approach yaw)
"""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, PoseArray, Pose, Point32, PoseStamped
from std_msgs.msg import String
import tf_transformations as tft


def yaw_from_quat(q):
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class Track:
    """单个障碍物轨迹（带 EMA 速度滤波）。"""
    ALPHA = 0.35   # EMA 系数（越小越平滑）

    def __init__(self, cx: float, cy: float, now: float, tid: int):
        self.tid   = tid
        self.pos   = (cx, cy)
        self.vel   = (0.0, 0.0)
        self.t     = now

    def update(self, cx: float, cy: float, now: float):
        dt  = max(1e-3, now - self.t)
        raw_vx = (cx - self.pos[0]) / dt
        raw_vy = (cy - self.pos[1]) / dt
        # EMA 滤波
        self.vel = (
            self.ALPHA * raw_vx + (1 - self.ALPHA) * self.vel[0],
            self.ALPHA * raw_vy + (1 - self.ALPHA) * self.vel[1],
        )
        self.pos = (cx, cy)
        self.t   = now


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.declare_parameters('', [
            ('cluster_dist',         0.25),
            ('cluster_min_pts',      2),
            ('cluster_max_pts',      60),
            ('max_range',            8.0),
            ('wall_min_pts',         15),      # 最少点数才判定为墙
            ('wall_min_extent',      0.8),
            ('dynamic_speed_thresh', 0.15),
            ('track_timeout',        1.2),
            ('track_match_dist',     0.4),
            ('gate_dist_thresh',     4.0),
            ('gate_width_min',       0.8),
            ('gate_width_max',       2.5),
            ('gate_front_angle_deg', 50.0),    # 只检测前方扇区内的门
        ])
        g = self.get_parameter
        self.clust_dist     = g('cluster_dist').value
        self.clust_min      = g('cluster_min_pts').value
        self.clust_max      = g('cluster_max_pts').value
        self.max_range      = g('max_range').value
        self.wall_min_pts   = g('wall_min_pts').value
        self.wall_min       = g('wall_min_extent').value
        self.dyn_thresh     = g('dynamic_speed_thresh').value
        self.track_to       = g('track_timeout').value
        self.track_match    = g('track_match_dist').value
        self.gate_dist      = g('gate_dist_thresh').value
        self.gate_min       = g('gate_width_min').value
        self.gate_max       = g('gate_width_max').value
        self.gate_front_a   = math.radians(g('gate_front_angle_deg').value)

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cb_scan, sensor_qos)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.cb_odom, sensor_qos)

        self.pub_obs       = self.create_publisher(
            PolygonStamped, '/perception/obstacle_points', 5)
        self.pub_dyn       = self.create_publisher(
            PoseArray, '/perception/dynamic_obstacles', 5)
        self.pub_gate      = self.create_publisher(
            String, '/perception/gate_event', 5)
        self.pub_gate_pose = self.create_publisher(
            PoseStamped, '/perception/gate_pose', 5)

        self.robot_pose = None   # (x, y, yaw)
        self.tracks: dict[int, Track] = {}
        self._track_id = 0

        self.get_logger().info('perception_node ready')

    # ── 回调 ────────────────────────────────────────────────────────────
    def cb_odom(self, msg: Odometry):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.robot_pose = (x, y, yaw)

    def cb_scan(self, msg: LaserScan):
        if self.robot_pose is None:
            return
        rx, ry, ryaw = self.robot_pose
        now = time.time()

        # ── 将激光点转换到世界坐标系 ──
        pts = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < 0.08 or r > self.max_range:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            lx    = r * math.cos(angle)
            ly    = r * math.sin(angle)
            wx    = rx + lx * math.cos(ryaw) - ly * math.sin(ryaw)
            wy    = ry + lx * math.sin(ryaw) + ly * math.cos(ryaw)
            pts.append((wx, wy))

        if not pts:
            return

        clusters = self._cluster(pts)

        static_centers = []
        dyn_centers    = []
        front_cones    = []   # 候选门锥桶（前方扇区）

        for cl in clusters:
            arr    = np.array(cl)
            cx     = float(np.mean(arr[:, 0]))
            cy     = float(np.mean(arr[:, 1]))
            n_pts  = len(cl)

            # 障碍尺寸
            spread  = arr - arr.mean(axis=0)
            extents = np.linalg.norm(spread, axis=1)
            extent  = float(extents.max() * 2)

            # 判定为墙面（大型细长结构）
            if n_pts >= self.wall_min_pts and extent > self.wall_min:
                # 主轴判断：PCA 最大特征值/最小特征值 > 5 ⇒ 细长 = 墙
                cov = np.cov(arr.T)
                if cov.ndim == 2:
                    eigvals = np.linalg.eigvalsh(cov)
                    ratio   = max(eigvals) / (min(eigvals) + 1e-6)
                    if ratio > 4.0:
                        continue   # 跳过墙

            # 速度追踪（带 EMA 滤波）
            trk  = self._update_track(cx, cy, now)
            spd  = math.hypot(*trk.vel)

            if spd > self.dyn_thresh:
                dyn_centers.append((cx, cy, trk.vel[0], trk.vel[1], spd))
            else:
                # 机器人到障碍的距离
                dist = math.hypot(cx - rx, cy - ry)
                static_centers.append((cx, cy))
                # 门候选：小尺寸（锥桶型）+ 在机器人前方扇区内
                if (extent < 0.7 and n_pts <= 25 and dist < self.gate_dist):
                    # 检查是否在前方 ±gate_front_a 扇区
                    bearing = math.atan2(cy - ry, cx - rx) - ryaw
                    bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
                    if abs(bearing) < self.gate_front_a:
                        front_cones.append((cx, cy, bearing))

        # ── 过期轨迹清理 ──
        expired = [k for k, v in self.tracks.items() if now - v.t > self.track_to]
        for k in expired:
            del self.tracks[k]

        # ── 门检测（改进版：要求两锥桶的连线垂直于前进方向）──
        gate_event  = 'clear'
        best_gate   = None    # (center_x, center_y, approach_yaw, gap)
        best_dist   = float('inf')
        for i in range(len(front_cones)):
            cx1, cy1, b1 = front_cones[i]
            for j in range(i + 1, len(front_cones)):
                cx2, cy2, b2 = front_cones[j]
                gap = math.hypot(cx1 - cx2, cy1 - cy2)
                if not (self.gate_min <= gap <= self.gate_max):
                    continue
                # 连线方向（世界坐标）
                line_yaw = math.atan2(cy2 - cy1, cx2 - cx1)
                # 连线应垂直于机器人前进方向（robot yaw）
                perp_diff = abs(
                    ((line_yaw - ryaw + math.pi / 2 + math.pi) % (2 * math.pi)) - math.pi
                )
                # 允许 ±35° 的误差
                if perp_diff < math.radians(35):
                    gate_event = 'approaching'
                    # 记录最近的门（多扇门时取最近）
                    gcx = (cx1 + cx2) / 2.0
                    gcy = (cy1 + cy2) / 2.0
                    d_to_gate = math.hypot(gcx - rx, gcy - ry)
                    if d_to_gate < best_dist:
                        best_dist = d_to_gate
                        # 进门方向 = 机器人当前朝向（在前方检测到才保存）
                        best_gate = (gcx, gcy, ryaw, gap)
        if gate_event != 'approaching':
            best_gate = None

        # ── 发布 ──
        obs_msg = PolygonStamped()
        obs_msg.header.stamp    = msg.header.stamp
        obs_msg.header.frame_id = 'odom'
        for cx, cy in static_centers:
            p = Point32(x=float(cx), y=float(cy), z=0.0)
            obs_msg.polygon.points.append(p)
        self.pub_obs.publish(obs_msg)

        dyn_msg = PoseArray()
        dyn_msg.header.stamp    = msg.header.stamp
        dyn_msg.header.frame_id = 'odom'
        for cx, cy, vx, vy, sp in dyn_centers:
            p = Pose()
            p.position.x = float(cx)
            p.position.y = float(cy)
            p.position.z = float(sp)    # 速度存在 z 字段
            yaw_vel = math.atan2(vy, vx)
            q = tft.quaternion_from_euler(0, 0, yaw_vel)
            p.orientation.x = q[0]; p.orientation.y = q[1]
            p.orientation.z = q[2]; p.orientation.w = q[3]
            dyn_msg.poses.append(p)
        self.pub_dyn.publish(dyn_msg)

        self.pub_gate.publish(String(data=gate_event))

        # ── 发布门中心位姿（供规划层生成穿门路径）──
        if best_gate is not None:
            gcx, gcy, g_yaw, _ = best_gate
            gp = PoseStamped()
            gp.header.stamp    = msg.header.stamp
            gp.header.frame_id = 'odom'
            gp.pose.position.x = float(gcx)
            gp.pose.position.y = float(gcy)
            q = tft.quaternion_from_euler(0, 0, g_yaw)
            gp.pose.orientation.x = q[0]; gp.pose.orientation.y = q[1]
            gp.pose.orientation.z = q[2]; gp.pose.orientation.w = q[3]
            self.pub_gate_pose.publish(gp)

    # ── 内部工具 ─────────────────────────────────────────────────────────
    def _cluster(self, pts: list) -> list:
        """按扫描顺序 1D 聚类。"""
        if not pts:
            return []
        clusters = [[pts[0]]]
        for i in range(1, len(pts)):
            prev = clusters[-1][-1]
            dist = math.hypot(pts[i][0] - prev[0], pts[i][1] - prev[1])
            if dist < self.clust_dist:
                clusters[-1].append(pts[i])
            else:
                clusters.append([pts[i]])
        return [c for c in clusters
                if self.clust_min <= len(c) <= self.clust_max]

    def _update_track(self, cx: float, cy: float, now: float) -> Track:
        """最近邻匹配 + 速度一致性检验，返回对应 Track。"""
        best_id   = None
        best_dist = self.track_match
        for tid, trk in self.tracks.items():
            d = math.hypot(cx - trk.pos[0], cy - trk.pos[1])
            if d >= best_dist:
                continue
            # 速度一致性：预测位置检验
            dt_pred = max(1e-3, now - trk.t)
            px = trk.pos[0] + trk.vel[0] * dt_pred
            py = trk.pos[1] + trk.vel[1] * dt_pred
            pred_err = math.hypot(cx - px, cy - py)
            if pred_err < self.track_match * 1.5:
                best_dist = d
                best_id   = tid

        if best_id is not None:
            self.tracks[best_id].update(cx, cy, now)
            return self.tracks[best_id]
        else:
            self._track_id += 1
            trk = Track(cx, cy, now, self._track_id)
            self.tracks[self._track_id] = trk
            return trk


def main():
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
