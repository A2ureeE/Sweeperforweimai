#!/usr/bin/env python3
"""
CILQR-based MPC controller node.

Subscribes:
  /reference_path              nav_msgs/Path          coverage/waypoint path
  /odom                       nav_msgs/Odometry      robot state
  /perception/obstacle_points geometry_msgs/PolygonStamped  static obstacles
  /perception/dynamic_obstacles geometry_msgs/PoseArray     dynamic obstacles

Publishes:
  /cmd_vel                    geometry_msgs/Twist    (v, omega)

Control flow per tick:
  1. Build / update Spline2D from /reference_path.
  2. Convert 5D state [x, y, v, yaw, steer] from /odom.
  3. Predict obstacle positions over MPC horizon.
  4. Run CILQR.solve() -> exec_u = [acc, steer_rate].
  5. Convert to diff-drive (v, omega) and publish.

Fallback: if solver fails or path not ready, publish zero velocity.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PolygonStamped, PoseArray, Point32
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import tf_transformations as tft

from .bicycle_dynamics import rk4, angdiff
from .spline import Spline2D, SplineProps
from .cilqr_solver import (
    CILQRSolver, MPCConfig, IterationConfig,
    VehicleConfig, RoadConfig, Obstacle, TrajPoint,
)


def yaw_from_quat(q):
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# -----------------------------------------------------------------------
# Config singleton (read from ROS params)
# -----------------------------------------------------------------------
def build_configs(node: Node):
    """Read all MPC parameters and return config objects."""
    g = node.get_parameter
    L = g('wheel_base').value

    mpc = MPCConfig(
        N=int(g('mpc_horizon').value),
        dt=g('mpc_dt').value,
        w_pos=g('w_pos').value,
        w_vel=g('w_vel').value,
        w_yaw=g('w_yaw').value,
        w_steer=g('w_steer').value,
        w_acc=g('w_acc').value,
        w_steer_rate=g('w_steer_rate').value,
        w_pos_term=g('w_pos_term').value,
        w_vel_term=g('w_vel_term').value,
        w_yaw_term=g('w_yaw_term').value,
        w_steer_term=g('w_steer_term').value,
        w_consistency=g('w_consistency').value,
        exp_q1=g('exp_q1').value,
        exp_q2=g('exp_q2').value,
        road_exp_q1=g('road_exp_q1').value,
        road_exp_q2=g('road_exp_q2').value,
        road_safe_margin=g('road_safe_margin').value,
    )

    iter_cfg = IterationConfig(
        max_iter=int(g('mpc_max_iter').value),
        init_lamb=g('init_lamb').value,
        lamb_decay=g('lamb_decay').value,
        lamb_amplify=g('lamb_amplify').value,
        max_lamb=g('max_lamb').value,
        tol=g('mpc_tol').value,
    )

    veh = VehicleConfig(
        wheelbase=L,
        width=g('vehicle_width').value,
        length=g('vehicle_length').value,
        velo_max=g('max_speed').value,
        velo_min=0.0,
        a_max=g('max_accel').value,
        a_min=-g('max_decel').value,
        stl_lim=g('max_steer_angle').value,
        max_steer_rate=g('max_steer_rate').value,
    )

    road = RoadConfig(
        width=g('road_width').value,
        velo_ref=g('ref_speed').value,
    )

    return mpc, iter_cfg, veh, road


# -----------------------------------------------------------------------
# Obstacle converter
# -----------------------------------------------------------------------
def polygons_to_obstacles(poly_msg: PolygonStamped, dyn_msg: PoseArray,
                          veh_cfg: VehicleConfig) -> list:
    """Convert perception msgs to Obstacle list."""
    obs_list = []
    for p in poly_msg.polygon.points:
        obs_list.append(Obstacle(
            x=float(p.x), y=float(p.y),
            vx=0.0, vy=0.0,
            width=0.6, length=0.6, d_safe=0.5,
        ))
    for p in dyn_msg.poses:
        yaw = yaw_from_quat(p.orientation)
        sp = float(p.position.z)
        obs_list.append(Obstacle(
            x=float(p.position.x), y=float(p.position.y),
            vx=sp * math.cos(yaw), vy=sp * math.sin(yaw),
            width=0.6, length=0.6, d_safe=0.5,
        ))
    return obs_list


# -----------------------------------------------------------------------
# Main node
# -----------------------------------------------------------------------
class MPCControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_controller_node')

        # ---- Parameters ----
        self.declare_parameters('', [
            ('control_rate_hz', 10.0),
            ('wheel_base', 1.05),
            ('max_speed', 1.3),
            ('max_accel', 0.8),
            ('max_decel', 1.0),
            ('max_steer_angle', 0.8727),
            ('max_steer_rate', 0.5),
            ('vehicle_width', 1.05),
            ('vehicle_length', 1.52),
            ('road_width', 4.0),
            ('ref_speed', 1.0),
            # MPC horizon & solver
            ('mpc_horizon', 20),
            ('mpc_dt', 0.1),
            ('mpc_max_iter', 15),
            ('mpc_tol', 0.01),
            ('init_lamb', 5.0),
            ('lamb_decay', 0.7),
            ('lamb_amplify', 2.0),
            ('max_lamb', 1e4),
            # State weights
            ('w_pos', 8.0),
            ('w_vel', 2.0),
            ('w_yaw', 1.0),
            ('w_steer', 5.0),
            ('w_acc', 1.0),
            ('w_steer_rate', 80.0),
            ('w_pos_term', 20.0),
            ('w_vel_term', 5.0),
            ('w_yaw_term', 5.0),
            ('w_steer_term', 5.0),
            ('w_consistency', 3.0),
            # Barrier params
            ('exp_q1', 1.0),
            ('exp_q2', 10.0),
            ('road_exp_q1', 5.0),
            ('road_exp_q2', 5.0),
            ('road_safe_margin', 0.5),
        ])

        # ---- Build configs ----
        self.mpc_cfg, self.iter_cfg, self.veh_cfg, self.road_cfg = \
            build_configs(self)

        # ---- Solver ----
        self.solver = CILQRSolver(
            self.mpc_cfg, self.iter_cfg, self.veh_cfg, self.road_cfg)

        # ---- Subscriptions ----
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_ref = self.create_subscription(
            Path, '/reference_path', self._cb_ref, 5)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self._cb_odom, sensor_qos)
        self.sub_obs = self.create_subscription(
            PolygonStamped, '/perception/obstacle_points',
            self._cb_obs, sensor_qos)
        self.sub_dyn = self.create_subscription(
            PoseArray, '/perception/dynamic_obstacles',
            self._cb_dyn, sensor_qos)

        # ---- Publishers ----
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 5)
        self.pub_vis = self.create_publisher(
            MarkerArray, '/mpc/trajectory_vis', 5)

        # ---- State ----
        self.robot: np.ndarray | None = None   # (5,) = [x, y, v, yaw, steer]
        self.steer: float = 0.0               # current steering angle
        self.ref_path: Path | None = None
        self.spline: Spline2D | None = None
        self.obstacles: list = []
        self.dyn_obstacles: list = []

        # ---- Warm-start state ----
        self.prev_u: list = []
        self.prev_x: list = []

        # ---- Timer ----
        rate = float(self.get_parameter('control_rate_hz').value)
        self.dt_ctrl = 1.0 / rate
        self.timer = self.create_timer(self.dt_ctrl, self._tick)
        self.get_logger().info(
            f'MPC controller ready (N={self.mpc_cfg.N}, dt={self.mpc_cfg.dt}s, '
            f'L={self.veh_cfg.wheelbase}m)')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _cb_ref(self, msg: Path):
        self.ref_path = msg
        if len(msg.poses) < 2:
            self.spline = None
            return
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        # Build non-periodic spline (open path, not closed loop)
        self.spline = Spline2D(
            np.array(xs), np.array(ys), periodic=False)

    def _cb_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.robot = np.array([x, y, v, yaw, self.steer])

    def _cb_obs(self, msg: PolygonStamped):
        self.obstacles = [(float(p.x), float(p.y)) for p in msg.polygon.points]

    def _cb_dyn(self, msg: PoseArray):
        self.dyn_obstacles = []
        for p in msg.poses:
            yaw = yaw_from_quat(p.orientation)
            sp = float(p.position.z)
            self.dyn_obstacles.append(
                (float(p.position.x), float(p.position.y),
                 sp * math.cos(yaw), sp * math.sin(yaw), sp))

    # ------------------------------------------------------------------
    # Tick
    # ------------------------------------------------------------------
    def _tick(self):
        cmd = Twist()
        if self.robot is None:
            self.pub_cmd.publish(cmd)
            return
        if self.spline is None:
            self._emergency_stop()
            self.pub_cmd.publish(cmd)
            return

        x0 = self.robot.copy()

        # ---- Build obstacle list ----
        obs_list = []
        for ox, oy in self.obstacles:
            obs_list.append(Obstacle(x=ox, y=oy, width=0.6, length=0.6, d_safe=0.5))
        for ox, oy, ovx, ovy, _ in self.dyn_obstacles:
            obs_list.append(Obstacle(
                x=ox, y=oy, vx=ovx, vy=ovy,
                width=0.6, length=0.6, d_safe=0.5))

        # ---- Run CILQR ----
        try:
            exec_u = self.solver.solve(
                x0,
                self.spline,
                [],          # path_pts (used for spline, already built)
                [],          # ref_traj (empty -> use road defaults)
                self.prev_u,
                self.prev_x,
                obs_list,
            )
        except Exception as e:
            self.get_logger().warn(f'CILQR solve failed: {e}', throttle_duration_sec=2.0)
            self._emergency_stop()
            self.pub_cmd.publish(cmd)
            return

        acc, steer_rate = float(exec_u[0]), float(exec_u[1])

        # ---- Update steer estimate ----
        self.steer = float(wrap_angle(self.steer + steer_rate * self.dt_ctrl))
        self.steer = max(-self.veh_cfg.stl_lim,
                         min(self.veh_cfg.stl_lim, self.steer))

        # ---- Convert [acc, steer_rate] -> [v, omega] ----
        # v += acc * dt
        v_new = max(0.0, min(self.robot[2] + acc * self.dt_ctrl,
                             self.veh_cfg.velo_max))
        # Ackermann: omega = v * tan(steer) / L
        omega = v_new * math.tan(self.steer) / self.veh_cfg.wheelbase

        cmd.linear.x = float(v_new)
        cmd.angular.z = float(omega)
        self.pub_cmd.publish(cmd)

        # ---- Update warm-start ----
        self._update_warm_start(x0, exec_u)

    def _emergency_stop(self):
        if self.robot is not None:
            v_new = max(0.0, self.robot[2] - self.veh_cfg.a_max * self.dt_ctrl)
            self.robot[2] = v_new

    def _update_warm_start(self, x0, exec_u):
        """Shift warm-start buffers (simplified, full version uses full trajectory)."""
        N = self.mpc_cfg.N
        if len(self.prev_u) == N:
            shifted = list(self.prev_u[1:])
            shifted.append(exec_u.copy())
            self.prev_u = shifted
        else:
            self.prev_u = [exec_u.copy() for _ in range(N)]

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------
    def _publish_vis(self, traj_x, ref_x):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        # Predicted trajectory
        m = Marker()
        m.header.stamp = stamp; m.header.frame_id = 'odom'
        m.ns = 'mpc_traj'; m.id = 0
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.05; m.color = ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.9)
        from geometry_msgs.msg import Point
        for xi in traj_x:
            m.points.append(Point(x=float(xi[0]), y=float(xi[1]), z=0.05))
        ma.markers.append(m)
        # Obstacles
        for i, obs in enumerate(self.obstacles):
            m2 = Marker()
            m2.header.stamp = stamp; m2.header.frame_id = 'odom'
            m2.ns = 'obs'; m2.id = i + 1
            m2.type = Marker.CYLINDER; m2.action = Marker.ADD
            m2.pose.position.x = float(obs[0])
            m2.pose.position.y = float(obs[1])
            m2.pose.position.z = 0.3; m2.pose.orientation.w = 1.0
            m2.scale.x = m2.scale.y = 0.6; m2.scale.z = 0.6
            m2.color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.8)
            ma.markers.append(m2)
        self.pub_vis.publish(ma)


def main():
    rclpy.init()
    node = MPCControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
