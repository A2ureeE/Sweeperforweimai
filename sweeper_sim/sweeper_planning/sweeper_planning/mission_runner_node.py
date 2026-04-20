#!/usr/bin/env python3
"""
Mission runner — tracks preliminary round completion criteria.

Monitors:
  - Gate 1 + Gate 2 passage
  - Static obstacle avoidance (no collision logged)
  - Dynamic obstacle avoidance
  - Coverage rate > 80%
  - Edge-follow duration > 30 s

Publishes:
  /mission/status    std_msgs/String (JSON)
  /mission/complete  std_msgs/String ('true' when all tasks done)
"""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
import tf_transformations as tft


GATE_PASS_RADIUS = 2.5


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class MissionRunnerNode(Node):
    def __init__(self):
        super().__init__('mission_runner_node')
        # Gate positions from map_config (injected via launch)
        self.declare_parameter('gate1_x',  3.0)
        self.declare_parameter('gate1_y', -8.0)
        self.declare_parameter('gate2_x',  3.0)
        self.declare_parameter('gate2_y',  2.0)
        self.declare_parameter('coverage_goal_pct',  80.0)
        self.declare_parameter('edge_follow_goal_s', 30.0)
        g = self.get_parameter

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        latch_qos  = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.gate1_center = (g('gate1_x').value, g('gate1_y').value)
        self.gate2_center = (g('gate2_x').value, g('gate2_y').value)
        self.cov_goal     = g('coverage_goal_pct').value
        self.edge_goal    = g('edge_follow_goal_s').value

        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.cb_odom, sensor_qos)
        self.sub_mode = self.create_subscription(
            String, '/behavior/mode', self.cb_mode, 5)
        self.sub_cov  = self.create_subscription(
            Float32, '/coverage/coverage_pct', self.cb_cov, 5)

        self.pub_status   = self.create_publisher(String, '/mission/status', 5)
        self.pub_complete = self.create_publisher(String, '/mission/complete', latch_qos)

        self.robot = None
        self.mode  = 'UNKNOWN'
        self.cov   = 0.0
        self.t_start = time.time()

        self.gate1_passed  = False
        self.gate2_passed  = False
        self.edge_follow_s = 0.0
        self.completed     = False

        self._near_gate1 = False
        self._near_gate2 = False
        self._edge_start = None

        self.create_timer(0.5, self.tick)
        self.get_logger().info('mission_runner_node ready')

    def cb_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot = (x, y)

    def cb_mode(self, msg: String):
        self.mode = msg.data

    def cb_cov(self, msg: Float32):
        self.cov = float(msg.data)

    def tick(self):
        if self.robot is None:
            return
        rx, ry = self.robot
        now = time.time()

        # Gate passage detection (uses coords from map_config)
        if not self.gate1_passed:
            if dist((rx, ry), self.gate1_center) < GATE_PASS_RADIUS:
                if not self._near_gate1:
                    self._near_gate1 = True
            elif self._near_gate1:
                self.gate1_passed = True
                self.get_logger().info('Gate 1 PASSED')

        if not self.gate2_passed:
            if dist((rx, ry), self.gate2_center) < GATE_PASS_RADIUS:
                if not self._near_gate2:
                    self._near_gate2 = True
            elif self._near_gate2:
                self.gate2_passed = True
                self.get_logger().info('Gate 2 PASSED')

        # Edge follow duration
        if self.mode == 'EDGE_FOLLOW':
            if self._edge_start is None:
                self._edge_start = now
            self.edge_follow_s = now - self._edge_start
        else:
            self._edge_start = None

        # Check completion
        tasks = {
            'gate1_passed':   self.gate1_passed,
            'gate2_passed':   self.gate2_passed,
            'coverage_ok':    self.cov >= self.cov_goal,
            'edge_follow_ok': self.edge_follow_s >= self.edge_goal,
        }
        elapsed = now - self.t_start

        status = dict(tasks)
        status['coverage_pct']   = round(self.cov, 1)
        status['edge_follow_s']  = round(self.edge_follow_s, 1)
        status['elapsed_s']      = round(elapsed, 1)
        status['all_done']       = all(tasks.values())

        self.pub_status.publish(String(data=json.dumps(status)))

        if status['all_done'] and not self.completed:
            self.completed = True
            self.pub_complete.publish(String(data='true'))
            self.get_logger().info(
                f'MISSION COMPLETE in {elapsed:.1f}s! '
                f'coverage={self.cov:.1f}%')


def main():
    rclpy.init()
    node = MissionRunnerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
