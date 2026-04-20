#!/usr/bin/env python3
"""
Moving pedestrian obstacle controller.

Drives the 'pedestrian' model on a looping path through the sweep area.
"""
import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist


WAYPOINTS = [
    (0.0,  0.0),
    (2.0,  3.0),
    (2.0,  6.0),
    (0.0,  6.0),
    (-2.0, 3.0),
    (-2.0, 0.0),
]
SPEED = 0.5  # m/s


class MovingObstacleNode(Node):
    def __init__(self):
        super().__init__('moving_obstacle_node')
        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state...')

        self.wp_idx = 0
        self.x = float(WAYPOINTS[0][0])
        self.y = float(WAYPOINTS[0][1])
        self.create_timer(0.1, self.tick)
        self.get_logger().info('moving_obstacle_node ready')

    def tick(self):
        tx, ty = WAYPOINTS[self.wp_idx]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)
        if dist < 0.1:
            self.wp_idx = (self.wp_idx + 1) % len(WAYPOINTS)
            tx, ty = WAYPOINTS[self.wp_idx]
            dx = tx - self.x
            dy = ty - self.y
            dist = math.hypot(dx, dy)

        if dist > 1e-4:
            step = SPEED * 0.1
            self.x += (dx / dist) * step
            self.y += (dy / dist) * step

        yaw = math.atan2(dy, dx)
        state = EntityState()
        state.name = 'pedestrian'
        state.pose.position.x = float(self.x)
        state.pose.position.y = float(self.y)
        state.pose.position.z = 0.0
        state.pose.orientation.z = math.sin(yaw / 2)
        state.pose.orientation.w = math.cos(yaw / 2)
        state.twist.linear.x = float(SPEED * dx / max(dist, 1e-4))
        state.twist.linear.y = float(SPEED * dy / max(dist, 1e-4))

        req = SetEntityState.Request()
        req.state = state
        self.cli.call_async(req)


def main():
    rclpy.init()
    node = MovingObstacleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
