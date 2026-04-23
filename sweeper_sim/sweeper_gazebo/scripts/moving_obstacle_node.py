#!/usr/bin/env python3
"""
Moving pedestrian obstacle controller.

Drives the 'pedestrian' model on a looping path through the sweep area.
Waypoints are auto-scaled to the map area so the path stays within
any reasonably-sized arena.
"""
import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

SPEED = 0.5  # m/s


class MovingObstacleNode(Node):
    def __init__(self):
        super().__init__('moving_obstacle_node')
        self.declare_parameters('', [
            ('area_x_min', -12.0),
            ('area_x_max',  14.5),
            ('area_y_min',  -9.0),
            ('area_y_max',   9.5),
        ])
        g = self.get_parameter
        xmin = float(g('area_x_min').value)
        xmax = float(g('area_x_max').value)
        ymin = float(g('area_y_min').value)
        ymax = float(g('area_y_max').value)

        cx = (xmin + xmax) / 2.0
        cy = (ymin + ymax) / 2.0
        scale = min(xmax - xmin, ymax - ymin) * 0.15

        self.waypoints = [
            (cx,             cy),
            (cx + scale,     cy + 1.5 * scale),
            (cx + scale,     cy + 3.0 * scale),
            (cx,             cy + 3.0 * scale),
            (cx - scale,     cy + 1.5 * scale),
            (cx - scale,     cy),
        ]

        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self._service_ready = False

        self.wp_idx = 0
        self.x = float(self.waypoints[0][0])
        self.y = float(self.waypoints[0][1])

        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f'moving_obstacle_node ready | area=[{xmin:.0f},{xmax:.0f}]x'
            f'[{ymin:.0f},{ymax:.0f}] center=({cx:.1f},{cy:.1f}) '
            f'scale={scale:.2f}m {len(self.waypoints)} waypoints')

    def tick(self):
        if not self._service_ready:
            if self.cli.service_is_ready():
                self._service_ready = True
                self.get_logger().info(
                    '/gazebo/set_entity_state available, starting movement')
            else:
                return

        tx, ty = self.waypoints[self.wp_idx]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)
        if dist < 0.1:
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            tx, ty = self.waypoints[self.wp_idx]
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
