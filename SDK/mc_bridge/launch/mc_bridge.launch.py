import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('mc_bridge'),
                       'config', 'mc_bridge.yaml')
    return LaunchDescription([
        Node(package='mc_bridge', executable='mc_bridge_node',
             name='mc_bridge_node', parameters=[cfg],
             output='screen'),
    ])
