import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('sweeper_behavior'),
                       'config', 'behavior.yaml')
    return LaunchDescription([
        Node(package='sweeper_behavior', executable='behavior_node',
             name='behavior_node',
             parameters=[cfg, {'use_sim_time': True}], output='screen'),
    ])
