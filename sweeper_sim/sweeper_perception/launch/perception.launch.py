import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('sweeper_perception'),
                       'config', 'perception.yaml')
    return LaunchDescription([
        Node(package='sweeper_perception', executable='perception_node',
             name='perception_node', parameters=[cfg, {'use_sim_time': True}],
             output='screen'),
    ])
