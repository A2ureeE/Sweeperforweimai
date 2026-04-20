import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('sweeper_control'),
                       'config', 'control.yaml')
    return LaunchDescription([
        Node(package='sweeper_control', executable='controller_node',
             name='controller_node',
             parameters=[cfg, {'use_sim_time': True}], output='screen'),
    ])
