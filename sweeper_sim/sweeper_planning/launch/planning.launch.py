import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('sweeper_planning'),
                       'config', 'planning.yaml')
    return LaunchDescription([
        Node(package='sweeper_planning', executable='coverage_node',
             name='coverage_node',
             parameters=[cfg, {'use_sim_time': True}], output='screen'),
        Node(package='sweeper_planning', executable='planner_node',
             name='planner_node',
             parameters=[cfg, {'use_sim_time': True}], output='screen'),
    ])
