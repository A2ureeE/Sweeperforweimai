import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('lidar_adapter'),
                       'config', 'lidar_adapter.yaml')
    return LaunchDescription([
        Node(package='lidar_adapter', executable='pc_to_scan_node',
             name='pc_to_scan_node', parameters=[cfg],
             output='screen'),
    ])
