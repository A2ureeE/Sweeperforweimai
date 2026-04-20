import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('rtk_to_odom'),
                       'config', 'rtk_to_odom.yaml')
    return LaunchDescription([
        Node(package='rtk_to_odom', executable='rtk_to_odom_node',
             name='rtk_to_odom_node', parameters=[cfg],
             output='screen'),
    ])
