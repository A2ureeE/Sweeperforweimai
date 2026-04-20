from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("rslidar_sdk")
    # RViz 配置文件路径
    rviz_config = os.path.join(pkg_share, "rviz", "rviz2.rviz")
    # 雷达配置文件路径
    config_front = os.path.join(pkg_share, "config", "config_front.yaml")  # 前雷达
    config_rear = os.path.join(pkg_share, "config", "config_rear.yaml")  # 后雷达


    

    return LaunchDescription(
        [
            Node(
                namespace="rslidar_sdk",
                package="rslidar_sdk",
                executable="rslidar_sdk_node",
                output="screen",
                parameters=[{"config_path": config_front}],
            ),
            Node(
                namespace="rviz2",
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
            ),
        ]
    )
