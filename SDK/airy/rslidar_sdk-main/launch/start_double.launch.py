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

    # 点云合并 launch 文件路径
    pkg_share_merge = get_package_share_directory("rslidar_pointcloud_merger")
    merge_launch = os.path.join(pkg_share_merge, "launch", "merge_two_lidars.launch.py")

    # 前雷达节点
    front_lidar_node = Node(
        namespace="front_lidar",
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        name="rslidar_front",
        output="screen",
        parameters=[{"config_path": config_front}],
    )

    # 后雷达节点
    rear_lidar_node = Node(
        namespace="rear_lidar",
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        name="rslidar_rear",
        output="screen",
        parameters=[{"config_path": config_rear}],
    )

    # RViz 可视化节点
    rviz_node = Node(
        namespace="",  # RViz 一般不加命名空间
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
    )

    # 点云合并 launch 文件
    merge_launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(merge_launch)
    )

    # 返回所有节点和 launch 文件
    return LaunchDescription(
        [front_lidar_node, rear_lidar_node, merge_launch_action, rviz_node]
        # [front_lidar_node, rear_lidar_node, merge_launch_action]
    )
