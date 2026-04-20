from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取配置文件路径
    config_dir = os.path.join(
        get_package_share_directory("rtk"), "config", "rtk_params.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="rtk",
                executable="rtk_node",
                name="rtk_node",
                parameters=[config_dir],  # 从YAML文件加载参数
                output="screen",
            ),
        ]
    )
