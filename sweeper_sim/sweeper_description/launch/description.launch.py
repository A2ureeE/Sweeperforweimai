"""
Robot description launch.

Publishes robot_description via robot_state_publisher.
Uses z200.urdf.xacro (derived from the official z200.urdf with the
unavailable libgazebo_ros_tricycle_drive_fixed.so swapped for the
standard libgazebo_ros_diff_drive.so — the original z200.urdf is
never modified).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    pkg = get_package_share_directory('sweeper_description')
    xacro_file = os.path.join(pkg, 'urdf', 'z200.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }]),
    ])
