"""Launch Gazebo with a configurable world and spawn the Z200 robot.

Accepts:
  world    — path to .world file (default: sweep_course.world)
  x/y/yaw — robot spawn pose   (default: from map_config.yaml)
  gui      — show Gazebo GUI   (default: true)
"""
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_map_cfg():
    """Load map_config.yaml and return the map section (with defaults)."""
    cfg_path = os.path.join(
        get_package_share_directory('sweeper_bringup'),
        'config', 'map_config.yaml')
    defaults = {
        'world_file': 'sweep_course.world',
        'spawn': {'x': -10.0, 'y': -8.0, 'z': 0.1, 'yaw': 0.0},
    }
    if os.path.exists(cfg_path):
        with open(cfg_path) as f:
            data = yaml.safe_load(f) or {}
        return data.get('map', defaults)
    return defaults


def generate_launch_description():
    pkg_gz   = get_package_share_directory('sweeper_gazebo')
    pkg_desc = get_package_share_directory('sweeper_description')

    map_cfg = _load_map_cfg()
    spawn   = map_cfg.get('spawn', {})
    default_world = os.path.join(pkg_gz, 'worlds',
                                 map_cfg.get('world_file', 'sweep_course.world'))

    world_arg = DeclareLaunchArgument('world', default_value=default_world,
                                      description='Path to Gazebo .world file')
    x_arg   = DeclareLaunchArgument('x',   default_value=str(spawn.get('x',   -10.0)))
    y_arg   = DeclareLaunchArgument('y',   default_value=str(spawn.get('y',    -8.0)))
    yaw_arg = DeclareLaunchArgument('yaw', default_value=str(spawn.get('yaw',   0.0)))
    gui_arg = DeclareLaunchArgument('gui', default_value='true')

    set_gz_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_gz, 'models') + ':' +
              os.environ.get('GAZEBO_MODEL_PATH', ''))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('gazebo_ros'),
                                   'launch', 'gazebo.launch.py'])]),
        launch_arguments={
            'world':   LaunchConfiguration('world'),
            'gui':     LaunchConfiguration('gui'),
            'verbose': 'true',
        }.items(),
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_desc, 'launch', 'description.launch.py')))

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'z200',
            '-x',   LaunchConfiguration('x'),
            '-y',   LaunchConfiguration('y'),
            '-z',   str(spawn.get('z', 0.1)) if isinstance(spawn, dict) else '0.1',
            '-Y',   LaunchConfiguration('yaw'),
        ],
        output='screen')

    mover = Node(
        package='sweeper_gazebo', executable='moving_obstacle_node',
        name='moving_obstacle_node', output='screen',
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        world_arg, x_arg, y_arg, yaw_arg, gui_arg,
        set_gz_model_path,
        gazebo,
        description,
        spawn,
        mover,
    ])
