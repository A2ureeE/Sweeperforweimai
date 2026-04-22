"""
Main simulation launch file.

Reads map_config.yaml once and injects all map-specific parameters
(area bounds, gate positions, spawn pose, scoring goals) into every
node that needs them.  Swap to a new map by editing map_config.yaml
and re-running — no code changes required.

Launch order:
  t=0  Gazebo + robot description + moving obstacle
  t=4s Perception, behavior, coverage, planner, controller
  t=6s Score logger + mission runner
"""
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# ─────────────────────────────────────────────
# Helper: load map_config.yaml
# ─────────────────────────────────────────────
def _load_map(pkg_bringup: str) -> dict:
    path = os.path.join(pkg_bringup, 'config', 'map_config.yaml')
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return data.get('map', {})


def generate_launch_description():
    pkg_gazebo     = get_package_share_directory('sweeper_gazebo')
    pkg_perception = get_package_share_directory('sweeper_perception')
    pkg_behavior   = get_package_share_directory('sweeper_behavior')
    pkg_planning   = get_package_share_directory('sweeper_planning')
    pkg_control    = get_package_share_directory('sweeper_control')
    pkg_bringup    = get_package_share_directory('sweeper_bringup')

    # ── Package config files ──────────────────────────────────────────
    cfg_perception = os.path.join(pkg_perception, 'config', 'perception.yaml')
    cfg_behavior   = os.path.join(pkg_behavior,   'config', 'behavior.yaml')
    cfg_planning   = os.path.join(pkg_planning,   'config', 'planning.yaml')
    cfg_control    = os.path.join(pkg_control,    'config', 'control.yaml')
    cfg_map        = os.path.join(pkg_bringup,    'config', 'map_config.yaml')

    # ── Read map config ───────────────────────────────────────────────
    m      = _load_map(pkg_bringup)
    area   = m.get('area',   {})
    spawn  = m.get('spawn',  {})
    edge   = m.get('edge',   {})
    sweep  = m.get('sweep',  {})
    score  = m.get('scoring', {})
    gates  = m.get('gates',  [])

    # Gate positions for mission runner
    gate1 = gates[0] if len(gates) > 0 else {}
    gate2 = gates[1] if len(gates) > 1 else {}

    # ── Map-derived parameters injected into nodes ────────────────────
    map_params = {
        'use_sim_time':     True,
        # Area
        'area_x_min':       float(area.get('x_min', -12.0)),
        'area_x_max':       float(area.get('x_max',  14.5)),
        'area_y_min':       float(area.get('y_min',  -9.0)),
        'area_y_max':       float(area.get('y_max',   9.5)),
        # Edge
        'edge_offset':      float(edge.get('sweep_offset',   0.80)),
        'edge_follow_offset': float(edge.get('follow_offset', 0.20)),
        # Sweep
        'sweep_spacing':    float(sweep.get('row_spacing',   1.00)),
        'min_turn_radius':  float(sweep.get('min_turn_radius', 0.95)),
    }

    # Gate params for mission runner
    gate_params = {
        'use_sim_time':       True,
        'gate1_x':  float(gate1.get('center', {}).get('x', 3.0)),
        'gate1_y':  float(gate1.get('center', {}).get('y', -8.0)),
        'gate2_x':  float(gate2.get('center', {}).get('x', 3.0)),
        'gate2_y':  float(gate2.get('center', {}).get('y',  2.0)),
        'coverage_goal_pct':  float(score.get('coverage_goal_pct', 80.0)),
        'edge_follow_goal_s': float(score.get('edge_follow_goal_s', 30.0)),
    }

    # ── Declare launch arguments ──────────────────────────────────────
    gui_arg  = DeclareLaunchArgument('gui',  default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    edge_follow_arg = DeclareLaunchArgument('enable_edge_follow', default_value='false')

    # ── 1. Gazebo + robot ─────────────────────────────────────────────
    world_file = os.path.join(
        pkg_gazebo, 'worlds',
        m.get('world_file', 'sweep_course.world'))

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'gui':   LaunchConfiguration('gui'),
            'world': world_file,
            'x':     str(spawn.get('x',   -10.0)),
            'y':     str(spawn.get('y',    -8.0)),
            'yaw':   str(spawn.get('yaw',   0.0)),
        }.items(),
    )

    # ── 2. Algorithm nodes ────────────────────────────────────────────
    perception_node = Node(
        package='sweeper_perception', executable='perception_node',
        name='perception_node',
        parameters=[cfg_perception, {'use_sim_time': True}],
        output='screen')

    behavior_node = Node(
        package='sweeper_behavior', executable='behavior_node',
        name='behavior_node',
        parameters=[
            cfg_behavior,
            map_params,
            {
                'enable_edge_follow': ParameterValue(
                    LaunchConfiguration('enable_edge_follow'),
                    value_type=bool)
            }
        ],
        output='screen')

    coverage_node = Node(
        package='sweeper_planning', executable='coverage_node',
        name='coverage_node',
        parameters=[cfg_planning, map_params],   # map_params overrides yaml
        output='screen')

    planner_node = Node(
        package='sweeper_planning', executable='planner_node',
        name='planner_node',
        parameters=[cfg_planning, {'use_sim_time': True}],
        output='screen')

    controller_node = Node(
        package='sweeper_control', executable='controller_node',
        name='controller_node',
        parameters=[cfg_control, map_params],
        output='screen')

    # ── 3. Score / mission nodes ──────────────────────────────────────
    score_logger_node = Node(
        package='sweeper_planning', executable='score_logger_node',
        name='score_logger_node',
        parameters=[cfg_planning, {'use_sim_time': True}],
        output='screen')

    mission_runner_node = Node(
        package='sweeper_planning', executable='mission_runner_node',
        name='mission_runner_node',
        parameters=[cfg_planning, gate_params],   # gate_params from map
        output='screen')

    # ── 4. RViz2 (conditional) ─────────────────────────────────────────
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'sweeper.rviz')
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen')

    # ── Launch sequence ───────────────────────────────────────────────
    delayed_nodes = TimerAction(
        period=4.0,
        actions=[
            perception_node,
            behavior_node,
            coverage_node,
            planner_node,
        ])

    delayed_controller = TimerAction(
        period=10.0,
        actions=[controller_node])

    delayed_loggers = TimerAction(
        period=6.0,
        actions=[
            score_logger_node,
            mission_runner_node,
        ])

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        edge_follow_arg,
        gazebo_launch,
        rviz_node,
        delayed_nodes,
        delayed_controller,
        delayed_loggers,
    ])
