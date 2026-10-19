#!/usr/bin/env python3
"""
Master ROS 2 + Gazebo Launch File (uav_simulation.launch.py)
============================================================
Starts the complete 3D Quadrotor UAV Simulation in Gazebo:
  1. Dynamically updates `worlds/uav_waypoint_world.sdf` from `config/uav_params.yaml`
     so Start Position, Waypoints, and the 3D Reference Trajectory are rendered in Gazebo.
  2. Launches Gazebo (`gz sim -r` / `ign gazebo -r` / `gazebo`) with the UAV world.
  3. Starts `robot_state_publisher` with `urdf/quadrotor.urdf`.
  4. Starts `waypoint_manager` (sequential waypoint tracking & switching).
  5. Starts `trajectory_node` (quintic minimum-jerk 3D reference trajectory generator).
  6. Starts `controller_node` (CACS 2024 Quaternion Backstepping Fractional-Exponent controller).
  7. Starts `gazebo_uav_node` (6-DOF UAV dynamics + live Gazebo pose & trajectory trail bridge).
  8. Starts `info_panel_node` (Right-side live coordinate & error HUD panel alongside Gazebo).
  9. Starts `data_logger` (CSV logger & RMSE performance evaluator).
 10. Optionally starts `rviz2` (enabled with `use_rviz:=true`).
"""

import os
import shutil
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('uav_simulation')
    config_file = LaunchConfiguration('config_file').perform(context)
    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui').perform(context)
    csv_output_path = LaunchConfiguration('csv_output_path').perform(context)

    urdf_path = os.path.join(pkg_share, 'urdf', 'quadrotor.urdf')
    rviz_path = os.path.join(pkg_share, 'rviz', 'uav_visualization.rviz')
    world_sdf_path = os.path.join(pkg_share, 'worlds', 'uav_waypoint_world.sdf')
    world_gen_script = os.path.join(pkg_share, 'scripts', 'generate_gazebo_world.py')

    # Regenerate Gazebo SDF world with current waypoints & reference trajectory from YAML
    writable_world_path = os.path.join('/tmp', 'uav_waypoint_world.sdf') if os.name != 'nt' else world_sdf_path
    try:
        from uav_simulation.waypoint_manager import load_waypoints_from_yaml
        import importlib.util
        spec = importlib.util.spec_from_file_location("gen_world", world_gen_script)
        gen_mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(gen_mod)
        gen_mod.build_world_sdf(config_file, writable_world_path)
    except Exception:
        writable_world_path = world_sdf_path

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_desc = f.read()

    actions = []

    # Launch Gazebo (supports `gz sim`, `ign gazebo`, or `gazebo`)
    if use_gazebo_gui.lower() == 'true':
        if shutil.which('gz'):
            gz_cmd = ['gz', 'sim', '-r', writable_world_path]
        elif shutil.which('ign'):
            gz_cmd = ['ign', 'gazebo', '-r', writable_world_path]
        elif shutil.which('gazebo'):
            gz_cmd = ['gazebo', '--verbose', writable_world_path]
        else:
            gz_cmd = None

        if gz_cmd:
            actions.append(
                ExecuteProcess(
                    cmd=gz_cmd,
                    output='screen'
                )
            )

    # Robot State Publisher
    actions.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        )
    )

    # 1. Waypoint Manager Node
    actions.append(
        Node(
            package='uav_simulation',
            executable='waypoint_manager',
            name='waypoint_manager',
            output='screen',
            parameters=[{'config_file': config_file}]
        )
    )

    # 2. Reference Trajectory Generator Node
    actions.append(
        Node(
            package='uav_simulation',
            executable='trajectory_node',
            name='trajectory_node',
            output='screen',
            parameters=[{'config_file': config_file}]
        )
    )

    # 3. Quaternion Backstepping Fractional-Exponent Controller Node (CACS 2024 Paper)
    actions.append(
        Node(
            package='uav_simulation',
            executable='controller_node',
            name='controller_node',
            output='screen',
            parameters=[{'config_file': config_file}]
        )
    )

    # 4. Gazebo 6-DOF UAV Dynamics & Pose/Trajectory Bridge Node
    actions.append(
        Node(
            package='uav_simulation',
            executable='gazebo_uav_node',
            name='gazebo_uav_node',
            output='screen',
            parameters=[{'config_file': config_file, 'world_name': 'uav_world'}]
        )
    )

    # 5. Right-Side Real-Time Coordinate Information Panel Node
    actions.append(
        Node(
            package='uav_simulation',
            executable='info_panel_node',
            name='info_panel_node',
            output='screen',
            parameters=[{'enable_gui': True}]
        )
    )

    # 6. CSV Data Logger & Performance Evaluator Node
    actions.append(
        Node(
            package='uav_simulation',
            executable='data_logger',
            name='data_logger',
            output='screen',
            parameters=[{'csv_output_path': csv_output_path, 'logging_frequency': 20.0}]
        )
    )

    # Optional RViz2 Visualization
    actions.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
            condition=IfCondition(use_rviz),
            output='screen'
        )
    )

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('uav_simulation')
    default_config = os.path.join(pkg_share, 'config', 'uav_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to YAML configuration file with start position, waypoints, and controller gains'
        ),
        DeclareLaunchArgument(
            'use_gazebo_gui',
            default_value='true',
            description='Launch Gazebo 3D simulation GUI'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Optionally launch RViz2 alongside Gazebo'
        ),
        DeclareLaunchArgument(
            'csv_output_path',
            default_value='uav_trajectory_log.csv',
            description='Output file path for time-series CSV log'
        ),
        OpaqueFunction(function=_launch_setup)
    ])
