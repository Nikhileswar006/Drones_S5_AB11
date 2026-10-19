#!/usr/bin/env python3
"""
Gazebo World SDF Generator (generate_gazebo_world.py)
=====================================================
Reads `config/uav_params.yaml` and generates `worlds/uav_waypoint_world.sdf` containing:
  1. 3D Inertial Coordinate Axes (e_x Red, e_y Green, e_z Blue)
  2. Start Position Marker (Blue Sphere at Start)
  3. All Target Waypoints (WP1, WP2, WP3, ...) + Waypoint Tolerance Spheres
  4. Persistent 3D Reference Trajectory Curve (Cyan glowing path along the quintic trajectory)
  5. Dynamic Active Waypoint Halo (`active_waypoint_halo`)
  6. Dynamic Reference Position Target Sphere (`reference_target_sphere`)
  7. 3D Quadrotor UAV (`quadrotor`) spawned at the user-defined Start position
"""

import os
import sys
import numpy as np

# Add package root to sys.path
PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from uav_simulation.waypoint_manager import load_waypoints_from_yaml
from uav_simulation.trajectory_node import QuinticSegmentTrajectoryGenerator


def build_world_sdf(yaml_path: str, output_sdf_path: str):
    start_pos, waypoints, tol = load_waypoints_from_yaml(yaml_path)
    traj = QuinticSegmentTrajectoryGenerator(start_pos, waypoints, max_velocity=2.5, max_acceleration=5.0)
    ref_pts = traj.sample_full_reference_path(num_points_per_segment=25)

    # Create visual spheres along the 3D reference trajectory inside Gazebo
    ref_visuals = []
    for idx, pt in enumerate(ref_pts):
        ref_visuals.append(f"""
      <visual name="ref_pt_{idx}">
        <pose>{pt[0]:.3f} {pt[1]:.3f} {pt[2]:.3f} 0 0 0</pose>
        <geometry><sphere><radius>0.055</radius></sphere></geometry>
        <material>
          <ambient>0.0 0.95 0.85 0.95</ambient>
          <diffuse>0.0 0.95 0.85 0.95</diffuse>
          <emissive>0.0 0.45 0.40 1.0</emissive>
        </material>
      </visual>""")

    # Create waypoint models inside Gazebo
    wp_models = []
    for i, wp in enumerate(waypoints):
        wp_models.append(f"""
    <!-- Waypoint {i+1}: ({wp[0]}, {wp[1]}, {wp[2]}) -->
    <model name="waypoint_{i+1}">
      <static>true</static>
      <pose>{wp[0]:.3f} {wp[1]:.3f} {wp[2]:.3f} 0 0 0</pose>
      <link name="link">
        <visual name="core_sphere">
          <geometry><sphere><radius>0.16</radius></sphere></geometry>
          <material>
            <ambient>0.9 0.2 0.95 1.0</ambient>
            <diffuse>0.9 0.2 0.95 1.0</diffuse>
            <emissive>0.35 0.05 0.4 1.0</emissive>
          </material>
        </visual>
        <visual name="tolerance_sphere">
          <geometry><sphere><radius>{max(tol, 0.20):.3f}</radius></sphere></geometry>
          <material>
            <ambient>0.2 0.95 0.4 0.30</ambient>
            <diffuse>0.2 0.95 0.4 0.30</diffuse>
          </material>
        </visual>
      </link>
    </model>""")

    # Create 3D Obstacle Tower models inside Gazebo (between waypoints)
    obs_models = []
    for obs in traj.obstacles:
        obs_id = obs['id']
        pos = obs['position']
        radius = obs['radius']
        height = obs['height']
        obs_models.append(f"""
    <!-- 3D Obstacle Tower {obs_id}: Position ({pos[0]:.2f}, {pos[1]:.2f}) Height {height:.2f}m -->
    <model name="obstacle_tower_{obs_id}">
      <static>true</static>
      <pose>{pos[0]:.3f} {pos[1]:.3f} {height / 2.0:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><cylinder><radius>{radius:.3f}</radius><length>{height:.3f}</length></cylinder></geometry>
        </collision>
        <visual name="body">
          <geometry><cylinder><radius>{radius:.3f}</radius><length>{height:.3f}</length></cylinder></geometry>
          <material>
            <ambient>0.25 0.28 0.35 1.0</ambient>
            <diffuse>0.95 0.50 0.10 1.0</diffuse>
            <emissive>0.15 0.05 0.0 1.0</emissive>
          </material>
        </visual>
        <visual name="warning_beacon">
          <pose>0 0 {height / 2.0 + 0.15:.3f} 0 0 0</pose>
          <geometry><sphere><radius>0.22</radius></sphere></geometry>
          <material>
            <ambient>1.0 0.1 0.1 1.0</ambient>
            <diffuse>1.0 0.1 0.1 1.0</diffuse>
            <emissive>1.0 0.2 0.2 1.0</emissive>
          </material>
        </visual>
      </link>
    </model>""")

    wp1 = waypoints[0] if waypoints else start_pos

    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="uav_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.005</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.18 0.20 0.24 1</ambient>
            <diffuse>0.18 0.20 0.24 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Inertial Coordinate Frame Axes (e_x Red, e_y Green, e_z Blue - Fig. 1 of Paper) -->
    <model name="world_coordinate_axes">
      <static>true</static>
      <pose>0 0 0.02 0 0 0</pose>
      <link name="axes_link">
        <visual name="ex_axis">
          <pose>1.0 0 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.025</radius><length>2.0</length></cylinder></geometry>
          <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
        </visual>
        <visual name="ey_axis">
          <pose>0 1.0 0 1.5708 0 0</pose>
          <geometry><cylinder><radius>0.025</radius><length>2.0</length></cylinder></geometry>
          <material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material>
        </visual>
        <visual name="ez_axis">
          <pose>0 0 1.0 0 0 0</pose>
          <geometry><cylinder><radius>0.025</radius><length>2.0</length></cylinder></geometry>
          <material><ambient>0.1 0.4 1 1</ambient><diffuse>0.1 0.4 1 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Start Position Marker: ({start_pos[0]}, {start_pos[1]}, {start_pos[2]}) -->
    <model name="start_marker">
      <static>true</static>
      <pose>{start_pos[0]:.3f} {start_pos[1]:.3f} {start_pos[2]:.3f} 0 0 0</pose>
      <link name="link">
        <visual name="start_sphere">
          <geometry><sphere><radius>0.18</radius></sphere></geometry>
          <material>
            <ambient>0.1 0.5 1.0 1.0</ambient>
            <diffuse>0.1 0.5 1.0 1.0</diffuse>
            <emissive>0.05 0.25 0.6 1.0</emissive>
          </material>
        </visual>
      </link>
    </model>

    <!-- Target Waypoints -->
    {"".join(wp_models)}

    <!-- 3D Obstacle Towers (Obstacles between waypoints to navigate around) -->
    {"".join(obs_models)}

    <!-- Dynamic Active Waypoint Highlight Halo -->
    <model name="active_waypoint_halo">
      <static>false</static>
      <pose>{wp1[0]:.3f} {wp1[1]:.3f} {wp1[2]:.3f} 0 0 0</pose>
      <link name="link">
        <gravity>false</gravity>
        <visual name="halo_visual">
          <geometry><sphere><radius>0.28</radius></sphere></geometry>
          <material>
            <ambient>1.0 0.90 0.0 0.55</ambient>
            <diffuse>1.0 0.90 0.0 0.55</diffuse>
            <emissive>0.7 0.6 0.0 0.55</emissive>
          </material>
        </visual>
      </link>
    </model>

    <!-- Instantaneous Reference Trajectory Position Sphere (X_ref, Y_ref, Z_ref) -->
    <model name="reference_target_sphere">
      <static>false</static>
      <pose>{start_pos[0]:.3f} {start_pos[1]:.3f} {start_pos[2]:.3f} 0 0 0</pose>
      <link name="link">
        <gravity>false</gravity>
        <visual name="ref_target_visual">
          <geometry><sphere><radius>0.11</radius></sphere></geometry>
          <material>
            <ambient>0.0 1.0 0.4 0.9</ambient>
            <diffuse>0.0 1.0 0.4 0.9</diffuse>
            <emissive>0.0 0.7 0.3 0.9</emissive>
          </material>
        </visual>
      </link>
    </model>

    <!-- Persistent Complete 3D Reference Trajectory (Start -> WP1 -> WP2 -> WP3) -->
    <model name="reference_trajectory_path">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="path_link">{"".join(ref_visuals)}
      </link>
    </model>

    <!-- 3D Quadrotor UAV Model (J = diag[0.1, 0.1, 0.12] from CACS 2024 Paper) -->
    <model name="quadrotor">
      <static>false</static>
      <pose>{start_pos[0]:.3f} {start_pos[1]:.3f} {start_pos[2]:.3f} 0 0 0</pose>
      <link name="base_link">
        <gravity>false</gravity>
        <inertial>
          <mass>1.20</mass>
          <inertia>
            <ixx>0.10</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
            <iyy>0.10</iyy><iyz>0.0</iyz><izz>0.12</izz>
          </inertia>
        </inertial>
        <visual name="fuselage_visual">
          <geometry><box><size>0.20 0.20 0.07</size></box></geometry>
          <material><ambient>0.15 0.2 0.3 1</ambient><diffuse>0.2 0.25 0.38 1</diffuse></material>
        </visual>
        <visual name="arm1_front">
          <pose>0.16 0 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.016</radius><length>0.32</length></cylinder></geometry>
          <material><ambient>0.95 0.15 0.15 1</ambient><diffuse>0.95 0.2 0.2 1</diffuse></material>
        </visual>
        <visual name="arm3_rear">
          <pose>-0.16 0 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.016</radius><length>0.32</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material>
        </visual>
        <visual name="arm24_cross">
          <pose>0 0 0 1.5708 0 0</pose>
          <geometry><cylinder><radius>0.016</radius><length>0.64</length></cylinder></geometry>
          <material><ambient>0.15 0.65 0.95 1</ambient><diffuse>0.15 0.65 0.95 1</diffuse></material>
        </visual>
        <visual name="rotor1">
          <pose>0.30 0 0.03 0 0 0</pose>
          <geometry><cylinder><radius>0.12</radius><length>0.014</length></cylinder></geometry>
          <material><ambient>1.0 0.25 0.1 0.9</ambient><diffuse>1.0 0.25 0.1 0.9</diffuse></material>
        </visual>
        <visual name="rotor2">
          <pose>0 -0.30 0.03 0 0 0</pose>
          <geometry><cylinder><radius>0.12</radius><length>0.014</length></cylinder></geometry>
          <material><ambient>0.0 0.9 0.95 0.9</ambient><diffuse>0.0 0.9 0.95 0.9</diffuse></material>
        </visual>
        <visual name="rotor3">
          <pose>-0.30 0 0.03 0 0 0</pose>
          <geometry><cylinder><radius>0.12</radius><length>0.014</length></cylinder></geometry>
          <material><ambient>0.0 0.9 0.95 0.9</ambient><diffuse>0.0 0.9 0.95 0.9</diffuse></material>
        </visual>
        <visual name="rotor4">
          <pose>0 0.30 0.03 0 0 0</pose>
          <geometry><cylinder><radius>0.12</radius><length>0.014</length></cylinder></geometry>
          <material><ambient>0.0 0.9 0.95 0.9</ambient><diffuse>0.0 0.9 0.95 0.9</diffuse></material>
        </visual>
        <!-- Body Frame Axes (bx, by, bz) -->
        <visual name="bx">
          <pose>0.25 0 0.05 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.008</radius><length>0.50</length></cylinder></geometry>
          <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
        </visual>
        <visual name="by">
          <pose>0 0.25 0.05 1.5708 0 0</pose>
          <geometry><cylinder><radius>0.008</radius><length>0.50</length></cylinder></geometry>
          <material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material>
        </visual>
        <visual name="bz">
          <pose>0 0 0.25 0 0 0</pose>
          <geometry><cylinder><radius>0.008</radius><length>0.50</length></cylinder></geometry>
          <material><ambient>0.1 0.4 1 1</ambient><diffuse>0.1 0.4 1 1</diffuse></material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
"""
    os.makedirs(os.path.dirname(os.path.abspath(output_sdf_path)), exist_ok=True)
    with open(output_sdf_path, 'w', encoding='utf-8') as f:
        f.write(sdf_content)
    print(f"Generated Gazebo SDF world: {output_sdf_path}")


if __name__ == '__main__':
    default_yaml = os.path.join(PKG_ROOT, 'config', 'uav_params.yaml')
    default_sdf = os.path.join(PKG_ROOT, 'worlds', 'uav_waypoint_world.sdf')
    build_world_sdf(default_yaml, default_sdf)
