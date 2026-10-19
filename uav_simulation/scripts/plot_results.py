#!/usr/bin/env python3
"""
UAV Trajectory & Paper Controller Plotting Script (plot_results.py)
===================================================================
Reads the recorded CSV file (`uav_trajectory_log.csv`) and generates all
required publication-quality figures:
  1. 3D Reference Trajectory vs Actual UAV Trajectory with Start & Waypoints
  2. X, Y, Z Position vs Time (Reference vs Actual)
  3. X, Y, Z Position Errors (e_x, e_y, e_z) & Euclidean Position Error (e) vs Time
  4. Roll, Pitch, Yaw Euler Angles vs Time
  5. Control Inputs (Total Thrust u_1 and Body Torques tau_x, tau_y, tau_z) vs Time
  6. Paper-Specific States (Quaternion Error q_err, Angular Velocity Error omega_err,
     Backstepping Variable delta) vs Time
"""

import os
import sys
import csv
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from uav_simulation.waypoint_manager import load_waypoints_from_yaml


def load_csv_data(csv_path: str) -> dict:
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise ValueError(f"CSV file is empty: {csv_path}")

    data = {}
    for key in rows[0].keys():
        data[key] = np.array([float(r[key]) for r in rows], dtype=float)
    return data


def generate_all_plots(csv_path: str, output_dir: str, yaml_path: str = None):
    os.makedirs(output_dir, exist_ok=True)
    d = load_csv_data(csv_path)
    start_pos, waypoints, tol = load_waypoints_from_yaml(yaml_path)

    t = d['timestamp']
    plt.rcParams.update({
        'font.size': 10,
        'axes.titlesize': 12,
        'axes.labelsize': 10,
        'figure.facecolor': 'white',
        'axes.grid': True,
        'grid.alpha': 0.35
    })

    # --------------------------------------------------------------------------
    # 1. 3D Reference Trajectory vs Actual UAV Trajectory
    # --------------------------------------------------------------------------
    fig = plt.figure(figsize=(10, 7.5), dpi=160)
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(
        d['reference_x'], d['reference_y'], d['reference_z'],
        '--', color='#00b4d8', linewidth=2.4, label='Reference Trajectory'
    )
    ax.plot(
        d['actual_x'], d['actual_y'], d['actual_z'],
        '-', color='#e63946', linewidth=2.0, label='Actual UAV Trajectory'
    )
    ax.scatter(
        [start_pos[0]], [start_pos[1]], [start_pos[2]],
        color='#1d3557', s=110, marker='o', label=f'Start ({start_pos[0]:.0f}, {start_pos[1]:.0f}, {start_pos[2]:.0f})'
    )
    ax.text(start_pos[0], start_pos[1], start_pos[2] + 0.25, "  Start", fontsize=9, fontweight='bold')

    wp_colors = ['#2a9d8f', '#f4a261', '#8338ec', '#fb8500']
    for i, wp in enumerate(waypoints):
        c = wp_colors[i % len(wp_colors)]
        ax.scatter(
            [wp[0]], [wp[1]], [wp[2]],
            color=c, s=130, marker='^', edgecolors='black',
            label=f'WP{i+1} ({wp[0]:.0f}, {wp[1]:.0f}, {wp[2]:.0f})'
        )
        ax.text(wp[0], wp[1], wp[2] + 0.25, f"  WP{i+1}", fontsize=9, fontweight='bold')

    # 3D Obstacle Columns
    from uav_simulation.trajectory_node import QuinticSegmentTrajectoryGenerator
    traj_gen = QuinticSegmentTrajectoryGenerator(start_pos, waypoints)
    for idx, obs in enumerate(traj_gen.obstacles):
        pos = obs['position']
        r = obs['radius']
        h = obs['height']
        z_c = np.linspace(0, h, 15)
        theta = np.linspace(0, 2 * np.pi, 20)
        theta_grid, z_grid = np.meshgrid(theta, z_c)
        x_grid = pos[0] + r * np.cos(theta_grid)
        y_grid = pos[1] + r * np.sin(theta_grid)
        ax.plot_surface(x_grid, y_grid, z_grid, color='#f97316', alpha=0.45, edgecolor='none')
        ax.scatter([pos[0]], [pos[1]], [h + 0.15], color='#d90429', s=70, marker='^', label='Obstacle Tower' if idx == 0 else "")
        ax.text(pos[0], pos[1], h + 0.35, f"Obs{idx+1}", color='#e85d04', fontsize=8, fontweight='bold')

    ax.set_title('3D Reference Trajectory vs. Actual UAV Trajectory (Obstacle Avoidance & Wind Disturbances)', fontweight='bold')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Altitude (m)')
    ax.legend(loc='upper left', fontsize=9)
    ax.view_init(elev=26, azim=-58)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'fig_3d_trajectory.png'))
    plt.close(fig)

    # --------------------------------------------------------------------------
    # 2. X, Y, Z Position vs Time (Reference vs Actual)
    # --------------------------------------------------------------------------
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, dpi=160)
    coords = [
        ('X', d['actual_x'], d['reference_x'], '#e63946', '#0077b6'),
        ('Y', d['actual_y'], d['reference_y'], '#2a9d8f', '#023e8a'),
        ('Z', d['actual_z'], d['reference_z'], '#f77f00', '#0096c7'),
    ]
    for ax_i, (name, act, ref, c_act, c_ref) in zip(axes, coords):
        ax_i.plot(t, ref, '--', color=c_ref, linewidth=2.2, label=f'{name}_ref (Reference)')
        ax_i.plot(t, act, '-', color=c_act, linewidth=1.8, label=f'{name} (Actual UAV)')
        ax_i.set_ylabel(f'{name} Position (m)')
        ax_i.legend(loc='lower right')
    axes[0].set_title('UAV 3D Position Tracking: Reference vs. Actual (X, Y, Z vs. Time)', fontweight='bold')
    axes[-1].set_xlabel('Time (s)')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'fig_xyz_vs_time.png'))
    plt.close(fig)

    # --------------------------------------------------------------------------
    # 3. Position Tracking Errors (e_x, e_y, e_z, and Euclidean Error e) vs Time
    # --------------------------------------------------------------------------
    fig, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True, dpi=160)
    axes[0].plot(t, d['error_x'], label='e_x = X - X_ref', color='#e63946', linewidth=1.7)
    axes[0].plot(t, d['error_y'], label='e_y = Y - Y_ref', color='#2a9d8f', linewidth=1.7)
    axes[0].plot(t, d['error_z'], label='e_z = Z - Z_ref', color='#0077b6', linewidth=1.7)
    axes[0].axhline(0.0, color='black', linestyle=':', linewidth=1.0)
    axes[0].set_ylabel('Axis Error (m)')
    axes[0].set_title('Position Tracking Errors vs. Time', fontweight='bold')
    axes[0].legend(loc='upper right')

    axes[1].plot(t, d['position_error'], label='Euclidean Error ||e|| = sqrt(e_x^2 + e_y^2 + e_z^2)', color='#7209b7', linewidth=2.0)
    axes[1].axhline(tol, color='#2b9348', linestyle='--', linewidth=1.3, label=f'Waypoint Tolerance ({tol:.2f} m)')
    axes[1].set_ylabel('Euclidean Error (m)')
    axes[1].set_xlabel('Time (s)')
    axes[1].legend(loc='upper right')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'fig_position_errors.png'))
    plt.close(fig)

    # --------------------------------------------------------------------------
    # 4. Attitude Euler Angles (Roll, Pitch, Yaw) vs Time
    # --------------------------------------------------------------------------
    fig, axes = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True, dpi=160)
    axes[0].plot(t, d['roll'], color='#d62828', linewidth=1.8, label='Roll (phi)')
    axes[0].set_ylabel('Roll (deg)')
    axes[0].set_title('UAV Attitude Angles (Roll, Pitch, Yaw) vs. Time', fontweight='bold')
    axes[0].legend(loc='upper right')

    axes[1].plot(t, d['pitch'], color='#0077b6', linewidth=1.8, label='Pitch (theta)')
    axes[1].set_ylabel('Pitch (deg)')
    axes[1].legend(loc='upper right')

    axes[2].plot(t, d['yaw'], color='#2a9d8f', linewidth=1.8, label='Yaw (psi)')
    axes[2].set_ylabel('Yaw (deg)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend(loc='upper right')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'fig_attitude_rpy.png'))
    plt.close(fig)

    # --------------------------------------------------------------------------
    # 5. Control Inputs (Total Thrust u_1 & Body Torques tau_1, tau_2, tau_3)
    # --------------------------------------------------------------------------
    fig, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True, dpi=160)
    axes[0].plot(t, d['thrust_u1'], color='#1d3557', linewidth=1.8, label='Total Collective Thrust u_1 (N)')
    axes[0].set_ylabel('Thrust u_1 (N)')
    axes[0].set_title('Control Inputs: Collective Thrust u_1 and Quaternion Backstepping Torques tau', fontweight='bold')
    axes[0].legend(loc='upper right')

    axes[1].plot(t, d['tau_x'], color='#e63946', linewidth=1.6, label='tau_1 (Roll Torque, N*m)')
    axes[1].plot(t, d['tau_y'], color='#2a9d8f', linewidth=1.6, label='tau_2 (Pitch Torque, N*m)')
    axes[1].plot(t, d['tau_z'], color='#0077b6', linewidth=1.6, label='tau_3 (Yaw Torque, N*m)')
    axes[1].set_ylabel('Torque tau (N*m)')
    axes[1].set_xlabel('Time (s)')
    axes[1].legend(loc='upper right')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'fig_control_inputs.png'))
    plt.close(fig)

    # --------------------------------------------------------------------------
    # 6. Paper Internal States during 3D Flight (q_err, omega_err, delta)
    # --------------------------------------------------------------------------
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, dpi=160)
    axes[0].plot(t, d['q_err0'], label='q_err0 (scalar -> 1)', linewidth=1.8)
    axes[0].plot(t, d['q_err1'], label='q_err1', linewidth=1.5)
    axes[0].plot(t, d['q_err2'], label='q_err2', linewidth=1.5)
    axes[0].plot(t, d['q_err3'], label='q_err3', linewidth=1.5)
    axes[0].set_ylabel('Quaternion Error q_err')
    axes[0].set_title('Paper Attitude Controller Internal States (q_err, omega_err, delta)', fontweight='bold')
    axes[0].legend(loc='lower right', ncol=4)

    axes[1].plot(t, d['omega_err_x'], label='omega_err_x')
    axes[1].plot(t, d['omega_err_y'], label='omega_err_y')
    axes[1].plot(t, d['omega_err_z'], label='omega_err_z')
    axes[1].set_ylabel('omega_err (rad/s)')
    axes[1].legend(loc='upper right', ncol=3)

    axes[2].plot(t, d['delta_x'], label='delta_1')
    axes[2].plot(t, d['delta_y'], label='delta_2')
    axes[2].plot(t, d['delta_z'], label='delta_3')
    axes[2].set_ylabel('delta (rad/s)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend(loc='upper right', ncol=3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'fig_paper_internal_states.png'))
    plt.close(fig)

    print(f"All trajectory & controller plots saved to: {output_dir}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Plot UAV simulation CSV logs.")
    parser.add_argument('--csv', default=os.path.join(PKG_ROOT, 'uav_trajectory_log.csv'))
    parser.add_argument('--out', default=os.path.join(PKG_ROOT, 'plots'))
    parser.add_argument('--config', default=os.path.join(PKG_ROOT, 'config', 'uav_params.yaml'))
    args = parser.parse_args()
    generate_all_plots(args.csv, args.out, args.config)
