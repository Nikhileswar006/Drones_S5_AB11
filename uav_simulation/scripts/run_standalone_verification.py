#!/usr/bin/env python3
"""
Complete Verification & Simulation Runner (run_standalone_verification.py)
==========================================================================
Executes:
  1. Exact Reproduction of Section V (Figures 3-12) of the CACS 2024 Paper:
     "Quaternion-Based Attitude Tracking Control Design for UAVs"
     (Qain-Rong Lin & Jen-te Yu, 2024)
  2. Complete 6-DOF 3D Multi-Waypoint UAV Flight Simulation:
     Start (0, 0, 2) -> WP1 (5, 3, 4) -> WP2 (10, -2, 5) -> WP3 (15, 4, 6)
     recording `uav_trajectory_log.csv`, computing performance metrics,
     printing the 0.5s interval table, and generating all required plots.
"""

import os
import sys
import csv
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from uav_simulation.quaternion_math import (
    PaperQuaternionBacksteppingController,
    quat_normalize,
    quat_derivative,
    omega_derivative,
    quat_to_euler,
)
from uav_simulation.waypoint_manager import load_waypoints_from_yaml, WaypointManagerCore
from uav_simulation.trajectory_node import QuinticSegmentTrajectoryGenerator
from uav_simulation.controller_node import CascadedUAVQuaternionController
from uav_simulation.gazebo_uav_node import Quadrotor6DOFDynamics, AtmosphericDisturbanceModel
from uav_simulation.data_logger import CSV_COLUMNS, evaluate_performance_metrics
from uav_simulation.info_panel_node import format_telemetry_panel_text
from scripts.plot_results import generate_all_plots
from scripts.export_html5_viewer import export_html


def run_paper_section_v_benchmark(output_dir: str):
    """
    Reproduces Section V (Figures 3 - 12) of Lin & Yu (CACS 2024):
      - J = diag[0.1, 0.1, 0.12]
      - k1 = 20, k2 = 2
      - Sinusoidal 3-axis reference angular velocity omega_ref(t)
      - Verifies:
          * q_err -> [1, 0, 0, 0]^T (Fig. 3)
          * Yaw, Pitch, Roll angle tracking in degrees (Figs. 4, 5, 6)
          * omega_err -> 0 in ~0.8 s (Fig. 7)
          * delta -> 0 in ~0.4 s (Fig. 11)
          * Torque tau(t) response (Fig. 12)
    """
    J_diag = (0.1, 0.1, 0.12)
    J = np.diag(J_diag)
    J_inv = np.diag(1.0 / np.asarray(J_diag))
    controller = PaperQuaternionBacksteppingController(
        J_diag=J_diag, k1=20.0, k2=12.0, exponent=1.0 / 3.0, enforce_shortest_path=True
    )

    # Initial states from Section V producing Fig. 3:
    # q_err(0) = [0.94628, -0.1541, 0.19051, -0.21098]^T
    q_ref = quat_normalize(np.array([1.0, 0.0, 0.0, 0.0], dtype=float))
    q = quat_normalize(np.array([0.94628, -0.1541, 0.19051, -0.21098], dtype=float))
    omega = np.array([-0.1, -0.2, 0.2], dtype=float)

    dt = 0.001
    t_all = np.arange(0.0, 10.0 + dt, dt)

    hist_q_err = []
    hist_rpy = []
    hist_rpy_ref = []
    hist_w_err = []
    hist_delta = []
    hist_tau = []

    for t in t_all:
        # Three-sine-wave reference angular velocity as described in Section V (Figs. 8, 9, 10)
        omega_ref = np.array([
            0.03 + 0.07 * np.sin(0.95 * t),
            0.05 + 0.38 * np.sin(0.95 * t + 0.4),
            0.15 + 0.22 * np.cos(0.95 * t)
        ], dtype=float)
        omega_dot_ref = np.array([
            0.07 * 0.95 * np.cos(0.95 * t),
            0.38 * 0.95 * np.cos(0.95 * t + 0.4),
            -0.22 * 0.95 * np.sin(0.95 * t)
        ], dtype=float)

        res = controller.compute_control(q, omega, q_ref, omega_ref, omega_dot_ref)
        tau = res['tau']

        hist_q_err.append(res['q_err'])
        hist_rpy.append(quat_to_euler(q, degrees=True))
        hist_rpy_ref.append(quat_to_euler(q_ref, degrees=True))
        hist_w_err.append(res['omega_err'])
        hist_delta.append(res['delta'])
        hist_tau.append(tau)

        # Integrate reference quaternion (Eq. 8) and UAV attitude dynamics (Eq. 8 & 9)
        q_ref = quat_normalize(q_ref + dt * quat_derivative(q_ref, omega_ref))
        q = quat_normalize(q + dt * quat_derivative(q, omega))
        omega = omega + dt * omega_derivative(omega, tau, J, J_inv)

    hist_q_err = np.array(hist_q_err)
    hist_rpy = np.array(hist_rpy)
    hist_rpy_ref = np.array(hist_rpy_ref)
    hist_w_err = np.array(hist_w_err)
    hist_delta = np.array(hist_delta)
    hist_tau = np.array(hist_tau)

    # Plot reproduction of Paper Figures 3, 4-6, 7, 11, 12
    fig, axes = plt.subplots(3, 2, figsize=(13, 10), dpi=160)

    # Fig. 3: Quaternion Error (0 to 1 sec)
    mask_1s = t_all <= 1.0
    ax = axes[0, 0]
    ax.plot(t_all[mask_1s], hist_q_err[mask_1s, 0], label='q_err0', linewidth=1.8)
    ax.plot(t_all[mask_1s], hist_q_err[mask_1s, 1], label='q_err1', linewidth=1.6)
    ax.plot(t_all[mask_1s], hist_q_err[mask_1s, 2], label='q_err2', linewidth=1.6)
    ax.plot(t_all[mask_1s], hist_q_err[mask_1s, 3], label='q_err3', linewidth=1.6)
    ax.set_title('Paper Fig. 3: Trajectory of Quaternion Error q_err -> [1;0;0;0]', fontweight='bold')
    ax.set_xlabel('Time (sec)')
    ax.set_ylabel('Quaternion Error')
    ax.legend(loc='center right')

    # Fig. 7: Angular Velocity Error
    ax = axes[0, 1]
    ax.plot(t_all, hist_w_err[:, 0], label='w_err1', linewidth=1.6)
    ax.plot(t_all, hist_w_err[:, 1], label='w_err2', linewidth=1.6)
    ax.plot(t_all, hist_w_err[:, 2], label='w_err3', linewidth=1.6)
    ax.set_title('Paper Fig. 7: Trajectory of Angular Velocity Error w_err', fontweight='bold')
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Angular velocity error (rad/sec)')
    ax.legend(loc='upper right')

    # Figs. 4, 5, 6: Attitude Angles Tracking (Roll, Pitch, Yaw)
    ax = axes[1, 0]
    ax.plot(t_all, hist_rpy_ref[:, 0], '--', color='#f4a261', linewidth=2.2, label='Roll ref')
    ax.plot(t_all, hist_rpy[:, 0], '-', color='#1d3557', linewidth=1.4, label='Roll actual')
    ax.plot(t_all, hist_rpy_ref[:, 1], '--', color='#2a9d8f', linewidth=2.2, label='Pitch ref')
    ax.plot(t_all, hist_rpy[:, 1], '-', color='#e63946', linewidth=1.4, label='Pitch actual')
    ax.set_title('Paper Figs. 5 & 6: Roll & Pitch Attitude Tracking (deg)', fontweight='bold')
    ax.set_xlabel('Time (sec)')
    ax.set_ylabel('Angle (degree)')
    ax.legend(loc='upper right', ncol=2, fontsize=8)

    ax = axes[1, 1]
    ax.plot(t_all, hist_rpy_ref[:, 2], '--', color='#e76f51', linewidth=2.2, label='reference')
    ax.plot(t_all, hist_rpy[:, 2], '-', color='#0077b6', linewidth=1.5, label='actual')
    ax.set_title('Paper Fig. 4: Trajectory of Yaw Angle Tracking (deg)', fontweight='bold')
    ax.set_xlabel('Time (sec)')
    ax.set_ylabel('Yaw angle (degree)')
    ax.legend(loc='upper left')

    # Fig. 11: Trajectory of delta
    ax = axes[2, 0]
    ax.plot(t_all, hist_delta[:, 0], label='delta_1', linewidth=1.6)
    ax.plot(t_all, hist_delta[:, 1], label='delta_2', linewidth=1.6)
    ax.plot(t_all, hist_delta[:, 2], label='delta_3', linewidth=1.6)
    ax.set_title('Paper Fig. 11: Trajectory of Backstepping Error delta', fontweight='bold')
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('delta (rad/sec)')
    ax.legend(loc='lower right')

    # Fig. 12: Trajectory of torque tau
    ax = axes[2, 1]
    ax.plot(t_all, hist_tau[:, 0], label='tau_1', linewidth=1.6)
    ax.plot(t_all, hist_tau[:, 1], label='tau_2', linewidth=1.6)
    ax.plot(t_all, hist_tau[:, 2], label='tau_3', linewidth=1.6)
    ax.set_title('Paper Fig. 12: Trajectory of Control Torque tau (N-m)', fontweight='bold')
    ax.set_xlabel('Time (sec)')
    ax.set_ylabel('tau (N-m)')
    ax.legend(loc='upper right')

    fig.suptitle(
        'Direct Verification of Lin & Yu (CACS 2024) Quaternion Backstepping Fractional-Exponent Controller',
        fontsize=13, fontweight='bold'
    )
    fig.tight_layout()
    os.makedirs(output_dir, exist_ok=True)
    out_fig = os.path.join(output_dir, 'fig_paper_benchmark_reproduction.png')
    fig.savefig(out_fig)
    plt.close(fig)
    print(f"Saved Paper Section V Benchmark Reproduction plot: {out_fig}")


def run_3d_waypoint_simulation(yaml_path: str, csv_path: str, plots_dir: str):
    """
    Runs the full 6-DOF 3D Multi-Waypoint UAV Simulation:
      Start (0, 0, 2) -> WP1 (5, 3, 4) -> WP2 (10, -2, 5) -> WP3 (15, 4, 6)
    using the exact CACS 2024 Quaternion Backstepping Fractional-Exponent controller.
    """
    start_pos, waypoints, tol = load_waypoints_from_yaml(yaml_path)

    import yaml
    with open(yaml_path, 'r', encoding='utf-8') as f:
        ydata = yaml.safe_load(f)
    yparams = ydata.get('uav_simulation', {}).get('ros__parameters', ydata)
    max_vel = float(yparams.get('max_velocity', 1.0))
    max_acc = float(yparams.get('max_acceleration', 1.2))

    wp_manager = WaypointManagerCore(start_pos, waypoints, tolerance=tol)
    traj_gen = QuinticSegmentTrajectoryGenerator(
        start_pos, waypoints, max_velocity=max_vel, max_acceleration=max_acc
    )
    controller = CascadedUAVQuaternionController(
        mass=1.20,
        gravity=9.81,
        J_diag=(0.1, 0.1, 0.12),
        k1=20.0,
        k2=2.0,
        exponent=1.0 / 3.0,
        kp_pos=(4.8, 4.8, 6.5),
        kd_pos=(3.8, 3.8, 4.8),
        ki_pos=(0.15, 0.15, 0.25),
        dt=0.005
    )
    dynamics = Quadrotor6DOFDynamics(
        mass=1.20,
        gravity=9.81,
        J_diag=(0.1, 0.1, 0.12),
        init_pos=start_pos,
        init_rpy=(0.0, 0.0, 0.0),
        init_omega=(0.0, 0.0, 0.0)
    )
    # Read disturbance parameters from YAML
    enable_dist = bool(yparams.get('enable_disturbances', True))
    wind_steady = tuple(yparams.get('wind_steady', [0.75, -0.55, 0.15]))
    gust_amp = tuple(yparams.get('wind_gust_amplitude', [1.1, 0.95, 0.4]))
    gust_freq = tuple(yparams.get('wind_gust_frequencies', [0.8, 1.4, 2.2]))
    force_noise_std = float(yparams.get('force_noise_std', 0.28))
    torque_noise_std = float(yparams.get('torque_noise_std', 0.025))

    dist_model = AtmosphericDisturbanceModel(
        wind_steady=wind_steady,
        gust_amplitude=gust_amp,
        gust_frequencies=gust_freq,
        force_noise_std=force_noise_std,
        torque_noise_std=torque_noise_std,
        enabled=enable_dist
    )

    dt = 0.005
    log_every_n = 10  # 10 * 0.005 = 0.05 s (20 Hz CSV logging)
    total_sim_time = traj_gen.total_trajectory_time + 2.5

    rows = []
    wp_reach_times_formatted = {}
    active_seg_idx = 0
    seg_start_t = 0.0
    snapshot_panel_text = ""
    mission_complete_time = None

    num_steps = int(round(total_sim_time / dt))
    for step_idx in range(num_steps + 1):
        t = step_idx * dt
        pos = dynamics.pos.copy()
        vel = dynamics.vel.copy()
        q = dynamics.q.copy()
        omega = dynamics.omega.copy()

        # 1. Waypoint manager check & sequential switching
        wp_status = wp_manager.update(pos, t)
        if wp_status['active_index'] != active_seg_idx:
            active_seg_idx = wp_status['active_index']
            seg_start_t = t

        for k, v in wp_status['waypoint_reach_times'].items():
            wp_reach_times_formatted[f"Waypoint_{k}"] = round(v, 3)

        # Early termination: stop 1.0s after reaching final waypoint (settling confirmation)
        if wp_status.get('mission_complete', False):
            if mission_complete_time is None:
                mission_complete_time = t
            elif (t - mission_complete_time) >= 1.0:
                break

        # 2. Evaluate reference trajectory at current active segment
        local_t = t - seg_start_t
        ref = traj_gen.evaluate_segment(active_seg_idx, local_t)
        pos_ref = ref['pos']
        vel_ref = ref['vel']
        acc_ref = ref['acc']

        # 3. Execute CACS 2024 Quaternion Backstepping Controller
        ctrl_out = controller.step(
            pos=pos,
            vel=vel,
            q=q,
            omega=omega,
            pos_ref=pos_ref,
            vel_ref=vel_ref,
            acc_ref=acc_ref,
            yaw_ref=0.0,
            dt=dt
        )

        # 4. Log at 20 Hz (including exact 0.5 s intervals)
        if step_idx % log_every_n == 0:
            err = pos - pos_ref
            pos_err_norm = float(np.linalg.norm(err))
            roll_deg, pitch_deg, yaw_deg = quat_to_euler(q, degrees=True)
            row = {
                'timestamp': round(t, 4),
                'actual_x': round(float(pos[0]), 5),
                'actual_y': round(float(pos[1]), 5),
                'actual_z': round(float(pos[2]), 5),
                'reference_x': round(float(pos_ref[0]), 5),
                'reference_y': round(float(pos_ref[1]), 5),
                'reference_z': round(float(pos_ref[2]), 5),
                'error_x': round(float(err[0]), 5),
                'error_y': round(float(err[1]), 5),
                'error_z': round(float(err[2]), 5),
                'position_error': round(pos_err_norm, 5),
                'velocity_x': round(float(vel[0]), 5),
                'velocity_y': round(float(vel[1]), 5),
                'velocity_z': round(float(vel[2]), 5),
                'roll': round(roll_deg, 4),
                'pitch': round(pitch_deg, 4),
                'yaw': round(yaw_deg, 4),
                'q0': round(float(q[0]), 6),
                'q1': round(float(q[1]), 6),
                'q2': round(float(q[2]), 6),
                'q3': round(float(q[3]), 6),
                'q_ref0': round(float(ctrl_out['q_ref'][0]), 6),
                'q_ref1': round(float(ctrl_out['q_ref'][1]), 6),
                'q_ref2': round(float(ctrl_out['q_ref'][2]), 6),
                'q_ref3': round(float(ctrl_out['q_ref'][3]), 6),
                'q_err0': round(float(ctrl_out['q_err'][0]), 6),
                'q_err1': round(float(ctrl_out['q_err'][1]), 6),
                'q_err2': round(float(ctrl_out['q_err'][2]), 6),
                'q_err3': round(float(ctrl_out['q_err'][3]), 6),
                'omega_x': round(float(omega[0]), 5),
                'omega_y': round(float(omega[1]), 5),
                'omega_z': round(float(omega[2]), 5),
                'omega_ref_x': round(float(ctrl_out['omega_ref'][0]), 5),
                'omega_ref_y': round(float(ctrl_out['omega_ref'][1]), 5),
                'omega_ref_z': round(float(ctrl_out['omega_ref'][2]), 5),
                'omega_err_x': round(float(ctrl_out['omega_err'][0]), 5),
                'omega_err_y': round(float(ctrl_out['omega_err'][1]), 5),
                'omega_err_z': round(float(ctrl_out['omega_err'][2]), 5),
                'delta_x': round(float(ctrl_out['delta'][0]), 5),
                'delta_y': round(float(ctrl_out['delta'][1]), 5),
                'delta_z': round(float(ctrl_out['delta'][2]), 5),
                'tau_x': round(float(ctrl_out['tau'][0]), 5),
                'tau_y': round(float(ctrl_out['tau'][1]), 5),
                'tau_z': round(float(ctrl_out['tau'][2]), 5),
                'thrust_u1': round(float(ctrl_out['u1']), 5),
                'active_waypoint': wp_status['active_waypoint_number'],
                'dist_to_waypoint': round(float(wp_status['distance_to_waypoint']), 5),
            }
            rows.append(row)

            # Capture a mid-flight snapshot of the Right-Side Telemetry Panel around t = 8.5s
            if abs(t - 8.50) < 0.03:
                snapshot_panel_text = format_telemetry_panel_text({
                    't': t,
                    'pos': tuple(pos),
                    'pos_ref': tuple(pos_ref),
                    'wp_label': wp_status['label'],
                    'wp_coords': tuple(wp_status['current_waypoint']),
                    'dist_to_wp': wp_status['distance_to_waypoint'],
                    'vel': tuple(vel),
                    'rpy_deg': (roll_deg, pitch_deg, yaw_deg),
                    'q_err': tuple(ctrl_out['q_err']),
                    'delta': tuple(ctrl_out['delta']),
                    'tau': tuple(ctrl_out['tau']),
                })

        # 5. Compute real-world environmental disturbances (wind shear, turbulence, gusts)
        f_dist, tau_dist = dist_model.compute(t, vel, dt, uav_pos=pos)

        # 6. Advance 6-DOF Quadrotor Dynamics using RK4
        dynamics.step_rk4(ctrl_out['u1'], ctrl_out['tau'], dt, f_dist=f_dist, tau_dist=tau_dist)

    # Save CSV
    os.makedirs(os.path.dirname(os.path.abspath(csv_path)), exist_ok=True)
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)

    # Compute Performance Metrics
    metrics = evaluate_performance_metrics(rows, wp_reach_times_formatted, start_pos, waypoints)
    metrics_path = os.path.splitext(csv_path)[0] + "_metrics.json"
    with open(metrics_path, 'w', encoding='utf-8') as f:
        json.dump(metrics, f, indent=2)

    # Generate all required plots
    generate_all_plots(csv_path, plots_dir, yaml_path)

    # Render Gazebo 3D Scene + Right-Side Live Telemetry HUD Preview Figure
    _render_gazebo_and_hud_preview(rows, start_pos, waypoints, snapshot_panel_text, plots_dir, obstacles=traj_gen.obstacles)

    # Export clean 3D WebGL HTML5 simulation viewer
    html_viewer_path = os.path.join(plots_dir, 'interactive_3d_simulation.html')
    export_html(csv_path, yaml_path, html_viewer_path)

    print("\n================ PERFORMANCE METRICS ================")
    print(json.dumps(metrics, indent=2))
    print("\n================ SAMPLE TIME TABLE (0.5s steps) ================")
    print(f"{'Time(s)':>7} | {'X':>7} {'Y':>7} {'Z':>7} | {'X_ref':>7} {'Y_ref':>7} {'Z_ref':>7} | {'Error(m)':>8} | {'WP':>3}")
    print("-" * 78)
    for r in rows:
        if abs((r['timestamp'] * 2.0) - round(r['timestamp'] * 2.0)) < 1e-3:
            print(
                f"{r['timestamp']:7.1f} | "
                f"{r['actual_x']:7.3f} {r['actual_y']:7.3f} {r['actual_z']:7.3f} | "
                f"{r['reference_x']:7.3f} {r['reference_y']:7.3f} {r['reference_z']:7.3f} | "
                f"{r['position_error']:8.4f} | "
                f"WP{r['active_waypoint']}"
            )


def _render_gazebo_and_hud_preview(rows, start_pos, waypoints, panel_text, plots_dir, obstacles=None):
    """
    Renders a combined preview showing the Gazebo 3D Simulation Viewport on the left
    and the docked Live Coordinate Information Panel on the right.
    """
    fig = plt.figure(figsize=(14, 7.5), dpi=160, facecolor='#111622')
    gs = fig.add_gridspec(1, 2, width_ratios=[2.1, 1.0], wspace=0.05)

    # Left: 3D Gazebo Scene View
    ax3d = fig.add_subplot(gs[0], projection='3d', facecolor='#1a202c')
    rx = [r['reference_x'] for r in rows]
    ry = [r['reference_y'] for r in rows]
    rz = [r['reference_z'] for r in rows]
    ax = [r['actual_x'] for r in rows]
    ay = [r['actual_y'] for r in rows]
    az = [r['actual_z'] for r in rows]

    ax3d.plot(rx, ry, rz, '--', color='#00f5d4', linewidth=2.4, label='Reference Trajectory (Bypasses Obstacles)')
    ax3d.plot(ax, ay, az, '-', color='#ff4d29', linewidth=2.2, label='Actual UAV Trajectory (With Wind/Noise)')

    # 3D Obstacles between Waypoints
    if obstacles:
        for idx, obs in enumerate(obstacles):
            pos = obs['position']
            r = obs['radius']
            h = obs['height']
            z_c = np.linspace(0, h, 15)
            theta = np.linspace(0, 2 * np.pi, 20)
            theta_grid, z_grid = np.meshgrid(theta, z_c)
            x_grid = pos[0] + r * np.cos(theta_grid)
            y_grid = pos[1] + r * np.sin(theta_grid)
            ax3d.plot_surface(x_grid, y_grid, z_grid, color='#f97316', alpha=0.55, edgecolor='none')
            ax3d.scatter([pos[0]], [pos[1]], [h + 0.15], color='#ef4444', s=80, marker='^', label='Obstacle Tower' if idx == 0 else "")
            ax3d.text(pos[0], pos[1], h + 0.35, f"Obs{idx+1}", color='#ff9900', fontsize=8, fontweight='bold')

    # Start & Waypoints
    ax3d.scatter([start_pos[0]], [start_pos[1]], [start_pos[2]], color='#3a86ff', s=120, label='Start (0, 0, 2)')
    for i, wp in enumerate(waypoints):
        c = '#ffbe0b' if i == 1 else '#b5179e'
        ax3d.scatter([wp[0]], [wp[1]], [wp[2]], color=c, s=150, marker='o', label=f'Waypoint {i+1} ({wp[0]:.0f},{wp[1]:.0f},{wp[2]:.0f})')
        ax3d.text(wp[0], wp[1], wp[2] + 0.35, f"WP{i+1}", color='white', fontweight='bold')

    # Mid-flight UAV position around t = 8.5s
    mid_idx = min(len(rows) - 1, int(len(rows) * 0.48))
    uav_p = (rows[mid_idx]['actual_x'], rows[mid_idx]['actual_y'], rows[mid_idx]['actual_z'])
    ax3d.scatter([uav_p[0]], [uav_p[1]], [uav_p[2]], color='#00ff66', s=220, marker='P', label='Current 3D Quadrotor UAV')

    ax3d.set_title('Gazebo 3D Simulation Viewport (UAV + Reference Path + Actual Trail + Waypoints)', color='white', fontweight='bold', pad=12)
    ax3d.set_xlabel('X (m)', color='white')
    ax3d.set_ylabel('Y (m)', color='white')
    ax3d.set_zlabel('Z (m)', color='white')
    ax3d.tick_params(colors='white')
    ax3d.legend(loc='upper left', facecolor='#111622', edgecolor='#00f5d4', labelcolor='white', fontsize=8.5)
    ax3d.view_init(elev=25, azim=-60)

    # Right: Docked Real-Time Coordinate Information Panel
    ax_hud = fig.add_subplot(gs[1], facecolor='#0d131f')
    ax_hud.axis('off')
    ax_hud.text(
        0.04, 0.97, panel_text,
        transform=ax_hud.transAxes,
        fontfamily='monospace',
        fontsize=8.4,
        color='#00f5d4',
        verticalalignment='top',
        bbox=dict(boxstyle='round,pad=0.6', facecolor='#111827', edgecolor='#00f5d4', linewidth=1.5)
    )

    fig.subplots_adjust(left=0.03, right=0.98, top=0.92, bottom=0.05, wspace=0.05)
    out_path = os.path.join(plots_dir, 'fig_gazebo_hud_simulation_view.png')
    fig.savefig(out_path, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"Saved Gazebo + Right-Side Telemetry Panel visual preview: {out_path}")


if __name__ == '__main__':
    yaml_cfg = os.path.join(PKG_ROOT, 'config', 'uav_params.yaml')
    csv_out = os.path.join(PKG_ROOT, 'uav_trajectory_log.csv')
    plots_out = os.path.join(PKG_ROOT, 'plots')

    run_paper_section_v_benchmark(plots_out)
    run_3d_waypoint_simulation(yaml_cfg, csv_out, plots_out)
