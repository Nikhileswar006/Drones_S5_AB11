#!/usr/bin/env python3
"""
Direct Live Gazebo 3D Simulation Runner (run_live_in_gazebo.py)
===============================================================
Launches the real Gazebo 3D Simulator (`gz sim -r worlds/uav_waypoint_world.sdf`),
opens the docked Right-Side Real-Time Coordinate Information Panel, and executes
the CACS 2024 Quaternion Backstepping Fractional-Exponent Controller in real time,
updating:
  1. The 3D Quadrotor UAV 6-DOF pose (X, Y, Z, Roll, Pitch, Yaw) inside Gazebo
  2. The Active Waypoint highlight halo inside Gazebo
  3. The Instantaneous Reference Position sphere inside Gazebo
  4. The Live Actual UAV Trajectory trail (`/marker` LINE_STRIP) inside Gazebo
  5. The Right-Side Real-Time Coordinate Information Panel
  6. The CSV log (`uav_trajectory_log.csv`)
"""

import os
import sys
import time
import shutil
import subprocess
import threading
import csv
import numpy as np

PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from uav_simulation.quaternion_math import quat_to_euler
from uav_simulation.waypoint_manager import load_waypoints_from_yaml, WaypointManagerCore
from uav_simulation.trajectory_node import QuinticSegmentTrajectoryGenerator
from uav_simulation.controller_node import CascadedUAVQuaternionController
from uav_simulation.gazebo_uav_node import Quadrotor6DOFDynamics
from uav_simulation.data_logger import CSV_COLUMNS
from uav_simulation.info_panel_node import format_telemetry_panel_text
from scripts.generate_gazebo_world import build_world_sdf


def update_gazebo_pose(gz_cli: str, model_name: str, pos, q=(1.0, 0.0, 0.0, 0.0)):
    req = (
        f'name: "{model_name}", '
        f'position: {{x: {pos[0]:.4f}, y: {pos[1]:.4f}, z: {pos[2]:.4f}}}, '
        f'orientation: {{w: {q[0]:.5f}, x: {q[1]:.5f}, y: {q[2]:.5f}, z: {q[3]:.5f}}}'
    )
    subprocess.Popen(
        [
            gz_cli, 'service', '-s', '/world/uav_world/set_pose',
            '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '60', '-r', req
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )


def update_gazebo_trail(gz_cli: str, actual_pts: list):
    if len(actual_pts) < 2:
        return
    pts_sub = actual_pts[::max(1, len(actual_pts) // 60)]
    if not np.allclose(pts_sub[-1], actual_pts[-1]):
        pts_sub.append(actual_pts[-1])
    pts_str = " ".join(
        [f"point: {{x: {p[0]:.3f}, y: {p[1]:.3f}, z: {p[2]:.3f}}}" for p in pts_sub]
    )
    marker_req = (
        f'ns: "actual_uav_trajectory", id: 2, action: ADD_MODIFY, '
        f'type: LINE_STRIP, visibility: GUI, '
        f'material: {{ambient: {{r: 1.0, g: 0.25, b: 0.05, a: 1.0}}, '
        f'diffuse: {{r: 1.0, g: 0.25, b: 0.05, a: 1.0}}}}, '
        f'{pts_str}'
    )
    subprocess.Popen(
        [
            gz_cli, 'service', '-s', '/marker',
            '--reqtype', 'gz.msgs.Marker', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '60', '-r', marker_req
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )


def run_live_gazebo_simulation():
    yaml_path = os.path.join(PKG_ROOT, 'config', 'uav_params.yaml')
    sdf_path = os.path.join('/tmp', 'uav_waypoint_world.sdf') if os.name != 'nt' else os.path.join(PKG_ROOT, 'worlds', 'uav_waypoint_world.sdf')
    csv_path = os.path.join(PKG_ROOT, 'uav_trajectory_log.csv')

    # 1. Regenerate Gazebo SDF world from current config/uav_params.yaml
    build_world_sdf(yaml_path, sdf_path)
    start_pos, waypoints, tol = load_waypoints_from_yaml(yaml_path)

    gz_cli = shutil.which('gz') or shutil.which('ign')
    if not gz_cli:
        print("ERROR: Gazebo ('gz' or 'ign') is not installed in this environment.")
        print("Run './setup_and_run_gazebo_wsl.sh' inside WSL Ubuntu to install Gazebo Harmonic automatically.")
        sys.exit(1)

    # 2. Launch Gazebo GUI with our generated SDF world
    print(f"Launching Gazebo 3D Simulator with world: {sdf_path} ...")
    gz_cmd = [gz_cli, 'sim', '-r', sdf_path] if os.path.basename(gz_cli) == 'gz' else [gz_cli, 'gazebo', '-r', sdf_path]
    gz_proc = subprocess.Popen(gz_cmd)

    # Give Gazebo 4 seconds to open its 3D viewport
    time.sleep(4.0)

    # Shared telemetry state for the Right-Side Panel
    shared_state = {
        't': 0.0,
        'pos': tuple(start_pos),
        'pos_ref': tuple(start_pos),
        'wp_label': 'Waypoint 1',
        'wp_coords': tuple(waypoints[0]),
        'dist_to_wp': float(np.linalg.norm(np.array(start_pos) - np.array(waypoints[0]))),
        'running': True,
    }

    # 3. Physics + Controller Loop Thread
    def sim_worker():
        wp_manager = WaypointManagerCore(start_pos, waypoints, tolerance=tol)
        traj_gen = QuinticSegmentTrajectoryGenerator(start_pos, waypoints, max_velocity=2.5, max_acceleration=5.0)
        controller = CascadedUAVQuaternionController(
            mass=1.20, gravity=9.81, J_diag=(0.1, 0.1, 0.12),
            k1=20.0, k2=2.0, exponent=1.0 / 3.0, dt=0.005
        )
        dynamics = Quadrotor6DOFDynamics(
            mass=1.20, gravity=9.81, J_diag=(0.1, 0.1, 0.12), init_pos=start_pos
        )

        dt = 0.005
        total_time = traj_gen.total_trajectory_time + 4.0
        num_steps = int(total_time / dt)
        active_seg_idx = 0
        seg_start_t = 0.0
        actual_points = [np.array(start_pos, dtype=float)]
        rows = []

        t_wall_start = time.perf_counter()
        for step_idx in range(num_steps + 1):
            if not shared_state['running']:
                break
            t = step_idx * dt
            pos = dynamics.pos.copy()
            vel = dynamics.vel.copy()
            q = dynamics.q.copy()
            omega = dynamics.omega.copy()

            wp_status = wp_manager.update(pos, t)
            if wp_status['active_index'] != active_seg_idx:
                active_seg_idx = wp_status['active_index']
                seg_start_t = t

            ref = traj_gen.evaluate_segment(active_seg_idx, t - seg_start_t)
            pos_ref = ref['pos']
            ctrl_out = controller.step(
                pos=pos, vel=vel, q=q, omega=omega,
                pos_ref=pos_ref, vel_ref=ref['vel'], acc_ref=ref['acc'],
                yaw_ref=0.0, dt=dt
            )
            dynamics.step_rk4(ctrl_out['u1'], ctrl_out['tau'], dt)

            # Update shared HUD telemetry
            shared_state['t'] = t
            shared_state['pos'] = tuple(pos)
            shared_state['pos_ref'] = tuple(pos_ref)
            shared_state['wp_label'] = wp_status['label']
            shared_state['wp_coords'] = tuple(wp_status['current_waypoint'])
            shared_state['dist_to_wp'] = wp_status['distance_to_waypoint']

            if np.linalg.norm(pos - actual_points[-1]) >= 0.04:
                actual_points.append(pos.copy())

            # Stream pose & trail to Gazebo at 25 Hz (every 8 steps = 0.04s)
            if step_idx % 8 == 0:
                update_gazebo_pose(gz_cli, 'quadrotor', pos, q)
                update_gazebo_pose(gz_cli, 'reference_target_sphere', pos_ref)
                update_gazebo_pose(gz_cli, 'active_waypoint_halo', wp_status['current_waypoint'])

            if step_idx % 20 == 0:
                update_gazebo_trail(gz_cli, actual_points)

            # Log CSV at 20 Hz
            if step_idx % 10 == 0:
                err = pos - pos_ref
                roll_deg, pitch_deg, yaw_deg = quat_to_euler(q, degrees=True)
                rows.append({
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
                    'position_error': round(float(np.linalg.norm(err)), 5),
                    'velocity_x': round(float(vel[0]), 5),
                    'velocity_y': round(float(vel[1]), 5),
                    'velocity_z': round(float(vel[2]), 5),
                    'roll': round(roll_deg, 4),
                    'pitch': round(pitch_deg, 4),
                    'yaw': round(yaw_deg, 4),
                    'q0': round(float(q[0]), 6), 'q1': round(float(q[1]), 6),
                    'q2': round(float(q[2]), 6), 'q3': round(float(q[3]), 6),
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
                })

            # Real-time pacing
            target_wall = t_wall_start + t
            sleep_s = target_wall - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)

        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            writer.writerows(rows)
        print(f"Saved Gazebo flight CSV log to: {csv_path}")

    worker = threading.Thread(target=sim_worker, daemon=True)
    worker.start()

    # 4. Open Right-Side Live Telemetry Panel Window docked next to Gazebo
    try:
        import tkinter as tk
        root = tk.Tk()
        root.title("Gazebo UAV Live Coordinate Panel")
        root.configure(bg="#111622")
        root.attributes("-topmost", True)
        screen_w = root.winfo_screenwidth()
        root.geometry(f"380x560+{max(0, screen_w - 395)}+50")

        lbl = tk.Label(
            root, text="", font=("Consolas", 10, "bold"),
            fg="#00f5d4", bg="#111622", justify=tk.LEFT, anchor="nw", padx=14, pady=12
        )
        lbl.pack(fill=tk.BOTH, expand=True)

        def refresh_hud():
            lbl.config(text=format_telemetry_panel_text(shared_state))
            if gz_proc.poll() is None:
                root.after(50, refresh_hud)
            else:
                root.destroy()

        refresh_hud()
        root.mainloop()
    except Exception:
        worker.join()

    shared_state['running'] = False
    if gz_proc.poll() is None:
        gz_proc.wait()


if __name__ == '__main__':
    run_live_gazebo_simulation()
