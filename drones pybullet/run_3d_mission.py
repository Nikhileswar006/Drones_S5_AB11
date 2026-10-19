"""
Main 3D UAV Mission Execution Script (PyBullet Controller)
Executes full 3D trajectory tracking from Start to Destination.

Workflow:
1. Generates smooth 5th-order minimum-jerk trajectory from P_start to P_goal.
2. Runs real-time PyBullet simulation with 3D debug trajectory rendering.
3. Cascades outer-loop position controller with the paper's Quaternion-Based
   Attitude Backstepping Controller (Eqs. 10 - 23 with delta^(1/3)).
4. Logs telemetry and generates publication-grade plots replicating Figures 3-12 of the paper.
"""

import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt

# Ensure local directory is in path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from uav_quaternion_backstepping import QuaternionMath, QuaternionBacksteppingController
from uav_trajectory import QuinticTrajectory3D, CurvedMissionTrajectory3D, MultiWaypointTrajectory3D, ObstacleAvoidanceTrajectory3D
from uav_position_controller import UAVPositionController
from uav_pybullet_env import UAVPyBulletEnv, LiveTrackingPanel, CameraController


def run_3d_mission(p_start=np.array([0.0, 0.0, 0.5]),
                   p_goal=np.array([4.0, 4.0, 2.5]),
                   waypoints=None,
                   flight_duration=12.0,
                   total_sim_time=15.0,
                   trajectory_type="multi_waypoint",
                   arch_height=1.5,
                   lateral_curve=1.5,
                   gui=True,
                   save_plots=True,
                   export_csv=True,
                   noise_std=0.0,
                   add_obstacles=False,
                   dynamic_obstacle=False,
                   reactive_lidar=False,
                   manual_cam=True):
    """
    Runs the 3D UAV navigation mission in PyBullet.

    Args:
        p_start: [x, y, z] start coordinates (m)
        p_goal: [x, y, z] destination coordinates (m)
        flight_duration: Time to reach destination along smooth trajectory (s)
        total_sim_time: Total simulation time including stabilization hover (s)
        trajectory_type: "quintic" (minimum-jerk) or "curved" (3D arch & lateral curve)
        arch_height: Vertical height of curve (m) if trajectory_type == "curved"
        lateral_curve: Horizontal curve offset (m) if trajectory_type == "curved"
        gui: Launch interactive 3D PyBullet visualizer
        save_plots: Save telemetry comparison figures to disk
        export_csv: Save timestamped coordinates to flight_coordinates.csv
    """
    print("=" * 70)
    print(" 3D UAV Trajectory Tracking with Quaternion Backstepping Control")
    print(f" Trajectory Type:      {trajectory_type.upper()}")
    print(f" Start Position:       {p_start.tolist()}")
    print(f" Destination Position: {p_goal.tolist()}")
    print(f" Flight Duration:      {flight_duration} s (Total Sim: {total_sim_time} s)")
    if trajectory_type == "curved":
        print(f" 3D Curve Settings:    Arch Height = {arch_height} m, Lateral Curve = {lateral_curve} m")
    print("=" * 70)

    # 1. Physics & Simulation Settings
    dt = 1.0 / 240.0  # 240 Hz PyBullet physics rate
    mass = 1.0        # kg
    # Inertia matching paper Section V: J = diag([0.1, 0.1, 0.12])
    J = np.diag([0.1, 0.1, 0.12])

    # 2. Base Waypoints Selection
    if waypoints is not None and len(waypoints) >= 2:
        planned_waypoints = [np.array(wp, dtype=np.float64) for wp in waypoints]
        p_start = planned_waypoints[0]
        p_goal = planned_waypoints[-1]
    else:
        # Default smooth path navigating through intermediate waypoints
        planned_waypoints = [
            p_start,
            np.array([p_start[0] + 1.5, p_start[1] + 1.0, p_start[2] + 1.3]),
            np.array([0.5 * (p_start[0] + p_goal[0]) - 0.8, 0.5 * (p_start[1] + p_goal[1]) + 1.5, max(p_start[2], p_goal[2]) + 1.2]),
            np.array([p_goal[0] - 1.2, p_goal[1] + 0.6, p_goal[2] + 0.5]),
            p_goal
        ]

    # 3. Initialize PyBullet Environment & Drone
    env = UAVPyBulletEnv(gui=gui, time_step=dt, mass=mass, inertia=J)
    initial_q = np.array([1.0, 0.0, 0.0, 0.0])  # Upright scalar-first
    env.build_drone(initial_pos=p_start, initial_orn=initial_q)

    # Spawn obstacles directly in the flight path between waypoints
    obstacle_specs = []
    if add_obstacles:
        obstacle_specs = env.add_default_obstacles(planned_waypoints)
        print(f"[OBSTACLE AVOIDANCE] Placed {len(obstacle_specs)} physical obstacles directly blocking nominal path.")

    # 4. Trajectory Planning
    if reactive_lidar:
        # PURE SENSOR-BASED REACTIVE NAVIGATION:
        # Zero prior map information! Drone follows nominal path and actively uses LiDAR to dodge obstacles!
        traj_gen = MultiWaypointTrajectory3D(waypoints=planned_waypoints, total_duration=flight_duration)
        flight_waypoints = planned_waypoints
        trajectory_type = "reactive_lidar"
        print("[LIDAR PERCEPTION] Operating in 100% blind mode — zero prior obstacle knowledge! Relying purely on onboard LiDAR.")
    elif add_obstacles and obstacle_specs:
        traj_gen = ObstacleAvoidanceTrajectory3D(
            waypoints=planned_waypoints,
            obstacles=obstacle_specs,
            safety_radius=0.75,
            total_duration=flight_duration
        )
        trajectory_type = "obstacle_avoidance"
        # Update planned waypoints to include the detour navigation waypoints
        flight_waypoints = traj_gen.waypoints
        print(f"[OBSTACLE AVOIDANCE] Successfully synthesized detour trajectory with {len(flight_waypoints)} waypoints.")
    elif trajectory_type == "curved":
        traj_gen = CurvedMissionTrajectory3D(p_start=p_start,
                                             p_goal=p_goal,
                                             duration=flight_duration,
                                             arch_height=arch_height,
                                             lateral_curve=lateral_curve)
        flight_waypoints = [p_start, traj_gen.p_mid, p_goal]
    elif trajectory_type == "quintic":
        traj_gen = QuinticTrajectory3D(p_start=p_start, p_goal=p_goal, duration=flight_duration)
        flight_waypoints = [p_start, p_goal]
    else:
        traj_gen = MultiWaypointTrajectory3D(waypoints=planned_waypoints, total_duration=flight_duration)
        flight_waypoints = planned_waypoints

    ref_waypoints = traj_gen.generate_waypoints(num_points=300)

    # 5. Initialize Controllers
    # Outer-loop position controller (SE(3) flatness)
    pos_controller = UAVPositionController(mass=mass, gravity=9.81)

    # Inner-loop attitude controller (Paper Eq. 10 - 23, k1=20, k2=2)
    att_controller = QuaternionBacksteppingController(J=J, k1=20.0, k2=2.0)

    # Create interactive 360° camera controller (touchpad/mouse driven)
    if gui:
        mid = 0.5 * (p_start + p_goal)
        cam = CameraController(target_pos=mid, distance=6.5, yaw=45.0, pitch=-30.0)
    else:
        cam = None

    # Draw all 3D waypoints and the reference flight path
    env.draw_all_waypoints(flight_waypoints)
    env.draw_reference_trajectory(ref_waypoints, color=[0.1, 0.9, 0.2], width=2.5)
    planned_waypoints = flight_waypoints

    # 5. Telemetry Data Logging
    log = {
        'time': [],
        'pos': [],
        'pos_ref': [],
        'vel': [],
        'vel_ref': [],
        'euler_deg': [],
        'euler_ref_deg': [],
        'q': [],
        'q_ref': [],
        'q_err': [],
        'omega': [],
        'omega_ref': [],
        'omega_err': [],
        'omega_err_d': [],
        'delta': [],
        'tau': [],
        'thrust': []
    }

    # 6. Main Simulation Loop
    num_steps = int(total_sim_time / dt)
    print("\nStarting flight simulation... Press Ctrl+C in terminal to abort.")
    if manual_cam:
        print("[INFO] Manual camera mode: use mouse inside PyBullet window to rotate/zoom.")
    if noise_std > 0:
        print(f"[INFO] Sensor noise enabled: sigma = {noise_std:.4f} m (Gaussian)")

    # Open the live tracking panel (rectangular dashboard on the left side of screen)
    tracking_panel = LiveTrackingPanel(title="UAV Live Tracking Dashboard") if gui else None
    if tracking_panel:
        print("\n[READY] Click 'START SIMULATION' on the dashboard when ready to fly!")
        # Define idle callback so user can inspect and rotate camera in 360° before launching
        def _preflight_idle():
            if cam is not None:
                cam.update(follow_pos=p_start, smooth=0.08)
            env.step()
        tracking_panel.wait_for_start(on_idle_callback=_preflight_idle)
        if tracking_panel.exit_requested:
            print("[INFO] User clicked EXIT before takeoff. Closing.")
            env.close()
            return

    start_wall_time = time.time()
    dynamic_obs_spawned = False
    spawn_time = 0.35 * flight_duration  # Obstacle suddenly appears at 35% of mission time

    for step in range(num_steps):
        t = step * dt

        # Dynamically spawn an obstacle directly in the drone's forward line of sight!
        if dynamic_obstacle and not dynamic_obs_spawned and t >= spawn_time:
            drone_curr = env.get_state()
            forward_pred = traj_gen.evaluate(min(t + 1.2, flight_duration))['pos']
            pop_obs = env.spawn_dynamic_obstacle(
                position=forward_pred,
                tower_height=max(forward_pred[2] + 0.5, 2.0),
                half_radius=0.28,
                label="! OBSTACLE APPEARED !"
            )
            print(f"\n[ALERT] SUDDEN OBSTACLE DETECTED AT t={t:.2f}s IN FRONT OF UAV: {forward_pred.round(2).tolist()}")
            if tracking_panel:
                tracking_panel.set_status("AVOIDING UNEXPECTED OBSTACLE!", color="#ff2200", bg="#330000")

            # Dynamic real-time escape trajectory replanning!
            rem_dur = max(flight_duration - t, 3.5)
            # Retain any upcoming waypoints ahead of the drone, ending with p_goal
            cur_idx = traj_gen.get_current_waypoint_index(t) if hasattr(traj_gen, 'get_current_waypoint_index') else 1
            rem_waypoints = [wp for wp in planned_waypoints[cur_idx:] if np.linalg.norm(wp - drone_curr['pos']) > 0.8]
            if len(rem_waypoints) == 0 or np.linalg.norm(rem_waypoints[-1] - p_goal) > 0.1:
                rem_waypoints.append(p_goal)

            traj_gen = ObstacleAvoidanceTrajectory3D.replan_from_current_state(
                current_pos=drone_curr['pos'],
                current_vel=drone_curr['vel'],
                target_waypoints=rem_waypoints,
                new_obstacle=pop_obs,
                remaining_duration=rem_dur,
                safety_radius=0.75
            )
            # Re-draw updated evasive route in PyBullet
            new_ref = traj_gen.generate_waypoints(num_points=200)
            env.draw_reference_trajectory(new_ref, color=[1.0, 0.4, 0.0], width=3.0)
            planned_waypoints = traj_gen.waypoints
            dynamic_obs_spawned = True

        # Evaluate reference trajectory
        eval_t = (t - spawn_time) if (dynamic_obs_spawned and t >= spawn_time) else t
        ref_state = traj_gen.evaluate(eval_t)
        pos_d = ref_state['pos']
        vel_d = ref_state['vel']
        acc_d = ref_state['acc']
        yaw_d = ref_state['yaw']

        # Get current state from PyBullet
        drone_state = env.get_state()
        # Apply Gaussian sensor noise to position measurement (simulates IMU/GPS noise)
        noise = np.random.normal(0.0, noise_std, 3) if noise_std > 0 else np.zeros(3)
        curr_pos = drone_state['pos'] + noise
        curr_vel = drone_state['vel']
        curr_q = drone_state['q']
        curr_omega = drone_state['omega']
        curr_euler = drone_state['euler_deg']

        # Active Onboard LiDAR Perception & Reactive Repulsive Force (APF)
        if not hasattr(env, '_smoothed_f_rep'):
            env._smoothed_f_rep = np.zeros(3)

        f_rep_instant = np.zeros(3)
        if reactive_lidar:
            # Drone scans space ahead using its onboard forward LiDAR
            hits = env.scan_lidar(drone_pos=drone_state['pos'], drone_orn=curr_q, max_range=2.5, num_rays=15, fov_deg=100.0)
            if hits:
                # Obstacle detected ahead! Compute repulsive force away from hit points
                min_dist = min(h['dist'] for h in hits)
                closest_hit = min(hits, key=lambda h: h['dist'])
                k_rep = 8.0
                # Repulsion directed purely horizontally and slightly upwards (never downward!)
                dir_xy = -closest_hit['dir'][:2]
                norm_xy = np.linalg.norm(dir_xy)
                if norm_xy > 1e-3:
                    dir_xy = dir_xy / norm_xy
                    # Perpendicular evasive vector
                    perp_xy = np.array([-dir_xy[1], dir_xy[0]])
                    evade_dir = 0.5 * dir_xy + 0.85 * perp_xy
                    evade_dir = evade_dir / np.linalg.norm(evade_dir)
                    rep_dir_3d = np.array([evade_dir[0], evade_dir[1], 0.25])
                else:
                    rep_dir_3d = np.array([0.0, 1.0, 0.25])

                rep_mag = k_rep * max(0.0, 2.0 - min_dist) / (min_dist + 0.2)
                f_rep_instant = rep_dir_3d * min(rep_mag, 6.0)

                if tracking_panel and step % 6 == 0:
                    tracking_panel.set_status(f"LIDAR DETECTED OBSTACLE AT {min_dist:.2f}m! EVADING", color="#ff5500", bg="#331100")

        # Smooth repulsive force with low-pass filter (alpha=0.15) to prevent sudden attitude jerks
        alpha = 0.15
        env._smoothed_f_rep = (1.0 - alpha) * env._smoothed_f_rep + alpha * f_rep_instant
        f_repulsion = env._smoothed_f_rep.copy()

        # Outer-loop: Compute required collective thrust and target attitude (q_ref, omega_ref)
        pos_res = pos_controller.compute_position_control(
            pos=curr_pos,
            vel=curr_vel,
            q_current=curr_q,
            target_pos=pos_d,
            target_vel=vel_d,
            target_acc=acc_d,
            target_yaw=yaw_d,
            dt=dt,
            obstacle_repulsion=f_repulsion
        )
        thrust = pos_res['thrust']
        q_ref = pos_res['q_ref']
        omega_ref = pos_res['omega_ref']
        d_omega_ref = pos_res['d_omega_ref']

        # Inner-loop: Compute torques using CACS 2024 Quaternion Backstepping Controller
        att_res = att_controller.compute_control(
            q=curr_q,
            omega=curr_omega,
            q_ref=q_ref,
            omega_ref=omega_ref,
            d_omega_ref=d_omega_ref,
            dt=dt
        )
        tau = att_res['tau']

        # Apply controls to PyBullet physics
        env.apply_control(thrust=thrust, torque=tau)
        env.step()

        # Update real-time 3D flight trail, camera, and live HUD on the left side
        if step % 3 == 0:
            env.update_flight_trail(drone_state['pos'])  # use true pos for trail

            # 360° interactive camera — reads touchpad/mouse events, smooth-follows drone
            if cam is not None:
                if manual_cam:
                    # user-controlled: cam reads events, target softly follows drone
                    cam.update(follow_pos=drone_state['pos'], smooth=0.05)
                else:
                    # auto-rotate mode: override yaw with time-based rotation
                    cam.yaw = 45.0 + t * 3.5
                    cam.update(follow_pos=drone_state['pos'], smooth=0.08)

            cur_wp = traj_gen.get_current_waypoint_index(t) if hasattr(traj_gen, 'get_current_waypoint_index') else 1
            tot_wps = len(planned_waypoints) - 1
            pos_err_mag = float(np.linalg.norm(curr_pos - pos_d))

            # Update the rectangular live tracking dashboard (tkinter panel on left)
            if tracking_panel is not None:
                if tracking_panel.exit_requested:
                    break          # User clicked EXIT during flight
                tracking_panel.update(
                    t=t,
                    current_pos=curr_pos,
                    target_pos=pos_d,
                    pos_error=pos_err_mag,
                    q_err=att_res['q_err'],
                    euler_deg=curr_euler,
                    thrust=thrust,
                    tau=tau,
                    current_wp_idx=cur_wp,
                    total_wps=tot_wps,
                    noise_std=noise_std
                )



        if step % 20 == 0:
            cur_wp = traj_gen.get_current_waypoint_index(t) if hasattr(traj_gen, 'get_current_waypoint_index') else 1
            tot_wps = len(planned_waypoints) - 1
            pos_err_mag = float(np.linalg.norm(curr_pos - pos_d))
            print(f"\r[LIVE TRACKING] t={t:5.2f}s | Target WP: {cur_wp}/{tot_wps} | Err={pos_err_mag:5.3f}m | Pos=[{curr_pos[0]:5.2f}, {curr_pos[1]:5.2f}, {curr_pos[2]:5.2f}] | Thrust={thrust:5.2f}N", end="", flush=True)

        # Telemetry logging (subsampled to ~120 Hz)
        if step % 2 == 0:
            euler_ref = QuaternionMath.to_euler_deg(q_ref)
            log['time'].append(t)
            log['pos'].append(curr_pos.copy())
            log['pos_ref'].append(pos_d.copy())
            log['vel'].append(curr_vel.copy())
            log['vel_ref'].append(vel_d.copy())
            log['euler_deg'].append(curr_euler.copy())
            log['euler_ref_deg'].append(euler_ref.copy())
            log['q'].append(curr_q.copy())
            log['q_ref'].append(q_ref.copy())
            log['q_err'].append(att_res['q_err'].copy())
            log['omega'].append(curr_omega.copy())
            log['omega_ref'].append(omega_ref.copy())
            log['omega_err'].append(att_res['omega_err'].copy())
            log['omega_err_d'].append(att_res['omega_err_d'].copy())
            log['delta'].append(att_res['delta'].copy())
            log['tau'].append(tau.copy())
            log['thrust'].append(thrust)

        # Real-time synchronization for GUI
        if gui and (step % 4 == 0):
            elapsed = time.time() - start_wall_time
            expected = t
            if expected > elapsed:
                time.sleep(expected - elapsed)

    print("\nMission completed successfully!")
    final_error = np.linalg.norm(curr_pos - p_goal)
    print(f"Final Position Error at Destination: {final_error:.4f} m")

    # ── Hold simulation open until user clicks EXIT ───────────────────────────
    if gui and tracking_panel is not None and not tracking_panel.exit_requested:
        tracking_panel.set_status("MISSION COMPLETE  |  Click EXIT to close",
                                  color="#ffee00", bg="#222200")
        hover_thrust = mass * 9.81
        print("\n[HOLD] Simulation window stays open. Click EXIT in the panel to close.")
        while not tracking_panel.exit_requested:
            try:
                # Keep drone hovering at final position
                env.apply_control(thrust=hover_thrust, torque=np.zeros(3))
                env.step()
                # Keep camera interactive during hold
                if cam is not None:
                    cam.update()
                tracking_panel.root.update()
            except Exception:
                break


    env.close()
    if tracking_panel is not None:
        tracking_panel.close()

    # Convert logged data to numpy arrays
    for k in log:
        log[k] = np.array(log[k])

    # Export coordinates to CSV
    if export_csv:
        csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "flight_coordinates.csv")
        data_matrix = np.column_stack([
            log['time'],
            log['pos_ref'][:, 0], log['pos_ref'][:, 1], log['pos_ref'][:, 2],
            log['pos'][:, 0], log['pos'][:, 1], log['pos'][:, 2],
            log['pos'][:, 0] - log['pos_ref'][:, 0],
            log['pos'][:, 1] - log['pos_ref'][:, 1],
            log['pos'][:, 2] - log['pos_ref'][:, 2],
            np.linalg.norm(log['pos'] - log['pos_ref'], axis=1)
        ])
        header = "time_sec,x_ref,y_ref,z_ref,x_actual,y_actual,z_actual,error_x,error_y,error_z,total_error"
        np.savetxt(csv_path, data_matrix, delimiter=",", header=header, comments="", fmt="%.5f")
        print(f"Exported all flight coordinates to CSV: {csv_path}")

        # Print sample coordinates table
        print("\n" + "=" * 80)
        print(f" SAMPLE TRAVELLED 3D COORDINATES ({trajectory_type.upper()} PATH)")
        print("=" * 80)
        print(f"{'Time (s)':<10} | {'Target Pos [X, Y, Z] (m)':<28} | {'Actual Flown Pos [X, Y, Z] (m)':<28} | {'Error (m)':<10}")
        print("-" * 80)
        sample_indices = np.linspace(0, len(log['time']) - 1, 11, dtype=int)
        for idx in sample_indices:
            t_val = log['time'][idx]
            pref = log['pos_ref'][idx]
            pact = log['pos'][idx]
            err = np.linalg.norm(pact - pref)
            ref_str = f"[{pref[0]:5.2f}, {pref[1]:5.2f}, {pref[2]:5.2f}]"
            act_str = f"[{pact[0]:5.2f}, {pact[1]:5.2f}, {pact[2]:5.2f}]"
            print(f"{t_val:7.2f} s  | {ref_str:<28} | {act_str:<28} | {err:7.4f} m")
        print("=" * 80 + "\n")

    # 7. Generate Output Plots
    if save_plots:
        plot_mission_results(log, planned_waypoints=planned_waypoints)

    return log


def plot_mission_results(log, output_dir=None, planned_waypoints=None):
    """
    Generates two comprehensive figure sets:
    1. 'uav_3d_flight_trajectory.png': 3D spatial flight path and XYZ tracking.
    2. 'uav_paper_metrics_reproduction.png': Exact reproduction of Figures 3-12
       from the CACS 2024 paper under full 3D flight conditions!
    """
    if output_dir is None:
        output_dir = os.path.dirname(os.path.abspath(__file__))

    t = log['time']
    pos = log['pos']
    pos_ref = log['pos_ref']
    q_err = log['q_err']
    euler = log['euler_deg']
    euler_ref = log['euler_ref_deg']
    omega = log['omega']
    omega_ref = log['omega_ref']
    omega_err = log['omega_err']
    delta = log['delta']
    tau = log['tau']

    # -------------------------------------------------------------
    # FIGURE 1: 3D Trajectory & Position Tracking
    # -------------------------------------------------------------
    fig = plt.figure(figsize=(14, 6))

    # 3D Path
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    ax1.plot(pos_ref[:, 0], pos_ref[:, 1], pos_ref[:, 2], 'g--', linewidth=2.2, label='Reference Trajectory')
    ax1.plot(pos[:, 0], pos[:, 1], pos[:, 2], 'b-', linewidth=2.0, label='Actual UAV Flight')
    ax1.scatter(pos_ref[0, 0], pos_ref[0, 1], pos_ref[0, 2], color='green', s=90, marker='o', label='Start')
    ax1.scatter(pos_ref[-1, 0], pos_ref[-1, 1], pos_ref[-1, 2], color='red', s=110, marker='*', label='Destination')

    if planned_waypoints is not None and len(planned_waypoints) > 2:
        wp_arr = np.array(planned_waypoints)
        ax1.scatter(wp_arr[1:-1, 0], wp_arr[1:-1, 1], wp_arr[1:-1, 2], color='orange', s=85, marker='D', label='Intermediate Waypoints')
        for i in range(1, len(planned_waypoints) - 1):
            ax1.text(wp_arr[i, 0], wp_arr[i, 1], wp_arr[i, 2] + 0.18, f"WP{i}", color='darkorange', fontsize=9, fontweight='bold')

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D UAV Navigation through Waypoints')
    ax1.legend(loc='best')
    ax1.grid(True)

    # Position Error vs Time
    ax2 = fig.add_subplot(1, 2, 2)
    pos_err = pos_ref - pos
    ax2.plot(t, pos_err[:, 0], 'r-', label='e_x (m)')
    ax2.plot(t, pos_err[:, 1], 'g-', label='e_y (m)')
    ax2.plot(t, pos_err[:, 2], 'b-', label='e_z (m)')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Tracking Error (m)')
    ax2.set_title('Position Tracking Errors (X, Y, Z)')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    fig1_path = os.path.join(output_dir, "uav_3d_flight_trajectory.png")
    plt.savefig(fig1_path, dpi=200)
    plt.close()
    print(f"Saved 3D flight trajectory plot: {fig1_path}")

    # -------------------------------------------------------------
    # FIGURE 2: Paper Metrics Reproduction (Matching Figs 3-12)
    # -------------------------------------------------------------
    fig, axes = plt.subplots(5, 2, figsize=(16, 18))
    fig.suptitle("Paper Metrics Reproduction: Quaternion Attitude Backstepping)", fontsize=15)

    # Subplot (0,0): Fig. 3 - Trajectory of quaternion error
    axes[0, 0].plot(t, q_err[:, 0], label=r'$q_{err0}$', color='tab:blue')
    axes[0, 0].plot(t, q_err[:, 1], label=r'$q_{err1}$', color='tab:orange')
    axes[0, 0].plot(t, q_err[:, 2], label=r'$q_{err2}$', color='tab:green')
    axes[0, 0].plot(t, q_err[:, 3], label=r'$q_{err3}$', color='tab:purple')
    axes[0, 0].set_title('Fig. 3. Trajectory of quaternion error')
    axes[0, 0].set_xlabel('Time (sec)')
    axes[0, 0].set_ylabel('Quaternion Error')
    axes[0, 0].grid(True)
    axes[0, 0].legend()

    # Subplot (0,1): Fig. 7 - Trajectory of angular velocity error
    axes[0, 1].plot(t, omega_err[:, 0], label=r'$\omega_{err1}$', color='tab:blue')
    axes[0, 1].plot(t, omega_err[:, 1], label=r'$\omega_{err2}$', color='tab:orange')
    axes[0, 1].plot(t, omega_err[:, 2], label=r'$\omega_{err3}$', color='tab:green')
    axes[0, 1].set_title('Fig. 7. Trajectory of angular velocity error')
    axes[0, 1].set_xlabel('Time (seconds)')
    axes[0, 1].set_ylabel(r'$\omega_{err}$ (rad/sec)')
    axes[0, 1].grid(True)
    axes[0, 1].legend()

    # Subplot (1,0): Fig. 4 - Trajectory of yaw angle tracking
    axes[1, 0].plot(t, euler[:, 0], label='actual', color='tab:blue')
    axes[1, 0].plot(t, euler_ref[:, 0], label='reference', linestyle='--', color='tab:orange')
    axes[1, 0].set_title('Fig. 4. Trajectory of yaw angle tracking')
    axes[1, 0].set_xlabel('Time (sec)')
    axes[1, 0].set_ylabel('Yaw angle (degree)')
    axes[1, 0].grid(True)
    axes[1, 0].legend()

    # Subplot (1,1): Fig. 8 - Trajectory of roll angular velocity tracking
    axes[1, 1].plot(t, omega[:, 0], label='actual', color='tab:blue')
    axes[1, 1].plot(t, omega_ref[:, 0], label='reference', linestyle='--', color='tab:orange')
    axes[1, 1].set_title('Fig. 8. Trajectory of roll angular velocity tracking')
    axes[1, 1].set_xlabel('Time (seconds)')
    axes[1, 1].set_ylabel(r'$\omega_1$ (rad/sec)')
    axes[1, 1].grid(True)
    axes[1, 1].legend()

    # Subplot (2,0): Fig. 5 - Trajectory of pitch angle tracking
    axes[2, 0].plot(t, euler[:, 1], label='actual', color='tab:blue')
    axes[2, 0].plot(t, euler_ref[:, 1], label='reference', linestyle='--', color='tab:orange')
    axes[2, 0].set_title('Fig. 5. Trajectory of pitch angle tracking')
    axes[2, 0].set_xlabel('Time (sec)')
    axes[2, 0].set_ylabel('Pitch angle (degree)')
    axes[2, 0].grid(True)
    axes[2, 0].legend()

    # Subplot (2,1): Fig. 9 - Trajectory of pitch angular velocity tracking
    axes[2, 1].plot(t, omega[:, 1], label='actual', color='tab:blue')
    axes[2, 1].plot(t, omega_ref[:, 1], label='reference', linestyle='--', color='tab:orange')
    axes[2, 1].set_title('Fig. 9. Trajectory of pitch angular velocity tracking')
    axes[2, 1].set_xlabel('Time (seconds)')
    axes[2, 1].set_ylabel(r'$\omega_2$ (rad/sec)')
    axes[2, 1].grid(True)
    axes[2, 1].legend()

    # Subplot (3,0): Fig. 6 - Trajectory of roll angle tracking
    axes[3, 0].plot(t, euler[:, 2], label='actual', color='tab:blue')
    axes[3, 0].plot(t, euler_ref[:, 2], label='reference', linestyle='--', color='tab:orange')
    axes[3, 0].set_title('Fig. 6. Trajectory of roll angle tracking')
    axes[3, 0].set_xlabel('Time (sec)')
    axes[3, 0].set_ylabel('Roll angle (degree)')
    axes[3, 0].grid(True)
    axes[3, 0].legend()

    # Subplot (3,1): Fig. 10 - Trajectory of yaw angular velocity tracking
    axes[3, 1].plot(t, omega[:, 2], label='actual', color='tab:blue')
    axes[3, 1].plot(t, omega_ref[:, 2], label='reference', linestyle='--', color='tab:orange')
    axes[3, 1].set_title('Fig. 10. Trajectory of yaw angular velocity tracking')
    axes[3, 1].set_xlabel('Time (seconds)')
    axes[3, 1].set_ylabel(r'$\omega_3$ (rad/sec)')
    axes[3, 1].grid(True)
    axes[3, 1].legend()

    # Subplot (4,0): Fig. 11 - Trajectory of delta
    axes[4, 0].plot(t, delta[:, 0], label=r'$\delta_1$', color='tab:blue')
    axes[4, 0].plot(t, delta[:, 1], label=r'$\delta_2$', color='tab:orange')
    axes[4, 0].plot(t, delta[:, 2], label=r'$\delta_3$', color='tab:green')
    axes[4, 0].set_title(r'Fig. 11. Trajectory of $\delta$')
    axes[4, 0].set_xlabel('Time (sec)')
    axes[4, 0].set_ylabel(r'$\delta$ (rad/sec)')
    axes[4, 0].grid(True)
    axes[4, 0].legend()

    # Subplot (4,1): Fig. 12 - Trajectory of torque
    axes[4, 1].plot(t, tau[:, 0], label=r'$\tau_1$', color='tab:blue')
    axes[4, 1].plot(t, tau[:, 1], label=r'$\tau_2$', color='tab:orange')
    axes[4, 1].plot(t, tau[:, 2], label=r'$\tau_3$', color='tab:green')
    axes[4, 1].set_title(r'Fig. 12. Trajectory of torque')
    axes[4, 1].set_xlabel('Time (sec)')
    axes[4, 1].set_ylabel(r'$\tau$ (N$\cdot$m)')
    axes[4, 1].grid(True)
    axes[4, 1].legend()

    plt.tight_layout()
    fig2_path = os.path.join(output_dir, "uav_paper_metrics_reproduction.png")
    plt.savefig(fig2_path, dpi=200)
    plt.close()
    print(f"Saved paper metrics reproduction plot: {fig2_path}")


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        description="3D UAV Trajectory Tracking — CACS 2024 Quaternion Backstepping Controller",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("--start", nargs=3, type=float, default=[0.0, 0.0, 0.5],
                        metavar=("X", "Y", "Z"),
                        help="Start position in meters (default: 0.0 0.0 0.5)")
    parser.add_argument("--goal", nargs=3, type=float, default=[4.0, 4.0, 2.5],
                        metavar=("X", "Y", "Z"),
                        help="Goal position in meters (default: 4.0 4.0 2.5)")
    parser.add_argument("--wp", nargs="+", type=float, default=None,
                        metavar="COORD",
                        help=(
                            "Intermediate waypoints only (X Y Z triplets).\n"
                            "Start and Goal are added automatically from --start and --goal.\n"
                            "Example: --start 0 0 0.5 --wp 1.5 1.2 1.8  2.5 3.2 3.0  1.2 4.2 2.6 --goal 4.0 4.0 2.5"
                        ))
    parser.add_argument("--waypoints", nargs="+", type=float, default=None,
                        metavar="COORD",
                        help=(
                            "Full waypoint list including start & goal (X Y Z triplets).\n"
                            "Overrides --start, --wp, --goal.\n"
                            "Example: --waypoints 0 0 0.5  2 1 2  4 3 3  6 6 2.5"
                        ))
    parser.add_argument("--duration", type=float, default=14.0,
                        help="Flight duration in seconds (default: 14.0)")
    parser.add_argument("--sim_time", type=float, default=17.0,
                        help="Total simulation time in seconds (default: 17.0)")
    parser.add_argument("--straight", action="store_true",
                        help="Fly a single straight quintic segment start→goal")
    parser.add_argument("--curved", action="store_true",
                        help="Fly a 3D curved Bezier arch start→goal")
    parser.add_argument("--arch", type=float, default=1.5,
                        help="Arch height for --curved mode (default: 1.5 m)")
    parser.add_argument("--lateral", type=float, default=1.5,
                        help="Lateral curve for --curved mode (default: 1.5 m)")
    parser.add_argument("--no_gui", action="store_true",
                        help="Run headless (no PyBullet window)")
    parser.add_argument("--auto_cam", action="store_true",
                        help="Enable auto-rotating camera that follows drone (default: free mouse control)")
    parser.add_argument("--noise", type=float, default=0.0,
                        metavar="SIGMA",
                        help="Add Gaussian sensor noise to position (std in meters, e.g. 0.02)")
    parser.add_argument("--obstacles", action="store_true",
                        help="Spawn static red box obstacles between waypoints")
    parser.add_argument("--popup_obstacle", action="store_true",
                        help="Simulate unexpected obstacle popping up in front of the drone during flight, triggering real-time evasion")
    parser.add_argument("--lidar", action="store_true",
                        help="Pure sensor-based reactive navigation: drone has zero prior knowledge and uses onboard forward LiDAR rays to detect & dodge obstacles")
    args = parser.parse_args()

    # ── Build waypoint sequence ───────────────────────────────────────────────
    if args.waypoints is not None:
        # --waypoints: full list including start & goal
        coords = args.waypoints
        if len(coords) % 3 != 0 or len(coords) < 6:
            print("ERROR: --waypoints needs at least 2 waypoints (6 numbers).")
            raise SystemExit(1)
        WAYPOINTS_SEQUENCE = [
            [coords[i], coords[i+1], coords[i+2]]
            for i in range(0, len(coords), 3)
        ]
    elif args.wp is not None:
        # --wp: intermediate only; prepend start, append goal
        coords = args.wp
        if len(coords) % 3 != 0:
            print("ERROR: --wp needs X Y Z triplets (multiple of 3 numbers).")
            raise SystemExit(1)
        middle = [
            [coords[i], coords[i+1], coords[i+2]]
            for i in range(0, len(coords), 3)
        ]
        WAYPOINTS_SEQUENCE = [list(args.start)] + middle + [list(args.goal)]
    else:
        # Default built-in 5-point sequence
        WAYPOINTS_SEQUENCE = [
            list(args.start),           # WP0: Start
            [1.5, 1.2, 1.8],            # WP1: Ascend & initial turn
            [2.5, 3.2, 3.0],            # WP2: Apex curve & lateral swing
            [1.2, 4.2, 2.6],            # WP3: Corridor maneuver
            list(args.goal),            # WP4: Goal
        ]

    print(f"[CMD] Waypoint sequence ({len(WAYPOINTS_SEQUENCE)} points):")
    for idx, wp in enumerate(WAYPOINTS_SEQUENCE):
        tag = "Start" if idx == 0 else ("Goal" if idx == len(WAYPOINTS_SEQUENCE)-1 else f"WP{idx}")
        print(f"  {tag}: {wp}")

    # ── Trajectory mode ───────────────────────────────────────────────────────
    if args.straight:
        traj_mode = "quintic"
        wps = None
    elif args.curved:
        traj_mode = "curved"
        wps = None
    else:
        traj_mode = "multi_waypoint"
        wps = WAYPOINTS_SEQUENCE

    # ── Run simulation ────────────────────────────────────────────────────────
    run_3d_mission(
        p_start=np.array(args.start, dtype=np.float64),
        p_goal=np.array(args.goal, dtype=np.float64),
        waypoints=wps,
        flight_duration=args.duration,
        total_sim_time=args.sim_time,
        trajectory_type=traj_mode,
        arch_height=args.arch,
        lateral_curve=args.lateral,
        gui=not args.no_gui,
        save_plots=True,
        export_csv=True,
        noise_std=args.noise,
        add_obstacles=args.obstacles,
        dynamic_obstacle=args.popup_obstacle,
        reactive_lidar=args.lidar,
        manual_cam=not args.auto_cam,   # default: free mouse; --auto_cam enables rotating camera
    )


