"""
Main 3D UAV Mission Execution Script (PyBullet + Controller)
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
from uav_trajectory import QuinticTrajectory3D, CurvedMissionTrajectory3D
from uav_position_controller import UAVPositionController
from uav_pybullet_env import UAVPyBulletEnv


def run_3d_mission(p_start=np.array([0.0, 0.0, 0.5]),
                   p_goal=np.array([4.0, 4.0, 2.5]),
                   flight_duration=10.0,
                   total_sim_time=13.0,
                   trajectory_type="quintic",
                   arch_height=1.5,
                   lateral_curve=1.5,
                   gui=True,
                   save_plots=True,
                   export_csv=True):
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
    print(" Based on: , Equations (1) - (23)")
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

    # 2. Initialize Trajectory Generator
    if trajectory_type == "curved":
        traj_gen = CurvedMissionTrajectory3D(p_start=p_start,
                                             p_goal=p_goal,
                                             duration=flight_duration,
                                             arch_height=arch_height,
                                             lateral_curve=lateral_curve)
    else:
        traj_gen = QuinticTrajectory3D(p_start=p_start, p_goal=p_goal, duration=flight_duration)

    ref_waypoints = traj_gen.generate_waypoints(num_points=200)

    # 3. Initialize Controllers
    # Outer-loop position controller (SE(3) flatness)
    pos_controller = UAVPositionController(mass=mass, gravity=9.81)

    # Inner-loop attitude controller (Paper Eq. 10 - 23, k1=20, k2=2)
    att_controller = QuaternionBacksteppingController(J=J, k1=20.0, k2=2.0)

    # 4. Initialize PyBullet Environment
    env = UAVPyBulletEnv(gui=gui, time_step=dt, mass=mass, inertia=J)
    initial_q = np.array([1.0, 0.0, 0.0, 0.0])  # Upright scalar-first
    env.build_drone(initial_pos=p_start, initial_orn=initial_q)

    # Draw 3D start/goal markers and reference trajectory line in space
    env.draw_markers(start_pos=p_start, goal_pos=p_goal)
    env.draw_reference_trajectory(ref_waypoints, color=[0.1, 0.9, 0.2], width=2.5)

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

    start_wall_time = time.time()
    for step in range(num_steps):
        t = step * dt

        # Evaluate reference trajectory
        ref_state = traj_gen.evaluate(t)
        pos_d = ref_state['pos']
        vel_d = ref_state['vel']
        acc_d = ref_state['acc']
        yaw_d = ref_state['yaw']

        # Get current state from PyBullet
        drone_state = env.get_state()
        curr_pos = drone_state['pos']
        curr_vel = drone_state['vel']
        curr_q = drone_state['q']
        curr_omega = drone_state['omega']
        curr_euler = drone_state['euler_deg']

        # Outer-loop: Compute required collective thrust and target attitude (q_ref, omega_ref)
        pos_res = pos_controller.compute_position_control(
            pos=curr_pos,
            vel=curr_vel,
            q_current=curr_q,
            target_pos=pos_d,
            target_vel=vel_d,
            target_acc=acc_d,
            target_yaw=yaw_d,
            dt=dt
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

        # Update real-time 3D flight trail and camera
        if step % 3 == 0:
            env.update_flight_trail(curr_pos)
            env.update_camera(curr_pos, distance=3.2, pitch=-25.0, yaw=45.0 + t * 4.0)

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

    env.close()

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
        plot_mission_results(log)

    return log


def plot_mission_results(log, output_dir=None):
    """
    Generates two comprehensive figure sets:
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
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D UAV Navigation from Start to Destination')
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
    fig.suptitle("Paper Metrics Reproduction: Quaternion Attitude Backstepping (Lin & Yu CACS 2024)", fontsize=15)

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
    fig2_path = os.path.join(output_dir, "uav    _metrics.png")
    plt.savefig(fig2_path, dpi=200)
    plt.close()
    print(f"Saved paper metrics reproduction plot: {fig2_path}")


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="3D UAV Trajectory Tracking Quaternion Backstepping Controller")
    parser.add_argument("--start", nargs=3, type=float, default=[0.0, 0.0, 0.5],
                        help="Start coordinates [x y z] in meters (default: 0.0 0.0 0.5)")
    parser.add_argument("--goal", nargs=3, type=float, default=[4.0, 4.0, 2.5],
                        help="Destination coordinates [x y z] in meters (default: 4.0 4.0 2.5)")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Flight duration to destination in seconds (default: 10.0)")
    parser.add_argument("--sim_time", type=float, default=13.0,
                        help="Total simulation time including hover in seconds (default: 13.0)")
    parser.add_argument("--curved", action="store_true",
                        help="Use 3D curved trajectory (altitude arch + lateral curve) instead of straight quintic")
    parser.add_argument("--arch", type=float, default=1.5,
                        help="Peak altitude arch height in meters for curved path (default: 1.5)")
    parser.add_argument("--lateral", type=float, default=1.5,
                        help="Lateral horizontal curve offset in meters for curved path (default: 1.5)")
    parser.add_argument("--no_gui", action="store_true", help="Run in headless mode without 3D window")
    args = parser.parse_args()

    # =========================================================================
    # >> CHANGE DESTINATION AND START POINTS HERE (OR PASS VIA CLI --goal X Y Z) <<
    # =========================================================================
    START_POINT = np.array(args.start, dtype=np.float64)
    DESTINATION_POINT = np.array(args.goal, dtype=np.float64)
    traj_mode = "curved" if args.curved else "quintic"

    # Run PyBullet simulation
    run_3d_mission(
        p_start=START_POINT,
        p_goal=DESTINATION_POINT,
        flight_duration=args.duration,
        total_sim_time=args.sim_time,
        trajectory_type=traj_mode,
        arch_height=args.arch,
        lateral_curve=args.lateral,
        gui=not args.no_gui,
        save_plots=True,
        export_csv=True
    )

