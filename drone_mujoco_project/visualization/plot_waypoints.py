import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_all_waypoint_results(data_dict: dict, switch_history: list = None, save_dir: str = "."):
    """
    Generates all 6 analytical verification plots for the multi-waypoint UAV flight:
      1. 3D Flight Trajectory (Start -> WP1 -> WP2 -> WP3 -> Goal)
      2. Position Error e_p(t) vs Time with Waypoint Switch Markers
      3. X, Y, Z Position Tracking Components vs Time
      4. Quaternion Tracking Error (qe0, qe1, qe2, qe3) vs Time
      5. Angular Velocity Error (w_err) vs Time
      6. Control Torques (tau_x, tau_y, tau_z) vs Time
    """
    os.makedirs(save_dir, exist_ok=True)

    t = data_dict['t']
    pos = data_dict['pos']
    target_pos = data_dict['target_pos']
    dist_err = data_dict['dist_err']
    q_err = data_dict['q_err']
    w_err = data_dict['w_err']
    tau = data_dict['tau']
    start_pos = data_dict['start_pos']
    waypoints = data_dict['waypoints']

    # Extract switch timestamps if available
    switch_times = [evt['time'] for evt in switch_history] if switch_history else []
    switch_names = [evt['name'] for evt in switch_history] if switch_history else []

    # =========================================================================
    # PLOT 1: 3D Trajectory (Start -> WP1 -> WP2 -> WP3 -> Goal)
    # =========================================================================
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d')

    # Reference straight lines between waypoints (Planned path)
    ref_points = np.vstack([start_pos, waypoints])
    ax.plot(ref_points[:, 0], ref_points[:, 1], ref_points[:, 2], 
            color='#FF8C00', linestyle='--', linewidth=2.0, alpha=0.9, label='Reference path — planned')

    # Actual UAV Flight Trajectory (Drone movement)
    ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], 
            color='#0066FF', linewidth=2.5, label='Actual path — drone movement')

    # Start Point (Blue)
    ax.scatter(start_pos[0], start_pos[1], start_pos[2], 
               color='blue', s=130, marker='o', label=f'Start: {start_pos.tolist()}')
    ax.text(start_pos[0], start_pos[1], start_pos[2] + 0.15, '  Start Point', color='blue', fontweight='bold')

    # Intermediate Waypoints (Orange / Gold)
    colors = ['#FF8C00', '#E65100', '#FF6D00']
    for i, wp in enumerate(waypoints[:-1]):
        c = colors[i % len(colors)]
        ax.scatter(wp[0], wp[1], wp[2], color=c, s=110, marker='D', label=f'WP{i+1}: {wp.tolist()}')
        ax.text(wp[0], wp[1], wp[2] + 0.15, f'  WP{i+1}', color=c, fontweight='bold')

    # Final Destination Goal (Green)
    goal = waypoints[-1]
    ax.scatter(goal[0], goal[1], goal[2], 
               color='green', s=180, marker='*', label=f'Destination: {goal.tolist()}')
    ax.text(goal[0], goal[1], goal[2] + 0.15, '  Destination', color='green', fontweight='bold')

    ax.set_title("1. Multi-Waypoint 3D Flight Trajectory Tracking", fontsize=13, fontweight='bold')
    ax.set_xlabel("X Position (m)", fontweight='bold')
    ax.set_ylabel("Y Position (m)", fontweight='bold')
    ax.set_zlabel("Z Position (m)", fontweight='bold')
    ax.legend(loc='upper left', fontsize=9)
    plt.tight_layout()
    path1 = os.path.join(save_dir, "waypoint_3d_trajectory.png")
    plt.savefig(path1, dpi=300)
    plt.close()
    print(f" [Plot 1/6] Saved: {path1}")

    # =========================================================================
    # PLOT 2: Position Error e_p(t) vs Time
    # =========================================================================
    plt.figure(figsize=(8, 4.8))
    plt.plot(t, dist_err, color='crimson', linewidth=2.2, label=r'$e_p(t) = ||P_{target}(t) - P(t)||$')
    plt.axhline(0.20, color='gray', linestyle=':', label='Switching Tolerance (0.20m)')
    plt.axhline(0.10, color='green', linestyle='--', label='Final Arrival Tolerance (0.10m)')

    # Add vertical lines for waypoint switch events
    for st, name in zip(switch_times, switch_names):
        plt.axvline(st, color='orange', linestyle='--', alpha=0.7)
        plt.text(st + 0.08, np.max(dist_err) * 0.85, f'{name}\n({st:.1f}s)', 
                 fontsize=8, rotation=90, verticalalignment='top', color='#B24700', fontweight='bold')

    plt.title(r"2. Position Error vs Time $e_p(t) = ||P_{target} - P(t)||$", fontsize=12, fontweight='bold')
    plt.xlabel("Time (s)", fontweight='bold')
    plt.ylabel("Distance to Active Target (m)", fontweight='bold')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc='upper right', fontsize=9)
    plt.tight_layout()
    path2 = os.path.join(save_dir, "waypoint_position_error.png")
    plt.savefig(path2, dpi=300)
    plt.close()
    print(f" [Plot 2/6] Saved: {path2}")

    # =========================================================================
    # PLOT 3: X, Y, Z Position Components vs Time
    # =========================================================================
    fig, axes = plt.subplots(3, 1, figsize=(8.5, 7.5), sharex=True)
    coords = ['X', 'Y', 'Z']
    colors_act = ['#1f77b4', '#ff7f0e', '#2ca02c']

    for i, ax in enumerate(axes):
        ax.plot(t, pos[:, i], color=colors_act[i], linewidth=2.2, label=f'Actual {coords[i]}')
        ax.plot(t, target_pos[:, i], 'k--', linewidth=1.5, alpha=0.8, label=f'Target {coords[i]}')
        for st in switch_times:
            ax.axvline(st, color='gray', linestyle=':', alpha=0.5)
        ax.set_ylabel(f"{coords[i]} (m)", fontweight='bold')
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.legend(loc='upper left', fontsize=9)

    axes[0].set_title("3. Multi-Waypoint Position Tracking (X, Y, Z vs Time)", fontsize=12, fontweight='bold')
    axes[2].set_xlabel("Time (s)", fontweight='bold')
    plt.tight_layout()
    path3 = os.path.join(save_dir, "waypoint_xyz_tracking.png")
    plt.savefig(path3, dpi=300)
    plt.close()
    print(f" [Plot 3/6] Saved: {path3}")

    # =========================================================================
    # PLOT 4: Quaternion Tracking Error vs Time (qe0, qe1, qe2, qe3)
    # =========================================================================
    plt.figure(figsize=(8.5, 4.8))
    plt.plot(t, q_err[:, 0], label=r'$q_{e,0}$ (Scalar $\to 1$)', color='black', linewidth=2.0)
    plt.plot(t, q_err[:, 1], label=r'$q_{e,1}$ (Roll error $\to 0$)', color='C0', linewidth=1.8)
    plt.plot(t, q_err[:, 2], label=r'$q_{e,2}$ (Pitch error $\to 0$)', color='C1', linewidth=1.8)
    plt.plot(t, q_err[:, 3], label=r'$q_{e,3}$ (Yaw error $\to 0$)', color='C2', linewidth=1.8)
    plt.axhline(1.0, color='black', linestyle=':', alpha=0.4)
    plt.axhline(0.0, color='black', linestyle=':', alpha=0.4)

    for st in switch_times:
        plt.axvline(st, color='gray', linestyle=':', alpha=0.5)

    plt.title(r"4. Quaternion Attitude Error $q_{err} = q_{ref}^* \otimes q$", fontsize=12, fontweight='bold')
    plt.xlabel("Time (s)", fontweight='bold')
    plt.ylabel("Quaternion Error Components", fontweight='bold')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc='upper right', fontsize=9)
    plt.tight_layout()
    path4 = os.path.join(save_dir, "waypoint_quaternion_error.png")
    plt.savefig(path4, dpi=300)
    plt.close()
    print(f" [Plot 4/6] Saved: {path4}")

    # =========================================================================
    # PLOT 5: Angular Velocity Error vs Time (w_err)
    # =========================================================================
    plt.figure(figsize=(8.5, 4.8))
    plt.plot(t, w_err[:, 0], label=r'$\omega_{e,x}$ (Roll rate error)', color='C0', linewidth=1.8)
    plt.plot(t, w_err[:, 1], label=r'$\omega_{e,y}$ (Pitch rate error)', color='C1', linewidth=1.8)
    plt.plot(t, w_err[:, 2], label=r'$\omega_{e,z}$ (Yaw rate error)', color='C2', linewidth=1.8)
    plt.axhline(0.0, color='black', linestyle=':', alpha=0.5)

    for st in switch_times:
        plt.axvline(st, color='gray', linestyle=':', alpha=0.5)

    plt.title(r"5. Angular Velocity Error $\omega_{err} = \omega - \omega_{ref}$", fontsize=12, fontweight='bold')
    plt.xlabel("Time (s)", fontweight='bold')
    plt.ylabel("Angular Velocity Error (rad/s)", fontweight='bold')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc='upper right', fontsize=9)
    plt.tight_layout()
    path5 = os.path.join(save_dir, "waypoint_angular_velocity_error.png")
    plt.savefig(path5, dpi=300)
    plt.close()
    print(f" [Plot 5/6] Saved: {path5}")

    # =========================================================================
    # PLOT 6: Control Torques vs Time (tau_x, tau_y, tau_z)
    # =========================================================================
    plt.figure(figsize=(8.5, 4.8))
    plt.plot(t, tau[:, 0], label=r'$\tau_x$ (Roll Control Torque)', color='C0', linewidth=1.8)
    plt.plot(t, tau[:, 1], label=r'$\tau_y$ (Pitch Control Torque)', color='C1', linewidth=1.8)
    plt.plot(t, tau[:, 2], label=r'$\tau_z$ (Yaw Control Torque)', color='C2', linewidth=1.8)
    plt.axhline(0.0, color='black', linestyle=':', alpha=0.5)

    for st in switch_times:
        plt.axvline(st, color='gray', linestyle=':', alpha=0.5)

    plt.title(r"6. Control Torques $\tau$ from Quaternion Backstepping Controller", fontsize=12, fontweight='bold')
    plt.xlabel("Time (s)", fontweight='bold')
    plt.ylabel("Torque (N·m)", fontweight='bold')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc='upper right', fontsize=9)
    plt.tight_layout()
    path6 = os.path.join(save_dir, "waypoint_control_torques.png")
    plt.savefig(path6, dpi=300)
    plt.close()
    print(f" [Plot 6/6] Saved: {path6}")

if __name__ == "__main__":
    results_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "results", "waypoint_simulation_results.npz")
    if os.path.exists(results_path):
        data = np.load(results_path, allow_pickle=True)
        data_dict = {k: data[k] for k in data.files}
        plot_all_waypoint_results(data_dict, save_dir=os.path.dirname(results_path))
    else:
        print(f"Results file {results_path} not found. Run simulate_waypoints.py first.")
