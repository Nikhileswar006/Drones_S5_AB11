import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(BASE_DIR, "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

def plot_base_paper_results(data_dict: dict, save_dir: str = RESULTS_DIR):
    """
    Step 22: Base paper reproduction plots.
    """
    t = data_dict['t']
    q_err = data_dict['q_err']
    w_err = data_dict['w_err']
    euler_deg = np.degrees(data_dict['euler'])
    tau = data_dict['tau']

    # 1. Quaternion Error Plot
    plt.figure(figsize=(7, 4.5))
    plt.plot(t, q_err[:, 0], label='$q_{err0}$', color='C0', linewidth=2)
    plt.plot(t, q_err[:, 1], label='$q_{err1}$', color='C1', linewidth=2)
    plt.plot(t, q_err[:, 2], label='$q_{err2}$', color='C2', linewidth=2)
    plt.plot(t, q_err[:, 3], label='$q_{err3}$', color='C3', linewidth=2)
    plt.title("Step 22: Trajectory of Quaternion Error $q_{err}$", fontsize=12, fontweight='bold')
    plt.xlabel("Time (sec)")
    plt.ylabel("Quaternion Error")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc='best')
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, "step22_quaternion_error.png"), dpi=300)
    plt.close()

    # 2. Angular Velocity Error Plot
    plt.figure(figsize=(7, 4.5))
    plt.plot(t, w_err[:, 0], label='$\\omega_{err1}$', color='C0', linewidth=2)
    plt.plot(t, w_err[:, 1], label='$\\omega_{err2}$', color='C1', linewidth=2)
    plt.plot(t, w_err[:, 2], label='$\\omega_{err3}$', color='C2', linewidth=2)
    plt.title("Step 22: Trajectory of Angular Velocity Error $\\omega_{err}$", fontsize=12, fontweight='bold')
    plt.xlabel("Time (sec)")
    plt.ylabel("Angular Velocity Error (rad/sec)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc='best')
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, "step22_angular_velocity_error.png"), dpi=300)
    plt.close()

    # 3. Roll, Pitch, Yaw Euler Angles Plot
    plt.figure(figsize=(7, 4.5))
    plt.plot(t, euler_deg[:, 0], label='Roll ($\\phi$)', color='C0', linewidth=2)
    plt.plot(t, euler_deg[:, 1], label='Pitch ($\\theta$)', color='C1', linewidth=2)
    plt.plot(t, euler_deg[:, 2], label='Yaw ($\\psi$)', color='C2', linewidth=2)
    plt.axhline(0, color='black', linestyle=':', alpha=0.5)
    plt.title("Step 22: Attitude Angles Stabilization (Roll, Pitch, Yaw)", fontsize=12, fontweight='bold')
    plt.xlabel("Time (sec)")
    plt.ylabel("Attitude Angle (deg)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc='best')
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, "step22_euler_angles.png"), dpi=300)
    plt.close()

    # 4. Control Torques Plot
    plt.figure(figsize=(7, 4.5))
    plt.plot(t, tau[:, 0], label='$\\tau_1$ (Roll Torque)', color='C0', linewidth=2)
    plt.plot(t, tau[:, 1], label='$\\tau_2$ (Pitch Torque)', color='C1', linewidth=2)
    plt.plot(t, tau[:, 2], label='$\\tau_3$ (Yaw Torque)', color='C2', linewidth=2)
    plt.title("Step 22: Control Torques $\\tau$", fontsize=12, fontweight='bold')
    plt.xlabel("Time (sec)")
    plt.ylabel("Torque (N·m)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc='best')
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, "step22_control_torques.png"), dpi=300)
    plt.close()

def plot_extension_results(data_dict: dict, save_dir: str = RESULTS_DIR):
    """
    Steps 36, 37, 38: Extension Destination Navigation Plots.
      - 3D Actual Trajectory
      - Position Error e(t) vs Time
      - X, Y, Z Position vs Time
    """
    t = data_dict['t']
    pos = data_dict['pos']
    target_pos = data_dict['target_pos']
    dist_err = data_dict['dist_err']

    p0 = pos[0]
    pd = target_pos

    # -------------------------------------------------------------
    # Step 36: 3D Actual Flight Trajectory Plot
    # -------------------------------------------------------------
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot actual flight trajectory
    ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], color='navy', linewidth=2.5, label='Actual Flight Path')
    
    # Plot Start and Goal Markers
    ax.scatter(p0[0], p0[1], p0[2], color='blue', s=100, marker='o', label=f'Start ({p0[0]}, {p0[1]}, {p0[2]})')
    ax.scatter(pd[0], pd[1], pd[2], color='green', s=150, marker='*', label=f'Goal ({pd[0]}, {pd[1]}, {pd[2]})')
    
    ax.set_title("Step 36: Actual 3D Flight Trajectory (Un-scripted Error-Driven Path)", fontsize=12, fontweight='bold')
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_zlabel("Z Position (m)")
    ax.legend(loc='best')
    plt.tight_layout()
    path_3d = os.path.join(save_dir, "step36_3d_actual_trajectory.png")
    plt.savefig(path_3d, dpi=300)
    plt.close()
    print(f"[Plot] Saved: {path_3d}")

    # -------------------------------------------------------------
    # Step 37: Distance Error vs Time Plot e(t) = ||Pd - P(t)||
    # -------------------------------------------------------------
    plt.figure(figsize=(7, 4.5))
    plt.plot(t, dist_err, color='crimson', linewidth=2.5, label='$e(t) = ||P_d - P(t)||$')
    plt.axhline(0.1, color='green', linestyle='--', label='Reaching Threshold (0.1m)')
    plt.title("Step 37: Position Error to Destination vs Time", fontsize=12, fontweight='bold')
    plt.xlabel("Time (sec)")
    plt.ylabel("Distance to Destination (m)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc='best')
    plt.tight_layout()
    path_err = os.path.join(save_dir, "step37_position_error_vs_time.png")
    plt.savefig(path_err, dpi=300)
    plt.close()
    print(f"[Plot] Saved: {path_err}")

    # -------------------------------------------------------------
    # Step 38: X, Y, Z Position Components vs Time
    # -------------------------------------------------------------
    plt.figure(figsize=(8, 5))
    plt.plot(t, pos[:, 0], label='X (Actual)', color='C0', linewidth=2)
    plt.axhline(pd[0], color='C0', linestyle='--', alpha=0.7, label=f'X Target ({pd[0]}m)')
    
    plt.plot(t, pos[:, 1], label='Y (Actual)', color='C1', linewidth=2)
    plt.axhline(pd[1], color='C1', linestyle='--', alpha=0.7, label=f'Y Target ({pd[1]}m)')
    
    plt.plot(t, pos[:, 2], label='Z (Actual)', color='C2', linewidth=2)
    plt.axhline(pd[2], color='C2', linestyle='--', alpha=0.7, label=f'Z Target ({pd[2]}m)')
    
    plt.title("Step 38: Position Components (X, Y, Z) vs Time", fontsize=12, fontweight='bold')
    plt.xlabel("Time (sec)")
    plt.ylabel("Position (m)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc='best')
    plt.tight_layout()
    path_xyz = os.path.join(save_dir, "step38_xyz_position_components.png")
    plt.savefig(path_xyz, dpi=300)
    plt.close()
    print(f"[Plot] Saved: {path_xyz}")
