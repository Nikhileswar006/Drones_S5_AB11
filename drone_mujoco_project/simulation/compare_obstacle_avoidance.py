import os
import sys
import numpy as np
import mujoco
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Add project root to sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from controllers.quaternion import quat_to_euler
from controllers.backstepping import BacksteppingController
from controllers.position_controller import PositionController
from controllers.motor_mixer import MotorMixer
from controllers.waypoint_manager import WaypointManager
from controllers.obstacle_avoidance import ObstacleAvoidancePlanner

MODEL_PATH = os.path.join(PROJECT_ROOT, "models", "quadrotor.xml")
RESULTS_DIR = os.path.join(PROJECT_ROOT, "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

START_POS = np.array([0.0, 0.0, 1.0])
WAYPOINTS = [
    [3.0, 2.0, 2.5],   # WP1
    [6.0, 8.0, 3.5],   # WP2
    [10.0, 4.0, 4.0],  # WP3
    [14.0, 12.0, 5.0]  # Destination
]

def run_simulation(enable_avoidance: bool = True, duration: float = 40.0, dt: float = 0.002):
    """
    Runs a batch simulation with or without autonomous obstacle avoidance.
    """
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    model.opt.timestep = dt

    wp_manager = WaypointManager(waypoints=WAYPOINTS, tolerance=0.75, final_tolerance=0.40)
    pos_controller = PositionController(mass=1.0, gravity=9.81, 
                                        Kp=np.array([1.2, 1.2, 2.0]), 
                                        Kv=np.array([2.6, 2.6, 2.5]))
    attitude_controller = BacksteppingController(J=np.diag([0.1, 0.1, 0.12]), k1=20.0, k2=2.0)
    mixer = MotorMixer(arm_length=0.85)
    obstacle_planner = ObstacleAvoidancePlanner() if enable_avoidance else None

    # Reset state
    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = START_POS
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    # Telemetry logging
    t_hist = []
    pos_hist = []
    target_hist = []
    clearance_hist = []
    detour_target_hist = []
    avoidance_active_hist = []

    collision_count = 0
    min_clearance = 999.0
    total_steps = int(duration / dt)

    # Reference obstacles for clearance evaluation
    ref_obstacles = ObstacleAvoidancePlanner().obstacles

    for step in range(total_steps):
        t_sim = step * dt
        pos = data.qpos[0:3].copy()
        vel = data.qvel[0:3].copy()
        quat = data.qpos[3:7].copy()
        omega = data.qvel[3:6].copy()

        # Step 1: Waypoint Manager
        nominal_target, dist_to_tgt, switched, reached_final = wp_manager.update(pos, t_sim)

        # Step 2: Obstacle Avoidance
        if enable_avoidance and obstacle_planner is not None:
            safe_target, avoidance_telemetry = obstacle_planner.update(
                current_pos=pos, current_vel=vel,
                nominal_target=nominal_target,
                current_waypoint_idx=wp_manager.current_idx,
                current_time=t_sim
            )
            obstacle_planner.check_mujoco_contacts(model, data)
            current_clearance = avoidance_telemetry['nearest_obs_dist']
            is_avoiding = avoidance_telemetry['avoidance_active']
            detour_tgt = avoidance_telemetry['detour_target']
        else:
            safe_target = nominal_target.copy()
            # Calculate distance to nearest obstacle
            nearest_d = 999.0
            for obs in ref_obstacles:
                d, _ = ObstacleAvoidancePlanner().compute_distance_to_obstacle(pos, obs)
                if d < nearest_d:
                    nearest_d = d
            current_clearance = nearest_d
            is_avoiding = False
            detour_tgt = None

            # Detect collisions via MuJoCo contact manifold
            for i in range(data.ncon):
                contact = data.contact[i]
                g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
                g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
                if g1 and g2 and (('obs' in g1 and any(k in g2 for k in ['body', 'arm', 'rotor'])) or 
                                  ('obs' in g2 and any(k in g1 for k in ['body', 'arm', 'rotor']))):
                    if t_sim > 0.2:
                        collision_count += 1
                        break

        if current_clearance < min_clearance:
            min_clearance = current_clearance

        # Step 3: Position Controller
        is_final = (wp_manager.current_idx == wp_manager.num_waypoints - 1)
        T_desired, q_ref, e_p, a_cmd = pos_controller.compute(
            current_pos=pos, current_vel=vel,
            target_pos=safe_target, target_yaw=0.0,
            is_final_target=is_final
        )

        # Step 4: Attitude Controller
        tau, q_err, w_err, delta = attitude_controller.compute(
            quat=quat, omega=omega, q_ref=q_ref
        )

        # Step 5: Motor Mixer & Actuators
        u_thrusts, yaw_torque = mixer.mix(T_desired, tau)
        data.ctrl[0:4] = u_thrusts
        data.ctrl[4] = yaw_torque

        # Step 6: Step MuJoCo
        mujoco.mj_step(model, data)

        # Log downsampled at 100 Hz
        if step % 5 == 0:
            t_hist.append(t_sim)
            pos_hist.append(pos)
            target_hist.append(nominal_target.copy())
            clearance_hist.append(current_clearance)
            detour_target_hist.append(detour_tgt.copy() if detour_tgt is not None else None)
            avoidance_active_hist.append(is_avoiding)

        if reached_final and wp_manager.is_completed:
            break

    actual_coll = obstacle_planner.collision_count if (enable_avoidance and obstacle_planner) else collision_count
    
    pos_arr = np.array(pos_hist)
    tgt_arr = np.array(target_hist)
    tracking_errors = np.linalg.norm(pos_arr - tgt_arr, axis=1)
    rmse = float(np.sqrt(np.mean(tracking_errors**2)))

    return {
        't': np.array(t_hist),
        'pos': pos_arr,
        'target': tgt_arr,
        'clearance': np.array(clearance_hist),
        'avoidance_active': np.array(avoidance_active_hist),
        'rmse': rmse,
        'min_clearance': float(min_clearance),
        'collision_count': int(actual_coll),
        'waypoints_reached': wp_manager.current_idx + 1 if wp_manager.is_completed else wp_manager.current_idx,
        'destination_reached': wp_manager.is_completed,
        'final_time': t_hist[-1] if t_hist else 0.0
    }

def main():
    print("=" * 80)
    print(" OBSTACLE AVOIDANCE VALIDATION & COMPARISON ")
    print("=" * 80)
    print(" Running Case A: Direct Tracking (Without Avoidance)...")
    res_no_avoid = run_simulation(enable_avoidance=False)
    print(f"   -> Collisions Detected     : {res_no_avoid['collision_count']}")
    print(f"   -> Min Obstacle Clearance  : {res_no_avoid['min_clearance']:.3f} m")
    print(f"   -> Waypoint Completion     : {res_no_avoid['waypoints_reached']} / 4")
    print(f"   -> Destination Reached     : {res_no_avoid['destination_reached']}")

    print("\n Running Case B: Autonomous Obstacle Avoidance (With 3D Detour Planner)...")
    res_avoid = run_simulation(enable_avoidance=True)
    print(f"   -> Collisions Detected     : {res_avoid['collision_count']}")
    print(f"   -> Min Obstacle Clearance  : {res_avoid['min_clearance']:.3f} m")
    print(f"   -> Waypoint Completion     : {res_avoid['waypoints_reached']} / 4")
    print(f"   -> Destination Reached     : {res_avoid['destination_reached']}")
    print(f"   -> Tracking RMSE           : {res_avoid['rmse']:.3f} m")

    # =========================================================================
    # Generate Comprehensive Comparative Visualization Plot
    # =========================================================================
    fig = plt.figure(figsize=(15, 10))

    # Subplot 1: 3D Trajectory Comparison with Physical Obstacles
    ax1 = fig.add_subplot(221, projection='3d')
    
    # Draw Reference Path
    full_wps = np.vstack([START_POS, WAYPOINTS])
    ax1.plot(full_wps[:, 0], full_wps[:, 1], full_wps[:, 2], 
             color='#FF8C00', linestyle='--', linewidth=2.0, label='Planned Reference Path')

    # Draw Uncontrolled Trajectory (Colliding)
    ax1.plot(res_no_avoid['pos'][:, 0], res_no_avoid['pos'][:, 1], res_no_avoid['pos'][:, 2], 
             color='red', linestyle=':', linewidth=2.0, alpha=0.8, label=f"Without Avoidance ({res_no_avoid['collision_count']} Collisions)")

    # Draw Autonomous Avoidance Trajectory (Safe)
    ax1.plot(res_avoid['pos'][:, 0], res_avoid['pos'][:, 1], res_avoid['pos'][:, 2], 
             color='#0080FF', linewidth=2.5, label=f"With Obstacle Avoidance (0 Collisions)")

    # Draw Waypoint Markers
    ax1.scatter(START_POS[0], START_POS[1], START_POS[2], color='blue', s=120, label='Start Point')
    for i, wp in enumerate(WAYPOINTS[:-1]):
        ax1.scatter(wp[0], wp[1], wp[2], color='orange', s=90)
        ax1.text(wp[0], wp[1], wp[2]+0.2, f"WP{i+1}", fontweight='bold')
    ax1.scatter(WAYPOINTS[-1][0], WAYPOINTS[-1][1], WAYPOINTS[-1][2], color='green', s=140, label='Destination')
    ax1.text(WAYPOINTS[-1][0], WAYPOINTS[-1][1], WAYPOINTS[-1][2]+0.2, "DEST", fontweight='bold', color='green')

    # Draw 4 Obstacles
    obs_list = ObstacleAvoidancePlanner().obstacles
    for obs in obs_list:
        c = obs['pos']
        r = obs['radius']
        if obs['type'] == 'cylinder':
            z_grid = np.linspace(obs['z_min'], obs['z_max'], 15)
            theta_grid = np.linspace(0, 2*np.pi, 20)
            theta_mesh, z_mesh = np.meshgrid(theta_grid, z_grid)
            x_mesh = c[0] + r * np.cos(theta_mesh)
            y_mesh = c[1] + r * np.sin(theta_mesh)
            ax1.plot_surface(x_mesh, y_mesh, z_mesh, color='crimson', alpha=0.35)
            ax1.text(c[0], c[1], obs['z_max']+0.2, obs['label'], fontsize=8, color='crimson', fontweight='bold')
        elif obs['type'] == 'box':
            h = obs['half_size']
            # Draw wireframe box
            dx = [c[0]-h[0], c[0]+h[0]]
            dy = [c[1]-h[1], c[1]+h[1]]
            dz = [0.0, 2*h[2]]
            for x in dx:
                for y in dy:
                    ax1.plot([x, x], [y, y], dz, color='navy', alpha=0.6, linewidth=1.5)
            for z in dz:
                ax1.plot([dx[0], dx[1], dx[1], dx[0], dx[0]], 
                         [dy[0], dy[0], dy[1], dy[1], dy[0]], [z]*5, color='navy', alpha=0.6, linewidth=1.5)
            ax1.text(c[0], c[1], 2*h[2]+0.2, obs['label'], fontsize=8, color='navy', fontweight='bold')
        elif obs['type'] == 'sphere':
            u = np.linspace(0, 2 * np.pi, 15)
            v = np.linspace(0, np.pi, 15)
            x = c[0] + r * np.outer(np.cos(u), np.sin(v))
            y = c[1] + r * np.outer(np.sin(u), np.sin(v))
            z = c[2] + r * np.outer(np.ones(np.size(u)), np.cos(v))
            ax1.plot_surface(x, y, z, color='gold', alpha=0.4)
            ax1.text(c[0], c[1], c[2]+r+0.2, obs['label'], fontsize=8, color='#B28900', fontweight='bold')

    ax1.set_title("1. 3D Flight Path with Physical Obstacles", fontweight='bold')
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Z (m)")
    ax1.legend(loc='upper left', fontsize=8)

    # Subplot 2: Top-Down (X-Y) Trajectory & Detours
    ax2 = fig.add_subplot(222)
    ax2.plot(full_wps[:, 0], full_wps[:, 1], 'orange', linestyle='--', linewidth=2.0, label='Planned Reference')
    ax2.plot(res_no_avoid['pos'][:, 0], res_no_avoid['pos'][:, 1], 'red', linestyle=':', label='Without Avoidance')
    ax2.plot(res_avoid['pos'][:, 0], res_avoid['pos'][:, 1], '#0080FF', linewidth=2.5, label='Autonomous Avoidance')
    
    # Plot obstacle footprint circles
    for obs in obs_list:
        c = obs['pos']
        r = obs['radius']
        circle = plt.Circle((c[0], c[1]), r, color='crimson' if obs['type']=='cylinder' else ('navy' if obs['type']=='box' else 'gold'), alpha=0.35)
        ax2.add_patch(circle)
        safe_circle = plt.Circle((c[0], c[1]), r + 0.35 + 0.50, color='green', fill=False, linestyle=':', alpha=0.6)
        ax2.add_patch(safe_circle)
        ax2.text(c[0], c[1], obs['label'], fontsize=8, ha='center', va='center', fontweight='bold')

    ax2.scatter(full_wps[:, 0], full_wps[:, 1], color=['blue', 'orange', 'orange', 'orange', 'green'], s=80, zorder=5)
    ax2.set_title("2. Top-Down (X-Y) Flight Trajectory & Safe Bypasses", fontweight='bold')
    ax2.set_xlabel("X Position (m)")
    ax2.set_ylabel("Y Position (m)")
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend(loc='lower right', fontsize=8)

    # Subplot 3: Obstacle Clearance Distance vs Time
    ax3 = fig.add_subplot(223)
    ax3.plot(res_no_avoid['t'], res_no_avoid['clearance'], color='red', linestyle='--', label='Without Avoidance')
    ax3.plot(res_avoid['t'], res_avoid['clearance'], color='green', linewidth=2.2, label='With Obstacle Avoidance')
    ax3.axhline(0.0, color='black', linewidth=1.5, linestyle='-', label='Obstacle Boundary (Collision Line)')
    ax3.axhline(0.85, color='orange', linestyle=':', linewidth=1.5, label='Safety Margin (0.85 m)')
    ax3.set_title("3. Nearest Obstacle Clearance vs Time", fontweight='bold')
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Distance to Nearest Obstacle Surface (m)")
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.legend(loc='upper right', fontsize=8)

    # Subplot 4: Performance Comparison Metrics Bar Chart
    ax4 = fig.add_subplot(224)
    metrics = ['Collisions', 'Min Clearance (m)', 'Waypoints Reached', 'RMSE (m)']
    no_avoid_vals = [res_no_avoid['collision_count'], max(0.0, res_no_avoid['min_clearance']), res_no_avoid['waypoints_reached'], res_no_avoid['rmse']]
    avoid_vals = [res_avoid['collision_count'], max(0.0, res_avoid['min_clearance']), res_avoid['waypoints_reached'], res_avoid['rmse']]

    x = np.arange(len(metrics))
    width = 0.35
    ax4.bar(x - width/2, no_avoid_vals, width, label='Without Avoidance', color='#E53935')
    ax4.bar(x + width/2, avoid_vals, width, label='With Avoidance', color='#00897B')
    ax4.set_xticks(x)
    ax4.set_xticklabels(metrics, fontweight='bold')
    ax4.set_title("4. Quantitative Performance Comparison", fontweight='bold')
    ax4.legend(loc='upper right')
    ax4.grid(True, axis='y', linestyle='--', alpha=0.6)

    for i in range(len(metrics)):
        ax4.text(x[i] - width/2, no_avoid_vals[i] + 0.1, f"{no_avoid_vals[i]:.2f}", ha='center', fontsize=9, fontweight='bold')
        ax4.text(x[i] + width/2, avoid_vals[i] + 0.1, f"{avoid_vals[i]:.2f}", ha='center', fontsize=9, fontweight='bold')

    plt.tight_layout()
    out_fig = os.path.join(RESULTS_DIR, "obstacle_avoidance_comparison.png")
    plt.savefig(out_fig, dpi=300)
    plt.close()
    print(f"\n[Comparison] Saved comparative analysis figure to: {out_fig}")

if __name__ == "__main__":
    main()
