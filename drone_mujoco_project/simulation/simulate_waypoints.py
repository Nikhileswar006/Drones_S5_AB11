import os
import sys
import time
import argparse
import numpy as np
import mujoco
import mujoco.viewer

# Add project root to sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from controllers.quaternion import quat_to_euler
from controllers.backstepping import BacksteppingController
from controllers.position_controller import PositionController
from controllers.motor_mixer import MotorMixer
from controllers.waypoint_manager import WaypointManager
from visualization.plot_waypoints import plot_all_waypoint_results

MODEL_PATH = os.path.join(PROJECT_ROOT, "models", "quadrotor.xml")
RESULTS_DIR = os.path.join(PROJECT_ROOT, "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

DEFAULT_WAYPOINTS = [
    [3.0, 2.0, 2.5],   # WP1 (Spaced out)
    [6.0, 8.0, 3.5],   # WP2 (Wide bank turn)
    [10.0, 4.0, 4.0],  # WP3 (Cross-field transit)
    [14.0, 12.0, 5.0]  # Destination Goal (Longer distance)
]

def run_waypoint_simulation(start_pos: np.ndarray, 
                            waypoints: list,
                            duration: float = 22.0,
                            tolerance: float = 0.75,
                            final_tolerance: float = 0.20,
                            gui: bool = True):
    print("=" * 80)
    print(" MULTI-WAYPOINT UAV SIMULATION -- QUATERNION BACKSTEPPING CONTROLLER ")
    print("=" * 80)
    print(f" Start Position P0        : {start_pos}")
    print(f" Intermediate Waypoints   :")
    for i, wp in enumerate(waypoints[:-1]):
        print(f"   * WP{i+1}                   : {wp}")
    print(f" Final Destination Goal   : {waypoints[-1]}")
    print(f" Tolerance (WP / Final)   : {tolerance}m / {final_tolerance}m")
    print(f" Duration                 : {duration} s")
    print(f" 3D Viewer GUI            : {'ENABLED' if gui else 'DISABLED'}")
    print("=" * 80)

    # 1. Instantiate MuJoCo Model & Data
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    dt = 0.002
    model.opt.timestep = dt

    # 2. Instantiate Controllers
    wp_manager = WaypointManager(waypoints=waypoints, tolerance=tolerance, final_tolerance=final_tolerance)
    pos_controller = PositionController(mass=1.0, gravity=9.81)
    attitude_controller = BacksteppingController(J=np.diag([0.1, 0.1, 0.12]), k1=20.0, k2=2.0)
    mixer = MotorMixer(arm_length=0.85)

    # 3. Reset Physics & Set Start Position
    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = start_pos
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0]) # Identity quat
    data.qvel[:] = 0.0

    # 4. Position Mocap Markers in MuJoCo World
    def set_mocap_pos(body_name, pos_vec):
        b_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if b_id != -1 and model.body_mocapid[b_id] != -1:
            data.mocap_pos[model.body_mocapid[b_id]] = pos_vec

    set_mocap_pos("Start_Point", start_pos)
    for idx, wp in enumerate(waypoints[:-1]):
        set_mocap_pos(f"WP{idx+1}", wp)
    set_mocap_pos("Destination", waypoints[-1])
    mujoco.mj_forward(model, data)

    # 5. Initialize Passive Viewer if GUI enabled
    viewer = None
    if gui:
        viewer = mujoco.viewer.launch_passive(model, data)
        viewer.opt.label = mujoco.mjtLabel.mjLABEL_BODY # Displays floating labels
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        viewer.cam.distance = 25.0
        viewer.cam.lookat[:] = [7.0, 6.0, 3.0]

    # 6. Telemetry Logging Container
    history = {
        't': [],
        'pos': [],
        'target_pos': [],
        'wp_idx': [],
        'vel': [],
        'quat': [],
        'q_err': [],
        'w_err': [],
        'euler': [],
        'omega': [],
        'tau': [],
        'motor_thrusts': [],
        'dist_err': [],
        'start_pos': start_pos,
        'waypoints': np.array(waypoints)
    }

    num_steps = int(duration / dt)
    log_interval = int(0.5 / dt) # Console output every 0.5s

    print("\n[Flight Started] Progressing through waypoints sequentially...")
    print("-" * 80)

    for step in range(num_steps):
        step_start_time = time.time()
        t = data.time

        # Read Current State from MuJoCo
        pos = data.qpos[0:3].copy()
        quat = data.qpos[3:7].copy()
        vel = data.qvel[0:3].copy()
        omega = data.qvel[3:6].copy()

        # Update Waypoint Manager to get active target
        active_target, dist_err, switched, completed = wp_manager.update(pos, t)

        # Outer Loop: Position Controller -> T_desired & q_ref
        T_desired, q_ref, e_p, a_cmd = pos_controller.compute(
            current_pos=pos, current_vel=vel, target_pos=active_target, target_yaw=0.0
        )

        # Inner Loop: Base Paper Attitude Backstepping Controller -> tau
        tau, q_err, w_err, delta = attitude_controller.compute(
            quat=quat, omega=omega, q_ref=q_ref, omega_ref=np.zeros(3), dt=dt
        )

        # Motor Mixer -> T1, T2, T3, T4
        T_motors, tau_z_reaction = mixer.mix(total_thrust=T_desired, tau=tau)

        # Apply Motor Actuation
        data.ctrl[0] = T_motors[0]     # Motor 1 (+X)
        data.ctrl[1] = T_motors[1]     # Motor 2 (+Y)
        data.ctrl[2] = T_motors[2]     # Motor 3 (-X)
        data.ctrl[3] = T_motors[3]     # Motor 4 (-Y)
        data.ctrl[4] = tau_z_reaction  # Yaw reaction torque

        # Advance MuJoCo Physics
        mujoco.mj_step(model, data)

        # Record Telemetry
        history['t'].append(t)
        history['pos'].append(pos.copy())
        history['target_pos'].append(active_target.copy())
        history['wp_idx'].append(wp_manager.current_idx)
        history['vel'].append(vel.copy())
        history['quat'].append(quat.copy())
        history['q_err'].append(q_err.copy())
        history['w_err'].append(w_err.copy())
        history['euler'].append(quat_to_euler(quat))
        history['omega'].append(omega.copy())
        history['tau'].append(tau.copy())
        history['motor_thrusts'].append(T_motors.copy())
        history['dist_err'].append(dist_err)

        # Console Progress Output
        if step % log_interval == 0 or step == num_steps - 1 or switched:
            wp_tag = wp_manager.current_name
            print(f" [Nav {t:5.2f}s] Drone: {pos.round(2)} | Target ({wp_tag}): {active_target.round(2)} | Error: {dist_err:5.2f}m")

        # Sync Viewer
        if viewer is not None and viewer.is_running():
            viewer.sync()
            elapsed = time.time() - step_start_time
            if dt > elapsed:
                time.sleep(dt - elapsed)

    if viewer is not None and viewer.is_running():
        print("[Viewer] Simulation finished. Closing 3D viewer.")
        viewer.close()

    # Convert lists to NumPy arrays
    data_dict = {
        't': np.array(history['t']),
        'pos': np.array(history['pos']),
        'target_pos': np.array(history['target_pos']),
        'wp_idx': np.array(history['wp_idx']),
        'vel': np.array(history['vel']),
        'quat': np.array(history['quat']),
        'q_err': np.array(history['q_err']),
        'w_err': np.array(history['w_err']),
        'euler': np.array(history['euler']),
        'omega': np.array(history['omega']),
        'tau': np.array(history['tau']),
        'motor_thrusts': np.array(history['motor_thrusts']),
        'dist_err': np.array(history['dist_err']),
        'start_pos': start_pos,
        'waypoints': np.array(waypoints),
        'switch_history': wp_manager.switch_history
    }

    # Save to NPZ
    npz_path = os.path.join(RESULTS_DIR, "waypoint_simulation_results.npz")
    np.savez_compressed(npz_path, **{k: v for k, v in data_dict.items() if k != 'switch_history'})
    print(f"\n[Telemetry] Saved multi-waypoint simulation telemetry to: {npz_path}")

    # Generate all 6 requested plots
    print("[Plots] Generating all 6 waypoint verification plots...")
    plot_all_waypoint_results(data_dict, wp_manager.switch_history, save_dir=RESULTS_DIR)

    final_pos = data_dict['pos'][-1]
    final_err = data_dict['dist_err'][-1]
    print("-" * 80)
    print(f"[Complete] Final Drone Position: {final_pos.round(3)} | Final Error to Destination: {final_err:.3f}m")
    print("=" * 80)

def main():
    parser = argparse.ArgumentParser(description="Multi-Waypoint Quadrotor UAV Simulation in MuJoCo")
    parser.add_argument("--start", nargs=3, type=float, default=[0.0, 0.0, 1.0], help="Start position [x y z]")
    parser.add_argument("--duration", type=float, default=22.0, help="Simulation duration (s)")
    parser.add_argument("--tolerance", type=float, default=0.75, help="Waypoint switching tolerance (m)")
    parser.add_argument("--final-tolerance", type=float, default=0.20, help="Final goal arrival tolerance (m)")
    parser.add_argument("--gui", action="store_true", default=True, help="Enable 3D viewer")
    parser.add_argument("--no-gui", dest="gui", action="store_false", help="Disable 3D viewer (headless)")
    parser.add_argument("--dashboard", action="store_true", help="Launch interactive split-screen 3D simulation + live results dashboard")
    args = parser.parse_args()

    if args.dashboard:
        from simulation.simulate_with_dashboard import run_simulation_with_dashboard
        run_simulation_with_dashboard(
            start_pos=np.array(args.start),
            waypoints=DEFAULT_WAYPOINTS,
            duration=args.duration,
            tolerance=args.tolerance,
            final_tolerance=args.final_tolerance
        )
        return

    run_waypoint_simulation(
        start_pos=np.array(args.start),
        waypoints=DEFAULT_WAYPOINTS,
        duration=args.duration,
        tolerance=args.tolerance,
        final_tolerance=args.final_tolerance,
        gui=args.gui
    )

if __name__ == "__main__":
    main()
