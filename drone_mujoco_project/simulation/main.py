import argparse
import mujoco
import mujoco.viewer
import numpy as np
import os
import sys
import time

# Base Path Setup
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE_DIR)

from controllers.quaternion import quat_to_euler
from controllers.position_controller import PositionController
from controllers.backstepping import BacksteppingController
from controllers.motor_mixer import MotorMixer
from visualization.plot_results import plot_extension_results, plot_base_paper_results

MODEL_PATH = os.path.join(BASE_DIR, "models", "quadrotor.xml")
RESULTS_DIR = os.path.join(BASE_DIR, "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

def parse_args():
    parser = argparse.ArgumentParser(description="Drone MuJoCo Project - Complete Cascaded Closed-Loop Simulation")
    parser.add_argument("--start", nargs=3, type=float, default=[0.0, 0.0, 1.0],
                        help="Start position P0 x y z (default: 0 0 1)")
    parser.add_argument("--target", nargs=3, type=float, default=[5.0, 5.0, 3.0],
                        help="Target destination position Pd x y z (default: 5 5 3)")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Simulation duration in seconds")
    parser.add_argument("--gui", action="store_true", default=True,
                        help="Enable interactive passive 3D viewer rendering")
    parser.add_argument("--no-gui", action="store_false", dest="gui",
                        help="Disable GUI viewer for headless execution")
    return parser.parse_args()

def main():
    args = parse_args()
    
    start_pos = np.array(args.start, dtype=float)
    target_pos = np.array(args.target, dtype=float)
    
    print("=" * 75)
    print(" DRONE MUJOCO PROJECT — COMPLETE DESTINATION-BASED SIMULATION ")
    print("=" * 75)
    print(f" Start Position P0       : {start_pos}")
    print(f" Destination Setpoint Pd : {target_pos}")
    print(f" Duration                : {args.duration} s")
    print(f" 3D Viewer GUI          : {'ENABLED' if args.gui else 'DISABLED'}")
    print("=" * 75)

    # 1. Instantiate MuJoCo Model & Data
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    dt = 0.002
    model.opt.timestep = dt

    # 2. Instantiate Controllers
    pos_controller = PositionController(mass=1.0, gravity=9.81)
    attitude_controller = BacksteppingController(J=np.diag([0.1, 0.1, 0.12]), k1=20.0, k2=2.0)
    mixer = MotorMixer(arm_length=0.4)

    # 3. Reset Physics & Set Start Position
    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = start_pos
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0]) # Identity quat
    data.qvel[:] = 0.0
    
    # Update Target Destination Marker in MuJoCo
    if model.nmocap > 0:
        data.mocap_pos[0] = target_pos
    mujoco.mj_forward(model, data)

    # 4. Initialize Passive Viewer if GUI enabled
    viewer = None
    if args.gui:
        viewer = mujoco.viewer.launch_passive(model, data)
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        viewer.cam.distance = 12.0
        viewer.cam.lookat[:] = (start_pos + target_pos) * 0.5

    # 5. Telemetry Container (Step 34)
    history = {
        't': [],
        'pos': [],
        'target_pos': target_pos,
        'vel': [],
        'quat': [],
        'q_err': [],
        'w_err': [],
        'euler': [],
        'omega': [],
        'tau': [],
        'motor_thrusts': [],
        'dist_err': []
    }

    num_steps = int(args.duration / dt)
    log_interval = int(0.5 / dt) # Console output every 0.5s
    destination_reached_time = None

    print(f"\n[Simulation] Flight Started: {start_pos} -> Destination Goal {target_pos}")
    print("-" * 75)

    # Step 33: Complete Simulation Loop
    for step in range(num_steps):
        step_start_time = time.time()
        t = data.time
        
        # -------------------------------------------------------------
        # STEP 11 & 25: Read Current State from MuJoCo
        # -------------------------------------------------------------
        pos = data.qpos[0:3].copy()
        quat = data.qpos[3:7].copy()  # [qw, qx, qy, qz]
        vel = data.qvel[0:3].copy()
        omega = data.qvel[3:6].copy()

        # -------------------------------------------------------------
        # STEPS 26-30: Position Controller -> T_desired & q_ref
        # -------------------------------------------------------------
        T_desired, q_ref, e_p, a_cmd = pos_controller.compute(
            current_pos=pos, current_vel=vel, target_pos=target_pos, target_yaw=0.0
        )

        # -------------------------------------------------------------
        # STEPS 14-19: Base Paper Attitude Backstepping Controller -> tau
        # -------------------------------------------------------------
        tau, q_err, w_err, delta = attitude_controller.compute(
            quat=quat, omega=omega, q_ref=q_ref, omega_ref=np.zeros(3), dt=dt
        )

        # -------------------------------------------------------------
        # STEP 32: Motor Mixer -> T1, T2, T3, T4
        # -------------------------------------------------------------
        T_motors, tau_z_reaction = mixer.mix(total_thrust=T_desired, tau=tau)

        # -------------------------------------------------------------
        # STEP 31 & 33: Apply Motor Controls & Step Physics
        # -------------------------------------------------------------
        data.ctrl[0] = T_motors[0]     # Motor 1 (+X)
        data.ctrl[1] = T_motors[1]     # Motor 2 (+Y)
        data.ctrl[2] = T_motors[2]     # Motor 3 (-X)
        data.ctrl[3] = T_motors[3]     # Motor 4 (-Y)
        data.ctrl[4] = tau_z_reaction  # Reaction Yaw Torque

        # Step 35: Distance to Goal Error
        dist_err = np.linalg.norm(target_pos - pos)
        vel_norm = np.linalg.norm(vel)
        
        if dist_err < 0.1 and vel_norm < 0.2 and destination_reached_time is None:
            destination_reached_time = t
            print(f"\n[DESTINATION REACHED!] Goal reached at t = {t:.2f}s | Final Distance = {dist_err:.3f}m | Velocity = {vel_norm:.3f}m/s\n")

        # Step 34: Record Telemetry Data
        history['t'].append(t)
        history['pos'].append(pos.copy())
        history['vel'].append(vel.copy())
        history['quat'].append(quat.copy())
        history['q_err'].append(q_err.copy())
        history['w_err'].append(w_err.copy())
        history['euler'].append(quat_to_euler(quat))
        history['omega'].append(omega.copy())
        history['tau'].append(tau.copy())
        history['motor_thrusts'].append(T_motors.copy())
        history['dist_err'].append(dist_err)

        # Console Log Output every 0.5s
        if step % log_interval == 0 or step == num_steps - 1:
            print(f" [Nav {t:5.2f}s] Drone: {pos.round(2)} | Goal: {target_pos.round(2)} | Position Error e_p: {e_p.round(2)} | Dist Error: {dist_err:5.2f}m")

        # Advance MuJoCo Physics Step
        mujoco.mj_step(model, data)

        # Sync 3D Viewer
        if viewer is not None:
            if step % 2 == 0:
                viewer.sync()
            elapsed_step = time.time() - step_start_time
            if elapsed_step < dt:
                time.sleep(dt - elapsed_step)

    print("-" * 75)
    print(f"[Simulation Complete] Final Position: {data.qpos[0:3].round(3)} | Final Distance to Goal: {history['dist_err'][-1]:.3f}m")
    if destination_reached_time is not None:
        print(f"[Summary] Destination Reaching Time: {destination_reached_time:.2f} seconds")
    print("-" * 75)

    # Save Output Telemetry
    data_dict = {key: (np.array(val) if isinstance(val, list) else val) for key, val in history.items()}
    npz_path = os.path.join(RESULTS_DIR, "final_destination_simulation.npz")
    np.savez_compressed(npz_path, **data_dict)
    print(f"[Telemetry] Saved simulation telemetry to {npz_path}")

    # Generate All Final Project Plots (Step 38)
    print("[Plots] Generating final project result figures...")
    plot_extension_results(data_dict, save_dir=RESULTS_DIR)
    plot_base_paper_results(data_dict, save_dir=RESULTS_DIR)
    print(f"[Done] All results and figures saved to: {RESULTS_DIR}")
    print("=" * 75)

    if viewer is not None:
        print("[Simulation] 3D Viewer open. Close window to finish script...")
        while viewer.is_running():
            time.sleep(0.1)
        viewer.close()

if __name__ == "__main__":
    main()
