import mujoco
import numpy as np
import os
import sys

# Base Path Setup
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE_DIR)

from controllers.quaternion import (
    quat_to_euler, quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion, quaternion_normalize
)
from controllers.backstepping import BacksteppingController

MODEL_PATH = os.path.join(BASE_DIR, "models", "quadrotor.xml")
RESULTS_DIR = os.path.join(BASE_DIR, "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

def run_step21_base_paper_verification():
    print("=" * 75)
    print(" STEP 21: BASE PAPER ATTITUDE CONTROLLER VERIFICATION ")
    print("=" * 75)

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    model.opt.timestep = 0.002
    
    quad_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")

    controller = BacksteppingController(J=np.diag([0.1, 0.1, 0.12]), k1=20.0, k2=2.0)

    # Initial perturbed attitude: Roll = 20 deg, Pitch = 10 deg, Yaw = 15 deg
    roll_init = np.radians(20.0)
    pitch_init = np.radians(10.0)
    yaw_init = np.radians(15.0)
    
    # Calculate initial quaternion from Euler angles
    cy = np.cos(yaw_init * 0.5)
    sy = np.sin(yaw_init * 0.5)
    cp = np.cos(pitch_init * 0.5)
    sp = np.sin(pitch_init * 0.5)
    cr = np.cos(roll_init * 0.5)
    sr = np.sin(roll_init * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    q_initial = quaternion_normalize(np.array([qw, qx, qy, qz]))

    q_target = np.array([1.0, 0.0, 0.0, 0.0]) # Target: Level (Roll=0, Pitch=0, Yaw=0)

    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = np.array([0.0, 0.0, 2.0]) # Hover altitude z=2m
    data.qpos[3:7] = q_initial
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    print(f"Initial Attitude (t=0.0s) : Roll=20.0 deg, Pitch=10.0 deg, Yaw=15.0 deg")
    print(f"Target Attitude (Level)   : Roll=0.0 deg, Pitch=0.0 deg, Yaw=0.0 deg")

    history = {
        't': [],
        'q_err': [],
        'w_err': [],
        'euler': [],
        'omega': [],
        'tau': []
    }

    num_steps = 1000 # 2.0 seconds simulation
    for step in range(num_steps):
        t = data.time
        
        # Read State Feedback
        quat = data.qpos[3:7].copy()
        omega = data.qvel[3:6].copy()

        # Compute Backstepping Control Torque
        tau, q_err, w_err, delta = controller.compute(
            quat=quat, omega=omega, q_ref=q_target, dt=0.002
        )

        # Transform body torque to world frame for MuJoCo xfrc_applied
        R = quaternion_to_rotation_matrix(quat)
        tau_world = np.dot(R, tau)

        # Apply Hover Thrust + World Torque to Body
        data.xfrc_applied[quad_body_id, 0:3] = np.array([0.0, 0.0, 9.81])
        data.xfrc_applied[quad_body_id, 3:6] = tau_world

        # Log telemetry
        history['t'].append(t)
        history['q_err'].append(q_err.copy())
        history['w_err'].append(w_err.copy())
        history['euler'].append(quat_to_euler(quat))
        history['omega'].append(omega.copy())
        history['tau'].append(tau.copy())

        # Advance MuJoCo Physics
        mujoco.mj_step(model, data)

    # Convert telemetry to NumPy arrays
    data_dict = {key: np.array(val) for key, val in history.items()}
    npz_path = os.path.join(RESULTS_DIR, "attitude_results.npz")
    np.savez_compressed(npz_path, **data_dict)
    print(f"[Telemetry] Saved base paper results to {npz_path}")

    # Print final results
    final_euler_deg = np.degrees(data_dict['euler'][-1])
    final_q_err = data_dict['q_err'][-1]

    print("\n--- Base Paper Verification Results (t=2.0s) ---")
    print(f"Final Euler Angles (deg): Roll={final_euler_deg[0]:.2f} deg, Pitch={final_euler_deg[1]:.2f} deg, Yaw={final_euler_deg[2]:.2f} deg")
    print(f"Final Quaternion Error  : {final_q_err.round(4)}")

    if abs(final_euler_deg[0]) < 1.0 and abs(final_euler_deg[1]) < 1.0 and abs(final_euler_deg[2]) < 1.0:
        print("\n" + "=" * 75)
        print(" [SUCCESS] STEP 21 BASE PAPER ATTITUDE VERIFICATION PASSED! ")
        print(" Tilted quadrotor successfully corrected to level orientation!")
        print("=" * 75)
        return data_dict
    else:
        print(f"\n[FAIL] Attitude verification incomplete: final euler = {final_euler_deg}")
        return data_dict

if __name__ == "__main__":
    run_step21_base_paper_verification()
