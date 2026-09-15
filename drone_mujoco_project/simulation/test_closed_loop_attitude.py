import mujoco
import numpy as np
import os
import sys

# Base Path Setup
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE_DIR)

from controllers.quaternion import quat_to_euler, quaternion_to_rotation_matrix
from controllers.backstepping import BacksteppingController

MODEL_PATH = os.path.join(BASE_DIR, "models", "quadrotor.xml")

def run_step20_closed_loop_attitude_test():
    print("=" * 70)
    print(" STEPS 11 & 20: CLOSED-LOOP ATTITUDE BACKSTEPPING TEST IN MUJOCO ")
    print("=" * 70)

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    model.opt.timestep = 0.002
    
    quad_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")

    controller = BacksteppingController(J=np.diag([0.1, 0.1, 0.12]), k1=20.0, k2=2.0)

    # Initial perturbed attitude (20 deg roll, -15 deg pitch)
    q_initial = np.array([0.9763, 0.1736, -0.1305, 0.0])
    q_initial /= np.linalg.norm(q_initial)
    
    q_target = np.array([1.0, 0.0, 0.0, 0.0]) # Target: Level attitude

    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = np.array([0.0, 0.0, 2.0]) # Hover altitude z=2m
    data.qpos[3:7] = q_initial
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    print(f"Initial Attitude (t=0.0s) : {quat_to_euler(q_initial).round(3)} rad")
    print(f"Target Attitude (Level)   : {quat_to_euler(q_target).round(3)} rad")

    # Step 20: Closed-Loop Simulation Loop (2.0 seconds / 1000 steps)
    num_steps = 1000
    for step in range(num_steps):
        # -------------------------------------------------------------
        # STEP 11: Read Drone State Feedback Dictionary from MuJoCo
        # -------------------------------------------------------------
        state = {
            "position": data.qpos[0:3].copy(),
            "quaternion": data.qpos[3:7].copy(), # [qw, qx, qy, qz]
            "linear_velocity": data.qvel[0:3].copy(),
            "angular_velocity": data.qvel[3:6].copy()
        }

        # -------------------------------------------------------------
        # STEPS 14-19: Compute Control Torque tau from Backstepping Controller
        # -------------------------------------------------------------
        tau, q_err, w_err, delta = controller.compute(
            quat=state["quaternion"],
            omega=state["angular_velocity"],
            q_ref=q_target,
            omega_ref=np.zeros(3),
            dt=0.002
        )

        # Transform body torque to world frame for MuJoCo xfrc_applied
        R = quaternion_to_rotation_matrix(state["quaternion"])
        tau_world = np.dot(R, tau)

        # -------------------------------------------------------------
        # STEP 20: Apply Hover Thrust + World Torque to Floating Body
        # -------------------------------------------------------------
        data.xfrc_applied[quad_body_id, 0:3] = np.array([0.0, 0.0, 9.81]) # Hover thrust
        data.xfrc_applied[quad_body_id, 3:6] = tau_world                  # Torques in world frame

        # Advance MuJoCo Physics
        mujoco.mj_step(model, data)

    # Read final state after 2.0 seconds
    final_quat = data.qpos[3:7]
    final_euler = quat_to_euler(final_quat)
    final_omega = data.qvel[3:6]
    final_q_err = controller.compute(quat=final_quat, omega=final_omega, q_ref=q_target)[1]

    print("\n--- Closed-Loop Simulation Results (t=2.0s) ---")
    print(f"Final Quaternion          : {final_quat.round(4)}")
    print(f"Final Euler Angles (rad)  : {final_euler.round(4)}")
    print(f"Final Quaternion Error    : {final_q_err.round(4)}")
    print(f"Final Angular Velocity    : {final_omega.round(4)}")

    if np.allclose(final_q_err, np.array([1.0, 0.0, 0.0, 0.0]), atol=1e-2):
        print("\n" + "=" * 70)
        print(" [SUCCESS] STEP 20 CLOSED-LOOP ATTITUDE CONTROLLER PASSED! ")
        print(" Drone attitude stabilized to level [1,0,0,0] within 2.0 seconds!")
        print("=" * 70)
        return True
    else:
        print(f"\n[FAIL] Closed-loop attitude convergence incomplete: q_err = {final_q_err}")
        return False

if __name__ == "__main__":
    run_step20_closed_loop_attitude_test()
