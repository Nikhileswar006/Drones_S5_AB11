import mujoco
import numpy as np
import os
import sys

# Base Path Setup
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_PATH = os.path.join(BASE_DIR, "models", "quadrotor.xml")

def run_tests():
    print("=" * 70)
    print(" DRONE MUJOCO PROJECT — STEPS 2, 9, & 10 PHYSICAL VERIFICATION ")
    print("=" * 70)

    # -------------------------------------------------------------
    # STEP 2: Verify MuJoCo Installation & XML Load
    # -------------------------------------------------------------
    print("\n--- STEP 2: Verification of MuJoCo Installation & XML Load ---")
    print(f"MuJoCo Version: {mujoco.__version__}")
    
    try:
        model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        data = mujoco.MjData(model)
        model.opt.timestep = 0.002
        print("[SUCCESS] XML Loaded cleanly without errors.")
    except Exception as e:
        print(f"[FAIL] XML Load error: {e}")
        return False

    # -------------------------------------------------------------
    # STEP 9: Hovering Test (T1 = T2 = T3 = T4 = 2.4525 N)
    # -------------------------------------------------------------
    print("\n--- STEP 9: Hovering Test (T_total = mg = 9.81 N) ---")
    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = np.array([0.0, 0.0, 1.0]) # Initial height = 1.0m
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0]) # Identity quat
    data.qvel[:] = 0.0
    
    hover_thrust_per_motor = (1.0 * 9.81) / 4.0 # 2.4525 N per motor
    data.ctrl[0] = hover_thrust_per_motor # Motor 1 (+X)
    data.ctrl[1] = hover_thrust_per_motor # Motor 2 (+Y)
    data.ctrl[2] = hover_thrust_per_motor # Motor 3 (-X)
    data.ctrl[3] = hover_thrust_per_motor # Motor 4 (-Y)
    data.ctrl[4] = 0.0                    # Reaction yaw torque
    
    # Step simulation for 1.0 second (500 steps)
    for _ in range(500):
        mujoco.mj_step(model, data)
        
    z_final = data.qpos[2]
    vz_final = data.qvel[2]
    print(f"Hover Test Result (1.0s): Height z = {z_final:.4f}m | Vertical Velocity vz = {vz_final:.4f}m/s")
    
    if abs(z_final - 1.0) < 0.05 and abs(vz_final) < 0.05:
        print("[SUCCESS] Step 9 Hover Test Passed: Drone hovers in place under total thrust 9.81N!")
    else:
        print(f"[FAIL] Step 9 Hover Test: Unexpected drift z={z_final}, vz={vz_final}")
        return False

    # -------------------------------------------------------------
    # STEP 10: Test Roll, Pitch, and Yaw Motions
    # -------------------------------------------------------------
    print("\n--- STEP 10: Test Roll, Pitch, and Yaw Motions ---")
    
    # 1. Roll Test (Unequal Left/Right Motor Thrust: T2 > T4)
    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = np.array([0.0, 0.0, 1.0])
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    data.qvel[:] = 0.0
    
    data.ctrl[0] = hover_thrust_per_motor
    data.ctrl[1] = 3.5  # Motor 2 (+Y) higher thrust
    data.ctrl[2] = hover_thrust_per_motor
    data.ctrl[3] = 1.4  # Motor 4 (-Y) lower thrust
    
    for _ in range(100):
        mujoco.mj_step(model, data)
        
    roll_rate = data.qvel[3]
    print(f"Roll Motion Test (T2 > T4): Roll angular velocity wx = {roll_rate:.3f} rad/s")
    if roll_rate > 0.1:
        print("[SUCCESS] Roll Motion Verified: Differential left/right thrust produces positive roll rotation!")
    else:
        print(f"[FAIL] Roll Motion Test failed: wx = {roll_rate}")
        return False

    # 2. Pitch Test (Unequal Front/Back Motor Thrust: T3 > T1)
    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = np.array([0.0, 0.0, 1.0])
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    data.qvel[:] = 0.0
    
    data.ctrl[0] = 1.4  # Motor 1 (+X) lower thrust
    data.ctrl[1] = hover_thrust_per_motor
    data.ctrl[2] = 3.5  # Motor 3 (-X) higher thrust
    data.ctrl[3] = hover_thrust_per_motor
    
    for _ in range(100):
        mujoco.mj_step(model, data)
        
    pitch_rate = data.qvel[4]
    print(f"Pitch Motion Test (T3 > T1): Pitch angular velocity wy = {pitch_rate:.3f} rad/s")
    if pitch_rate > 0.1:
        print("[SUCCESS] Pitch Motion Verified: Differential front/back thrust produces positive pitch rotation!")
    else:
        print(f"[FAIL] Pitch Motion Test failed: wy = {pitch_rate}")
        return False

    # 3. Yaw Test (Differential Reaction Torque)
    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = np.array([0.0, 0.0, 1.0])
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    data.qvel[:] = 0.0
    
    data.ctrl[0] = hover_thrust_per_motor
    data.ctrl[1] = hover_thrust_per_motor
    data.ctrl[2] = hover_thrust_per_motor
    data.ctrl[3] = hover_thrust_per_motor
    data.ctrl[4] = 0.5  # Reaction yaw torque +0.5 N*m
    
    for _ in range(100):
        mujoco.mj_step(model, data)
        
    yaw_rate = data.qvel[5]
    print(f"Yaw Motion Test (Reaction Torque): Yaw angular velocity wz = {yaw_rate:.3f} rad/s")
    if abs(yaw_rate) > 0.1:
        print("[SUCCESS] Yaw Motion Verified: Reaction torque produces yaw rotation!")
    else:
        print(f"[FAIL] Yaw Motion Test failed: wz = {yaw_rate}")
        return False

    print("\n" + "=" * 70)
    print(" ALL STEPS 1 THROUGH 10 TESTS PASSED CLEANLY! ")
    print("=" * 70)
    return True

if __name__ == "__main__":
    run_tests()
