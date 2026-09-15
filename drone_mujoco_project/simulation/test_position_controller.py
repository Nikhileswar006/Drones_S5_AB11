import numpy as np
import os
import sys

# Base Path Setup
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE_DIR)

from controllers.position_controller import PositionController
from controllers.quaternion import quat_to_euler

def run_step30_position_controller_test():
    print("=" * 70)
    print(" STEPS 23-30: DESTINATION POSITION CONTROLLER UNIT TEST ")
    print("=" * 70)

    # Step 23: Start Position P0 = [0, 0, 1]
    P0 = np.array([0.0, 0.0, 1.0])
    V0 = np.array([0.0, 0.0, 0.0])

    # Step 24: Target Destination Pd = [5, 5, 3]
    Pd = np.array([5.0, 5.0, 3.0])

    pos_controller = PositionController(mass=1.0, gravity=9.81)

    # Step 25-30 Execution
    T_desired, q_ref, e_p, a_cmd = pos_controller.compute(
        current_pos=P0, current_vel=V0, target_pos=Pd, target_yaw=0.0
    )

    euler_ref_deg = np.degrees(quat_to_euler(q_ref))

    print(f"Step 23 Start Position P0   : {P0}")
    print(f"Step 24 Target Destination Pd: {Pd}")
    print(f"Step 26 Position Error e_p  : {e_p}")
    print(f"Step 27 Acceleration Cmd a  : {a_cmd.round(3)} m/s^2")
    print(f"Step 28 Desired Thrust T    : {T_desired:.3f} N (Hover = 9.81 N)")
    print(f"Step 29 Desired Euler Angles: Roll={euler_ref_deg[0]:.2f} deg, Pitch={euler_ref_deg[1]:.2f} deg, Yaw={euler_ref_deg[2]:.2f} deg")
    print(f"Step 30 Reference Quaternion: {q_ref.round(4)}")

    # Verification assertions
    assert np.allclose(e_p, np.array([5.0, 5.0, 2.0])), "[FAIL] Step 26 Error calculation failed"
    assert T_desired > 9.81, "[FAIL] Step 28 Thrust calculation failed"
    assert euler_ref_deg[1] > 0.0, "[FAIL] Step 29 Pitch inclination toward +X destination failed"
    assert euler_ref_deg[0] < 0.0, "[FAIL] Step 29 Roll inclination toward +Y destination failed"

    print("\n" + "=" * 70)
    print(" [SUCCESS] STEPS 23-30 DESTINATION CONTROLLER UNIT TEST PASSED! ")
    print(" Destination error e_p successfully mapped to thrust T and quaternion q_ref!")
    print("=" * 70)
    return True

if __name__ == "__main__":
    run_step30_position_controller_test()
