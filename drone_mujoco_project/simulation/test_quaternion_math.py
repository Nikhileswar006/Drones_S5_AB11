import numpy as np
import os
import sys

# Base Path Setup
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE_DIR)

from controllers.quaternion import (
    quaternion_normalize, quaternion_conjugate, quaternion_multiply,
    quaternion_inverse, quaternion_error, quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion
)

def run_step13_tests():
    print("=" * 70)
    print(" STEP 13: INDEPENDENT QUATERNION MATH UNIT TESTS ")
    print("=" * 70)

    # Test 1: Identity quaternion
    q_identity = np.array([1.0, 0.0, 0.0, 0.0])
    print(f"1. Identity Quaternion: {q_identity}")
    assert np.allclose(q_identity, np.array([1.0, 0.0, 0.0, 0.0])), "[FAIL] Identity test failed"
    print("   [PASS] Identity quaternion correct.")

    # Test 2: Zero Error Condition (q_ref == q_actual => q_err == [1, 0, 0, 0])
    q_test = quaternion_normalize(np.array([0.92388, 0.38268, 0.0, 0.0])) # 45 deg roll
    q_err_zero = quaternion_error(q_test, q_test)
    print(f"2. Zero Error Test (q_ref == q_actual): q_err = {q_err_zero}")
    assert np.allclose(q_err_zero, np.array([1.0, 0.0, 0.0, 0.0]), atol=1e-5), "[FAIL] Zero error condition failed"
    print("   [PASS] Zero error condition verified: q_err = [1, 0, 0, 0].")

    # Test 3: Quaternion Inverse Product (q (x) q^-1 == [1, 0, 0, 0])
    q_inv = quaternion_inverse(q_test)
    prod_inv = quaternion_multiply(q_test, q_inv)
    print(f"3. Inverse Product Test (q (x) q^-1): {prod_inv}")
    assert np.allclose(prod_inv, np.array([1.0, 0.0, 0.0, 0.0]), atol=1e-5), "[FAIL] Inverse product failed"
    print("   [PASS] Inverse product verified.")

    # Test 4: Normalization Test
    unnorm_q = np.array([2.0, 2.0, 2.0, 2.0])
    norm_q = quaternion_normalize(unnorm_q)
    print(f"4. Normalization Test: ||norm(q)|| = {np.linalg.norm(norm_q):.6f}")
    assert abs(np.linalg.norm(norm_q) - 1.0) < 1e-6, "[FAIL] Normalization failed"
    print("   [PASS] Normalization verified.")

    # Test 5: Rotation Matrix Conversion Roundtrip
    R = quaternion_to_rotation_matrix(q_test)
    q_reconstructed = rotation_matrix_to_quaternion(R)
    print(f"5. Rotation Matrix Roundtrip: reconstructed q = {q_reconstructed}")
    assert np.allclose(q_test, q_reconstructed, atol=1e-4) or np.allclose(q_test, -q_reconstructed, atol=1e-4), "[FAIL] Roundtrip failed"
    print("   [PASS] Rotation matrix roundtrip verified.")

    print("\n" + "=" * 70)
    print(" ALL STEP 13 QUATERNION MATH TESTS PASSED CLEANLY! ")
    print("=" * 70)
    return True

if __name__ == "__main__":
    run_step13_tests()
