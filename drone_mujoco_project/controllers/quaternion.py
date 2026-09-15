import numpy as np

def quaternion_normalize(q: np.ndarray) -> np.ndarray:
    """
    Step 12.1: Normalize quaternion q / ||q||.
    Hamilton convention [q_w, q_x, q_y, q_z].
    """
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm

def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    """
    Step 12.2: Conjugate q* = [q_w, -q_x, -q_y, -q_z].
    """
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quaternion_multiply(q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """
    Step 12.3: Quaternion product q1 (x) q2 in Hamilton convention.
    """
    qw, qx, qy, qz = q
    rw, rx, ry, rz = r
    
    w = qw*rw - qx*rx - qy*ry - qz*rz
    x = qw*rx + qx*rw + qy*rz - qz*ry
    y = qw*ry - qx*rz + qy*rw + qz*rx
    z = qw*rz + qx*ry - qy*rx + qz*rw
    
    return np.array([w, x, y, z])

def quaternion_inverse(q: np.ndarray) -> np.ndarray:
    """
    Step 12.4: Quaternion inverse q^-1 = q* / ||q||^2.
    """
    q_norm_sq = np.dot(q, q)
    if q_norm_sq < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return quaternion_conjugate(q) / q_norm_sq

def quaternion_error(q_ref: np.ndarray, q_actual: np.ndarray) -> np.ndarray:
    """
    Step 12.5 & Base Paper Eq. 10: Quaternion Error q_err = q_ref* (x) q_actual.
    Ensures shortest rotation path (unwinding prevention).
    """
    q_ref_conj = quaternion_conjugate(q_ref)
    q_err = quaternion_multiply(q_ref_conj, q_actual)
    q_err = quaternion_normalize(q_err)
    
    if q_err[0] < 0.0:
        q_err = -q_err
        
    return q_err

def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Convert Hamilton quaternion [q_w, q_x, q_y, q_z] to 3x3 Rotation Matrix R."""
    q = quaternion_normalize(q)
    w, x, y, z = q
    
    R = np.array([
        [1.0 - 2.0*(y**2 + z**2), 2.0*(x*y - w*z),     2.0*(x*z + w*y)],
        [2.0*(x*y + w*z),     1.0 - 2.0*(x**2 + z**2), 2.0*(y*z - w*x)],
        [2.0*(x*z - w*y),     2.0*(y*z + w*x),     1.0 - 2.0*(x**2 + y**2)]
    ])
    return R

def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 Rotation Matrix R to Hamilton quaternion [q_w, q_x, q_y, q_z]."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S

    q = np.array([w, x, y, z])
    return quaternion_normalize(q)

def quat_to_euler(q: np.ndarray) -> np.ndarray:
    """Convert Hamilton quaternion [q_w, q_x, q_y, q_z] to Euler angles [roll, pitch, yaw] in radians."""
    q = quaternion_normalize(q)
    w, x, y, z = q
    
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x**2 + y**2))
    sin_pitch = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sin_pitch)
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))
    
    return np.array([roll, pitch, yaw])
