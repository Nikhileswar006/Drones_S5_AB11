"""
Mathematical Utilities & Paper Equations (1)-(23) Implementation
================================================================
Paper: "Quaternion-Based Attitude Tracking Control Design for UAVs"
Authors: Qain-Rong Lin, Jen-te Yu (CACS 2024)

Quaternion Convention in the Paper (Eq. 1-2):
    q = [q0, q1, q2, q3]^T
where:
    q0       : scalar part (cos(theta/2))
    q1,q2,q3 : vector part (sin(theta/2) * axis)
"""

import numpy as np
from typing import Tuple


# ==============================================================================
# Section II: Preliminaries for Quaternions (Equations 1 - 7)
# ==============================================================================

def quat_conj(q: np.ndarray) -> np.ndarray:
    """
    Equation (3): Conjugate of a quaternion.
    Conj(q) = q^* = [q0, -q1, -q2, -q3]^T
    """
    q = np.asarray(q, dtype=float).flatten()
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quat_norm(q: np.ndarray) -> float:
    """
    Equation (4): Norm (length) of a quaternion.
    Norm(q) = ||q|| = sqrt(q0^2 + q1^2 + q2^2 + q3^2)
    """
    q = np.asarray(q, dtype=float).flatten()
    return float(np.sqrt(np.dot(q, q)))


def quat_inv(q: np.ndarray) -> np.ndarray:
    """
    Equation (5): Inverse of a quaternion.
    Inv(q) = q^(-1) = q^* / ||q||^2
    """
    n2 = np.dot(q, q)
    if n2 < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return quat_conj(q) / n2


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """
    Equation (6): Normalization of a quaternion.
    Normalize(q) = q / ||q||
    """
    n = quat_norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return np.asarray(q, dtype=float).flatten() / n


def quat_multiply(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    """
    Equation (7): Non-commutative Hamilton multiplication of two quaternions p (x) q.
    p (x) q = [ p0*q0 - p1*q1 - p2*q2 - p3*q3 ]
              [ p0*q1 + p1*q0 + p2*q3 - p3*q2 ]
              [ p0*q2 - p1*q3 + p2*q0 + p3*q1 ]
              [ p0*q3 + p1*q2 - p2*q1 + p3*q0 ]
    """
    p0, p1, p2, p3 = np.asarray(p, dtype=float).flatten()
    q0, q1, q2, q3 = np.asarray(q, dtype=float).flatten()
    return np.array([
        p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3,
        p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2,
        p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1,
        p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0
    ], dtype=float)


# ==============================================================================
# Section III: Attitude Dynamics of UAVs (Equations 8 - 9)
# ==============================================================================

def quat_derivative(q: np.ndarray, omega: np.ndarray) -> np.ndarray:
    """
    Equation (8): Quaternion differential equation.
    q_dot = 0.5 * q (x) [0; omega]
    """
    omega_quat = np.array([0.0, omega[0], omega[1], omega[2]], dtype=float)
    return 0.5 * quat_multiply(q, omega_quat)


def omega_derivative(omega: np.ndarray, tau: np.ndarray, J: np.ndarray, J_inv: np.ndarray) -> np.ndarray:
    """
    Equation (9): Angular velocity differential equation.
    omega_dot = J^(-1) * [tau - omega x (J * omega)]
    """
    omega = np.asarray(omega, dtype=float).flatten()
    tau = np.asarray(tau, dtype=float).flatten()
    coriolis = np.cross(omega, J @ omega)
    return J_inv @ (tau - coriolis)


# ==============================================================================
# Section IV: Paper Controller Design (Equations 10 - 23)
# ==============================================================================

def fractional_power(vec: np.ndarray, exponent: float = 1.0 / 3.0) -> np.ndarray:
    """
    Real-valued signed fractional exponent used in Equation (22) & (23):
    delta^(1/3) = sign(delta) * |delta|^(1/3) component-wise.
    """
    vec = np.asarray(vec, dtype=float).flatten()
    return np.sign(vec) * (np.abs(vec) ** exponent)


class PaperQuaternionBacksteppingController:
    """
    Exact implementation of Section IV of Q.-R. Lin and J.-t. Yu (CACS 2024):
    "Quaternion-Based Attitude Tracking Control Design for UAVs"
    """

    def __init__(
        self,
        J_diag: Tuple[float, float, float] = (0.1, 0.1, 0.12),
        k1: float = 20.0,
        k2: float = 2.0,
        exponent: float = 1.0 / 3.0,
        enforce_shortest_path: bool = True
    ):
        self.J = np.diag(np.asarray(J_diag, dtype=float))
        self.J_inv = np.diag(1.0 / np.asarray(J_diag, dtype=float))
        self.k1 = float(k1)
        self.k2 = float(k2)
        self.exponent = float(exponent)
        self.enforce_shortest_path = enforce_shortest_path
        self.identity_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    def compute_control(
        self,
        q: np.ndarray,
        omega: np.ndarray,
        q_ref: np.ndarray,
        omega_ref: np.ndarray,
        omega_dot_ref: np.ndarray
    ) -> dict:
        """
        Executes Equations (10) through (22) to compute the control torque tau.

        Parameters
        ----------
        q             : Current UAV unit quaternion [q0, q1, q2, q3]
        omega         : Current UAV body angular velocity [wx, wy, wz] (rad/s)
        q_ref         : Desired unit quaternion [q_ref0, q_ref1, q_ref2, q_ref3]
        omega_ref     : Desired angular velocity [w_ref_x, w_ref_y, w_ref_z] (rad/s)
        omega_dot_ref : Desired angular acceleration [dw_ref_x, dw_ref_y, dw_ref_z] (rad/s^2)

        Returns
        -------
        dict containing:
            tau          : Total 3D control torque [tau_1, tau_2, tau_3] (N*m)
            tau1         : Feedforward + nonlinear cancellation torque (Eq. 21)
            tau2         : Fractional backstepping stabilizing torque (Eq. 22)
            q_err        : Quaternion tracking error (Eq. 10)
            omega_err    : Angular velocity tracking error (Eq. 11)
            omega_err_d  : Virtual control desired angular velocity error (Eq. 13)
            omega_dot_err_d : Time derivative of virtual control (Eq. 20)
            delta        : Backstepping second error variable (Eq. 16)
            q_ref_aligned: Hemisphere-aligned reference quaternion
        """
        q = quat_normalize(q)
        q_ref = quat_normalize(q_ref)
        omega = np.asarray(omega, dtype=float).flatten()
        omega_ref = np.asarray(omega_ref, dtype=float).flatten()
        omega_dot_ref = np.asarray(omega_dot_ref, dtype=float).flatten()

        # Hemisphere continuity check (prevents unwinding in 3D flight)
        if self.enforce_shortest_path and np.dot(q_ref, q) < 0.0:
            q_ref = -q_ref

        # Equation (10): Quaternion tracking error
        # q_err = q_ref^* (x) q
        q_err = quat_multiply(quat_conj(q_ref), q)
        q_err = quat_normalize(q_err)

        # Equation (11): Angular velocity error
        # omega_err = omega - omega_ref
        omega_err = omega - omega_ref

        # Equation (12): Quaternion error dynamics
        # q_dot_err = 0.5 * q_err (x) [0; omega_err]
        q_dot_err = quat_derivative(q_err, omega_err)

        # Equation (13): Designed virtual control omega_err_d
        # [0; omega_err_d] = -k1 * q_err^* (x) (q_err - [1; 0; 0; 0])
        q_err_minus_I = q_err - self.identity_quat
        virtual_quat = -self.k1 * quat_multiply(quat_conj(q_err), q_err_minus_I)
        omega_err_d = virtual_quat[1:4]

        # Equation (16): Backstepping error variable delta
        # delta = omega_err - omega_err_d
        delta = omega_err - omega_err_d

        # Equation (20): Time derivative of virtual control omega_dot_err_d
        # [0; omega_dot_err_d] = d/dt { -k1 * q_err^* (x) (q_err - [1; 0; 0; 0]) }
        # Using product rule with q_dot_err from Eq. (12):
        d_virtual_quat = -self.k1 * (
            quat_multiply(quat_conj(q_dot_err), q_err_minus_I)
            + quat_multiply(quat_conj(q_err), q_dot_err)
        )
        omega_dot_err_d = d_virtual_quat[1:4]

        # Equation (21): tau_1 eliminates known nonlinearities and tracks derivatives
        # tau_1 = omega x (J * omega) + J * omega_dot_ref + J * omega_dot_err_d
        tau1 = (
            np.cross(omega, self.J @ omega)
            + (self.J @ omega_dot_ref)
            + (self.J @ omega_dot_err_d)
        )

        # Equation (22): tau_2 ensures finite-time convergence of delta via fractional power 1/3
        # tau_2 = -k2 * J * delta^(1/3)
        tau2 = -self.k2 * (self.J @ fractional_power(delta, self.exponent))

        # Total control torque: tau = tau_1 + tau_2
        tau = tau1 + tau2

        return {
            'tau': tau,
            'tau1': tau1,
            'tau2': tau2,
            'q_err': q_err,
            'omega_err': omega_err,
            'omega_err_d': omega_err_d,
            'omega_dot_err_d': omega_dot_err_d,
            'delta': delta,
            'q_ref_aligned': q_ref,
        }


# ==============================================================================
# Conversions between Quaternions, Rotation Matrices, and Euler Angles
# ==============================================================================

def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """
    Converts unit quaternion q = [q0, q1, q2, q3] to 3x3 rotation matrix R in SO(3)
    mapping body frame vectors to inertial frame vectors.
    """
    q0, q1, q2, q3 = quat_normalize(q)
    return np.array([
        [1.0 - 2.0 * (q2 * q2 + q3 * q3), 2.0 * (q1 * q2 - q0 * q3),       2.0 * (q1 * q3 + q0 * q2)],
        [2.0 * (q1 * q2 + q0 * q3),       1.0 - 2.0 * (q1 * q1 + q3 * q3), 2.0 * (q2 * q3 - q0 * q1)],
        [2.0 * (q1 * q3 - q0 * q2),       2.0 * (q2 * q3 + q0 * q1),       1.0 - 2.0 * (q1 * q1 + q2 * q2)]
    ], dtype=float)


def rotmat_to_quat(R: np.ndarray) -> np.ndarray:
    """
    Converts a 3x3 rotation matrix R in SO(3) to a unit quaternion q = [q0, q1, q2, q3].
    """
    tr = np.trace(R)
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        q0 = 0.25 * S
        q1 = (R[2, 1] - R[1, 2]) / S
        q2 = (R[0, 2] - R[2, 0]) / S
        q3 = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q0 = (R[2, 1] - R[1, 2]) / S
        q1 = 0.25 * S
        q2 = (R[0, 1] + R[1, 0]) / S
        q3 = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q0 = (R[0, 2] - R[2, 0]) / S
        q1 = (R[0, 1] + R[1, 0]) / S
        q2 = 0.25 * S
        q3 = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q0 = (R[1, 0] - R[0, 1]) / S
        q1 = (R[0, 2] + R[2, 0]) / S
        q2 = (R[1, 2] + R[2, 1]) / S
        q3 = 0.25 * S
    q = np.array([q0, q1, q2, q3], dtype=float)
    if q[0] < 0.0:
        q = -q
    return quat_normalize(q)


def quat_to_euler(q: np.ndarray, degrees: bool = False) -> Tuple[float, float, float]:
    """
    Converts unit quaternion q = [q0, q1, q2, q3] to Euler angles (roll, pitch, yaw)
    following the ZYX intrinsic convention (matching MATLAB quat2eul used in Section V).
    Returns (roll, pitch, yaw).
    """
    q0, q1, q2, q3 = quat_normalize(q)

    # Roll (phi - x-axis rotation)
    sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (theta - y-axis rotation)
    sinp = 2.0 * (q0 * q2 - q3 * q1)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # Yaw (psi - z-axis rotation)
    siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    if degrees:
        return float(np.degrees(roll)), float(np.degrees(pitch)), float(np.degrees(yaw))
    return float(roll), float(pitch), float(yaw)


def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Converts Euler angles (roll, pitch, yaw) in radians to unit quaternion [q0, q1, q2, q3].
    """
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    q0 = cr * cp * cy + sr * sp * sy
    q1 = sr * cp * cy - cr * sp * sy
    q2 = cr * sp * cy + sr * cp * sy
    q3 = cr * cp * sy - sr * sp * cy
    return quat_normalize(np.array([q0, q1, q2, q3], dtype=float))


def desired_acc_to_quaternion(
    a_des: np.ndarray,
    yaw_des: float = 0.0,
    max_tilt_rad: float = np.radians(35.0)
) -> np.ndarray:
    """
    Maps a desired 3D inertial acceleration vector a_des = [ax, ay, az + g]^T and desired yaw angle
    psi_ref into the desired attitude quaternion q_ref = [q0, q1, q2, q3]^T for the quadrotor
    (Figure 1 of the paper).
    """
    a_des = np.asarray(a_des, dtype=float).flatten()
    if a_des[2] < 1.0:
        a_des[2] = 1.0

    # Limit horizontal tilt so the quadrotor never flips
    horiz_norm = np.hypot(a_des[0], a_des[1])
    max_horiz = a_des[2] * np.tan(max_tilt_rad)
    if horiz_norm > max_horiz and horiz_norm > 1e-8:
        scale = max_horiz / horiz_norm
        a_des[0] *= scale
        a_des[1] *= scale

    norm_a = np.linalg.norm(a_des)
    if norm_a < 1e-8:
        return euler_to_quat(0.0, 0.0, yaw_des)

    # Desired body Z axis b_z points along total thrust vector
    b_z = a_des / norm_a

    # Intermediate heading vector c_x in horizontal plane
    c_x = np.array([np.cos(yaw_des), np.sin(yaw_des), 0.0], dtype=float)

    # Desired body Y axis b_y = (b_z x c_x) / ||b_z x c_x||
    b_y = np.cross(b_z, c_x)
    norm_by = np.linalg.norm(b_y)
    if norm_by < 1e-8:
        b_y = np.array([0.0, 1.0, 0.0], dtype=float)
    else:
        b_y = b_y / norm_by

    # Desired body X axis b_x = b_y x b_z
    b_x = np.cross(b_y, b_z)

    R_des = np.column_stack((b_x, b_y, b_z))
    return rotmat_to_quat(R_des)
