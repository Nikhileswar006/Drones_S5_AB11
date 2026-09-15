"""
Quaternion Mathematics and Attitude Backstepping Controller
"Quaternion-Based Attitude Tracking Control Design for UAVs"
Equations (1) - (23)
"""

import numpy as np


class QuaternionMath:
    """
    Implements Quaternion algebra strictly following the paper.
    Convention: q = [q0, q1, q2, q3]^T where q0 is the scalar part and
    [q1, q2, q3] is the vector part (scalar-first).
    """

    @staticmethod
    def norm(q: np.ndarray) -> float:
        """Eq. (4): ||q|| = sqrt(q0^2 + q1^2 + q2^2 + q3^2)"""
        return float(np.linalg.norm(q))

    @staticmethod
    def normalize(q: np.ndarray) -> np.ndarray:
        """Eq. (6): Normalize(q) = q / ||q||"""
        n = np.linalg.norm(q)
        if n < 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return q / n

    @staticmethod
    def conjugate(q: np.ndarray) -> np.ndarray:
        """Eq. (3): q* = [q0, -q1, -q2, -q3]^T"""
        return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)

    @staticmethod
    def inverse(q: np.ndarray) -> np.ndarray:
        """Eq. (5): Inv(q) = q* / ||q||^2"""
        n_sq = float(np.dot(q, q))
        if n_sq < 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return QuaternionMath.conjugate(q) / n_sq

    @staticmethod
    def multiply(p: np.ndarray, q: np.ndarray) -> np.ndarray:
        """
        Eq. (7): Quaternion multiplication p (x) q
        [ p0*q0 - p1*q1 - p2*q2 - p3*q3 ]
        [ p0*q1 + p1*q0 + p2*q3 - p3*q2 ]
        [ p0*q2 - p1*q3 + p2*q0 + p3*q1 ]
        [ p0*q3 + p1*q2 - p2*q1 + p3*q0 ]
        """
        p0, p1, p2, p3 = p[0], p[1], p[2], p[3]
        q0, q1, q2, q3 = q[0], q[1], q[2], q[3]

        return np.array([
            p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3,
            p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2,
            p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1,
            p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0
        ], dtype=np.float64)

    @staticmethod
    def to_rotation_matrix(q: np.ndarray) -> np.ndarray:
        """
        Converts normalized quaternion [q0, q1, q2, q3] (scalar first)
        to standard 3x3 rotation matrix R.
        """
        q = QuaternionMath.normalize(q)
        q0, q1, q2, q3 = q

        r00 = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        r01 = 2.0 * (q1 * q2 - q0 * q3)
        r02 = 2.0 * (q1 * q3 + q0 * q2)

        r10 = 2.0 * (q1 * q2 + q0 * q3)
        r11 = 1.0 - 2.0 * (q1 * q1 + q3 * q3)
        r12 = 2.0 * (q2 * q3 - q0 * q1)

        r20 = 2.0 * (q1 * q3 - q0 * q2)
        r21 = 2.0 * (q2 * q3 + q0 * q1)
        r22 = 1.0 - 2.0 * (q1 * q1 + q2 * q2)

        return np.array([
            [r00, r01, r02],
            [r10, r11, r12],
            [r20, r21, r22]
        ], dtype=np.float64)

    @staticmethod
    def from_rotation_matrix(R: np.ndarray) -> np.ndarray:
        """
        Converts 3x3 rotation matrix to quaternion [q0, q1, q2, q3] (scalar first).
        """
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0:
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

        q = np.array([q0, q1, q2, q3], dtype=np.float64)
        return QuaternionMath.normalize(q)

    @staticmethod
    def to_euler_deg(q: np.ndarray) -> np.ndarray:
        """
        Converts scalar-first quaternion to Euler angles [yaw, pitch, roll] in degrees,
        matching Figures 4, 5, 6 of the paper.
        """
        q = QuaternionMath.normalize(q)
        q0, q1, q2, q3 = q

        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1.0:
            pitch = np.copysign(np.pi / 2.0, sinp)
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.rad2deg(np.array([yaw, pitch, roll]))


class QuaternionBacksteppingController:
    """
    Quaternion-Based Attitude Tracking Backstepping Controller
    """

    def __init__(self,
                 J: np.ndarray = None,
                 k1: float = 20.0,
                 k2: float = 2.0):
        """
        Parameters from Section V of the paper:
        - J = diag([0.1, 0.1, 0.12]) (kg*m^2)
        - k1 = 20.0
        - k2 = 2.0
        """
        if J is None:
            self.J = np.diag([0.1, 0.1, 0.12])
        else:
            self.J = np.array(J, dtype=np.float64)

        self.J_inv = np.linalg.inv(self.J)
        self.k1 = float(k1)
        self.k2 = float(k2)

        # State tracking for derivatives
        self.last_w_err_d = None
        self.last_time = None

    def reset(self):
        self.last_w_err_d = None
        self.last_time = None

    def compute_control(self,
                        q: np.ndarray,
                        omega: np.ndarray,
                        q_ref: np.ndarray,
                        omega_ref: np.ndarray,
                        d_omega_ref: np.ndarray = None,
                        dt: float = None) -> dict:
        """
        Equations (10) - (23).

        Args:
            q: Current attitude quaternion [q0, q1, q2, q3] (scalar first)
            omega: Current body angular velocity [p, q, r] (rad/s)
            q_ref: Desired attitude quaternion [q0, q1, q2, q3] (scalar first)
            omega_ref: Desired angular velocity [p_ref, q_ref, r_ref] (rad/s)
            d_omega_ref: Time derivative of desired angular velocity (rad/s^2), defaults to zeros
            dt: Timestep (s) for finite difference fallback if needed

        Returns:
            dict containing:
                tau: Total control torque vector [tau_x, tau_y, tau_z] (N*m)
                tau1: Elimination torque (Eq. 21)
                tau2: Fractional finite-time torque (Eq. 22)
                q_err: Quaternion error vector (Eq. 10)
                omega_err: Angular velocity error (Eq. 11)
                omega_err_d: Virtual control vector (Eq. 13)
                delta: Auxiliary error variable (Eq. 16)
        """
        q = QuaternionMath.normalize(q)
        q_ref = QuaternionMath.normalize(q_ref)

        # Handle quaternion double cover: if dot(q, q_ref) < 0, flip q_ref to prevent unwinding
        if np.dot(q, q_ref) < 0.0:
            q_ref = -q_ref

        if d_omega_ref is None:
            d_omega_ref = np.zeros(3, dtype=np.float64)

        # Eq. (10): q_err = q_ref* (x) q
        q_ref_star = QuaternionMath.conjugate(q_ref)
        q_err = QuaternionMath.multiply(q_ref_star, q)

        # Eq. (11): omega_err = omega - omega_ref
        omega_err = omega - omega_ref

        # Eq. (12): dot(q_err) = 0.5 * q_err (x) [0; omega_err]
        omega_err_quat = np.array([0.0, omega_err[0], omega_err[1], omega_err[2]])
        dot_q_err = 0.5 * QuaternionMath.multiply(q_err, omega_err_quat)

        # Eq. (13): [0; omega_err_d] = -k1 * q_err* (x) (q_err - [1; 0; 0; 0])
        # Vector components are: omega_err_d = -k1 * q_err[1:4]
        identity_quat = np.array([1.0, 0.0, 0.0, 0.0])
        err_diff = q_err - identity_quat
        q_err_star = QuaternionMath.conjugate(q_err)
        virtual_full = -self.k1 * QuaternionMath.multiply(q_err_star, err_diff)
        omega_err_d = virtual_full[1:4]

        # Eq. (16): delta = omega_err - omega_err_d
        delta = omega_err - omega_err_d

        # Eq. (20): [0; dot(omega_err_d)] = d/dt { -k1 * q_err* (x) (q_err - [1; 0; 0; 0]) }
        # Analytically: dot(omega_err_d) = -k1 * dot_q_err[1:4]
        dot_omega_err_d = -self.k1 * dot_q_err[1:4]

        self.last_w_err_d = omega_err_d.copy()

        # Eq. (21): tau1 = omega x (J * omega) + J * dot(omega_ref) + J * dot(omega_err_d)
        gyroscopic_term = np.cross(omega, self.J @ omega)
        tau1 = gyroscopic_term + self.J @ d_omega_ref + self.J @ dot_omega_err_d

        # Eq. (22): tau2 = -k2 * J * delta^(1/3)
        # Fractional exponent 1/3 evaluated with sign preservation: sign(delta) * |delta|^(1/3)
        delta_frac = np.sign(delta) * (np.abs(delta) ** (1.0 / 3.0))
        tau2 = -self.k2 * (self.J @ delta_frac)

        # Total control torque
        tau = tau1 + tau2

        return {
            'tau': tau,
            'tau1': tau1,
            'tau2': tau2,
            'q_err': q_err,
            'omega_err': omega_err,
            'omega_err_d': omega_err_d,
            'delta': delta
        }
