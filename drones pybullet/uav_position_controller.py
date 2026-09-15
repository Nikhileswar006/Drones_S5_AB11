"""
Outer-Loop UAV 3D Position Controller
Converts 3D position/velocity/acceleration tracking errors into:
1. Total collective thrust T
2. Reference attitude quaternion q_ref (scalar-first)
3. Feedforward reference angular velocity omega_ref and acceleration dot(omega_ref)

Designed to work seamlessly with the Quaternion Backstepping Attitude Controller.
"""

import numpy as np
from uav_quaternion_backstepping import QuaternionMath


class UAVPositionController:
    """
    Cascaded Position Controller for 3D trajectory tracking.
    Uses differential flatness of quadrotor dynamics on SE(3).
    """

    def __init__(self,
                 mass: float = 1.0,
                 gravity: float = 9.81,
                 Kp_pos: np.ndarray = None,
                 Kd_pos: np.ndarray = None,
                 Ki_pos: np.ndarray = None,
                 max_thrust: float = 30.0,
                 max_tilt_deg: float = 45.0):
        self.mass = float(mass)
        self.g = float(gravity)

        # Default gains tuned for smooth, robust 3D tracking
        self.Kp = np.array([6.5, 6.5, 9.0]) if Kp_pos is None else np.array(Kp_pos, dtype=np.float64)
        self.Kd = np.array([4.0, 4.0, 5.5]) if Kd_pos is None else np.array(Kd_pos, dtype=np.float64)
        self.Ki = np.array([0.2, 0.2, 0.5]) if Ki_pos is None else np.array(Ki_pos, dtype=np.float64)

        self.max_thrust = float(max_thrust)
        self.max_tilt_rad = np.deg2rad(max_tilt_deg)

        self.pos_integral = np.zeros(3, dtype=np.float64)
        self.last_q_ref = None
        self.last_omega_ref = None
        self.last_time = None

    def reset(self):
        self.pos_integral = np.zeros(3, dtype=np.float64)
        self.last_q_ref = None
        self.last_omega_ref = None
        self.last_time = None

    def compute_position_control(self,
                                 pos: np.ndarray,
                                 vel: np.ndarray,
                                 q_current: np.ndarray,
                                 target_pos: np.ndarray,
                                 target_vel: np.ndarray,
                                 target_acc: np.ndarray,
                                 target_yaw: float = 0.0,
                                 dt: float = 0.004) -> dict:
        """
        Computes thrust and reference orientation from position tracking error.

        Args:
            pos: Current 3D position [x, y, z] (m)
            vel: Current 3D velocity [vx, vy, vz] (m/s)
            q_current: Current quaternion [q0, q1, q2, q3] (scalar first)
            target_pos: Desired position [xd, yd, zd] (m)
            target_vel: Desired velocity [vxd, vyd, vzd] (m/s)
            target_acc: Desired acceleration [axd, ayd, azd] (m/s^2)
            target_yaw: Desired yaw angle (rad)
            dt: Timestep (s)

        Returns:
            dict containing:
                thrust: Total collective thrust magnitude (N)
                q_ref: Desired quaternion [q0, q1, q2, q3] (scalar first)
                omega_ref: Desired angular velocity [p, q, r] (rad/s)
                d_omega_ref: Desired angular acceleration [dp, dq, dr] (rad/s^2)
                pos_error: Position tracking error [ex, ey, ez] (m)
        """
        pos_error = target_pos - pos
        vel_error = target_vel - vel

        # Update integral error with anti-windup clamping
        if dt > 0:
            self.pos_integral += pos_error * dt
            self.pos_integral = np.clip(self.pos_integral, -2.0, 2.0)

        # Desired virtual acceleration:
        # a_des = a_d + Kp*e_p + Kd*e_v + Ki*int(e_p) + g * [0, 0, 1]
        a_des = (target_acc +
                 self.Kp * pos_error +
                 self.Kd * vel_error +
                 self.Ki * self.pos_integral +
                 np.array([0.0, 0.0, self.g]))

        # Limit maximum tilt to maintain stability
        # a_des_xy / a_des_z <= tan(max_tilt)
        a_z = max(a_des[2], 1.0)
        a_xy_norm = np.linalg.norm(a_des[:2])
        max_xy = a_z * np.tan(self.max_tilt_rad)
        if a_xy_norm > max_xy:
            a_des[:2] = a_des[:2] * (max_xy / a_xy_norm)

        # Desired body z-axis points along desired acceleration
        z_b_des = a_des / np.linalg.norm(a_des)

        # Construct desired body x and y axes with desired yaw
        x_c = np.array([np.cos(target_yaw), np.sin(target_yaw), 0.0])
        y_b_des = np.cross(z_b_des, x_c)
        norm_yb = np.linalg.norm(y_b_des)
        if norm_yb < 1e-4:
            # Singularity (pointing straight up/down)
            y_b_des = np.array([0.0, 1.0, 0.0])
        else:
            y_b_des = y_b_des / norm_yb

        x_b_des = np.cross(y_b_des, z_b_des)

        R_des = np.column_stack([x_b_des, y_b_des, z_b_des])
        q_ref = QuaternionMath.from_rotation_matrix(R_des)

        # Compute collective thrust T:
        # Current body z-axis in world frame:
        R_curr = QuaternionMath.to_rotation_matrix(q_current)
        z_b_curr = R_curr[:, 2]
        thrust = self.mass * float(np.dot(a_des, z_b_curr))
        thrust = np.clip(thrust, 0.0, self.max_thrust)

        # Compute feedforward omega_ref via quaternion derivative:
        # dot(q_ref) = 0.5 * q_ref (x) [0; omega_ref]
        # => [0; omega_ref] = 2 * q_ref* (x) dot(q_ref)
        if self.last_q_ref is not None and dt > 1e-5:
            # Ensure continuity across antipodal representation
            if np.dot(q_ref, self.last_q_ref) < 0.0:
                q_ref = -q_ref

            dot_q_ref = (q_ref - self.last_q_ref) / dt
            q_ref_star = QuaternionMath.conjugate(q_ref)
            w_quat = 2.0 * QuaternionMath.multiply(q_ref_star, dot_q_ref)
            omega_ref = w_quat[1:4]

            # Angular acceleration feedforward
            if self.last_omega_ref is not None:
                d_omega_ref = (omega_ref - self.last_omega_ref) / dt
            else:
                d_omega_ref = np.zeros(3)
        else:
            omega_ref = np.zeros(3)
            d_omega_ref = np.zeros(3)

        self.last_q_ref = q_ref.copy()
        self.last_omega_ref = omega_ref.copy()

        return {
            'thrust': thrust,
            'q_ref': q_ref,
            'omega_ref': omega_ref,
            'd_omega_ref': d_omega_ref,
            'pos_error': pos_error,
            'R_des': R_des
        }
