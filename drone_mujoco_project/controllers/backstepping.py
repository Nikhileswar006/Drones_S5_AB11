import numpy as np
from typing import Tuple
from controllers.quaternion import quaternion_error

class BacksteppingController:
    """
    Steps 14-19: Base Paper Attitude Backstepping Controller (CACS 2024).
    
    Inputs:
      q        : Current quaternion [qw, qx, qy, qz]
      q_ref    : Desired quaternion [qw, qx, qy, qz]
      omega    : Current angular velocity [wx, wy, wz]
      omega_ref: Desired angular velocity [wx, wy, wz]
      
    Outputs:
      tau      : Control torques [tau_x, tau_y, tau_z]
    """
    def __init__(self, J: np.ndarray = np.diag([0.1, 0.1, 0.12]), k1: float = 20.0, k2: float = 2.0):
        self.J = J
        self.J_inv = np.linalg.inv(J)
        self.k1 = k1
        self.k2 = k2

    def compute(self, quat: np.ndarray, omega: np.ndarray, 
                q_ref: np.ndarray, omega_ref: np.ndarray = np.zeros(3),
                omega_ref_dot: np.ndarray = np.zeros(3),
                dt: float = 0.002) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Calculates control torque tau matching base paper equations (10) to (22).
        """
        # Step 15 & Eq. 10: Quaternion Error q_err = q_ref* (x) q
        q_err = quaternion_error(q_ref, quat)
        q_e0 = q_err[0]
        q_ev = q_err[1:] # Vector part [q_e1, q_e2, q_e3]
        
        # Step 16 & Eq. 11: Velocity Error w_err = w - w_ref
        w_err = omega - omega_ref
        
        # Step 17 & Eq. 13: Designed Virtual Control w_err_d = -k1 * q_ev
        w_err_d = -self.k1 * q_ev
        
        # Step 18 & Eq. 16: Backstepping Error delta = w_err - w_err_d
        delta = w_err - w_err_d
        
        # Analytical Error Derivative w_err_d_dot = -k1 * q_ev_dot (Eq. 20)
        q_ev_dot = 0.5 * (q_e0 * w_err + np.cross(q_ev, w_err))
        w_err_d_dot = -self.k1 * q_ev_dot
        
        # Step 19 & Eq. 22: Fractional term tau2 = -k2 * J * (delta ^ (1/3))
        delta_pow_third = np.sign(delta) * (np.abs(delta) ** (1.0 / 3.0))
        tau2 = -self.k2 * np.dot(self.J, delta_pow_third)
        
        # Step 19 & Eq. 21: Cancellation term tau1 = w x (J w) + J * w_ref_dot + J * w_err_d_dot
        gyroscopic = np.cross(omega, np.dot(self.J, omega))
        tau1 = gyroscopic + np.dot(self.J, omega_ref_dot) + np.dot(self.J, w_err_d_dot)
        
        # Step 19: Total Torque tau = tau1 + tau2
        tau = tau1 + tau2
        
        return tau, q_err, w_err, delta
