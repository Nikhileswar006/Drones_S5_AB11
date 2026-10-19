import numpy as np
from typing import Tuple
from controllers.quaternion import quaternion_normalize

class PositionController:
   
    def __init__(self, mass: float = 1.0, gravity: float = 9.81, 
                 Kp: np.ndarray = np.array([1.2, 1.2, 2.0]), 
                 Kv: np.ndarray = np.array([2.6, 2.6, 2.5])):
        self.m = mass
        self.g = gravity
        self.Kp = np.array(Kp, dtype=float)
        self.Kv = np.array(Kv, dtype=float)

    def compute(self, current_pos: np.ndarray, current_vel: np.ndarray,
                target_pos: np.ndarray, target_yaw: float = 0.0,
                is_final_target: bool = False) -> Tuple[float, np.ndarray, np.ndarray, np.ndarray]:
        # Step 26: Destination error e_p = P_d - P
        e_p = target_pos - current_pos
        
        # Velocity damping error e_v = -V
        e_v = -current_vel
        
        # Use base gains by default (unchanged for intermediate waypoints)
        Kp_use = self.Kp
        Kv_use = self.Kv

        # --- Final destination braking only ---
        # Apply deceleration ONLY when approaching the final destination
        # to prevent the overshoot / long looping blue trail.
        # Intermediate waypoints fly through with full gains for smooth flight.
        if is_final_target:
            dist = float(np.linalg.norm(e_p))
            brake_radius = 4.0  # Start braking 4m before destination
            if dist < brake_radius:
                norm_d = dist / brake_radius
                # Ramp: Kp scales from 100% at 4m down to 35% at 0m for soft docking
                ramp = 0.35 + 0.65 * norm_d
                Kp_use = self.Kp * ramp
                # Damping boost: Kv scales up to 3.5x at destination to kill residual momentum
                Kv_use = self.Kv * (1.0 + 2.5 * (1.0 - norm_d))

        # Step 27: Acceleration command a_cmd = Kp * e_p + Kv * e_v
        # Position error and velocity damping
        a_cmd = Kp_use * e_p - Kv_use * current_vel

        # Near final destination, limit maximum tilt/speed to ensure a crisp, straight stop
        if is_final_target:
            dist = float(np.linalg.norm(e_p))
            if dist < 2.5:
                max_tilt_acc = max(1.2, 4.5 * (dist / 2.5))
                a_cmd[0] = np.clip(a_cmd[0], -max_tilt_acc, max_tilt_acc)
                a_cmd[1] = np.clip(a_cmd[1], -max_tilt_acc, max_tilt_acc)
        
        # Smooth physical tilt limits (+- 4.5 m/s^2 corresponds to ~25 deg tilt)
        a_cmd[0] = np.clip(a_cmd[0], -4.5, 4.5)
        a_cmd[1] = np.clip(a_cmd[1], -4.5, 4.5)
        a_cmd[2] = np.clip(a_cmd[2], -3.5, 5.0)
        
        # Step 28: Total thrust T_desired = m * (g + a_z)
        total_z_acc = self.g + a_cmd[2]
        total_z_acc = max(0.5, total_z_acc)
        T_desired = self.m * total_z_acc
        
        # Step 29: Generate desired roll phi_ref and pitch theta_ref
        psi_ref = float(target_yaw)
        sin_psi = np.sin(psi_ref)
        cos_psi = np.cos(psi_ref)
        
        # phi_ref = asin((ax * sin(psi) - ay * cos(psi)) / (g + az))
        sin_phi = (a_cmd[0] * sin_psi - a_cmd[1] * cos_psi) / total_z_acc
        sin_phi = np.clip(sin_phi, -0.45, 0.45) # Max +-25 deg tilt
        phi_ref = np.arcsin(sin_phi)
        
        # theta_ref = atan((ax * cos(psi) + ay * sin(psi)) / (g + az))
        theta_ref = np.arctan2(a_cmd[0] * cos_psi + a_cmd[1] * sin_psi, total_z_acc)
        theta_ref = np.clip(theta_ref, -0.45, 0.45)
        
        # Step 30: Convert (phi_ref, theta_ref, psi_ref) to Hamilton quaternion q_ref
        cy = np.cos(psi_ref * 0.5)
        sy = np.sin(psi_ref * 0.5)
        cp = np.cos(theta_ref * 0.5)
        sp = np.sin(theta_ref * 0.5)
        cr = np.cos(phi_ref * 0.5)
        sr = np.sin(phi_ref * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        q_ref = quaternion_normalize(np.array([qw, qx, qy, qz]))

        return T_desired, q_ref, e_p, a_cmd
