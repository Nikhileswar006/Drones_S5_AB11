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
        dist = float(np.linalg.norm(e_p))
        
        # Velocity damping error e_v = -V
        e_v = -current_vel

        # --- Proximity-based braking near target to prevent overshoot ---
        # When within the braking zone, ramp down position gain and ramp up
        # velocity damping so the drone decelerates smoothly.
        brake_radius = 2.0  # Start braking at 2m from target
        Kp_eff = self.Kp.copy()
        Kv_eff = self.Kv.copy()

        if dist < brake_radius:
            # Linear ramp: at dist==brake_radius → factor=1, at dist==0 → factor=0.15
            ramp = max(0.15, dist / brake_radius)
            Kp_eff = self.Kp * ramp
            # Boost damping inversely: stronger braking closer to target
            Kv_eff = self.Kv * (1.0 + 2.5 * (1.0 - ramp))

        # For final destination, apply even stronger braking
        if is_final_target and dist < brake_radius:
            Kv_eff = Kv_eff * 1.5  # Extra damping at destination

        # Step 27: Acceleration command a_cmd = Kp * e_p + Kv * e_v
        a_cmd = Kp_eff * e_p + Kv_eff * e_v
        
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
