import numpy as np
from typing import Tuple

class MotorMixer:
    """
    Step 32: Plus-Configuration Motor Mixer.
    
    Converts desired total thrust T and body control torques tau = [tau_x, tau_y, tau_z]
    into individual motor thrust forces T1, T2, T3, T4 and reaction yaw torque.
    
    Rotor Layout (Plus Configuration):
      Motor 1: Front (+X)
      Motor 2: Left (+Y)
      Motor 3: Back (-X)
      Motor 4: Right (-Y)
    """
    def __init__(self, arm_length: float = 0.4, min_thrust: float = 0.0, max_thrust: float = 20.0):
        self.L = arm_length
        self.min_thrust = min_thrust
        self.max_thrust = max_thrust

    def mix(self, total_thrust: float, tau: np.ndarray) -> Tuple[np.ndarray, float]:
        """
        Inputs:
          total_thrust : Desired vertical thrust T (N)
          tau          : Control torque vector [tau_x, tau_y, tau_z] (N*m)
          
        Outputs:
          T_motors     : Array [T1, T2, T3, T4] of motor thrusts (N)
          tau_z_reaction: Reaction yaw torque (N*m)
        """
        tau_x, tau_y, tau_z = tau
        
        # Base per-motor thrust
        T_base = total_thrust / 4.0
        
        # Plus configuration mixing matrix equations
        T1 = T_base - (tau_y / (2.0 * self.L)) # Front (+X)
        T2 = T_base + (tau_x / (2.0 * self.L)) # Left (+Y)
        T3 = T_base + (tau_y / (2.0 * self.L)) # Back (-X)
        T4 = T_base - (tau_x / (2.0 * self.L)) # Right (-Y)
        
        T_motors = np.array([T1, T2, T3, T4])
        
        # Saturate thrust commands within physical motor bounds [0, 20 N]
        T_motors = np.clip(T_motors, self.min_thrust, self.max_thrust)
        
        return T_motors, float(tau_z)
