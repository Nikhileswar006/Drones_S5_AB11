"""
Smooth 3D Trajectory Generator for UAV Navigation
Generates C^2 continuous (position, velocity, acceleration, jerk) reference trajectories
from a defined start position to a destination in 3D space.
"""

import numpy as np


class QuinticTrajectory3D:
    """
    Minimum-jerk (5th order polynomial) trajectory connecting
    P_start to P_goal over duration T.
    Ensures zero boundary velocity and acceleration for buttery-smooth flight.
    """

    def __init__(self,
                 p_start: np.ndarray,
                 p_goal: np.ndarray,
                 duration: float = 8.0,
                 target_yaw: float = 0.0):
        self.p_start = np.array(p_start, dtype=np.float64)
        self.p_goal = np.array(p_goal, dtype=np.float64)
        self.duration = max(float(duration), 1.0)
        self.target_yaw = float(target_yaw)

    def evaluate(self, t: float) -> dict:
        """
        Evaluates the trajectory at time t.

        Returns:
            pos: Desired 3D position [x, y, z] (m)
            vel: Desired 3D velocity [vx, vy, vz] (m/s)
            acc: Desired 3D acceleration [ax, ay, az] (m/s^2)
            jerk: Desired 3D jerk [jx, jy, jz] (m/s^3)
            yaw: Desired heading angle (rad)
            yaw_rate: Desired yaw rate (rad/s)
        """
        delta = self.p_goal - self.p_start

        if t <= 0.0:
            return {
                'pos': self.p_start.copy(),
                'vel': np.zeros(3),
                'acc': np.zeros(3),
                'jerk': np.zeros(3),
                'yaw': self.target_yaw,
                'yaw_rate': 0.0
            }

        if t >= self.duration:
            return {
                'pos': self.p_goal.copy(),
                'vel': np.zeros(3),
                'acc': np.zeros(3),
                'jerk': np.zeros(3),
                'yaw': self.target_yaw,
                'yaw_rate': 0.0
            }

        tau = t / self.duration
        T = self.duration

        # 5th-order polynomial basis functions: s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
        s = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)
        ds = (30.0 * (tau ** 2) - 60.0 * (tau ** 3) + 30.0 * (tau ** 4)) / T
        dds = (60.0 * tau - 180.0 * (tau ** 2) + 120.0 * (tau ** 3)) / (T ** 2)
        ddds = (60.0 - 360.0 * tau + 360.0 * (tau ** 2)) / (T ** 3)

        pos = self.p_start + delta * s
        vel = delta * ds
        acc = delta * dds
        jerk = delta * ddds

        # Desired yaw: can point forward along velocity vector or fixed target_yaw
        speed_xy = np.linalg.norm(vel[:2])
        if speed_xy > 0.05:
            yaw = np.arctan2(vel[1], vel[0])
            yaw_rate = (acc[1] * vel[0] - acc[0] * vel[1]) / (speed_xy ** 2 + 1e-6)
        else:
            yaw = self.target_yaw
            yaw_rate = 0.0

        return {
            'pos': pos,
            'vel': vel,
            'acc': acc,
            'jerk': jerk,
            'yaw': yaw,
            'yaw_rate': yaw_rate
        }

    def generate_waypoints(self, num_points: int = 150) -> np.ndarray:
        """Returns array of [num_points, 3] points for visualization."""
        t_vals = np.linspace(0, self.duration, num_points)
        pts = np.zeros((num_points, 3))
        for i, t in enumerate(t_vals):
            pts[i] = self.evaluate(t)['pos']
        return pts


class CurvedMissionTrajectory3D:
    """
    Smooth 3D trajectory connecting start to destination via a 3D curved path.
    Supports both vertical arching (arch_height) and horizontal curve (lateral_curve),
    creating a realistic 3D curved flight maneuver.
    """

    def __init__(self,
                 p_start: np.ndarray,
                 p_goal: np.ndarray,
                 duration: float = 10.0,
                 arch_height: float = 1.5,
                 lateral_curve: float = 1.5):
        self.p_start = np.array(p_start, dtype=np.float64)
        self.p_goal = np.array(p_goal, dtype=np.float64)
        self.duration = max(float(duration), 1.0)
        self.arch_height = float(arch_height)
        self.lateral_curve = float(lateral_curve)

        # Calculate 3D curved midpoint
        self.p_mid = 0.5 * (self.p_start + self.p_goal)
        self.p_mid[2] += self.arch_height

        # Calculate lateral perpendicular vector in XY plane
        delta_xy = self.p_goal[:2] - self.p_start[:2]
        norm_xy = np.linalg.norm(delta_xy)
        if norm_xy > 1e-4:
            perp_xy = np.array([-delta_xy[1], delta_xy[0]]) / norm_xy
            self.p_mid[0] += self.lateral_curve * perp_xy[0]
            self.p_mid[1] += self.lateral_curve * perp_xy[1]
        else:
            self.p_mid[0] += self.lateral_curve

    def evaluate(self, t: float) -> dict:
        t_clamped = np.clip(t, 0.0, self.duration)
        tau = t_clamped / self.duration
        T = self.duration

        # Quadratic Bezier blended with quintic timing for C^2 continuity
        s = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)
        ds = (30.0 * (tau ** 2) - 60.0 * (tau ** 3) + 30.0 * (tau ** 4)) / T
        dds = (60.0 * tau - 180.0 * (tau ** 2) + 120.0 * (tau ** 3)) / (T ** 2)

        # Curve: B(s) = (1-s)^2 P0 + 2(1-s)s P1 + s^2 P2
        P0 = self.p_start
        P1 = self.p_mid
        P2 = self.p_goal

        B = ((1.0 - s) ** 2) * P0 + 2.0 * (1.0 - s) * s * P1 + (s ** 2) * P2
        dB_ds = 2.0 * (1.0 - s) * (P1 - P0) + 2.0 * s * (P2 - P1)
        d2B_ds2 = 2.0 * (P2 - 2.0 * P1 + P0)

        # Chain rule
        pos = B
        vel = dB_ds * ds if (0.0 < t < self.duration) else np.zeros(3)
        acc = (d2B_ds2 * (ds ** 2) + dB_ds * dds) if (0.0 < t < self.duration) else np.zeros(3)

        speed_xy = np.linalg.norm(vel[:2])
        if speed_xy > 0.05:
            yaw = np.arctan2(vel[1], vel[0])
            yaw_rate = (acc[1] * vel[0] - acc[0] * vel[1]) / (speed_xy ** 2 + 1e-6)
        else:
            yaw = np.arctan2(self.p_goal[1] - self.p_start[1], self.p_goal[0] - self.p_start[0])
            yaw_rate = 0.0

        return {
            'pos': pos,
            'vel': vel,
            'acc': acc,
            'jerk': np.zeros(3),
            'yaw': yaw,
            'yaw_rate': yaw_rate
        }

    def generate_waypoints(self, num_points: int = 150) -> np.ndarray:
        t_vals = np.linspace(0, self.duration, num_points)
        pts = np.zeros((num_points, 3))
        for i, t in enumerate(t_vals):
            pts[i] = self.evaluate(t)['pos']
        return pts
