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


class MultiWaypointTrajectory3D:
    """
    Smooth C^2 continuous trajectory navigating through multiple 3D waypoints:
    P_0 (Start) -> P_1 -> P_2 -> ... -> P_M (Destination)
    Uses piecewise quintic polynomial segments with continuous tangent velocities.
    The drone smoothly flows through every intermediate point without stopping.
    """

    def __init__(self, waypoints: list, total_duration: float = 14.0):
        """
        Args:
            waypoints: List of 3D points [p0, p1, p2, ..., pM], each as array-like [x, y, z]
            total_duration: Total flight time across all segments in seconds
        """
        self.waypoints = [np.array(wp, dtype=np.float64) for wp in waypoints]
        if len(self.waypoints) < 2:
            raise ValueError("At least 2 waypoints (start and goal) are required.")

        self.num_pts = len(self.waypoints)
        self.num_segments = self.num_pts - 1
        self.total_duration = max(float(total_duration), 1.0)

        # Compute distances and durations for each segment
        seg_dists = [np.linalg.norm(self.waypoints[i + 1] - self.waypoints[i]) for i in range(self.num_segments)]
        total_dist = sum(seg_dists)
        if total_dist < 1e-4:
            total_dist = 1.0
            seg_dists = [total_dist / self.num_segments] * self.num_segments

        # Segment durations proportional to distance
        self.durations = [self.total_duration * (d / total_dist) for d in seg_dists]
        # Ensure minimum duration per segment
        for i in range(self.num_segments):
            self.durations[i] = max(self.durations[i], 0.5)

        # Re-normalize total duration
        self.durations = [d * (self.total_duration / sum(self.durations)) for d in self.durations]

        # Cumulative segment transition times
        self.cum_times = [0.0]
        for d in self.durations:
            self.cum_times.append(self.cum_times[-1] + d)

        # Compute continuous boundary velocities at each waypoint (Catmull-Rom style tangents)
        self.velocities = [np.zeros(3)]  # Start at rest
        for i in range(1, self.num_pts - 1):
            dt_prev = self.durations[i - 1]
            dt_next = self.durations[i]
            # Smooth average tangent velocity
            v_i = (self.waypoints[i + 1] - self.waypoints[i - 1]) / (dt_prev + dt_next)
            # Limit maximum velocity through waypoints
            v_norm = np.linalg.norm(v_i)
            max_v = 2.5
            if v_norm > max_v:
                v_i = v_i * (max_v / v_norm)
            self.velocities.append(v_i)
        self.velocities.append(np.zeros(3))  # Stop at rest at destination

        # Precompute quintic polynomial coefficients for each segment
        # x(tau) = a0 + a1*tau + a2*tau^2 + a3*tau^3 + a4*tau^4 + a5*tau^5
        self.coeffs = []
        for k in range(self.num_segments):
            Tk = self.durations[k]
            P0 = self.waypoints[k]
            P1 = self.waypoints[k + 1]
            V0 = self.velocities[k]
            V1 = self.velocities[k + 1]

            a0 = P0
            a1 = Tk * V0
            a2 = np.zeros(3)
            a3 = 10.0 * (P1 - P0) - (6.0 * V0 + 4.0 * V1) * Tk
            a4 = -15.0 * (P1 - P0) + (8.0 * V0 + 7.0 * V1) * Tk
            a5 = 6.0 * (P1 - P0) - 3.0 * (V0 + V1) * Tk

            self.coeffs.append((a0, a1, a2, a3, a4, a5, Tk))

    def get_current_waypoint_index(self, t: float) -> int:
        """Returns the index of the next waypoint being targeted."""
        for i in range(self.num_segments):
            if t <= self.cum_times[i + 1]:
                return i + 1
        return self.num_pts - 1

    def evaluate(self, t: float) -> dict:
        # Before start
        if t <= 0.0:
            p0 = self.waypoints[0]
            yaw0 = np.arctan2(self.waypoints[1][1] - p0[1], self.waypoints[1][0] - p0[0])
            return {'pos': p0.copy(), 'vel': np.zeros(3), 'acc': np.zeros(3), 'jerk': np.zeros(3), 'yaw': yaw0, 'yaw_rate': 0.0}

        # After finish
        if t >= self.total_duration:
            pf = self.waypoints[-1]
            yawf = np.arctan2(pf[1] - self.waypoints[-2][1], pf[0] - self.waypoints[-2][0])
            return {'pos': pf.copy(), 'vel': np.zeros(3), 'acc': np.zeros(3), 'jerk': np.zeros(3), 'yaw': yawf, 'yaw_rate': 0.0}

        # Find active segment
        seg_idx = 0
        for i in range(self.num_segments):
            if t < self.cum_times[i + 1]:
                seg_idx = i
                break

        t_local = t - self.cum_times[seg_idx]
        a0, a1, a2, a3, a4, a5, Tk = self.coeffs[seg_idx]
        tau = np.clip(t_local / Tk, 0.0, 1.0)

        # Quintic evaluation
        pos = a0 + a1 * tau + a2 * (tau ** 2) + a3 * (tau ** 3) + a4 * (tau ** 4) + a5 * (tau ** 5)
        d_pos_dtau = a1 + 2.0 * a2 * tau + 3.0 * a3 * (tau ** 2) + 4.0 * a4 * (tau ** 3) + 5.0 * a5 * (tau ** 4)
        d2_pos_dtau2 = 2.0 * a2 + 6.0 * a3 * tau + 12.0 * a4 * (tau ** 2) + 20.0 * a5 * (tau ** 3)
        d3_pos_dtau3 = 6.0 * a3 + 24.0 * a4 * tau + 60.0 * a5 * (tau ** 2)

        vel = d_pos_dtau / Tk
        acc = d2_pos_dtau2 / (Tk ** 2)
        jerk = d3_pos_dtau3 / (Tk ** 3)

        # Calculate smooth heading (yaw) along velocity
        speed_xy = np.linalg.norm(vel[:2])
        if speed_xy > 0.08:
            yaw = np.arctan2(vel[1], vel[0])
            yaw_rate = (acc[1] * vel[0] - acc[0] * vel[1]) / (speed_xy ** 2 + 1e-6)
        else:
            delta_target = self.waypoints[seg_idx + 1] - self.waypoints[seg_idx]
            yaw = np.arctan2(delta_target[1], delta_target[0])
            yaw_rate = 0.0

        return {
            'pos': pos,
            'vel': vel,
            'acc': acc,
            'jerk': jerk,
            'yaw': yaw,
            'yaw_rate': yaw_rate
        }

    def generate_waypoints(self, num_points: int = 250) -> np.ndarray:
        t_vals = np.linspace(0, self.total_duration, num_points)
        pts = np.zeros((num_points, 3))
        for i, t in enumerate(t_vals):
            pts[i] = self.evaluate(t)['pos']
        return pts


class ObstacleAvoidanceTrajectory3D(MultiWaypointTrajectory3D):
    """
    Intelligent Obstacle-Avoidance Trajectory Generator.
    Takes user nominal waypoints and physical obstacles placed directly in the line of flight.
    Dynamically computes smooth collision-free detour waypoints (swerving horizontally or vertically)
    around each obstacle with a safety clearance buffer, and stitches them into a smooth
    C^2 continuous quintic multi-waypoint flight path.
    """

    def __init__(self, waypoints: list, obstacles: list = None,
                 safety_radius: float = 0.85, total_duration: float = 14.0):
        """
        Args:
            waypoints: Nominal guide waypoints [p0, p1, ..., pM]
            obstacles: List of dicts: [{'pos': [x,y,z], 'radius': r, 'height': h}, ...] or [x,y,z] positions
            safety_radius: Minimum clearance radius around obstacle boundary (m)
            total_duration: Total flight duration (s)
        """
        self.nominal_waypoints = [np.array(wp, dtype=np.float64) for wp in waypoints]
        self.obstacle_specs = obstacles if obstacles is not None else []
        self.safety_radius = float(safety_radius)

        # Plan collision-free detour waypoints
        planned_path = self._plan_avoidance_path(self.nominal_waypoints, self.obstacle_specs, self.safety_radius)
        super().__init__(waypoints=planned_path, total_duration=total_duration)

    @staticmethod
    def _plan_avoidance_path(waypoints, obstacles, clearance):
        if not obstacles or len(waypoints) < 2:
            return waypoints

        final_path = [waypoints[0]]

        for i in range(len(waypoints) - 1):
            p_curr = waypoints[i]
            p_next = waypoints[i + 1]
            seg_vec = p_next - p_curr
            seg_len = np.linalg.norm(seg_vec)
            if seg_len < 1e-4:
                continue
            seg_dir = seg_vec / seg_len

            # Check all obstacles along this segment
            intersections = []
            for obs in obstacles:
                if isinstance(obs, dict):
                    obs_pos = np.array(obs.get('pos', [0, 0, 0]), dtype=np.float64)
                    r_obs = float(obs.get('radius', 0.35)) + clearance
                    h_obs = float(obs.get('height', 1.5))
                else:
                    obs_pos = np.array(obs, dtype=np.float64)
                    r_obs = 0.45 + clearance
                    h_obs = 1.8

                # Projection of obstacle center onto line segment
                v_to_obs = obs_pos - p_curr
                proj = np.dot(v_to_obs, seg_dir)
                if 0.15 * seg_len < proj < 0.85 * seg_len:
                    closest_pt = p_curr + proj * seg_dir
                    dist_to_center_xy = np.linalg.norm(closest_pt[:2] - obs_pos[:2])

                    if dist_to_center_xy < r_obs:
                        # Obstacle blocks the direct path!
                        intersections.append({
                            'proj': proj,
                            'obs_pos': obs_pos,
                            'radius': r_obs,
                            'height': h_obs,
                            'closest_pt': closest_pt
                        })

            # Sort intersections by distance along segment
            intersections.sort(key=lambda x: x['proj'])

            for item in intersections:
                obs_pos = item['obs_pos']
                r_detour = max(item['radius'], 0.75)

                # Compute perpendicular horizontal avoidance vector
                perp_2d = np.array([-seg_dir[1], seg_dir[0], 0.0])
                norm_perp = np.linalg.norm(perp_2d[:2])
                if norm_perp > 1e-3:
                    perp_2d = perp_2d / norm_perp
                else:
                    perp_2d = np.array([0.0, 1.0, 0.0])

                # Determine which side requires a smaller detour
                vec_curr_to_obs = obs_pos - p_curr
                side_sign = 1.0 if np.dot(perp_2d, vec_curr_to_obs) <= 0 else -1.0
                avoid_dir = side_sign * perp_2d

                # Pre-detour (enter avoidance corridor smoothly)
                pre_pt = obs_pos - seg_dir * (r_detour * 0.85) + avoid_dir * (r_detour * 0.7)
                pre_pt[2] = 0.5 * (p_curr[2] + p_next[2])

                # Apex detour (safely clearing obstacle center)
                apex_pt = obs_pos + avoid_dir * (r_detour * 1.15)
                apex_pt[2] = max(0.5 * (p_curr[2] + p_next[2]), obs_pos[2] + 0.35)

                # Post-detour (rejoin route towards destination)
                post_pt = obs_pos + seg_dir * (r_detour * 0.85) + avoid_dir * (r_detour * 0.7)
                post_pt[2] = 0.5 * (apex_pt[2] + p_next[2])

                final_path.append(pre_pt)
                final_path.append(apex_pt)
                final_path.append(post_pt)

            final_path.append(p_next)

        return final_path

    @classmethod
    def replan_from_current_state(cls, current_pos, current_vel, target_waypoints,
                                  new_obstacle, remaining_duration, safety_radius=0.85):
        """
        Dynamically replans the trajectory when an obstacle suddenly appears in front of the drone.
        Smoothly branches out from current [pos, vel] avoiding the new obstacle, then reconnects
        to the remaining target waypoints.
        """
        curr_p = np.array(current_pos, dtype=np.float64)
        curr_v = np.array(current_vel, dtype=np.float64)

        # Build nominal route starting from current position
        guide_waypoints = [curr_p] + [np.array(wp, dtype=np.float64) for wp in target_waypoints]

        # Use collision avoidance path calculation
        avoidance_wps = cls._plan_avoidance_path(
            waypoints=guide_waypoints,
            obstacles=[new_obstacle],
            clearance=safety_radius
        )
        if avoidance_wps is None or len(avoidance_wps) < 2:
            avoidance_wps = guide_waypoints

        # Create new trajectory instance
        new_traj = cls(waypoints=avoidance_wps, obstacles=[],
                       safety_radius=safety_radius, total_duration=remaining_duration)

        # Match initial velocity with drone's current velocity for C^1 continuity
        if len(new_traj.coeffs) > 0:
            Tk = new_traj.durations[0]
            P0 = new_traj.waypoints[0]
            P1 = new_traj.waypoints[1]
            V0 = curr_v
            V1 = new_traj.velocities[1]

            a0 = P0
            a1 = Tk * V0
            a2 = np.zeros(3)
            a3 = 10.0 * (P1 - P0) - (6.0 * V0 + 4.0 * V1) * Tk
            a4 = -15.0 * (P1 - P0) + (8.0 * V0 + 7.0 * V1) * Tk
            a5 = 6.0 * (P1 - P0) - 3.0 * (V0 + V1) * Tk
            new_traj.coeffs[0] = (a0, a1, a2, a3, a4, a5, Tk)

        return new_traj


