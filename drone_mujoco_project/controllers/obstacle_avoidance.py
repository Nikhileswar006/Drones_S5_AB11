import numpy as np
from typing import List, Dict, Tuple, Optional, Any

class ObstacleAvoidancePlanner:
    """
    Autonomous 3D Obstacle Avoidance Module for Quadrotor UAV.
    
    Architecture Position in Pipeline:
    Waypoint Manager -> Reference Path Planner -> Obstacle Avoidance -> Position Controller
                     -> Quaternion-based Backstepping Controller -> MuJoCo Dynamics
                     
    Methodology:
    Dual-layer Hybrid 3D Local Planner:
      1. Geometric Adaptive 3-Waypoint Detour Corridor Generation:
         - Phase 0  PRE-APEX : lateral sweep starts BEFORE the obstacle so the
           approach path to the apex itself never clips through obstacle geometry.
         - Phase 1  APEX     : fully alongside the obstacle at bypass distance.
         - Phase 2  EXIT     : past the obstacle, merging back to nominal path.
         Monotonic forward-progress tracking prevents hovering or back-tracking.
      2. Reactive Artificial Potential Field (APF) Boundary Buffer:
         - Continuously monitors real-time Euclidean distance from the drone's
           physical outer envelope to obstacle boundaries and applies smooth
           repulsive biasing as a safety layer.
    """

    def __init__(self, drone_radius: float = 1.15, safety_margin: float = 0.35, detect_dist: float = 3.5):
        """
        Args:
            drone_radius: Physical bounding radius of the UAV frame from center to outer rotor tip (meters).
            safety_margin: Additional buffer distance required around obstacles (meters).
            detect_dist: Forward lookahead distance along path corridor to trigger avoidance (meters).
        """
        self.drone_radius = drone_radius
        self.safety_margin = safety_margin
        self.detect_dist = detect_dist
        self.min_safe_dist = drone_radius + safety_margin  # 1.50m minimum center-to-surface clearance

        # Compact, proportional physical arena obstacles matching models/quadrotor.xml
        # NOTE: Obs1_Pillar moved to [2.20, 1.50] to give the drone reaction space from Start.
        self.obstacles: List[Dict[str, Any]] = [
            {
                'id': 'Obs1_Pillar',
                'label': 'Industrial Pillar',
                'type': 'cylinder',
                'pos': np.array([2.20, 1.50, 0.85]),
                'radius': 0.16,
                'half_height': 0.85,
                'z_min': 0.0,
                'z_max': 1.70,
                'preferred_bypass': np.array([-0.707, 0.707, 0.0]),  # Adjusted left-perpendicular bypass
                'r_eff': 2.50,
                'segment_idx': 0  # Start -> WP1
            },
            {
                'id': 'Obs2_Tower',
                'label': 'Urban Tower',
                'type': 'box',
                'pos': np.array([4.5, 5.0, 1.00]),
                'half_size': np.array([0.20, 0.20, 1.00]),
                'radius': 0.28,  # Horizontal circumscribed radius sqrt(0.20^2 + 0.20^2)
                'z_min': 0.0,
                'z_max': 2.00,
                'preferred_bypass': np.array([-0.894, 0.447, 0.0]),  # Left bypass (away from Obs3)
                'r_eff': 1.75,
                'segment_idx': 1  # WP1 -> WP2
            },
            {
                'id': 'Obs3_Sphere',
                'label': 'Aerial Hazard Sphere',
                'type': 'sphere',
                'pos': np.array([8.0, 6.0, 3.75]),
                'radius': 0.22,
                'z_min': 3.53,
                'z_max': 3.97,
                'preferred_bypass': np.array([0.707, 0.707, 0.0]),  # Northeast bypass
                'r_eff': 2.00,   # Increased from 1.80 to improve lateral clearance
                'segment_idx': 2  # WP2 -> WP3
            },
            {
                'id': 'Obs4_Pylon',
                'label': 'Tall Pylon',
                'type': 'cylinder',
                'pos': np.array([12.0, 8.0, 1.40]),
                'radius': 0.18,
                'half_height': 1.40,
                'z_min': 0.0,
                'z_max': 2.80,
                'preferred_bypass': np.array([0.894, -0.447, 0.0]),  # Right bypass toward open space
                'r_eff': 1.65,
                'segment_idx': 3  # WP3 -> Destination
            }
        ]

        # Avoidance State Machine
        self.state = "CLEAR"  # "CLEAR", "AVOIDING", "RESUMING"
        self.active_obstacle: Optional[Dict[str, Any]] = None
        self.prev_waypoint_idx: int = -1
        self.cleared_obstacles: set = set()
        self.detour_points: List[np.ndarray] = []  # [pre_apex, apex, exit]
        self.detour_sub_idx: int = 0               # 0=pre_apex, 1=apex, 2=exit
        self.current_detour_wp: Optional[np.ndarray] = None
        self.planned_detour_path: List[np.ndarray] = []
        self.last_switch_time: float = 0.0

        # Safety & Collision Telemetry
        self.collision_detected = False
        self.collision_count = 0
        self.min_clearance_recorded = 999.0

    def compute_distance_to_obstacle(self, pos: np.ndarray, obs: Dict[str, Any]) -> Tuple[float, np.ndarray]:
        """
        Calculates Euclidean surface distance and normal unit vector from drone center to obstacle surface.
        """
        p = np.array(pos, dtype=float)
        c = obs['pos']

        if obs['type'] == 'sphere':
            diff = p - c
            dist_center = float(np.linalg.norm(diff))
            normal = diff / max(dist_center, 1e-6)
            surface_dist = dist_center - obs['radius']
            return surface_dist, normal

        elif obs['type'] == 'cylinder':
            diff_xy = p[:2] - c[:2]
            dist_xy = float(np.linalg.norm(diff_xy))
            normal_xy = diff_xy / max(dist_xy, 1e-6)

            z_min = obs['z_min']
            z_max = obs['z_max']

            if z_min <= p[2] <= z_max:
                surface_dist = dist_xy - obs['radius']
                normal = np.array([normal_xy[0], normal_xy[1], 0.0])
            elif p[2] > z_max:
                dz = p[2] - z_max
                surface_dist = float(np.sqrt(max(0.0, dist_xy - obs['radius'])**2 + dz**2))
                normal = np.array([normal_xy[0]*0.7, normal_xy[1]*0.7, 0.7])
            else:
                dz = z_min - p[2]
                surface_dist = float(np.sqrt(max(0.0, dist_xy - obs['radius'])**2 + dz**2))
                normal = np.array([normal_xy[0]*0.7, normal_xy[1]*0.7, -0.7])

            return surface_dist, normal / max(np.linalg.norm(normal), 1e-6)

        elif obs['type'] == 'box':
            half = obs['half_size']
            diff = p - c
            dx = max(0.0, abs(diff[0]) - half[0])
            dy = max(0.0, abs(diff[1]) - half[1])
            dz = max(0.0, abs(diff[2]) - half[2])

            if dx == 0 and dy == 0 and dz == 0:
                # Inside box
                surface_dist = -min(half[0] - abs(diff[0]), half[1] - abs(diff[1]), half[2] - abs(diff[2]))
                normal = diff / max(np.linalg.norm(diff), 1e-6)
            else:
                surface_dist = float(np.sqrt(dx**2 + dy**2 + dz**2))
                normal = np.array([
                    np.sign(diff[0]) * dx,
                    np.sign(diff[1]) * dy,
                    np.sign(diff[2]) * dz
                ])
                normal = normal / max(np.linalg.norm(normal), 1e-6)

            return surface_dist, normal

        return 999.0, np.array([0.0, 0.0, 1.0])

    def get_nearest_obstacle(self, pos: np.ndarray) -> Tuple[Optional[Dict[str, Any]], float, np.ndarray]:
        """Finds the nearest obstacle to current UAV position."""
        min_d = 999.0
        nearest_obs = None
        best_norm = np.array([0.0, 0.0, 1.0])

        for obs in self.obstacles:
            d, norm = self.compute_distance_to_obstacle(pos, obs)
            if d < min_d:
                min_d = d
                nearest_obs = obs
                best_norm = norm

        if min_d < self.min_clearance_recorded:
            self.min_clearance_recorded = min_d

        return nearest_obs, min_d, best_norm

    def generate_detour_waypoints(self, obs: Dict[str, Any], current_pos: np.ndarray, target_pos: np.ndarray) -> List[np.ndarray]:
        """
        Computes a 3-stage detour corridor to guarantee the approach path to each
        waypoint does NOT clip through obstacle geometry.

        Stage 0 - PRE-APEX  : Drone starts moving sideways BEFORE reaching the obstacle.
                               Placed at obs_center - path_norm*R*0.50 + bypass*R*0.85
                               → the line from current_pos to pre_apex clears the obstacle.
        Stage 1 - APEX      : Fully alongside the obstacle (obs_center + bypass*R*0.90).
        Stage 2 - EXIT      : Past the obstacle, blending back toward nominal path
                               (obs_center + bypass*R*0.25 + path_norm*R*1.20).
        """
        c = obs['pos']
        R_eff = obs.get('r_eff', 1.65)

        path_dir = target_pos - current_pos
        path_norm = path_dir / max(np.linalg.norm(path_dir), 1e-6)
        bypass_u = obs['preferred_bypass'] / np.linalg.norm(obs['preferred_bypass'])

        # ── Stage 0: Pre-Apex ────────────────────────────────────────────────
        # Located before the obstacle (negative path direction) and to bypass side.
        # The path from current_pos → pre_apex swings wide enough to clear the obstacle.
        pre_apex = c.copy()
        pre_apex[:2] += bypass_u[:2] * R_eff * 0.85 - path_norm[:2] * R_eff * 0.50

        # ── Stage 1: Apex ────────────────────────────────────────────────────
        apex_wp = c.copy()
        apex_wp[:2] += bypass_u[:2] * R_eff * 0.90

        # ── Stage 2: Exit ────────────────────────────────────────────────────
        exit_wp = c.copy()
        exit_wp[:2] += bypass_u[:2] * R_eff * 0.25 + path_norm[:2] * R_eff * 1.20

        # ── Altitude profiles per obstacle ───────────────────────────────────
        if obs['id'] == 'Obs1_Pillar':
            pre_apex[2] = 1.80
            apex_wp[2]  = 2.00
            exit_wp[2]  = 2.30
        elif obs['id'] == 'Obs2_Tower':
            pre_apex[2] = 2.40
            apex_wp[2]  = 2.70
            exit_wp[2]  = 3.20
        elif obs['id'] == 'Obs3_Sphere':
            pre_apex[2] = 3.80
            apex_wp[2]  = 4.10
            exit_wp[2]  = 4.00
        elif obs['id'] == 'Obs4_Pylon':
            pre_apex[2] = 3.80
            apex_wp[2]  = 4.20
            exit_wp[2]  = 4.70

        # ── Safety clamp: if pre_apex is behind current position, start lateral ──
        # movement immediately at the drone's current XY location.
        pre_apex_fwd = float(np.dot(pre_apex - current_pos, path_norm))
        if pre_apex_fwd < 0.3:
            # Obstacle is already very close – begin bypass at current XY
            pre_apex = current_pos.copy()
            pre_apex[:2] += bypass_u[:2] * R_eff * 0.85
            pre_apex[2] = apex_wp[2]

        return [pre_apex, apex_wp, exit_wp]

    def update(self,
               current_pos: np.ndarray,
               current_vel: np.ndarray,
               nominal_target: np.ndarray,
               current_waypoint_idx: int,
               current_time: float) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Updates the obstacle avoidance logic and generates safe position reference.
        """
        # Step A: Reset State Machine whenever waypoint advances
        if current_waypoint_idx != self.prev_waypoint_idx:
            self.state = "CLEAR"
            self.active_obstacle = None
            self.detour_points = []
            self.detour_sub_idx = 0
            self.current_detour_wp = None
            self.planned_detour_path = []
            self.prev_waypoint_idx = current_waypoint_idx

        # 1. Evaluate Nearest Obstacle Proximity
        nearest_obs, nearest_dist, normal_vec = self.get_nearest_obstacle(current_pos)

        # 2. Check if an obstacle on the current flight segment needs avoidance
        segment_obs = None
        for obs in self.obstacles:
            if obs['segment_idx'] == current_waypoint_idx:
                segment_obs = obs
                break

        avoidance_active = False

        if segment_obs is not None and segment_obs['id'] not in self.cleared_obstacles:
            s_dist, s_norm = self.compute_distance_to_obstacle(current_pos, segment_obs)
            c = segment_obs['pos']

            # Vector from drone to obstacle and target
            to_obs = c - current_pos
            to_target = nominal_target - current_pos
            dist_to_obs = float(np.linalg.norm(to_obs))
            path_dir = to_target / max(np.linalg.norm(to_target), 1e-6)

            # Corridor progression coordinate s:
            # s < 0  → obstacle is ahead of drone; s > 0 → obstacle is behind drone
            s_proj = float(np.dot(current_pos - c, path_dir))

            # ── State Machine Transitions ──────────────────────────────────────
            if self.state == "CLEAR":
                # Trigger avoidance when approaching obstacle within detection horizon
                if s_proj < 0.1 and dist_to_obs < self.detect_dist:
                    self.state = "AVOIDING"
                    self.active_obstacle = segment_obs
                    self.detour_points = self.generate_detour_waypoints(
                        segment_obs, current_pos, nominal_target)
                    self.detour_sub_idx = 0
                    self.current_detour_wp = self.detour_points[0]  # Start at pre-apex
                    self.last_switch_time = current_time

                    self.planned_detour_path = [
                        current_pos.copy(),
                        self.detour_points[0].copy(),  # pre-apex
                        self.detour_points[1].copy(),  # apex
                        self.detour_points[2].copy(),  # exit
                        nominal_target.copy()
                    ]
                    print(f" [Avoidance] Obstacle detected: {segment_obs['label']}! "
                          f"Detour initiated -> Pre-Apex: {self.detour_points[0].round(2)}, "
                          f"Apex: {self.detour_points[1].round(2)}")

            elif self.state == "AVOIDING":
                if self.detour_sub_idx == 0:
                    # ── Phase 0: tracking pre-apex ──────────────────────────
                    pre_wp = self.detour_points[0]
                    dist_to_pre = float(np.linalg.norm(pre_wp - current_pos))
                    if dist_to_pre < 0.80:
                        self.detour_sub_idx = 1
                        self.current_detour_wp = self.detour_points[1]
                        print(f" [Avoidance] Pre-apex reached, moving alongside "
                              f"{segment_obs['label']} -> Apex: {self.detour_points[1].round(2)}")

                elif self.detour_sub_idx == 1:
                    # ── Phase 1: tracking apex ──────────────────────────────
                    apex_wp = self.detour_points[1]
                    dist_to_apex = float(np.linalg.norm(apex_wp - current_pos))

                    # Advance to exit as soon as drone has passed the obstacle plane or nears apex
                    if s_proj > -0.10 or dist_to_apex < 0.85:
                        self.detour_sub_idx = 2
                        self.current_detour_wp = self.detour_points[2]
                        print(f" [Avoidance] Apex cleared, heading to exit waypoint: "
                              f"{self.detour_points[2].round(2)}")

                elif self.detour_sub_idx == 2:
                    # ── Phase 2: tracking exit ──────────────────────────────
                    exit_wp = self.detour_points[2]
                    dist_to_exit = float(np.linalg.norm(exit_wp - current_pos))

                    if s_proj > 0.45 or dist_to_exit < 0.85:
                        self.state = "RESUMING"
                        self.last_switch_time = current_time
                        print(f" [Avoidance] Safely cleared {segment_obs['label']}. "
                              f"Resuming nominal flight toward waypoint!")

            elif self.state == "RESUMING":
                # Mark as cleared so it never pulls the UAV backward
                if s_proj > 0.60 or s_dist > self.min_safe_dist + 0.15:
                    self.cleared_obstacles.add(segment_obs['id'])
                    self.state = "CLEAR"
                    self.active_obstacle = None
                    self.current_detour_wp = None
                    self.planned_detour_path = []

        # 3. Determine Safe Target Position Output
        if self.state == "AVOIDING" and self.current_detour_wp is not None:
            avoidance_active = True
            safe_target = self.current_detour_wp.copy()
        else:
            safe_target = nominal_target.copy()

        # 4. Reactive Artificial Potential Field (Layer 2 Safety Shield)
        if nearest_dist < self.min_safe_dist and nearest_obs is not None:
            penetration = max(0.0, self.min_safe_dist - nearest_dist)
            k_rep = 2.5
            repulsion = normal_vec * (k_rep * penetration)
            safe_target = safe_target + repulsion
            avoidance_active = True

        # 5. Pack Detailed Telemetry
        telemetry = {
            'avoidance_active': avoidance_active,
            'state': self.state,
            'active_obs_id': self.active_obstacle['id'] if self.active_obstacle else None,
            'active_obs_name': self.active_obstacle['label'] if self.active_obstacle else "None",
            'num_obstacles': len(self.obstacles),
            'nearest_obs_id': nearest_obs['id'] if nearest_obs else "None",
            'nearest_obs_dist': float(nearest_dist),
            'detour_target': self.current_detour_wp.copy() if self.current_detour_wp is not None else None,
            'detour_path': [p.copy() for p in self.planned_detour_path] if self.planned_detour_path else [],
            'collision_detected': self.collision_detected,
            'collision_count': self.collision_count,
            'min_clearance': float(self.min_clearance_recorded)
        }

        return safe_target, telemetry

    def check_mujoco_contacts(self, model: Any, data: Any):
        """
        Uses true MuJoCo physics solver contact manifold to verify collision-free operation.
        """
        import mujoco
        in_collision_now = False

        for i in range(data.ncon):
            contact = data.contact[i]
            geom1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)

            if geom1 is None or geom2 is None:
                continue

            # Check if one geom is drone and other is obstacle or ground
            g1_is_drone = any(k in geom1 for k in ['main_body', 'canopy', 'arm', 'rotor', 'motor', 'arrow'])
            g2_is_drone = any(k in geom2 for k in ['main_body', 'canopy', 'arm', 'rotor', 'motor', 'arrow'])

            g1_is_obs = 'obs' in geom1 or geom1 == 'ground'
            g2_is_obs = 'obs' in geom2 or geom2 == 'ground'

            if (g1_is_drone and g2_is_obs) or (g2_is_drone and g1_is_obs):
                # Ignore ground contact only at launch moment t < 0.2s
                if data.time > 0.2:
                    in_collision_now = True
                    self.collision_count += 1
                    if not self.collision_detected:
                        print(f" [COLLISION] Impact between {geom1} and {geom2} at t={data.time:.2f}s!")
                    break

        self.collision_detected = in_collision_now

    def reset(self):
        """Resets planner state back to mission start."""
        self.state = "CLEAR"
        self.active_obstacle = None
        self.prev_waypoint_idx = -1
        self.cleared_obstacles.clear()
        self.detour_points = []
        self.detour_sub_idx = 0
        self.current_detour_wp = None
        self.planned_detour_path = []
        self.collision_detected = False
        self.collision_count = 0
        self.min_clearance_recorded = 999.0
