import numpy as np
from typing import List, Tuple, Dict, Any

class WaypointManager:
    """
    Manages sequential 3D waypoints for UAV trajectory navigation.
    The drone tracks one active target at a time:
        Start -> WP1 -> WP2 -> WP3 -> Final Goal
    """
    def __init__(self, waypoints: List[List[float]], tolerance: float = 0.20, final_tolerance: float = 0.10):
        """
        Args:
            waypoints: Ordered list of 3D waypoint coordinates [[x, y, z], ...]
            tolerance: Distance threshold in meters to consider an intermediate waypoint reached
            final_tolerance: Distance threshold in meters for the final destination
        """
        self.waypoints = [np.array(wp, dtype=float) for wp in waypoints]
        self.num_waypoints = len(self.waypoints)
        self.tolerance = tolerance
        self.final_tolerance = final_tolerance
        self.current_idx = 0
        self.is_completed = False
        self.switch_history = [] # Records (timestamp, waypoint_index, waypoint_coords)

    @property
    def current_target(self) -> np.ndarray:
        """Returns the currently active 3D target coordinate."""
        return self.waypoints[self.current_idx]

    @property
    def current_name(self) -> str:
        """Returns the human-readable label of the current target."""
        if self.current_idx == self.num_waypoints - 1:
            return "Destination"
        return f"WP{self.current_idx + 1}"

    def get_current_target(self) -> np.ndarray:
        return self.current_target

    def get_current_target_name(self) -> str:
        return self.current_name

    def reset(self):
        """Resets the manager state back to the first waypoint."""
        self.current_idx = 0
        self.is_completed = False
        self.switch_history.clear()

    def update(self, current_pos: np.ndarray, current_time: float) -> Tuple[np.ndarray, float, bool, bool]:
        """
        Updates the waypoint tracking based on current drone position.
        
        Args:
            current_pos: [x, y, z] of the UAV
            current_time: Current simulation timestamp in seconds
            
        Returns:
            active_target (np.ndarray): The 3D coordinate to track
            dist_to_target (float): Current Euclidean distance to active target
            switched (bool): True if a waypoint was just reached and switched on this step
            all_reached (bool): True if the final destination has been reached
        """
        target = self.current_target
        dist_to_target = float(np.linalg.norm(target - current_pos))
        switched = False

        # Determine applicable tolerance
        active_tol = self.final_tolerance if (self.current_idx == self.num_waypoints - 1) else self.tolerance

        if dist_to_target < active_tol:
            if self.current_idx < self.num_waypoints - 1:
                # Log transition
                prev_name = self.current_name
                self.switch_history.append({
                    'time': current_time,
                    'idx': self.current_idx,
                    'name': prev_name,
                    'pos': target.copy()
                })
                self.current_idx += 1
                switched = True
                target = self.current_target
                dist_to_target = float(np.linalg.norm(target - current_pos))
                print(f" [Waypoint] Reached {prev_name} at t={current_time:.2f}s! Switching to target {self.current_name}: {target}")
            else:
                if not self.is_completed:
                    self.is_completed = True
                    self.switch_history.append({
                        'time': current_time,
                        'idx': self.current_idx,
                        'name': "Destination Goal",
                        'pos': target.copy()
                    })
                    print(f" [Waypoint] Final Destination Goal reached at t={current_time:.2f}s with error {dist_to_target:.3f}m!")

        return target, dist_to_target, switched, self.is_completed
