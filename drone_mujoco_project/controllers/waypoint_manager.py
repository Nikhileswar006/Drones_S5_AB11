
import numpy as np
from typing import List, Tuple, Dict, Any


class WaypointManager:
    """
    Sequential 3D waypoint manager for UAV navigation.

    Features:
    - Distance-based switching for intermediate waypoints
    - Passed-waypoint detection with cross-track corridor checking
    - Strict distance-based final destination detection
    - Waypoint transition history
    - Safe handling after mission completion
    """

    def __init__(
        self,
        waypoints: List[List[float]],
        tolerance: float = 0.50,
        final_tolerance: float = 0.10,
        corridor_width: float = 1.0,
    ):
        if not waypoints:
            raise ValueError("At least one waypoint is required.")

        self.waypoints = [
            np.asarray(wp, dtype=float) for wp in waypoints
        ]

        if any(wp.shape != (3,) for wp in self.waypoints):
            raise ValueError(
                "Every waypoint must contain exactly [x, y, z]."
            )

        if tolerance <= 0:
            raise ValueError("tolerance must be positive.")

        if final_tolerance <= 0:
            raise ValueError("final_tolerance must be positive.")

        if corridor_width <= 0:
            raise ValueError("corridor_width must be positive.")

        self.num_waypoints = len(self.waypoints)
        self.tolerance = float(tolerance)
        self.final_tolerance = float(final_tolerance)
        self.corridor_width = float(corridor_width)

        self.current_idx = 0
        self.is_completed = False
        self.switch_history: List[Dict[str, Any]] = []

    @property
    def current_target(self) -> np.ndarray:
        """Return a copy of the active target."""
        return self.waypoints[self.current_idx].copy()

    @property
    def current_name(self) -> str:
        """Return the active waypoint label."""
        if self.current_idx == self.num_waypoints - 1:
            return "Destination"

        return f"WP{self.current_idx + 1}"

    def get_current_target(self) -> np.ndarray:
        return self.current_target

    def get_current_target_name(self) -> str:
        return self.current_name

    def reset(self):
        """Reset the manager before a new simulation run."""
        self.current_idx = 0
        self.is_completed = False
        self.switch_history.clear()

    def _has_passed_waypoint(
        self,
        current_pos: np.ndarray,
    ) -> bool:
        """
        Detect whether the drone has passed the active intermediate
        waypoint while remaining close to the previous-to-current
        waypoint segment.
        """
        if self.current_idx <= 0:
            return False

        previous_wp = self.waypoints[self.current_idx - 1]
        target_wp = self.waypoints[self.current_idx]

        segment = target_wp - previous_wp
        segment_length_sq = float(np.dot(segment, segment))

        # Avoid division by zero for duplicate waypoints.
        if segment_length_sq < 1e-9:
            return False

        relative_pos = current_pos - previous_wp

        # Projection along the previous-to-target segment.
        progress = float(np.dot(relative_pos, segment))

        # The drone must be beyond the target plane.
        if progress < segment_length_sq:
            return False

        # Calculate distance from the drone to the segment's line.
        projection = (
            previous_wp
            + (progress / segment_length_sq) * segment
        )

        cross_track_error = float(
            np.linalg.norm(current_pos - projection)
        )

        return cross_track_error <= self.corridor_width

    def _record_transition(
        self,
        current_time: float,
        name: str,
        position: np.ndarray,
        distance: float,
    ):
        self.switch_history.append({
            "time": float(current_time),
            "idx": self.current_idx,
            "name": name,
            "pos": position.copy(),
            "distance": float(distance),
        })

    def update(
        self,
        current_pos: np.ndarray,
        current_time: float,
    ) -> Tuple[np.ndarray, float, bool, bool]:
        """
        Update the active waypoint.

        Returns:
            target       : Active target position (3D)
            distance     : Distance from drone to returned target
            switched     : True if an intermediate waypoint changed
            all_reached  : True if final destination was reached
        """
        current_pos = np.asarray(current_pos, dtype=float)

        if current_pos.shape != (3,):
            raise ValueError(
                "current_pos must contain exactly [x, y, z]."
            )

        if not np.all(np.isfinite(current_pos)):
            raise ValueError("current_pos must contain finite values.")

        switched = False

        # Once complete, do not switch targets again.
        if self.is_completed:
            target = self.current_target
            distance = float(np.linalg.norm(target - current_pos))
            return target, distance, False, True

        target = self.current_target
        distance = float(np.linalg.norm(target - current_pos))

        is_final = self.current_idx == self.num_waypoints - 1

        if is_final:
            # Never use passed-waypoint detection for the destination.
            if distance <= self.final_tolerance:
                self._record_transition(
                    current_time,
                    "Destination",
                    target,
                    distance,
                )
                self.is_completed = True

                print(
                    f"[Waypoint] Destination reached at "
                    f"t={current_time:.2f}s; "
                    f"error={distance:.3f} m"
                )

            return (
                self.current_target,
                distance,
                False,
                self.is_completed,
            )

        # Check whether the active intermediate waypoint was reached.
        reached = distance <= self.tolerance
        passed = False if reached else self._has_passed_waypoint(
            current_pos
        )

        if reached or passed:
            previous_name = self.current_name

            self._record_transition(
                current_time,
                previous_name,
                target,
                distance,
            )

            self.current_idx += 1
            switched = True

            reason = (
                "within tolerance"
                if reached
                else "passed within corridor"
            )

            print(
                f"[Waypoint] {previous_name} reached by {reason} "
                f"at t={current_time:.2f}s. "
                f"Next target: {self.current_name}"
            )

        # Return the active target AFTER a possible switch.
        target = self.current_target
        distance = float(np.linalg.norm(target - current_pos))

        return target, distance, switched, self.is_completed
