#!/usr/bin/env python3
"""
Waypoint Manager Node (waypoint_manager.py)
===========================================
Manages the user-defined starting position and arbitrary sequence of 3D target waypoints:
    Start -> Waypoint 1 -> Waypoint 2 -> Waypoint 3 -> ...
Evaluates the Euclidean waypoint switching criterion:
    sqrt((x - x_wp)^2 + (y - y_wp)^2 + (z - z_wp)^2) < WAYPOINT_TOLERANCE
"""

from __future__ import annotations
import os
import yaml
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float64MultiArray, String
    from geometry_msgs.msg import PointStamped
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object


def load_waypoints_from_yaml(yaml_path: str = None):
    """Helper to load start position, waypoints list, and tolerance from YAML."""
    default_start = [0.0, 0.0, 2.0]
    default_wps = [
        [5.0, 3.0, 4.0],
        [10.0, -2.0, 5.0],
        [15.0, 4.0, 6.0]
    ]
    default_tol = 0.20

    if yaml_path and os.path.exists(yaml_path):
        with open(yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        params = data.get('uav_simulation', {}).get('ros__parameters', data)
        start = list(params.get('initial_position', default_start))
        if 'waypoints' in params and isinstance(params['waypoints'], list):
            wps = [list(wp) for wp in params['waypoints']]
        elif 'waypoints_flat' in params:
            flat = list(params['waypoints_flat'])
            wps = [flat[i:i + 3] for i in range(0, len(flat), 3)]
        else:
            wps = default_wps
        tol = float(params.get('waypoint_tolerance', default_tol))
        return start, wps, tol

    return default_start, default_wps, default_tol


class WaypointManagerCore:
    """
    Pure Python state machine for sequential 3D waypoint management and switching.
    Used identically by both the ROS 2 node and standalone verification.
    """

    def __init__(self, start_pos, waypoints, tolerance: float = 0.20):
        self.start_pos = np.asarray(start_pos, dtype=float).flatten()
        self.waypoints = [np.asarray(wp, dtype=float).flatten() for wp in waypoints]
        self.tolerance = float(tolerance)
        self.active_index = 0  # 0 corresponds to Waypoint 1
        self.mission_complete = False
        self.waypoint_reach_times = {}
        self.segment_start_time = 0.0

    @property
    def num_waypoints(self) -> int:
        return len(self.waypoints)

    @property
    def current_waypoint(self) -> np.ndarray:
        if self.num_waypoints == 0:
            return self.start_pos.copy()
        idx = min(self.active_index, self.num_waypoints - 1)
        return self.waypoints[idx].copy()

    @property
    def previous_waypoint(self) -> np.ndarray:
        if self.active_index <= 0:
            return self.start_pos.copy()
        idx = min(self.active_index - 1, self.num_waypoints - 1)
        return self.waypoints[idx].copy()

    @property
    def active_label(self) -> str:
        if self.mission_complete:
            return f"Waypoint {self.num_waypoints} (Reached - Holding)"
        return f"Waypoint {self.active_index + 1}"

    def compute_distance(self, current_pos: np.ndarray) -> float:
        """
        Computes Euclidean distance to active waypoint:
        d = sqrt((x - x_wp)^2 + (y - y_wp)^2 + (z - z_wp)^2)
        """
        pos = np.asarray(current_pos, dtype=float).flatten()
        wp = self.current_waypoint
        return float(np.linalg.norm(pos - wp))

    def update(self, current_pos: np.ndarray, sim_time: float) -> dict:
        """
        Checks if the UAV has entered the waypoint acceptance sphere:
            sqrt((x - x_wp)^2 + (y - y_wp)^2 + (z - z_wp)^2) < WAYPOINT_TOLERANCE
        and switches sequentially to the next waypoint.
        """
        dist = self.compute_distance(current_pos)
        switched = False

        if not self.mission_complete and dist <= self.tolerance:
            wp_num = self.active_index + 1
            if wp_num not in self.waypoint_reach_times:
                self.waypoint_reach_times[wp_num] = float(sim_time)

            if self.active_index < self.num_waypoints - 1:
                self.active_index += 1
                self.segment_start_time = float(sim_time)
                switched = True
                dist = self.compute_distance(current_pos)
            else:
                self.mission_complete = True
                switched = True

        return {
            'active_index': self.active_index,
            'active_waypoint_number': min(self.active_index + 1, self.num_waypoints),
            'total_waypoints': self.num_waypoints,
            'current_waypoint': self.current_waypoint,
            'previous_waypoint': self.previous_waypoint,
            'distance_to_waypoint': dist,
            'switched': switched,
            'mission_complete': self.mission_complete,
            'label': self.active_label,
            'waypoint_reach_times': self.waypoint_reach_times,
        }


class WaypointManagerNode(Node):
    """ROS 2 Node wrapping WaypointManagerCore."""

    def __init__(self):
        super().__init__('waypoint_manager')

        self.declare_parameter('config_file', '')
        self.declare_parameter('initial_position', [0.0, 0.0, 2.0])
        self.declare_parameter('waypoints_flat', [5.0, 3.0, 4.0, 10.0, -2.0, 5.0, 15.0, 4.0, 6.0])
        self.declare_parameter('waypoint_tolerance', 0.20)

        config_file = self.get_parameter('config_file').value
        if config_file and os.path.exists(config_file):
            start_pos, waypoints, tolerance = load_waypoints_from_yaml(config_file)
        else:
            start_pos = list(self.get_parameter('initial_position').value)
            flat = list(self.get_parameter('waypoints_flat').value)
            waypoints = [flat[i:i + 3] for i in range(0, len(flat), 3)]
            tolerance = float(self.get_parameter('waypoint_tolerance').value)

        self.core = WaypointManagerCore(start_pos, waypoints, tolerance)
        self.current_pos = np.array(start_pos, dtype=float)
        self.start_ros_time = self.get_clock().now().nanoseconds * 1e-9

        # Subscribers & Publishers
        self.odom_sub = self.create_subscription(
            Odometry, '/uav/odometry', self.odom_callback, 20
        )
        self.state_pub = self.create_publisher(
            Float64MultiArray, '/uav/waypoint_state', 20
        )
        self.label_pub = self.create_publisher(
            String, '/uav/active_waypoint_label', 20
        )
        self.target_pub = self.create_publisher(
            PointStamped, '/uav/active_waypoint', 20
        )

        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info(
            f"Waypoint Manager initialized | Start: {start_pos} | "
            f"Waypoints ({len(waypoints)}): {waypoints} | Tolerance: {tolerance} m"
        )

    def odom_callback(self, msg: Odometry):
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ], dtype=float)

    def timer_callback(self):
        sim_time = self.get_clock().now().nanoseconds * 1e-9 - self.start_ros_time
        status = self.core.update(self.current_pos, sim_time)

        if status['switched']:
            if status['mission_complete']:
                self.get_logger().info(
                    f"[t={sim_time:.2f}s] Final Waypoint {status['total_waypoints']} reached! "
                    f"Holding position at {status['current_waypoint'].tolist()}."
                )
            else:
                self.get_logger().info(
                    f"[t={sim_time:.2f}s] Reached Waypoint {status['active_index']}! "
                    f"Switching to {status['label']}: {status['current_waypoint'].tolist()}"
                )

        wp = status['current_waypoint']
        prev_wp = status['previous_waypoint']

        state_msg = Float64MultiArray()
        state_msg.data = [
            float(status['active_index']),
            float(status['total_waypoints']),
            float(wp[0]), float(wp[1]), float(wp[2]),
            float(prev_wp[0]), float(prev_wp[1]), float(prev_wp[2]),
            float(status['distance_to_waypoint']),
            1.0 if status['mission_complete'] else 0.0
        ]
        self.state_pub.publish(state_msg)

        label_msg = String()
        label_msg.data = status['label']
        self.label_pub.publish(label_msg)

        pt_msg = PointStamped()
        pt_msg.header.stamp = self.get_clock().now().to_msg()
        pt_msg.header.frame_id = 'world'
        pt_msg.point.x = float(wp[0])
        pt_msg.point.y = float(wp[1])
        pt_msg.point.z = float(wp[2])
        self.target_pub.publish(pt_msg)


def main(args=None):
    if not ROS2_AVAILABLE:
        print("ROS 2 (rclpy) is not sourced in this shell. Use run_standalone_verification.py or source ROS 2.")
        return
    rclpy.init(args=args)
    node = WaypointManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
