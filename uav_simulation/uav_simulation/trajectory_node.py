#!/usr/bin/env python3
"""
Trajectory Generator Node (trajectory_node.py)
==============================================
Generates smooth, minimum-jerk (quintic polynomial) 3D reference trajectories
for every pair of consecutive waypoints:
    Start -> WP1 -> WP2 -> WP3 -> ...
Provides continuous position p_ref(t), velocity v_ref(t), acceleration a_ref(t),
and jerk j_ref(t) required by the Quaternion-Based Attitude Tracking Controller
(Lin & Yu, CACS 2024), and publishes the complete persistent reference trajectory
for visualization in Gazebo and RViz2.
"""

from __future__ import annotations
import os
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float64MultiArray
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object

from uav_simulation.waypoint_manager import load_waypoints_from_yaml


class QuinticSegmentTrajectoryGenerator:
    """
    Generates piecewise minimum-jerk 5th-order polynomial trajectories between
    consecutive 3D waypoints:
        p_0 (Start) -> p_1 (WP1) -> p_2 (WP2) -> p_3 (WP3) -> ...
    Each segment i has duration T_i chosen from the Euclidean segment length
    and configured max_velocity / max_acceleration so that velocity and
    acceleration are zero at each waypoint and C^2-continuous everywhere.
    """

    def __init__(
        self,
        start_pos,
        waypoints,
        max_velocity: float = 1.0,
        max_acceleration: float = 1.2,
        obstacles: list | None = None,
        enable_obstacle_avoidance: bool = True
    ):
        self.start_pos = np.asarray(start_pos, dtype=float).flatten()
        self.waypoints = [np.asarray(wp, dtype=float).flatten() for wp in waypoints]
        self.max_velocity = max(float(max_velocity), 0.2)
        self.max_acceleration = max(float(max_acceleration), 0.2)
        self.enable_obstacle_avoidance = bool(enable_obstacle_avoidance)

        # Build segment list: [(p_start, p_end, duration_T), ...]
        self.nodes = [self.start_pos] + self.waypoints
        self.segments = []
        self.cumulative_times = [0.0]
        self.obstacles = []

        # Setup obstacles and avoidance offsets for each flight segment
        for i in range(len(self.nodes) - 1):
            p_a = self.nodes[i]
            p_b = self.nodes[i + 1]
            dist = float(np.linalg.norm(p_b - p_a))
            t_vel = 1.875 * dist / self.max_velocity
            t_acc = np.sqrt(5.7735 * dist / self.max_acceleration)
            duration = max(t_vel, t_acc, 2.0)

            # Determine obstacle for this segment
            obs_info = None
            avoid_offset = np.zeros(3, dtype=float)

            if self.enable_obstacle_avoidance and dist > 1.2:
                # Calculate lateral perpendicular normal in horizontal plane
                v_seg = p_b - p_a
                v_xy = np.array([v_seg[0], v_seg[1], 0.0], dtype=float)
                len_xy = float(np.linalg.norm(v_xy))
                if len_xy > 1e-3:
                    n_lat = np.array([-v_xy[1], v_xy[0], 0.0]) / len_xy
                else:
                    n_lat = np.array([1.0, 0.0, 0.0], dtype=float)

                # Alternate side for each obstacle: left or right (+ / -) for a realistic slalom bypass
                sign = 1.0 if (i % 2 == 0) else -1.0
                obs_radius = 0.45
                obs_height = max(p_a[2], p_b[2]) + 0.8
                obs_center = 0.5 * (p_a + p_b)
                # Obstacle is placed directly on the segment path line
                obs_pos = np.array([obs_center[0], obs_center[1], obs_height / 2.0], dtype=float)

                # Avoidance offset vector (pushes drone safely past the cylinder)
                avoid_offset = sign * 1.15 * n_lat

                obs_info = {
                    'id': i + 1,
                    'name': f"Obstacle_{i+1}",
                    'position': [float(obs_pos[0]), float(obs_pos[1]), float(obs_pos[2])],
                    'radius': float(obs_radius),
                    'height': float(obs_height),
                    'avoidance_offset': [float(avoid_offset[0]), float(avoid_offset[1]), float(avoid_offset[2])],
                    'segment_index': i
                }
                self.obstacles.append(obs_info)

            self.segments.append({
                'index': i,
                'p_start': p_a,
                'p_end': p_b,
                'distance': dist,
                'duration': duration,
                't_start': self.cumulative_times[-1],
                't_end': self.cumulative_times[-1] + duration,
                'avoidance_offset': avoid_offset,
                'obstacle': obs_info
            })
            self.cumulative_times.append(self.cumulative_times[-1] + duration)

        self.total_trajectory_time = self.cumulative_times[-1]

    def evaluate_segment(self, seg_idx: int, local_t: float) -> dict:
        """
        Evaluates position, velocity, acceleration, and jerk on segment `seg_idx`
        at local segment elapsed time `local_t` in [0, T_i].
        """
        if not self.segments:
            zeros = np.zeros(3, dtype=float)
            return {
                'pos': self.start_pos.copy(),
                'vel': zeros.copy(),
                'acc': zeros.copy(),
                'jerk': zeros.copy(),
                'segment_index': 0,
                's': 1.0
            }

        seg_idx = int(np.clip(seg_idx, 0, len(self.segments) - 1))
        seg = self.segments[seg_idx]
        T = seg['duration']
        delta_p = seg['p_end'] - seg['p_start']

        s = float(np.clip(local_t / T, 0.0, 1.0))
        s2 = s * s
        s3 = s2 * s
        s4 = s3 * s
        s5 = s4 * s

        # Quintic time-scaling s(t) in [0, 1]
        poly_p = 10.0 * s3 - 15.0 * s4 + 6.0 * s5
        poly_v = (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / T
        poly_a = (60.0 * s - 180.0 * s2 + 120.0 * s3) / (T * T)
        poly_j = (60.0 - 360.0 * s + 360.0 * s2) / (T * T * T)

        if local_t <= 0.0 or local_t >= T:
            poly_v = 0.0
            poly_a = 0.0
            poly_j = 0.0

        pos = seg['p_start'] + delta_p * poly_p
        vel = delta_p * poly_v
        acc = delta_p * poly_a
        jerk = delta_p * poly_j

        # Obstacle avoidance detour modification (Smooth C^2 septic polynomial)
        avoid_offset = seg.get('avoidance_offset', None)
        if avoid_offset is not None and np.linalg.norm(avoid_offset) > 1e-4:
            avoid_vec = np.asarray(avoid_offset, dtype=float)
            # P(s) = 64 * s^3 * (1 - s)^3 satisfies P(0)=0, P(1)=0, P'(0)=0, P'(1)=0, P''(0)=0, P''(1)=0, P(0.5)=1.0
            p_avoid = 64.0 * s3 * ((1.0 - s)**3)
            # P'(s) = 192 * s^2 * (1 - s)^2 * (1 - 2*s)
            dp_avoid = 192.0 * s2 * ((1.0 - s)**2) * (1.0 - 2.0 * s)
            # P''(s) = 192 * s * (1 - s) * (1 - 6*s + 6*s^2)
            ddp_avoid = 192.0 * s * (1.0 - s) * (1.0 - 6.0 * s + 6.0 * s2)
            # P'''(s) = 192 * (1 - 12*s + 30*s^2 - 20*s^3)
            dddp_avoid = 192.0 * (1.0 - 12.0 * s + 30.0 * s2 - 20.0 * s3)

            if local_t <= 0.0 or local_t >= T:
                dp_avoid = 0.0
                ddp_avoid = 0.0
                dddp_avoid = 0.0

            pos = pos + avoid_vec * p_avoid
            vel = vel + avoid_vec * (dp_avoid / T)
            acc = acc + avoid_vec * (ddp_avoid / (T * T))
            jerk = jerk + avoid_vec * (dddp_avoid / (T * T * T))

        return {
            'pos': pos,
            'vel': vel,
            'acc': acc,
            'jerk': jerk,
            'segment_index': seg_idx,
            's': s,
            'target_waypoint': seg['p_end'].copy()
        }

    def sample_full_reference_path(self, num_points_per_segment: int = 80) -> np.ndarray:
        """
        Returns an (N, 3) array of 3D coordinates sampling the complete reference
        trajectory across all segments (Start -> WP1 -> WP2 -> WP3 -> ...).
        """
        if not self.segments:
            return self.start_pos.reshape(1, 3)

        pts = []
        for i, seg in enumerate(self.segments):
            t_vals = np.linspace(0.0, seg['duration'], num_points_per_segment, endpoint=(i == len(self.segments) - 1))
            for t_val in t_vals:
                res = self.evaluate_segment(i, float(t_val))
                pts.append(res['pos'])
        return np.array(pts, dtype=float)


class TrajectoryNode(Node):
    """
    ROS 2 Node that generates the continuous 3D reference state and persistent
    reference trajectory path for Gazebo and RViz2.
    """

    def __init__(self):
        super().__init__('trajectory_node')

        self.declare_parameter('config_file', '')
        self.declare_parameter('initial_position', [0.0, 0.0, 2.0])
        self.declare_parameter('waypoints_flat', [5.0, 3.0, 4.0, 10.0, -2.0, 5.0, 15.0, 4.0, 6.0])
        self.declare_parameter('max_velocity', 2.5)
        self.declare_parameter('max_acceleration', 5.0)
        self.declare_parameter('control_frequency', 200.0)

        config_file = self.get_parameter('config_file').value
        if config_file and os.path.exists(config_file):
            start_pos, waypoints, _ = load_waypoints_from_yaml(config_file)
        else:
            start_pos = list(self.get_parameter('initial_position').value)
            flat = list(self.get_parameter('waypoints_flat').value)
            waypoints = [flat[i:i + 3] for i in range(0, len(flat), 3)]

        max_vel = float(self.get_parameter('max_velocity').value)
        max_acc = float(self.get_parameter('max_acceleration').value)
        freq = float(self.get_parameter('control_frequency').value)

        self.generator = QuinticSegmentTrajectoryGenerator(
            start_pos, waypoints, max_vel, max_acc
        )

        # Active segment state synchronized with waypoint_manager
        self.active_segment_idx = 0
        self.segment_switch_ros_time = self.get_clock().now().nanoseconds * 1e-9
        self.mission_complete = False

        # Precompute complete reference path message
        self.full_ref_points = self.generator.sample_full_reference_path(num_points_per_segment=80)

        self.wp_sub = self.create_subscription(
            Float64MultiArray, '/uav/waypoint_state', self.wp_state_callback, 20
        )
        self.ref_state_pub = self.create_publisher(
            Float64MultiArray, '/uav/reference_state', 20
        )
        self.ref_path_pub = self.create_publisher(
            Path, '/uav/reference_path', 10
        )

        self.control_timer = self.create_timer(1.0 / max(freq, 20.0), self.publish_reference_state)
        self.path_timer = self.create_timer(1.0, self.publish_reference_path)
        self.publish_reference_path()

        self.get_logger().info(
            f"Trajectory Generator ready | {len(self.generator.segments)} segments | "
            f"Total nominal duration: {self.generator.total_trajectory_time:.2f} s"
        )

    def wp_state_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 10:
            new_idx = int(round(msg.data[0]))
            self.mission_complete = (msg.data[9] > 0.5)
            if new_idx != self.active_segment_idx:
                self.active_segment_idx = new_idx
                self.segment_switch_ros_time = self.get_clock().now().nanoseconds * 1e-9

    def publish_reference_state(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        local_t = now_s - self.segment_switch_ros_time
        ref = self.generator.evaluate_segment(self.active_segment_idx, local_t)

        msg = Float64MultiArray()
        # [x_ref, y_ref, z_ref, vx_ref, vy_ref, vz_ref, ax_ref, ay_ref, az_ref, jx_ref, jy_ref, jz_ref, seg_idx]
        msg.data = [
            float(ref['pos'][0]), float(ref['pos'][1]), float(ref['pos'][2]),
            float(ref['vel'][0]), float(ref['vel'][1]), float(ref['vel'][2]),
            float(ref['acc'][0]), float(ref['acc'][1]), float(ref['acc'][2]),
            float(ref['jerk'][0]), float(ref['jerk'][1]), float(ref['jerk'][2]),
            float(ref['segment_index'])
        ]
        self.ref_state_pub.publish(msg)

    def publish_reference_path(self):
        path_msg = Path()
        now_msg = self.get_clock().now().to_msg()
        path_msg.header.stamp = now_msg
        path_msg.header.frame_id = 'world'

        for pt in self.full_ref_points:
            pose = PoseStamped()
            pose.header.stamp = now_msg
            pose.header.frame_id = 'world'
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = float(pt[2])
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.ref_path_pub.publish(path_msg)


def main(args=None):
    if not ROS2_AVAILABLE:
        print("ROS 2 (rclpy) is not sourced in this shell.")
        return
    rclpy.init(args=args)
    node = TrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
