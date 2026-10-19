#!/usr/bin/env python3
"""
CSV Data Logger & Performance Evaluator Node (data_logger.py)
=============================================================
Continuously records UAV time-series data to CSV with the exact columns required:
  timestamp, actual_x, actual_y, actual_z, reference_x, reference_y, reference_z,
  error_x, error_y, error_z, position_error, velocity_x, velocity_y, velocity_z,
  roll, pitch, yaw
plus paper-specific quaternion, angular velocity, backstepping error delta, and
control torque columns.

Also computes comprehensive Controller Performance Metrics:
  - RMSE of X position
  - RMSE of Y position
  - RMSE of Z position
  - Overall 3D position RMSE
  - Maximum tracking error
  - Mean tracking error
  - Settling time (position and attitude)
  - Waypoint reaching time (for each waypoint)
  - Trajectory tracking accuracy (%)
"""

from __future__ import annotations
import os
import csv
import json
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float64MultiArray
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object

from uav_simulation.quaternion_math import quat_to_euler


CSV_COLUMNS = [
    'timestamp',
    'actual_x',
    'actual_y',
    'actual_z',
    'reference_x',
    'reference_y',
    'reference_z',
    'error_x',
    'error_y',
    'error_z',
    'position_error',
    'velocity_x',
    'velocity_y',
    'velocity_z',
    'roll',
    'pitch',
    'yaw',
    # Paper-specific attitude & control variables
    'q0', 'q1', 'q2', 'q3',
    'q_ref0', 'q_ref1', 'q_ref2', 'q_ref3',
    'q_err0', 'q_err1', 'q_err2', 'q_err3',
    'omega_x', 'omega_y', 'omega_z',
    'omega_ref_x', 'omega_ref_y', 'omega_ref_z',
    'omega_err_x', 'omega_err_y', 'omega_err_z',
    'delta_x', 'delta_y', 'delta_z',
    'tau_x', 'tau_y', 'tau_z',
    'thrust_u1',
    'active_waypoint',
    'dist_to_waypoint'
]


def evaluate_performance_metrics(rows: list, waypoint_reach_times: dict = None, start_pos=None, waypoints=None) -> dict:
    """
    Computes all required quantitative controller performance metrics from logged rows.
    """
    if not rows:
        return {}

    t = np.array([float(r['timestamp']) for r in rows])
    ex = np.array([float(r['error_x']) for r in rows])
    ey = np.array([float(r['error_y']) for r in rows])
    ez = np.array([float(r['error_z']) for r in rows])
    e_pos = np.array([float(r['position_error']) for r in rows])

    rmse_x = float(np.sqrt(np.mean(ex ** 2)))
    rmse_y = float(np.sqrt(np.mean(ey ** 2)))
    rmse_z = float(np.sqrt(np.mean(ez ** 2)))
    overall_rmse = float(np.sqrt(np.mean(ex ** 2 + ey ** 2 + ez ** 2)))
    max_error = float(np.max(e_pos))
    mean_error = float(np.mean(e_pos))

    # Settling time after reaching final waypoint segment (within 0.05 m of steady state)
    above_thresh = np.where(e_pos > 0.05)[0]
    if len(above_thresh) == 0:
        settling_time = 0.0
    elif above_thresh[-1] < len(t) - 1:
        settling_time = float(t[above_thresh[-1] + 1])
    else:
        settling_time = float(t[-1])

    # Total path length for relative trajectory tracking accuracy
    total_span = 18.5
    if start_pos is not None and waypoints is not None:
        pts = [np.asarray(start_pos, dtype=float)] + [np.asarray(w, dtype=float) for w in waypoints]
        total_span = sum(float(np.linalg.norm(pts[i+1] - pts[i])) for i in range(len(pts) - 1))

    tracking_accuracy_pct = float(max(0.0, 100.0 * (1.0 - (overall_rmse / max(total_span, 1.0)))))

    # Attitude quaternion & angular velocity RMSE (Paper metrics)
    q_err_vec_norm = np.array([
        np.linalg.norm([float(r['q_err1']), float(r['q_err2']), float(r['q_err3'])])
        for r in rows
    ])
    w_err_norm = np.array([
        np.linalg.norm([float(r['omega_err_x']), float(r['omega_err_y']), float(r['omega_err_z'])])
        for r in rows
    ])
    delta_norm = np.array([
        np.linalg.norm([float(r['delta_x']), float(r['delta_y']), float(r['delta_z'])])
        for r in rows
    ])

    return {
        'rmse_x_m': rmse_x,
        'rmse_y_m': rmse_y,
        'rmse_z_m': rmse_z,
        'overall_position_rmse_m': overall_rmse,
        'max_tracking_error_m': max_error,
        'mean_tracking_error_m': mean_error,
        'settling_time_s': settling_time,
        'waypoint_reaching_times_s': waypoint_reach_times or {},
        'trajectory_tracking_accuracy_pct': tracking_accuracy_pct,
        'quaternion_vector_error_rmse': float(np.sqrt(np.mean(q_err_vec_norm ** 2))),
        'angular_velocity_error_rmse_rad_s': float(np.sqrt(np.mean(w_err_norm ** 2))),
        'backstepping_delta_rmse_rad_s': float(np.sqrt(np.mean(delta_norm ** 2))),
    }


class DataLoggerNode(Node):
    """
    ROS 2 Node that logs UAV trajectory & attitude tracking data to CSV
    and outputs performance evaluation metrics upon completion.
    """

    def __init__(self):
        super().__init__('data_logger')
        self.declare_parameter('csv_output_path', 'uav_trajectory_log.csv')
        self.declare_parameter('logging_frequency', 20.0)

        self.csv_path = str(self.get_parameter('csv_output_path').value)
        freq = float(self.get_parameter('logging_frequency').value)

        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.rows = []
        self.wp_reach_times = {}

        # Current buffered values
        self.pos = np.array([0.0, 0.0, 2.0])
        self.vel = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.omega = np.zeros(3)
        self.pos_ref = np.array([0.0, 0.0, 2.0])
        self.q_ref = np.array([1.0, 0.0, 0.0, 0.0])
        self.q_err = np.array([1.0, 0.0, 0.0, 0.0])
        self.omega_ref = np.zeros(3)
        self.omega_err = np.zeros(3)
        self.delta = np.zeros(3)
        self.tau = np.zeros(3)
        self.u1 = 1.2 * 9.81
        self.active_wp = 1
        self.dist_to_wp = 0.0

        # Open CSV file for real-time streaming write
        os.makedirs(os.path.dirname(os.path.abspath(self.csv_path)), exist_ok=True)
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_COLUMNS)
        self.csv_writer.writeheader()

        self.odom_sub = self.create_subscription(Odometry, '/uav/odometry', self.odom_cb, 20)
        self.ref_sub = self.create_subscription(Float64MultiArray, '/uav/reference_state', self.ref_cb, 20)
        self.wp_sub = self.create_subscription(Float64MultiArray, '/uav/waypoint_state', self.wp_cb, 20)
        self.diag_sub = self.create_subscription(Float64MultiArray, '/uav/controller_diagnostics', self.diag_cb, 20)

        self.timer = self.create_timer(1.0 / max(freq, 1.0), self.log_step)
        self.get_logger().info(f"CSV Data Logger recording to: {os.path.abspath(self.csv_path)}")

    def odom_cb(self, msg: Odometry):
        self.pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        self.vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        self.q = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])
        self.omega = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])

    def ref_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 3:
            self.pos_ref = np.array(msg.data[0:3])

    def wp_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 9:
            new_wp = int(round(msg.data[0])) + 1
            now_t = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
            if new_wp > self.active_wp:
                self.wp_reach_times[f"Waypoint_{self.active_wp}"] = round(now_t, 3)
            self.active_wp = new_wp
            self.dist_to_wp = float(msg.data[8])

    def diag_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 21:
            self.q_ref = np.array(msg.data[0:4])
            self.q_err = np.array(msg.data[4:8])
            self.omega_ref = np.array(msg.data[8:11])
            self.omega_err = np.array(msg.data[11:14])
            self.delta = np.array(msg.data[14:17])
            self.tau = np.array(msg.data[17:20])
            self.u1 = float(msg.data[20])

    def log_step(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
        err = self.pos - self.pos_ref
        pos_err_norm = float(np.linalg.norm(err))
        roll_deg, pitch_deg, yaw_deg = quat_to_euler(self.q, degrees=True)

        row = {
            'timestamp': round(t, 4),
            'actual_x': round(float(self.pos[0]), 5),
            'actual_y': round(float(self.pos[1]), 5),
            'actual_z': round(float(self.pos[2]), 5),
            'reference_x': round(float(self.pos_ref[0]), 5),
            'reference_y': round(float(self.pos_ref[1]), 5),
            'reference_z': round(float(self.pos_ref[2]), 5),
            'error_x': round(float(err[0]), 5),
            'error_y': round(float(err[1]), 5),
            'error_z': round(float(err[2]), 5),
            'position_error': round(pos_err_norm, 5),
            'velocity_x': round(float(self.vel[0]), 5),
            'velocity_y': round(float(self.vel[1]), 5),
            'velocity_z': round(float(self.vel[2]), 5),
            'roll': round(roll_deg, 4),
            'pitch': round(pitch_deg, 4),
            'yaw': round(yaw_deg, 4),
            'q0': round(float(self.q[0]), 6),
            'q1': round(float(self.q[1]), 6),
            'q2': round(float(self.q[2]), 6),
            'q3': round(float(self.q[3]), 6),
            'q_ref0': round(float(self.q_ref[0]), 6),
            'q_ref1': round(float(self.q_ref[1]), 6),
            'q_ref2': round(float(self.q_ref[2]), 6),
            'q_ref3': round(float(self.q_ref[3]), 6),
            'q_err0': round(float(self.q_err[0]), 6),
            'q_err1': round(float(self.q_err[1]), 6),
            'q_err2': round(float(self.q_err[2]), 6),
            'q_err3': round(float(self.q_err[3]), 6),
            'omega_x': round(float(self.omega[0]), 5),
            'omega_y': round(float(self.omega[1]), 5),
            'omega_z': round(float(self.omega[2]), 5),
            'omega_ref_x': round(float(self.omega_ref[0]), 5),
            'omega_ref_y': round(float(self.omega_ref[1]), 5),
            'omega_ref_z': round(float(self.omega_ref[2]), 5),
            'omega_err_x': round(float(self.omega_err[0]), 5),
            'omega_err_y': round(float(self.omega_err[1]), 5),
            'omega_err_z': round(float(self.omega_err[2]), 5),
            'delta_x': round(float(self.delta[0]), 5),
            'delta_y': round(float(self.delta[1]), 5),
            'delta_z': round(float(self.delta[2]), 5),
            'tau_x': round(float(self.tau[0]), 5),
            'tau_y': round(float(self.tau[1]), 5),
            'tau_z': round(float(self.tau[2]), 5),
            'thrust_u1': round(float(self.u1), 5),
            'active_waypoint': self.active_wp,
            'dist_to_waypoint': round(self.dist_to_wp, 5),
        }
        self.rows.append(row)
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def save_summary_and_close(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        metrics = evaluate_performance_metrics(self.rows, self.wp_reach_times)
        if metrics:
            summary_path = os.path.splitext(self.csv_path)[0] + "_metrics.json"
            with open(summary_path, 'w', encoding='utf-8') as f:
                json.dump(metrics, f, indent=2)
            self.get_logger().info(f"Performance metrics saved to: {summary_path}")


def main(args=None):
    if not ROS2_AVAILABLE:
        print("ROS 2 (rclpy) is not sourced in this shell.")
        return
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_summary_and_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
