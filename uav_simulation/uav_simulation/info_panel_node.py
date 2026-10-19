#!/usr/bin/env python3
"""
Real-Time Coordinate Information Panel Node (info_panel_node.py)
================================================================
Displays a live right-side telemetry panel docked on the right side of the
Gazebo simulation window (and publishes 3D text overlay + /uav/telemetry_text).

Continuously displays:
  - Time: t = ...
  - Current Position: X, Y, Z
  - Reference Position: X_ref, Y_ref, Z_ref
  - Position Error: e_x, e_y, e_z
  - Euclidean Position Error: e
  - Current Waypoint: Waypoint 1 / Waypoint 2 / Waypoint 3
  - Distance to Current Waypoint: ...
  - Velocity: Vx, Vy, Vz
  - Attitude (Euler Angles): Roll, Pitch, Yaw
  - Quaternion & Backstepping States from Paper: q_err, delta, Torque tau
"""

from __future__ import annotations
import threading
import numpy as np

try:
    import tkinter as tk
    TK_AVAILABLE = True
except ImportError:
    TK_AVAILABLE = False

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float64MultiArray, String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object

from uav_simulation.quaternion_math import quat_to_euler


def format_telemetry_panel_text(state: dict) -> str:
    """
    Formats all required real-time coordinate, error, waypoint, velocity,
    and attitude information exactly as specified.
    """
    t = state.get('t', 0.0)
    x, y, z = state.get('pos', (0.0, 0.0, 2.0))
    xr, yr, zr = state.get('pos_ref', (0.0, 0.0, 2.0))
    ex, ey, ez = x - xr, y - yr, z - zr
    e_norm = float(np.sqrt(ex * ex + ey * ey + ez * ez))
    wp_label = state.get('wp_label', 'Waypoint 1')
    wp_coords = state.get('wp_coords', (5.0, 3.0, 4.0))
    dist_wp = state.get('dist_to_wp', 0.0)
    vx, vy, vz = state.get('vel', (0.0, 0.0, 0.0))
    roll_deg, pitch_deg, yaw_deg = state.get('rpy_deg', (0.0, 0.0, 0.0))
    q_err = state.get('q_err', (1.0, 0.0, 0.0, 0.0))
    delta = state.get('delta', (0.0, 0.0, 0.0))
    tau = state.get('tau', (0.0, 0.0, 0.0))

    lines = [
        "==========================================",
        "   UAV REAL-TIME COORDINATE INFO PANEL    ",
        "==========================================",
        f"Time:",
        f"  t     = {t:8.3f} s",
        "",
        f"Current Position (m):",
        f"  X     = {x:8.3f} m",
        f"  Y     = {y:8.3f} m",
        f"  Z     = {z:8.3f} m",
        "",
        f"Reference Position (m):",
        f"  X_ref = {xr:8.3f} m",
        f"  Y_ref = {yr:8.3f} m",
        f"  Z_ref = {zr:8.3f} m",
        "",
        f"Position Error (m):",
        f"  e_x   = {ex:+8.4f} m",
        f"  e_y   = {ey:+8.4f} m",
        f"  e_z   = {ez:+8.4f} m",
        "",
        f"Euclidean Position Error:",
        f"  e     = {e_norm:8.4f} m",
        "",
        f"Current Waypoint:",
        f"  {wp_label}",
        f"  Target: ({wp_coords[0]:.1f}, {wp_coords[1]:.1f}, {wp_coords[2]:.1f}) m",
        "",
        f"Distance to Current Waypoint:",
        f"  d_wp  = {dist_wp:8.3f} m",
        "",
        f"Linear Velocity:",
        f"  Vx    = {vx:+8.3f} m/s",
        f"  Vy    = {vy:+8.3f} m/s",
        f"  Vz    = {vz:+8.3f} m/s",
        "",
        f"Attitude Angles (Euler):",
        f"  Roll  = {roll_deg:+8.2f} deg",
        f"  Pitch = {pitch_deg:+8.2f} deg",
        f"  Yaw   = {yaw_deg:+8.2f} deg",
        "------------------------------------------",
        f"Control States:",
        f"  q_err = [{q_err[0]:+.4f}, {q_err[1]:+.4f}, {q_err[2]:+.4f}, {q_err[3]:+.4f}]",
        f"  delta = [{delta[0]:+.4f}, {delta[1]:+.4f}, {delta[2]:+.4f}]",
        f"  tau   = [{tau[0]:+.4f}, {tau[1]:+.4f}, {tau[2]:+.4f}]",
        "==========================================",
    ]
    return "\n".join(lines)


class InfoPanelNode(Node):
    """
    ROS 2 Node that subscribes to UAV state, reference trajectory, and waypoint
    topics, and updates the live Right-Side Coordinate Information Panel window
    docked alongside Gazebo.
    """

    def __init__(self):
        super().__init__('info_panel_node')
        self.declare_parameter('enable_gui', True)
        self.enable_gui = bool(self.get_parameter('enable_gui').value)

        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.state = {
            't': 0.0,
            'pos': (0.0, 0.0, 2.0),
            'pos_ref': (0.0, 0.0, 2.0),
            'wp_label': 'Waypoint 1',
            'wp_coords': (5.0, 3.0, 4.0),
            'dist_to_wp': 6.164,
            'vel': (0.0, 0.0, 0.0),
            'rpy_deg': (0.0, 0.0, 0.0),
            'q_err': (1.0, 0.0, 0.0, 0.0),
            'delta': (0.0, 0.0, 0.0),
            'tau': (0.0, 0.0, 0.0),
        }

        self.odom_sub = self.create_subscription(
            Odometry, '/uav/odometry', self.odom_callback, 20
        )
        self.ref_sub = self.create_subscription(
            Float64MultiArray, '/uav/reference_state', self.ref_callback, 20
        )
        self.wp_sub = self.create_subscription(
            Float64MultiArray, '/uav/waypoint_state', self.wp_callback, 20
        )
        self.label_sub = self.create_subscription(
            String, '/uav/active_waypoint_label', self.label_callback, 20
        )
        self.diag_sub = self.create_subscription(
            Float64MultiArray, '/uav/controller_diagnostics', self.diag_callback, 20
        )
        self.text_pub = self.create_publisher(String, '/uav/telemetry_text', 10)

        self.timer = self.create_timer(0.05, self.publish_telemetry)
        self.get_logger().info("Live Coordinate Information Panel Node initialized.")

    def odom_callback(self, msg: Odometry):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        self.state['t'] = now_s - self.start_time
        self.state['pos'] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        self.state['vel'] = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        )
        q = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ], dtype=float)
        self.state['rpy_deg'] = quat_to_euler(q, degrees=True)

    def ref_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 3:
            self.state['pos_ref'] = (msg.data[0], msg.data[1], msg.data[2])

    def wp_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 9:
            self.state['wp_coords'] = (msg.data[2], msg.data[3], msg.data[4])
            self.state['dist_to_wp'] = msg.data[8]

    def label_callback(self, msg: String):
        self.state['wp_label'] = msg.data

    def diag_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 20:
            self.state['q_err'] = tuple(msg.data[4:8])
            self.state['delta'] = tuple(msg.data[14:17])
            self.state['tau'] = tuple(msg.data[17:20])

    def publish_telemetry(self):
        txt = format_telemetry_panel_text(self.state)
        msg = String()
        msg.data = txt
        self.text_pub.publish(msg)


def run_tkinter_hud(node: InfoPanelNode):
    """
    Creates a docked right-side HUD window on top of the Gazebo simulation window.
    """
    root = tk.Tk()
    root.title("Gazebo UAV Telemetry Panel")
    root.configure(bg="#111622")
    root.attributes("-topmost", True)

    # Position on the right side of the display
    screen_w = root.winfo_screenwidth()
    panel_w = 390
    panel_h = 780
    pos_x = max(0, screen_w - panel_w - 15)
    pos_y = 45
    root.geometry(f"{panel_w}x{panel_h}+{pos_x}+{pos_y}")

    label = tk.Label(
        root,
        text="",
        font=("Consolas", 10, "bold"),
        fg="#00f5d4",
        bg="#111622",
        justify=tk.LEFT,
        anchor="nw",
        padx=14,
        pady=12
    )
    label.pack(fill=tk.BOTH, expand=True)

    def refresh():
        label.config(text=format_telemetry_panel_text(node.state))
        root.after(50, refresh)

    refresh()
    root.mainloop()


def main(args=None):
    if not ROS2_AVAILABLE:
        print("ROS 2 (rclpy) is not sourced in this shell.")
        return
    rclpy.init(args=args)
    node = InfoPanelNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    if TK_AVAILABLE and node.enable_gui and os.environ.get('DISPLAY', os.name == 'nt'):
        try:
            run_tkinter_hud(node)
        except Exception as exc:
            node.get_logger().warn(f"GUI HUD fallback to console ({exc})")
            spin_thread.join()
    else:
        spin_thread.join()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
