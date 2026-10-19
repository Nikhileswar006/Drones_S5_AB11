#!/usr/bin/env python3
"""
Interactive 3D UAV Flight & Real-Time Coordinate Panel Viewer
=============================================================
Provides an immediate, interactive 3D simulation display that you can run
directly on Windows (or Linux/WSL) with one command:
    python scripts/interactive_3d_viewer.py

Features:
  - Left Viewport: Interactive 3D visualization showing the 3D Quadrotor UAV
    (with 4 arms, rotors, coordinate axes), Start marker (0,0,2), Waypoints
    (WP1, WP2, WP3), the persistent Cyan Reference Path, and the growing Orange
    Actual UAV Trajectory trail.
  - Right Viewport: The live Real-Time Coordinate Information Panel displaying
    all required variables updating continuously frame-by-frame.
  - Controls: Play/Pause, Reset, and Speed Slider.
  - 3D Interaction: Click and drag with mouse to orbit/rotate in 3D, right-click to zoom.
"""

import os
import sys
import csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider

PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from uav_simulation.waypoint_manager import load_waypoints_from_yaml
from uav_simulation.info_panel_node import format_telemetry_panel_text


def load_flight_data(csv_path: str):
    if not os.path.exists(csv_path):
        raise FileNotFoundError(
            f"Flight log not found at {csv_path}. Run 'python scripts/run_standalone_verification.py' first!"
        )
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    return rows


class InteractiveUAVViewer:
    def __init__(self, csv_path: str, yaml_path: str):
        self.rows = load_flight_data(csv_path)
        self.start_pos, self.waypoints, self.tol = load_waypoints_from_yaml(yaml_path)
        self.num_frames = len(self.rows)
        self.current_frame = 0
        self.is_paused = False
        self.playback_speed = 1.0

        # Precompute coordinate arrays
        self.t = np.array([float(r['timestamp']) for r in self.rows])
        self.rx = np.array([float(r['reference_x']) for r in self.rows])
        self.ry = np.array([float(r['reference_y']) for r in self.rows])
        self.rz = np.array([float(r['reference_z']) for r in self.rows])
        self.ax = np.array([float(r['actual_x']) for r in self.rows])
        self.ay = np.array([float(r['actual_y']) for r in self.rows])
        self.az = np.array([float(r['actual_z']) for r in self.rows])

        # Setup figure with dark high-tech HUD aesthetic
        self.fig = plt.figure(figsize=(15, 8.5), facecolor='#0e1420')
        self.fig.canvas.manager.set_window_title(
            "UAV 3D Simulation & Real-Time Telemetry (CACS 2024 Quaternion Controller)"
        )

        # 3D Viewport on Left
        self.ax3d = self.fig.add_axes([0.03, 0.12, 0.65, 0.83], projection='3d', facecolor='#151b28')
        # HUD Panel on Right
        self.ax_hud = self.fig.add_axes([0.69, 0.12, 0.29, 0.83], facecolor='#0a0e17')
        self.ax_hud.axis('off')

        self._setup_3d_scene()
        self._setup_hud_panel()
        self._setup_controls()

        # Animation interval (40 ms -> 25 FPS)
        self.anim = FuncAnimation(
            self.fig, self._update, interval=40, blit=False, cache_frame_data=False
        )

    def _setup_3d_scene(self):
        self.ax3d.set_title(
            "3D Quadrotor UAV Flight Simulation (Gazebo / RViz View)",
            color='#00f5d4', fontsize=12, fontweight='bold', pad=14
        )
        self.ax3d.set_xlabel('X Position (m)', color='#a0aec0', labelpad=8)
        self.ax3d.set_ylabel('Y Position (m)', color='#a0aec0', labelpad=8)
        self.ax3d.set_zlabel('Z Altitude (m)', color='#a0aec0', labelpad=8)
        self.ax3d.tick_params(colors='#718096')

        # Bounds with padding
        all_x = np.concatenate(([self.start_pos[0]], [w[0] for w in self.waypoints], self.ax))
        all_y = np.concatenate(([self.start_pos[1]], [w[1] for w in self.waypoints], self.ay))
        all_z = np.concatenate(([self.start_pos[2]], [w[2] for w in self.waypoints], self.az))

        self.ax3d.set_xlim([min(all_x) - 1.0, max(all_x) + 1.5])
        self.ax3d.set_ylim([min(all_y) - 1.5, max(all_y) + 1.5])
        self.ax3d.set_zlim([0.0, max(all_z) + 1.5])
        self.ax3d.view_init(elev=24, azim=-58)

        # 1. World Coordinate Axes at (0, 0, 0)
        self.ax3d.quiver(0, 0, 0, 1.5, 0, 0, color='#e63946', arrow_length_ratio=0.15, linewidth=2)
        self.ax3d.quiver(0, 0, 0, 0, 1.5, 0, color='#2a9d8f', arrow_length_ratio=0.15, linewidth=2)
        self.ax3d.quiver(0, 0, 0, 0, 0, 1.5, color='#0077b6', arrow_length_ratio=0.15, linewidth=2)
        self.ax3d.text(1.7, 0, 0, "X", color='#e63946', fontweight='bold')
        self.ax3d.text(0, 1.7, 0, "Y", color='#2a9d8f', fontweight='bold')
        self.ax3d.text(0, 0, 1.7, "Z", color='#0077b6', fontweight='bold')

        # 2. Start Marker (Blue sphere)
        self.ax3d.scatter(
            [self.start_pos[0]], [self.start_pos[1]], [self.start_pos[2]],
            color='#3a86ff', s=130, marker='o', label=f'Start ({self.start_pos[0]:.0f},{self.start_pos[1]:.0f},{self.start_pos[2]:.0f})'
        )
        self.ax3d.text(
            self.start_pos[0], self.start_pos[1], self.start_pos[2] + 0.35,
            " Start", color='#3a86ff', fontweight='bold'
        )

        # 3. Waypoint Markers (Spheres + Labels)
        wp_colors = ['#ff007f', '#ffbe0b', '#00f5d4']
        for i, wp in enumerate(self.waypoints):
            c = wp_colors[i % len(wp_colors)]
            self.ax3d.scatter(
                [wp[0]], [wp[1]], [wp[2]],
                color=c, s=150, marker='^', edgecolors='white',
                label=f'WP{i+1} ({wp[0]:.0f},{wp[1]:.0f},{wp[2]:.0f})'
            )
            self.ax3d.text(
                wp[0], wp[1], wp[2] + 0.35,
                f" WP{i+1}", color=c, fontweight='bold'
            )

        # 4. Persistent Complete Reference Trajectory (Cyan dashed line)
        self.ax3d.plot(
            self.rx, self.ry, self.rz,
            '--', color='#00f5d4', linewidth=2.0, alpha=0.85,
            label='Reference Trajectory (Quintic Path)'
        )

        # 5. Dynamic Actual Trajectory Trail (Orange/Red line)
        self.trail_line, = self.ax3d.plot([], [], [], '-', color='#ff4d29', linewidth=2.2, label='Actual UAV Trajectory')

        # 6. Current UAV Marker (Fuselage + 4 Rotor Disks + Coordinate Frame)
        self.uav_scatter = self.ax3d.scatter([], [], [], color='#ffff00', s=180, marker='o', edgecolors='red', label='Current UAV')
        self.arm1_line, = self.ax3d.plot([], [], [], '-', color='#ff0000', linewidth=3)
        self.arm2_line, = self.ax3d.plot([], [], [], '-', color='#00ffcc', linewidth=2)

        # 7. Active Waypoint Highlight Halo
        self.active_wp_scatter = self.ax3d.scatter([], [], [], color='#ffff00', s=280, alpha=0.45)

        self.ax3d.legend(
            loc='upper left', facecolor='#0e1420', edgecolor='#00f5d4',
            labelcolor='white', fontsize=8.5
        )

    def _setup_hud_panel(self):
        # Draw background container box
        init_text = format_telemetry_panel_text(self._get_state_dict(0))
        self.hud_text_artist = self.ax_hud.text(
            0.04, 0.98, init_text,
            transform=self.ax_hud.transAxes,
            fontfamily='monospace',
            fontsize=8.5,
            color='#00f5d4',
            verticalalignment='top',
            bbox=dict(boxstyle='round,pad=0.7', facecolor='#111827', edgecolor='#00f5d4', linewidth=1.5)
        )

    def _setup_controls(self):
        # Play / Pause Button
        ax_btn_pause = self.fig.add_axes([0.05, 0.03, 0.10, 0.05])
        self.btn_pause = Button(ax_btn_pause, 'Pause', color='#1e293b', hovercolor='#334155')
        self.btn_pause.label.set_color('white')
        self.btn_pause.on_clicked(self._toggle_pause)

        # Reset Button
        ax_btn_reset = self.fig.add_axes([0.17, 0.03, 0.10, 0.05])
        self.btn_reset = Button(ax_btn_reset, 'Restart', color='#1e293b', hovercolor='#334155')
        self.btn_reset.label.set_color('white')
        self.btn_reset.on_clicked(self._reset)

        # Speed Slider
        ax_slider = self.fig.add_axes([0.33, 0.04, 0.28, 0.03], facecolor='#1e293b')
        self.slider_speed = Slider(ax_slider, 'Speed', 0.2, 5.0, valinit=1.0, color='#00f5d4')
        self.slider_speed.label.set_color('white')
        self.slider_speed.valtext.set_color('white')
        self.slider_speed.on_changed(self._change_speed)

        # Progress / Time Display
        self.ax_time_text = self.fig.text(0.64, 0.04, "", color='#00f5d4', fontsize=10, fontweight='bold')

    def _toggle_pause(self, event):
        self.is_paused = not self.is_paused
        self.btn_pause.label.set_text('Play' if self.is_paused else 'Pause')

    def _reset(self, event):
        self.current_frame = 0

    def _change_speed(self, val):
        self.playback_speed = float(val)

    def _get_state_dict(self, idx: int) -> dict:
        r = self.rows[idx]
        pos = (float(r['actual_x']), float(r['actual_y']), float(r['actual_z']))
        pos_ref = (float(r['reference_x']), float(r['reference_y']), float(r['reference_z']))
        vel = (float(r['velocity_x']), float(r['velocity_y']), float(r['velocity_z']))
        rpy_deg = (float(r['roll']), float(r['pitch']), float(r['yaw']))
        q_err = (float(r['q_err0']), float(r['q_err1']), float(r['q_err2']), float(r['q_err3']))
        delta = (float(r['delta_x']), float(r['delta_y']), float(r['delta_z']))
        tau = (float(r['tau_x']), float(r['tau_y']), float(r['tau_z']))

        wp_num = int(r.get('active_waypoint', 1))
        wp_idx = min(wp_num - 1, len(self.waypoints) - 1)
        wp_coords = tuple(self.waypoints[wp_idx])

        return {
            't': float(r['timestamp']),
            'pos': pos,
            'pos_ref': pos_ref,
            'wp_label': f"Waypoint {wp_num}",
            'wp_coords': wp_coords,
            'dist_to_wp': float(r.get('dist_to_waypoint', 0.0)),
            'vel': vel,
            'rpy_deg': rpy_deg,
            'q_err': q_err,
            'delta': delta,
            'tau': tau,
        }

    def _update(self, _):
        if not self.is_paused:
            step_inc = max(1, int(round(self.playback_speed * 1.5)))
            self.current_frame = (self.current_frame + step_inc) % self.num_frames

        idx = self.current_frame
        curr_x = self.ax[idx]
        curr_y = self.ay[idx]
        curr_z = self.az[idx]

        # 1. Update actual trajectory trail
        self.trail_line.set_data(self.ax[:idx + 1], self.ay[:idx + 1])
        self.trail_line.set_3d_properties(self.az[:idx + 1])

        # 2. Update UAV body model (fuselage + arms)
        self.uav_scatter._offsets3d = ([curr_x], [curr_y], [curr_z])

        # Draw Quadrotor X/Y Cross Arms (Arm length 0.35m)
        L = 0.35
        self.arm1_line.set_data([curr_x - L, curr_x + L], [curr_y, curr_y])
        self.arm1_line.set_3d_properties([curr_z, curr_z])
        self.arm2_line.set_data([curr_x, curr_x], [curr_y - L, curr_y + L])
        self.arm2_line.set_3d_properties([curr_z, curr_z])

        # 3. Update active waypoint halo
        r = self.rows[idx]
        wp_num = int(r.get('active_waypoint', 1))
        wp_idx = min(wp_num - 1, len(self.waypoints) - 1)
        active_wp = self.waypoints[wp_idx]
        self.active_wp_scatter._offsets3d = ([active_wp[0]], [active_wp[1]], [active_wp[2]])

        # 4. Update right-side telemetry panel text
        state = self._get_state_dict(idx)
        self.hud_text_artist.set_text(format_telemetry_panel_text(state))

        # 5. Update progress bar text
        self.ax_time_text.set_text(f"t = {state['t']:.2f} s / {self.t[-1]:.2f} s")

        return self.trail_line, self.uav_scatter, self.hud_text_artist


def main():
    csv_file = os.path.join(PKG_ROOT, 'uav_trajectory_log.csv')
    yaml_file = os.path.join(PKG_ROOT, 'config', 'uav_params.yaml')

    if not os.path.exists(csv_file):
        print("Generating fresh simulation log...")
        from scripts.run_standalone_verification import run_3d_waypoint_simulation
        run_3d_waypoint_simulation(yaml_file, csv_file, os.path.join(PKG_ROOT, 'plots'))

    print("\nStarting Interactive 3D UAV Flight & Telemetry Viewer...")
    print("Controls: Click & Drag with Mouse to rotate in 3D, Right-Click to Zoom.")
    viewer = InteractiveUAVViewer(csv_file, yaml_file)
    plt.show()


if __name__ == '__main__':
    main()
