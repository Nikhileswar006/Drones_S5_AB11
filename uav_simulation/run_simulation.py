#!/usr/bin/env python3
"""
Interactive UAV Waypoint Configurator & Simulation Launcher
===========================================================
Allows you to change the start coordinates and target waypoints dynamically
every time you run the simulation—either through an interactive Graphical UI (GUI),
command-line arguments, or terminal prompts!

Usage:
  1. Graphical Waypoint Editor (Recommended - opens GUI to add/remove/edit coordinates):
       python run_simulation.py

  2. Command-Line Arguments (Directly specify any coordinates):
       python run_simulation.py --start 0 0 2 --waypoints "3,2,3" "8,-1,4" "12,3,5"

  3. Terminal Interactive Prompt:
       python run_simulation.py --prompt
"""

import os
import sys
import yaml
import argparse
import webbrowser
import numpy as np

PKG_ROOT = os.path.abspath(os.path.dirname(__file__))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from uav_simulation.waypoint_manager import load_waypoints_from_yaml
from scripts.generate_gazebo_world import build_world_sdf
from scripts.run_standalone_verification import run_3d_waypoint_simulation
from scripts.export_html5_viewer import export_html


def parse_start_coords(raw_input, default_start=None):
    """
    Parses start position from CLI input.
    Accepts:
      ['0', '0', '2'] -> [0.0, 0.0, 2.0]
      ['0,0,2']       -> [0.0, 0.0, 2.0]
      ['0, 0, 2']     -> [0.0, 0.0, 2.0]
      '(0, 0, 2)'     -> [0.0, 0.0, 2.0]
    """
    if not raw_input:
        return default_start or [0.0, 0.0, 2.0]
    if isinstance(raw_input, (list, tuple)):
        flat = []
        for item in raw_input:
            parts = str(item).replace(',', ' ').replace('(', '').replace(')', '').split()
            flat.extend([float(p) for p in parts if p.strip()])
        if len(flat) == 3:
            return [float(x) for x in flat]
    elif isinstance(raw_input, str):
        parts = raw_input.replace(',', ' ').replace('(', '').replace(')', '').split()
        flat = [float(p) for p in parts if p.strip()]
        if len(flat) == 3:
            return [float(x) for x in flat]
    raise ValueError(f"Could not parse starting position [X Y Z] from: {raw_input}")


def parse_waypoints_coords(raw_input):
    """
    Parses waypoints from CLI input.
    Accepts:
      ['4,2,3', '8,-3,5', '12,1,4', '16,5,6'] -> 4 waypoints
      ['4 2 3', '8 -3 5'] -> 2 waypoints
      ['4', '2', '3', '8', '-3', '5'] -> 2 waypoints
      ['(4,2,3)', '(8,-3,5)'] -> 2 waypoints
    """
    if not raw_input:
        return []

    temp_wps = []
    flat_nums = []
    all_grouped = True

    for item in raw_input:
        cleaned = str(item).replace('(', '').replace(')', '').strip()
        parts = cleaned.replace(',', ' ').split()
        if len(parts) == 3:
            temp_wps.append([float(parts[0]), float(parts[1]), float(parts[2])])
        else:
            all_grouped = False
        for p in parts:
            if p.strip():
                flat_nums.append(float(p))

    if all_grouped and len(temp_wps) > 0:
        return temp_wps

    if len(flat_nums) % 3 != 0 or len(flat_nums) == 0:
        raise ValueError(
            f"Waypoints coordinates must have multiples of 3 values (X Y Z for each waypoint). "
            f"Got {len(flat_nums)} numbers: {flat_nums}"
        )

    return [flat_nums[i:i + 3] for i in range(0, len(flat_nums), 3)]


def save_waypoints_to_yaml(
    yaml_path: str,
    start_pos: list,
    waypoints: list,
    tolerance: float = 0.20,
    max_velocity: float = None,
    max_acceleration: float = None,
    enable_disturbances: bool = None,
    wind_steady: list = None,
    force_noise_std: float = None,
    torque_noise_std: float = None,
    noise_factor: float = None
):
    """Updates config/uav_params.yaml with new user coordinates and noise parameters."""
    with open(yaml_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    params = data.get('uav_simulation', {}).get('ros__parameters', data)
    params['initial_position'] = [float(x) for x in start_pos]
    params['waypoints'] = [[float(c) for c in wp] for wp in waypoints]
    flat = []
    for wp in waypoints:
        flat.extend([float(c) for c in wp])
    params['waypoints_flat'] = flat
    params['waypoint_tolerance'] = float(tolerance)
    if max_velocity is not None:
        params['max_velocity'] = float(max_velocity)
    if max_acceleration is not None:
        params['max_acceleration'] = float(max_acceleration)

    # Noise & disturbance customization
    if noise_factor is not None:
        factor = max(0.0, float(noise_factor))
        if factor == 0.0:
            params['enable_disturbances'] = False
        else:
            params['enable_disturbances'] = True
            params['wind_steady'] = [round(0.75 * factor, 3), round(-0.55 * factor, 3), round(0.15 * factor, 3)]
            params['wind_gust_amplitude'] = [round(1.10 * factor, 3), round(0.95 * factor, 3), round(0.40 * factor, 3)]
            params['force_noise_std'] = round(0.28 * factor, 4)
            params['torque_noise_std'] = round(0.025 * factor, 4)

    if enable_disturbances is not None:
        params['enable_disturbances'] = bool(enable_disturbances)
    if wind_steady is not None:
        params['wind_steady'] = [float(w) for w in wind_steady]
    if force_noise_std is not None:
        params['force_noise_std'] = float(force_noise_std)
    if torque_noise_std is not None:
        params['torque_noise_std'] = float(torque_noise_std)

    with open(yaml_path, 'w', encoding='utf-8') as f:
        yaml.dump(data, f, default_flow_style=None, sort_keys=False)


def execute_pipeline(
    start_pos: list,
    waypoints: list,
    tolerance: float = 0.20,
    max_velocity: float = None,
    max_acceleration: float = None,
    enable_disturbances: bool = None,
    wind_steady: list = None,
    force_noise_std: float = None,
    torque_noise_std: float = None,
    noise_factor: float = None,
    open_browser: bool = True,
    open_desktop: bool = False
):
    yaml_path = os.path.join(PKG_ROOT, 'config', 'uav_params.yaml')
    sdf_path = os.path.join(PKG_ROOT, 'worlds', 'uav_waypoint_world.sdf')
    csv_path = os.path.join(PKG_ROOT, 'uav_trajectory_log.csv')
    plots_dir = os.path.join(PKG_ROOT, 'plots')
    html_path = os.path.join(plots_dir, 'interactive_3d_simulation.html')

    print("\n" + "=" * 65)
    print("   UPDATING SIMULATION WITH CUSTOM COORDINATES & NOISE")
    print("=" * 65)
    print(f"Start Position: ({start_pos[0]}, {start_pos[1]}, {start_pos[2]}) m")
    print(f"Waypoints ({len(waypoints)} total):")
    for i, wp in enumerate(waypoints):
        print(f"  WP{i+1}: ({wp[0]}, {wp[1]}, {wp[2]}) m")
    print(f"Waypoint Tolerance: {tolerance} m")
    if max_velocity is not None:
        print(f"Max Velocity: {max_velocity} m/s")
    if max_acceleration is not None:
        print(f"Max Acceleration: {max_acceleration} m/s^2")
    if noise_factor is not None:
        print(f"Noise Factor: {noise_factor}x ({'Disabled' if noise_factor == 0 else 'Active'})")
    if force_noise_std is not None:
        print(f"Force Noise Std: {force_noise_std} N")
    if torque_noise_std is not None:
        print(f"Torque Noise Std: {torque_noise_std} N*m")
    if wind_steady is not None:
        print(f"Steady Wind Vector: ({wind_steady[0]}, {wind_steady[1]}, {wind_steady[2]}) m/s")
    print("-" * 65)

    # 1. Update YAML
    save_waypoints_to_yaml(
        yaml_path, start_pos, waypoints, tolerance,
        max_velocity, max_acceleration,
        enable_disturbances, wind_steady, force_noise_std, torque_noise_std, noise_factor
    )
    print("[1/4] Saved custom parameters to config/uav_params.yaml")

    # 2. Re-generate Gazebo World SDF
    build_world_sdf(yaml_path, sdf_path)
    print("[2/4] Regenerated Gazebo 3D World (worlds/uav_waypoint_world.sdf)")

    # 3. Simulate 6-DOF Quadrotor with Paper Quaternion Controller
    print("[3/4] Running 6-DOF Quaternion Backstepping Simulation...")
    run_3d_waypoint_simulation(yaml_path, csv_path, plots_dir)

    # 4. Export Updated Interactive 3D WebGL Viewer
    export_html(csv_path, yaml_path, html_path)
    print(f"[4/4] Generated 3D WebGL Simulation: {html_path}")
    print("=" * 65)
    print("SUCCESS! Simulation updated with your custom coordinates.")

    # Synchronize to Antigravity brain artifact folder if available
    artifact_dir = r"C:\Users\hp\.gemini\antigravity\brain\3fb867a2-f851-489d-bd4a-d5f3959bbf8c"
    if os.path.exists(artifact_dir):
        import shutil
        try:
            shutil.copy2(html_path, os.path.join(artifact_dir, 'interactive_3d_simulation.html'))
            for f in os.listdir(plots_dir):
                if f.endswith('.png'):
                    shutil.copy2(os.path.join(plots_dir, f), os.path.join(artifact_dir, f))
        except Exception:
            pass

    if open_browser:
        print("Launching 3D WebGL Viewer in browser...")
        try:
            webbrowser.open('file://' + os.path.abspath(html_path))
        except Exception:
            pass

    if open_desktop:
        print("Launching Desktop Interactive 3D Window...")
        try:
            from scripts.interactive_3d_viewer import InteractiveUAVViewer
            import matplotlib.pyplot as plt
            viewer = InteractiveUAVViewer(csv_path, yaml_path)
            plt.show()
        except Exception as e:
            print(f"Desktop viewer could not launch: {e}")


# ==============================================================================
# Graphical User Interface (GUI) for Modifying Coordinates
# ==============================================================================

def launch_coordinate_gui():
    import tkinter as tk
    from tkinter import ttk, messagebox

    yaml_path = os.path.join(PKG_ROOT, 'config', 'uav_params.yaml')
    current_start, current_wps, current_tol = load_waypoints_from_yaml(yaml_path)

    root = tk.Tk()
    root.title("UAV 3D Waypoint & Coordinate Configurator")
    root.geometry("640x720")
    root.minsize(580, 600)
    root.configure(bg="#0f172a")

    # Style
    style = ttk.Style()
    style.theme_use('clam')
    style.configure('TLabel', background='#0f172a', foreground='#e2e8f0', font=('Segoe UI', 10))
    style.configure('Header.TLabel', background='#0f172a', foreground='#00f5d4', font=('Segoe UI', 12, 'bold'))

    # Title Banner
    header_frame = tk.Frame(root, bg="#1e293b", padx=16, pady=12)
    header_frame.pack(fill=tk.X)
    tk.Label(
        header_frame,
        text="UAV SIMULATION COORDINATE CONFIGURATOR",
        font=('Segoe UI', 13, 'bold'),
        fg="#00f5d4",
        bg="#1e293b"
    ).pack(anchor='w')
    tk.Label(
        header_frame,
        text="Enter any starting coordinates and target waypoints. Click 'Run & View in 3D' to simulate.",
        font=('Segoe UI', 9),
        fg="#94a3b8",
        bg="#1e293b"
    ).pack(anchor='w')

    # Main Scrollable / Form Frame
    main_frame = tk.Frame(root, bg="#0f172a", padx=18, pady=12)
    main_frame.pack(fill=tk.BOTH, expand=True)

    # 1. Starting Position Inputs
    start_box = tk.LabelFrame(main_frame, text=" 1. Starting Position (m) ", font=('Segoe UI', 10, 'bold'), fg='#38bdf8', bg='#111827', padx=12, pady=10)
    start_box.pack(fill=tk.X, pady=8)

    start_entries = []
    coords_labels = ['X:', 'Y:', 'Z:']
    for i, lbl in enumerate(coords_labels):
        tk.Label(start_box, text=lbl, bg='#111827', fg='#94a3b8', font=('Segoe UI', 10, 'bold')).grid(row=0, column=i * 2, padx=(10, 4), pady=4)
        entry = tk.Entry(start_box, width=8, font=('Consolas', 11), bg='#1e293b', fg='#ffffff', insertbackground='white')
        entry.insert(0, str(current_start[i]))
        entry.grid(row=0, column=i * 2 + 1, padx=(0, 15), pady=4)
        start_entries.append(entry)

    # 2. Tolerance Input
    tk.Label(start_box, text="Tolerance (m):", bg='#111827', fg='#94a3b8', font=('Segoe UI', 9)).grid(row=0, column=6, padx=(10, 4))
    tol_entry = tk.Entry(start_box, width=6, font=('Consolas', 11), bg='#1e293b', fg='#ffffff', insertbackground='white')
    tol_entry.insert(0, str(current_tol))
    tol_entry.grid(row=0, column=7, padx=(0, 4))

    # 3. Dynamic Waypoints List
    wp_box = tk.LabelFrame(main_frame, text=" 2. Target Waypoints (X, Y, Z in meters) ", font=('Segoe UI', 10, 'bold'), fg='#facc15', bg='#111827', padx=12, pady=10)
    wp_box.pack(fill=tk.BOTH, expand=True, pady=8)

    # Canvas + Scrollbar for arbitrary number of waypoints
    canvas = tk.Canvas(wp_box, bg='#111827', highlightthickness=0)
    scrollbar = tk.Scrollbar(wp_box, orient=tk.VERTICAL, command=canvas.yview)
    wp_list_frame = tk.Frame(canvas, bg='#111827')
    wp_list_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
    canvas.create_window((0, 0), window=wp_list_frame, anchor="nw")
    canvas.configure(yscrollcommand=scrollbar.set)
    canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    wp_rows = []

    def refresh_wp_rows():
        for widget in wp_list_frame.winfo_children():
            widget.destroy()

        for idx, wp_coords in enumerate(wp_rows):
            row_frame = tk.Frame(wp_list_frame, bg='#182234', padx=8, pady=4, relief=tk.RIDGE, bd=1)
            row_frame.pack(fill=tk.X, pady=3, padx=2)

            tk.Label(
                row_frame,
                text=f"Waypoint {idx + 1}:",
                font=('Segoe UI', 10, 'bold'),
                fg='#facc15',
                bg='#182234',
                width=12,
                anchor='w'
            ).pack(side=tk.LEFT)

            for c_idx, c_name in enumerate(['X', 'Y', 'Z']):
                tk.Label(row_frame, text=f"{c_name}:", bg='#182234', fg='#94a3b8').pack(side=tk.LEFT, padx=(4, 2))
                e = tk.Entry(row_frame, width=7, font=('Consolas', 11), bg='#0f172a', fg='#ffffff', insertbackground='white')
                e.insert(0, str(wp_coords[c_idx]))
                e.pack(side=tk.LEFT, padx=(0, 8))
                wp_coords[f'entry_{c_name}'] = e

            # Delete single button
            btn_del = tk.Button(
                row_frame,
                text="✕",
                font=('Segoe UI', 8, 'bold'),
                bg='#ef4444',
                fg='white',
                relief=tk.FLAT,
                command=lambda i=idx: remove_single_wp(i)
            )
            btn_del.pack(side=tk.RIGHT, padx=4)

    def add_wp(coords=None):
        if coords is None:
            if wp_rows:
                last_c = [float(wp_rows[-1]['entry_X'].get()), float(wp_rows[-1]['entry_Y'].get()), float(wp_rows[-1]['entry_Z'].get())]
                coords = [last_c[0] + 5.0, last_c[1] + 2.0, last_c[2] + 1.0]
            else:
                coords = [5.0, 3.0, 4.0]
        wp_rows.append([coords[0], coords[1], coords[2]])
        refresh_wp_rows()

    def remove_single_wp(idx):
        if len(wp_rows) <= 1:
            messagebox.showwarning("Warning", "You need at least 1 target waypoint!")
            return
        # Sync current text values before deleting
        sync_values()
        wp_rows.pop(idx)
        refresh_wp_rows()

    def sync_values():
        for row in wp_rows:
            if 'entry_X' in row:
                try:
                    row[0] = float(row['entry_X'].get())
                    row[1] = float(row['entry_Y'].get())
                    row[2] = float(row['entry_Z'].get())
                except ValueError:
                    pass

    # Load initial waypoints
    for wp in current_wps:
        wp_rows.append([wp[0], wp[1], wp[2]])
    refresh_wp_rows()

    # Buttons Frame for Adding/Removing Waypoints & Presets
    btn_sub_frame = tk.Frame(main_frame, bg="#0f172a")
    btn_sub_frame.pack(fill=tk.X, pady=4)

    tk.Button(
        btn_sub_frame,
        text="+ Add Waypoint",
        font=('Segoe UI', 9, 'bold'),
        bg='#0284c7',
        fg='white',
        relief=tk.FLAT,
        padx=10,
        pady=4,
        command=lambda: add_wp()
    ).pack(side=tk.LEFT, padx=3)

    # Presets dropdown
    def apply_preset(name):
        sync_values()
        wp_rows.clear()
        if name == "Default (Paper/Task)":
            start_entries[0].delete(0, tk.END); start_entries[0].insert(0, "0.0")
            start_entries[1].delete(0, tk.END); start_entries[1].insert(0, "0.0")
            start_entries[2].delete(0, tk.END); start_entries[2].insert(0, "2.0")
            for p in [[5.0, 3.0, 4.0], [10.0, -2.0, 5.0], [15.0, 4.0, 6.0]]:
                wp_rows.append(p)
        elif name == "Square Circuit":
            for p in [[0.0, 8.0, 3.0], [8.0, 8.0, 4.0], [8.0, 0.0, 3.0], [0.0, 0.0, 2.0]]:
                wp_rows.append(p)
        elif name == "Ascending Spiral":
            for angle in np.linspace(0, 4 * np.pi, 6):
                r = 6.0
                wp_rows.append([round(r * np.cos(angle), 1), round(r * np.sin(angle), 1), round(2.0 + 1.2 * angle, 1)])
        elif name == "Zig-Zag High Altitude":
            for p in [[4.0, 4.0, 5.0], [8.0, -4.0, 6.0], [12.0, 4.0, 7.0], [16.0, -4.0, 8.0]]:
                wp_rows.append(p)
        refresh_wp_rows()

    preset_var = tk.StringVar(value="Presets...")
    preset_menu = ttk.Combobox(
        btn_sub_frame,
        textvariable=preset_var,
        values=["Default (Paper/Task)", "Square Circuit", "Ascending Spiral", "Zig-Zag High Altitude"],
        state="readonly",
        width=20
    )
    preset_menu.pack(side=tk.RIGHT, padx=4)
    preset_menu.bind("<<ComboboxSelected>>", lambda e: apply_preset(preset_var.get()))

    # Action Launch Buttons
    action_frame = tk.Frame(root, bg="#1e293b", padx=16, pady=14)
    action_frame.pack(fill=tk.X, side=tk.BOTTOM)

    def on_launch(browser=True, desktop=False, gazebo=False):
        try:
            start_p = [float(e.get().strip()) for e in start_entries]
            tol_val = float(tol_entry.get().strip())
            wps_p = []
            for row in wp_rows:
                x = float(row['entry_X'].get().strip())
                y = float(row['entry_Y'].get().strip())
                z = float(row['entry_Z'].get().strip())
                wps_p.append([x, y, z])
        except ValueError as err:
            messagebox.showerror("Invalid Input", f"Please enter valid numeric coordinates:\n{err}")
            return

        if not wps_p:
            messagebox.showwarning("Warning", "At least one target waypoint is required.")
            return

        execute_pipeline(start_p, wps_p, tol_val, open_browser=browser, open_desktop=desktop)

        if gazebo:
            import subprocess
            bat_path = os.path.join(PKG_ROOT, 'LAUNCH_IN_GAZEBO.bat')
            sh_path = os.path.join(PKG_ROOT, 'setup_and_run_gazebo_wsl.sh')
            if os.name == 'nt':
                subprocess.Popen(['cmd.exe', '/c', 'start', 'cmd.exe', '/k', bat_path])
            else:
                subprocess.Popen(['bash', sh_path])

    btn_gz = tk.Button(
        action_frame,
        text="🌐 Launch in Gazebo",
        font=('Segoe UI', 10, 'bold'),
        bg='#f59e0b',
        fg='#0f172a',
        relief=tk.FLAT,
        padx=12,
        pady=8,
        cursor='hand2',
        command=lambda: on_launch(browser=False, desktop=False, gazebo=True)
    )
    btn_gz.pack(side=tk.LEFT, padx=5)

    btn_web = tk.Button(
        action_frame,
        text="🚀 View in 3D Browser",
        font=('Segoe UI', 10, 'bold'),
        bg='#00f5d4',
        fg='#0f172a',
        relief=tk.FLAT,
        padx=12,
        pady=8,
        cursor='hand2',
        command=lambda: on_launch(browser=True, desktop=False, gazebo=False)
    )
    btn_web.pack(side=tk.LEFT, padx=5)

    btn_desk = tk.Button(
        action_frame,
        text="🎮 Desktop 3D Window",
        font=('Segoe UI', 10, 'bold'),
        bg='#334155',
        fg='#e2e8f0',
        relief=tk.FLAT,
        padx=10,
        pady=8,
        cursor='hand2',
        command=lambda: on_launch(browser=False, desktop=True, gazebo=False)
    )
    btn_desk.pack(side=tk.LEFT, padx=5)

    root.mainloop()


# ==============================================================================
# CLI / Terminal Interactive Mode
# ==============================================================================

def run_cli_prompt():
    yaml_path = os.path.join(PKG_ROOT, 'config', 'uav_params.yaml')
    current_start, current_wps, current_tol = load_waypoints_from_yaml(yaml_path)

    print("\n" + "=" * 65)
    print("   UAV WAYPOINT CONFIGURATOR (TERMINAL PROMPT)")
    print("=" * 65)

    # Start position
    start_str = input(f"Enter Starting Position (X Y Z) [default: {current_start[0]} {current_start[1]} {current_start[2]}]: ").strip()
    if start_str:
        start_pos = [float(v) for v in start_str.replace(',', ' ').split()]
    else:
        start_pos = current_start

    # Waypoint tolerance
    tol_str = input(f"Enter Waypoint Tolerance in meters [default: {current_tol}]: ").strip()
    tol = float(tol_str) if tol_str else current_tol

    # Waypoints
    print("\nHow would you like to enter waypoints?")
    print("  1. Keep current default waypoints: " + " -> ".join([str(tuple(w)) for w in current_wps]))
    print("  2. Enter new sequential waypoints")
    choice = input("Choice (1 or 2) [default: 2]: ").strip() or "2"

    waypoints = []
    if choice == "1":
        waypoints = current_wps
    else:
        num_str = input("How many target waypoints? [e.g. 2, 3, 4, ...]: ").strip()
        num_wps = int(num_str) if num_str else 3
        for i in range(num_wps):
            wp_input = input(f"Enter Waypoint {i+1} coordinates (X Y Z): ").strip()
            wp = [float(v) for v in wp_input.replace(',', ' ').split()]
            waypoints.append(wp)

    execute_pipeline(start_pos, waypoints, tol, open_browser=True, open_desktop=False)


def main():
    parser = argparse.ArgumentParser(
        description="Configure UAV Coordinates & Run Real-World 3D Simulation.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run with custom start and 4 waypoints:
  python run_simulation.py --start 0 0 2 --waypoints "4,2,3" "8,-3,5" "12,1,4" "16,5,6"

  # Run with custom waypoints only (uses current start position):
  python run_simulation.py --waypoints "5,3,4" "10,-2,5" "15,4,6"

  # Space-separated flat coordinates:
  python run_simulation.py --start 0 0 2 --waypoints 4 2 3 8 -3 5 12 1 4 16 5 6

  # Headless mode without auto-opening browser:
  python run_simulation.py --start 0 0 2 --waypoints "4,2,3" "8,-3,5" --no-browser

  # Interactive modes:
  python run_simulation.py --gui       # Opens Graphical Coordinate Window
  python run_simulation.py --prompt    # Interactive terminal prompts
        """
    )
    parser.add_argument('--gui', action='store_true', help="Open the graphical waypoint editor window.")
    parser.add_argument('--prompt', action='store_true', help="Enter coordinates via terminal prompts.")
    parser.add_argument('--start', nargs='+', metavar='COORD', help="Set starting position, e.g. --start 0 0 2 or --start '0,0,2'.")
    parser.add_argument('--waypoints', '-w', nargs='+', metavar='WP', help="List of waypoints, e.g. --waypoints '4,2,3' '8,-3,5' '12,1,4' '16,5,6'.")
    parser.add_argument('--tolerance', type=float, default=0.20, help="Waypoint acceptance tolerance (m) [default: 0.20].")
    parser.add_argument('--max-velocity', '--max-vel', type=float, default=None, help="Maximum velocity limit (m/s) [default from yaml: 1.0].")
    parser.add_argument('--max-acceleration', '--max-acc', type=float, default=None, help="Maximum acceleration limit (m/s^2) [default from yaml: 1.2].")
    parser.add_argument('--noise-factor', type=float, default=None, help="Scale all air noise & wind together (e.g. 0.0=none, 0.5=mild, 1.0=normal, 2.0=strong).")
    parser.add_argument('--no-noise', action='store_true', help="Disable all wind, turbulence, and noise disturbances completely.")
    parser.add_argument('--force-noise', type=float, default=None, help="Standard deviation of aerodynamic force noise in Newtons [default: 0.28].")
    parser.add_argument('--torque-noise', type=float, default=None, help="Standard deviation of aerodynamic torque noise in N*m [default: 0.025].")
    parser.add_argument('--wind', nargs=3, type=float, metavar=('Wx', 'Wy', 'Wz'), help="Steady 3D wind velocity vector in m/s [default: 0.75 -0.55 0.15].")
    parser.add_argument('--no-browser', action='store_true', help="Do not automatically open the browser.")
    parser.add_argument('--desktop', action='store_true', help="Open desktop matplotlib window.")
    parser.add_argument('--gazebo', action='store_true', help="Launch live inside Gazebo 3D Simulator (via WSL2 Ubuntu).")

    args = parser.parse_args()

    yaml_path = os.path.join(PKG_ROOT, 'config', 'uav_params.yaml')
    current_start, current_wps, current_tol = load_waypoints_from_yaml(yaml_path)

    # Determine noise settings
    noise_factor = 0.0 if args.no_noise else args.noise_factor
    enable_dist = False if args.no_noise else None

    # 1. If custom coordinates or noise passed via CLI
    if args.waypoints or args.start or (args.noise_factor is not None) or args.no_noise or (args.force_noise is not None) or (args.torque_noise is not None) or (args.wind is not None):
        try:
            start_pos = parse_start_coords(args.start, current_start) if args.start else current_start
            waypoints = parse_waypoints_coords(args.waypoints) if args.waypoints else current_wps
        except Exception as e:
            print(f"Error parsing coordinates: {e}")
            sys.exit(1)

        execute_pipeline(
            start_pos=start_pos,
            waypoints=waypoints,
            tolerance=args.tolerance,
            max_velocity=args.max_velocity,
            max_acceleration=args.max_acceleration,
            enable_disturbances=enable_dist,
            wind_steady=args.wind,
            force_noise_std=args.force_noise,
            torque_noise_std=args.torque_noise,
            noise_factor=noise_factor,
            open_browser=(not args.no_browser and not args.gazebo),
            open_desktop=args.desktop
        )
        if args.gazebo:
            import subprocess
            bat_path = os.path.join(PKG_ROOT, 'LAUNCH_IN_GAZEBO.bat')
            sh_path = os.path.join(PKG_ROOT, 'setup_and_run_gazebo_wsl.sh')
            if os.name == 'nt':
                subprocess.Popen(['cmd.exe', '/c', 'start', 'cmd.exe', '/k', bat_path])
            else:
                subprocess.Popen(['bash', sh_path])
        return

    # 2. Gazebo standalone flag
    if args.gazebo:
        import subprocess
        bat_path = os.path.join(PKG_ROOT, 'LAUNCH_IN_GAZEBO.bat')
        sh_path = os.path.join(PKG_ROOT, 'setup_and_run_gazebo_wsl.sh')
        if os.name == 'nt':
            subprocess.Popen(['cmd.exe', '/c', 'start', 'cmd.exe', '/k', bat_path])
        else:
            subprocess.Popen(['bash', sh_path])
        return

    # 3. Terminal prompt flag
    if args.prompt:
        run_cli_prompt()
        return

    # 4. Default: Launch GUI if possible, fallback to terminal prompt
    try:
        launch_coordinate_gui()
    except Exception as e:
        print(f"GUI could not open ({e}). Falling back to terminal prompt...")
        run_cli_prompt()


if __name__ == '__main__':
    main()
