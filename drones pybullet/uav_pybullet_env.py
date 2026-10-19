"""
PyBullet 3D Simulation Environment for Quadrotor UAV
Features:
- Procedural multi-body Quadrotor with realistic visual frame and rotor discs
- Dynamic 3D path visualization (Green = Reference Trajectory, Cyan = Actual Flight)
- Visual Start/Destination markers (Green & Red 3D Spheres)
- Body coordinate frame indicators
- Real-time smooth camera tracking
"""

import os
import time
import numpy as np
import pybullet as p
import pybullet_data
import tkinter as tk

from uav_quaternion_backstepping import QuaternionMath


# ─────────────────────────────────────────────────────────────────────────────
# Interactive 360° Camera Controller (touchpad / mouse)
# ─────────────────────────────────────────────────────────────────────────────

class CameraController:
    """
    Full 360° interactive camera for PyBullet driven by mouse / touchpad events.

    Controls (works with both mouse and laptop touchpad):
        Left-click  + drag   →  Rotate (yaw full 360°, pitch -89° … -1°)
        Right-click + drag   →  Zoom in / out
        Scroll wheel         →  Zoom in / out
        Camera smoothly follows the drone target position.
    """

    _MOUSE_MOVE   = 1   # PyBullet MOUSE_MOVE_EVENT
    _MOUSE_BUTTON = 2   # PyBullet MOUSE_BUTTON_EVENT

    def __init__(self, target_pos=None, distance: float = 6.0,
                 yaw: float = 45.0, pitch: float = -30.0):
        self.target   = np.array(target_pos if target_pos is not None
                                 else [0.0, 0.0, 1.0], dtype=float)
        self.distance = float(distance)
        self.yaw      = float(yaw)
        self.pitch    = float(pitch)

        # Sensitivity knobs
        self.sens_rotate = 0.45   # degrees per pixel for left-drag rotate
        self.sens_zoom   = 0.30   # metres per pixel for right-drag zoom
        self.scroll_step = 0.50   # metres per scroll click

        # Internal state
        self._lmb = False   # left mouse button held?
        self._rmb = False   # right mouse button held?
        self._px  = 0       # previous mouse X
        self._py  = 0       # previous mouse Y

        self._apply()       # set initial camera view

    # ── Internal ──────────────────────────────────────────────────────────────
    def _apply(self):
        """Push current yaw/pitch/distance/target into PyBullet."""
        p.resetDebugVisualizerCamera(
            cameraDistance    = max(0.5, self.distance),
            cameraYaw         = self.yaw % 360.0,
            cameraPitch       = float(self.pitch),
            cameraTargetPosition = self.target.tolist()
        )

    # ── Public API ────────────────────────────────────────────────────────────
    def set_target(self, pos):
        """Instantly move camera target to pos."""
        self.target = np.array(pos, dtype=float)

    def update(self, follow_pos=None, smooth: float = 0.06):
        """
        Call once per frame (or every few physics steps).

        Args:
            follow_pos: If given, camera target smoothly tracks this 3-D point
                        (the drone position).  None = target stays fixed.
            smooth:     Blending factor 0..1 for target tracking (lower = slower).
        """
        dirty = False

        # ── Smooth-follow the drone ──────────────────────────────────────
        if follow_pos is not None:
            new_target = np.array(follow_pos, dtype=float)
            self.target = (1.0 - smooth) * self.target + smooth * new_target
            dirty = True

        # ── Read PyBullet mouse events ───────────────────────────────────
        for ev in p.getMouseEvents():
            etype, mx, my, btn, bstate = ev

            if etype == self._MOUSE_MOVE:
                dx = mx - self._px
                dy = my - self._py

                if self._lmb and (abs(dx) > 0 or abs(dy) > 0):
                    # Left drag → rotate yaw (full 360°) and pitch
                    self.yaw   += dx * self.sens_rotate
                    self.pitch  = float(np.clip(
                        self.pitch - dy * self.sens_rotate,
                        -89.0, -1.0
                    ))
                    dirty = True

                if self._rmb and abs(dy) > 0:
                    # Right drag (up=zoom in, down=zoom out)
                    self.distance = float(np.clip(
                        self.distance + dy * self.sens_zoom,
                        0.5, 40.0
                    ))
                    dirty = True

                self._px, self._py = mx, my

            elif etype == self._MOUSE_BUTTON:
                # Track left / right button state
                if btn == 0:                         # left button
                    pressed = bstate in (1, 2, 3)
                    self._lmb = pressed
                    if pressed:
                        self._px, self._py = mx, my
                elif btn == 2:                       # right button
                    pressed = bstate in (1, 2, 3)
                    self._rmb = pressed
                    if pressed:
                        self._px, self._py = mx, my

                # Scroll: button index 3 = scroll-up, 4 = scroll-down
                # (platform-dependent; both orderings handled)
                elif btn == 3:
                    self.distance = float(np.clip(
                        self.distance - self.scroll_step, 0.5, 40.0))
                    dirty = True
                elif btn == 4:
                    self.distance = float(np.clip(
                        self.distance + self.scroll_step, 0.5, 40.0))
                    dirty = True

        if dirty:
            self._apply()


# ─────────────────────────────────────────────────────────────────────────────
# Live Tracking Panel  (tkinter rectangular dashboard)
# ─────────────────────────────────────────────────────────────────────────────

class LiveTrackingPanel:
    """
    A dark-themed rectangular tkinter window that shows real-time UAV telemetry.
    Opens on the LEFT side of the screen next to the PyBullet 3D view.
    Only closes when the user clicks the EXIT button — never auto-closes.
    """

    _BG       = "#0d0d0d"
    _TITLE_BG = "#1a1a2e"
    _TITLE_FG = "#00ff88"
    _SEP_FG   = "#2a2a2a"
    _LABEL_FG = "#888888"

    def __init__(self, title: str = "UAV LIVE TRACKING"):
        self.exit_requested = False    # Set True when user clicks EXIT
        self.started = False           # Set True when user clicks START

        self.root = tk.Tk()
        self.root.title(title)
        self.root.configure(bg=self._BG)
        self.root.geometry("440x670+0+30")
        self.root.resizable(False, False)
        self.root.attributes("-topmost", True)
        # Intercept the window-close (X) button so it uses our exit flag
        self.root.protocol("WM_DELETE_WINDOW", self._on_exit)

        self._build_ui()
        self.root.update()

    def _on_start(self):
        self.started = True
        self.start_btn.config(state=tk.DISABLED, text="  MISSION RUNNING  ", bg="#005522")

    def _on_exit(self):
        self.exit_requested = True

    def wait_for_start(self, on_idle_callback=None):
        """
        Pauses and waits until the user clicks the green 'START SIMULATION' button.
        Allows the user to adjust their 3D camera / view before takeoff.
        Calls on_idle_callback on each tick (e.g. to keep camera responsive).
        """
        self.set_status("READY: CLICK 'START SIMULATION'", color="#ffff00", bg="#333300")
        while not self.started and not self.exit_requested:
            self.root.update()
            if on_idle_callback is not None:
                on_idle_callback()
            time.sleep(0.015)
        if not self.exit_requested:
            self.set_status("MISSION IN FLIGHT", color="#00ff88", bg="#002200")

    def _build_ui(self):
        # ── Title bar ──────────────────────────────────────────────
        hdr = tk.Frame(self.root, bg=self._TITLE_BG, pady=6)
        hdr.pack(fill=tk.X)
        tk.Label(hdr, text="  UAV LIVE TRACKING RESULTS  ",
                 bg=self._TITLE_BG, fg=self._TITLE_FG,
                 font=("Courier New", 12, "bold")).pack()

        # ── Status bar ─────────────────────────────────────────────
        self._status_var = tk.StringVar(value="[ INITIALIZING... ]")
        status_bar = tk.Label(self.root, textvariable=self._status_var,
                              bg="#002200", fg="#00ff88",
                              font=("Courier New", 10, "bold"), pady=4)
        status_bar.pack(fill=tk.X)
        self._status_bar = status_bar

        # ── Body ───────────────────────────────────────────────────
        body = tk.Frame(self.root, bg=self._BG, padx=16, pady=8)
        body.pack(fill=tk.BOTH, expand=True)

        rows = [
            ("time",     "Time",             "#ffffff"),
            ("waypoint", "Waypoint",         "#ffaa00"),
            None,
            ("pos_err",  "Position Error",   "#ff5555"),
            ("act_pos",  "Actual Position",  "#66ccff"),
            ("tgt_pos",  "Target Position",  "#66ff99"),
            None,
            ("roll",     "Roll",             "#ffee55"),
            ("pitch",    "Pitch",            "#ffee55"),
            ("yaw_ang",  "Yaw",              "#ffee55"),
            None,
            ("q_err",    "Quat Error",       "#cc88ff"),
            None,
            ("thrust",   "Thrust",           "#ff9922"),
            ("torque",   "Torques",          "#ff9922"),
            None,
            ("noise",    "Sensor Noise",     "#aaaaaa"),
        ]

        self._val_labels = {}
        for row in rows:
            if row is None:
                tk.Frame(body, bg=self._SEP_FG, height=1).pack(fill=tk.X, pady=3)
                continue
            key, lbl_text, val_color = row
            rf = tk.Frame(body, bg=self._BG)
            rf.pack(fill=tk.X, pady=2)
            tk.Label(rf, text=f"{lbl_text}:",
                     bg=self._BG, fg=self._LABEL_FG,
                     font=("Courier New", 10), width=16, anchor="w").pack(side=tk.LEFT)
            val = tk.Label(rf, text="---", bg=self._BG, fg=val_color,
                           font=("Courier New", 10, "bold"), anchor="w")
            val.pack(side=tk.LEFT, fill=tk.X, expand=True)
            self._val_labels[key] = val

        # ── START & EXIT Buttons ───────────────────────────────────
        btn_frame = tk.Frame(self.root, bg=self._BG, pady=8)
        btn_frame.pack(fill=tk.X, padx=16)

        self.start_btn = tk.Button(
            btn_frame, text=" ▶  START SIMULATION ",
            bg="#00aa33", fg="#ffffff", activebackground="#00cc44", activeforeground="#ffffff",
            font=("Courier New", 11, "bold"),
            relief=tk.FLAT, cursor="hand2",
            command=self._on_start
        )
        self.start_btn.pack(fill=tk.X, pady=(0, 6), ipady=7)

        self.exit_btn = tk.Button(
            btn_frame, text=" ✕  EXIT SIMULATION ",
            bg="#aa0000", fg="#ffffff", activebackground="#cc0000", activeforeground="#ffffff",
            font=("Courier New", 11, "bold"),
            relief=tk.FLAT, cursor="hand2",
            command=self._on_exit
        )
        self.exit_btn.pack(fill=tk.X, ipady=5)

    def set_status(self, text: str, color: str = "#00ff88", bg: str = "#002200"):
        self._status_var.set(f"[ {text} ]")
        self._status_bar.config(fg=color, bg=bg)

    def update(self, t, current_pos, target_pos, pos_error, q_err, euler_deg,
               thrust, tau, current_wp_idx=1, total_wps=1, noise_std=0.0):
        lv = self._val_labels
        lv["time"].config(    text=f"{t:.2f} s")
        lv["waypoint"].config(text=f"WP {current_wp_idx}  /  {total_wps}")
        lv["pos_err"].config( text=f"{pos_error:.3f} m")
        lv["act_pos"].config( text=f"[{current_pos[0]:+6.2f}, {current_pos[1]:+6.2f}, {current_pos[2]:+6.2f}] m")
        lv["tgt_pos"].config( text=f"[{target_pos[0]:+6.2f}, {target_pos[1]:+6.2f}, {target_pos[2]:+6.2f}] m")
        lv["roll"].config(    text=f"{euler_deg[2]:+6.1f} deg")
        lv["pitch"].config(   text=f"{euler_deg[1]:+6.1f} deg")
        lv["yaw_ang"].config( text=f"{euler_deg[0]:+6.1f} deg")
        lv["q_err"].config(   text=f"[{q_err[0]:.3f}, {q_err[1]:.3f}, {q_err[2]:.3f}, {q_err[3]:.3f}]")
        lv["thrust"].config(  text=f"{thrust:.2f} N")
        lv["torque"].config(  text=f"[{tau[0]:+.2f}, {tau[1]:+.2f}, {tau[2]:+.2f}] N*m")
        lv["noise"].config(   text=f"sigma = {noise_std:.4f} m  (Gaussian)")
        try:
            self.root.update()
        except tk.TclError:
            pass

    def close(self):
        try:
            self.root.destroy()
        except Exception:
            pass


class UAVPyBulletEnv:
    """
    Simulates a Quadrotor in PyBullet using rigid-body dynamics
    with direct collective thrust and body torque actuation.
    """

    def __init__(self,
                 gui: bool = True,
                 time_step: float = 1.0 / 240.0,
                 mass: float = 1.0,
                 inertia: np.ndarray = None,
                 gravity: float = 9.81):
        self.gui = gui
        self.dt = time_step
        self.mass = float(mass)
        self.gravity = float(gravity)

        if inertia is None:
            # Matches paper Section V: diag([0.1, 0.1, 0.12])
            self.J = np.diag([0.1, 0.1, 0.12])
        else:
            self.J = np.array(inertia, dtype=np.float64)

        # Connect to PyBullet
        if self.gui:
            self.client_id = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        else:
            self.client_id = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -self.gravity)
        p.setTimeStep(self.dt)

        # Load plane
        self.plane_id = p.loadURDF("plane.urdf")

        # Create Drone
        self.drone_id = None
        self.rotor_visual_ids = []
        self.trail_line_ids = []
        self.last_trail_pos = None
        self.hud_text_id = -1
        self.ref_trajectory_line_ids = []

        self.arm_length = 0.22  # meters

    def build_drone(self, initial_pos: np.ndarray, initial_orn: np.ndarray = None):
        """
        Creates a custom quadrotor multi-body directly in PyBullet with
        arms, center body chassis, and 4 rotor discs.
        """
        if initial_orn is None:
            initial_orn = np.array([1.0, 0.0, 0.0, 0.0])  # Scalar-first identity

        # Convert scalar-first [w, x, y, z] to PyBullet [x, y, z, w]
        pb_orn = [initial_orn[1], initial_orn[2], initial_orn[3], initial_orn[0]]

        # Center body collision and visual shapes
        body_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.08, 0.08, 0.025])
        body_vis = p.createVisualShape(p.GEOM_BOX,
                                       halfExtents=[0.08, 0.08, 0.025],
                                       rgbaColor=[0.15, 0.15, 0.18, 1.0])

        # Arms and rotors definition
        link_masses = [0.03, 0.03, 0.03, 0.03, 0.01, 0.01, 0.01, 0.01]
        link_col_shapes = [-1] * 8
        link_vis_shapes = []
        link_positions = []
        link_orientations = [[0, 0, 0, 1]] * 8
        link_inertial_positions = [[0, 0, 0]] * 8
        link_inertial_orientations = [[0, 0, 0, 1]] * 8
        link_parent_indices = [0] * 8
        link_joint_types = [p.JOINT_FIXED] * 8
        link_joint_axes = [[0, 0, 1]] * 8

        # Arm angles for 'X' configuration: 45, 135, 225, 315 degrees
        angles = [np.pi / 4.0, 3.0 * np.pi / 4.0, 5.0 * np.pi / 4.0, 7.0 * np.pi / 4.0]
        L = self.arm_length

        # 4 Arms
        for angle in angles:
            x = (L / 2.0) * np.cos(angle)
            y = (L / 2.0) * np.sin(angle)
            z = 0.0
            arm_vis = p.createVisualShape(p.GEOM_CYLINDER,
                                          radius=0.01,
                                          length=L,
                                          rgbaColor=[0.25, 0.25, 0.28, 1.0])
            link_vis_shapes.append(arm_vis)
            link_positions.append([x, y, z])

        # 4 Rotor Discs (Front rotors red, rear rotors blue)
        for i, angle in enumerate(angles):
            x = L * np.cos(angle)
            y = L * np.sin(angle)
            z = 0.02
            color = [0.9, 0.15, 0.15, 0.75] if i in [0, 1] else [0.15, 0.45, 0.95, 0.75]
            rotor_vis = p.createVisualShape(p.GEOM_CYLINDER,
                                            radius=0.08,
                                            length=0.005,
                                            rgbaColor=color)
            link_vis_shapes.append(rotor_vis)
            link_positions.append([x, y, z])

        self.drone_id = p.createMultiBody(
            baseMass=self.mass,
            baseCollisionShapeIndex=body_col,
            baseVisualShapeIndex=body_vis,
            basePosition=initial_pos.tolist(),
            baseOrientation=pb_orn,
            linkMasses=link_masses,
            linkCollisionShapeIndices=link_col_shapes,
            linkVisualShapeIndices=link_vis_shapes,
            linkPositions=link_positions,
            linkOrientations=link_orientations,
            linkInertialFramePositions=link_inertial_positions,
            linkInertialFrameOrientations=link_inertial_orientations,
            linkParentIndices=link_parent_indices,
            linkJointTypes=link_joint_types,
            linkJointAxis=link_joint_axes
        )

        # Set specific base inertia matrix matching paper [0.1, 0.1, 0.12]
        p.changeDynamics(self.drone_id, -1,
                         linearDamping=0.05,
                         angularDamping=0.02,
                         localInertiaDiagonal=[self.J[0, 0], self.J[1, 1], self.J[2, 2]])

        self.last_trail_pos = np.array(initial_pos, dtype=np.float64)

    def draw_markers(self, start_pos: np.ndarray, goal_pos: np.ndarray):
        """Draws 3D start (Green) and goal (Red) spheres with text labels."""
        self.draw_all_waypoints([start_pos, goal_pos])

    def draw_all_waypoints(self, waypoints: list):
        """Draws Start (Green), intermediate waypoints (Amber/Yellow), and Destination (Red)."""
        if not self.gui:
            return

        num_pts = len(waypoints)
        for i, wp in enumerate(waypoints):
            pos = np.array(wp, dtype=np.float64)
            if i == 0:
                color = [0.1, 0.9, 0.2, 0.85]  # Green for Start
                label = "START (WP 0)"
                size = 1.3
            elif i == num_pts - 1:
                color = [0.95, 0.15, 0.15, 0.9]  # Red for Destination
                label = f"DESTINATION (WP {i})"
                size = 1.4
            else:
                color = [1.0, 0.75, 0.05, 0.85]  # Amber/Yellow for intermediate points
                label = f"WAYPOINT {i}"
                size = 1.1

            vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.14, rgbaColor=color)
            p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis, basePosition=pos.tolist())
            p.addUserDebugText(label, pos + np.array([0, 0, 0.22]), textColorRGB=color[:3], textSize=size)

    def update_live_hud(self, t: float, current_pos: np.ndarray, target_pos: np.ndarray,
                        pos_error: float, q_err: np.ndarray, euler_deg: np.ndarray,
                        thrust: float, tau: np.ndarray, current_wp_idx: int = 1, total_wps: int = 1,
                        cam_yaw: float = 45.0):
        """
        Updates on-screen HUD telemetry dashboard always positioned on the LEFT side of the camera view.
        Uses 100% pure ASCII characters to guarantee crisp PyBullet rendering.
        """
        if not self.gui:
            return

        # Calculate camera-relative LEFT vector in world coordinates
        # Camera is oriented at cam_yaw (deg); perpendicular left direction is:
        rad = np.deg2rad(cam_yaw)
        left_dir = np.array([np.sin(rad), -np.cos(rad), 0.0])

        # Position HUD 1.4 meters to the LEFT of the drone and 0.5 meters elevated
        hud_pos = current_pos + 1.4 * left_dir + np.array([0.0, 0.0, 0.55])

        # 100% Pure ASCII string - absolutely NO unicode symbols or box characters that break PyBullet!
        hud_text = (
            "================ LIVE TRACKING RESULTS ================\n"
            f"Time: {t:5.2f} s   |   Target Waypoint: WP {current_wp_idx} of {total_wps}\n"
            f"3D Pos Error: {pos_error:6.3f} m  (ex: {current_pos[0]-target_pos[0]:+5.2f}, ey: {current_pos[1]-target_pos[1]:+5.2f}, ez: {current_pos[2]-target_pos[2]:+5.2f})\n"
            f"Actual Pos:   [{current_pos[0]:5.2f}, {current_pos[1]:5.2f}, {current_pos[2]:5.2f}] m\n"
            f"Target Pos:   [{target_pos[0]:5.2f}, {target_pos[1]:5.2f}, {target_pos[2]:5.2f}] m\n"
            f"Euler Angles: Roll: {euler_deg[2]:+5.1f} deg | Pitch: {euler_deg[1]:+5.1f} deg | Yaw: {euler_deg[0]:+5.1f} deg\n"
            f"Quat Error:   [{q_err[0]:.3f}, {q_err[1]:.3f}, {q_err[2]:.3f}, {q_err[3]:.3f}]\n"
            f"Thrust: {thrust:5.2f} N  |  Torques: [{tau[0]:+5.2f}, {tau[1]:+5.2f}, {tau[2]:+5.2f}] N*m\n"
            "======================================================="
        )

        if self.hud_text_id == -1:
            self.hud_text_id = p.addUserDebugText(
                text=hud_text,
                textPosition=hud_pos.tolist(),
                textColorRGB=[1.0, 1.0, 0.0],  # Bright yellow
                textSize=1.25
            )
        else:
            self.hud_text_id = p.addUserDebugText(
                text=hud_text,
                textPosition=hud_pos.tolist(),
                textColorRGB=[1.0, 1.0, 0.0],
                textSize=1.25,
                replaceItemUniqueId=self.hud_text_id
            )

    def clear_reference_trajectory(self):
        """Clears previously rendered reference trajectory lines in PyBullet."""
        if hasattr(self, 'ref_trajectory_line_ids') and self.ref_trajectory_line_ids:
            for line_id in self.ref_trajectory_line_ids:
                try:
                    p.removeUserDebugItem(line_id)
                except Exception:
                    pass
            self.ref_trajectory_line_ids = []

    def draw_reference_trajectory(self, waypoints: np.ndarray, color=[0.1, 0.9, 0.2], width=2.5):
        """Renders 3D trajectory line segments in PyBullet."""
        if not self.gui or len(waypoints) < 2:
            return

        self.clear_reference_trajectory()
        for i in range(len(waypoints) - 1):
            lid = p.addUserDebugLine(waypoints[i].tolist(),
                                     waypoints[i + 1].tolist(),
                                     lineColorRGB=color,
                                     lineWidth=width)
            self.ref_trajectory_line_ids.append(lid)

    def update_flight_trail(self, current_pos: np.ndarray, min_dist: float = 0.04):
        """Draws live flight path behind drone."""
        if not self.gui:
            return

        dist = np.linalg.norm(current_pos - self.last_trail_pos)
        if dist >= min_dist:
            line_id = p.addUserDebugLine(
                self.last_trail_pos.tolist(),
                current_pos.tolist(),
                lineColorRGB=[0.1, 0.7, 1.0],  # Cyan/Blue flight path
                lineWidth=3.0,
                lifeTime=0  # Persistent line
            )
            self.trail_line_ids.append(line_id)
            self.last_trail_pos = current_pos.copy()

    def update_camera(self, current_pos: np.ndarray, distance: float = 3.5, pitch: float = -28.0, yaw: float = 45.0):
        """Smoothly updates camera viewpoint following the drone."""
        if not self.gui:
            return
        p.resetDebugVisualizerCamera(cameraDistance=distance,
                                     cameraYaw=yaw,
                                     cameraPitch=pitch,
                                     cameraTargetPosition=current_pos.tolist())

    def get_state(self) -> dict:
        """
        Reads position, velocity, orientation quaternion (scalar-first), and
        body angular velocity from PyBullet.
        """
        pos, pb_orn = p.getBasePositionAndOrientation(self.drone_id)
        lin_vel, ang_vel_world = p.getBaseVelocity(self.drone_id)

        # PyBullet quaternion: [x, y, z, w] -> convert to paper's scalar-first [w, x, y, z]
        q_scalar_first = np.array([pb_orn[3], pb_orn[0], pb_orn[1], pb_orn[2]], dtype=np.float64)
        q_scalar_first = QuaternionMath.normalize(q_scalar_first)

        # Transform world angular velocity to body frame:
        # omega_body = R^T * omega_world
        R_world_to_body = QuaternionMath.to_rotation_matrix(q_scalar_first).T
        omega_body = R_world_to_body @ np.array(ang_vel_world, dtype=np.float64)

        return {
            'pos': np.array(pos, dtype=np.float64),
            'vel': np.array(lin_vel, dtype=np.float64),
            'q': q_scalar_first,
            'omega': omega_body,
            'euler_deg': QuaternionMath.to_euler_deg(q_scalar_first)
        }

    def apply_control(self, thrust: float, torque: np.ndarray):
        """
        Applies total thrust along body Z-axis and torques [tau_x, tau_y, tau_z]
        in the body frame.
        """
        # Body frame collective thrust along +Z axis
        p.applyExternalForce(self.drone_id, -1,
                             forceObj=[0.0, 0.0, float(thrust)],
                             posObj=[0.0, 0.0, 0.0],
                             flags=p.LINK_FRAME)

        # Body frame control torques
        p.applyExternalTorque(self.drone_id, -1,
                              torqueObj=torque.tolist(),
                              flags=p.LINK_FRAME)

    def add_obstacle(self, position, half_extents=(0.15, 0.15, 0.5),
                     color=(0.85, 0.25, 0.1, 1.0)):
        """
        Adds a static solid box obstacle at `position` with given half_extents.
        The obstacle has collision so the drone physically interacts with it.
        """
        pos = list(position)
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=list(half_extents))
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=list(half_extents),
                                  rgbaColor=list(color))
        p.createMultiBody(baseMass=0,
                          baseCollisionShapeIndex=col,
                          baseVisualShapeIndex=vis,
                          basePosition=pos)

    def add_default_obstacles(self, waypoints):
        """
        Places solid obstacles directly in the flight path between waypoints
        so the drone must actively avoid them.
        Returns the list of obstacle specifications for trajectory avoidance planning.
        """
        if len(waypoints) < 2:
            return []

        obstacle_specs = []
        for i in range(len(waypoints) - 1):
            a = np.array(waypoints[i], dtype=np.float64)
            b = np.array(waypoints[i + 1], dtype=np.float64)
            seg_vec = b - a
            seg_len = np.linalg.norm(seg_vec)
            if seg_len < 1.0:
                continue

            # Position obstacle directly along the flight line (at 50% midpoint)
            mid = a + 0.5 * seg_vec
            tower_h = max(mid[2] + 0.4, 1.2)
            half_h = tower_h * 0.5
            obs_pos = np.array([mid[0], mid[1], half_h], dtype=np.float64)
            half_radius = 0.22

            self.add_obstacle(position=obs_pos,
                              half_extents=(half_radius, half_radius, half_h),
                              color=(0.95, 0.2, 0.1, 0.95))

            # Visual warning beacon ring/label on top of obstacle
            p.addUserDebugText(f"OBSTACLE {i+1}",
                               [obs_pos[0], obs_pos[1], tower_h + 0.25],
                               textColorRGB=[1.0, 0.2, 0.1], textSize=1.2)

            obstacle_specs.append({
                'pos': obs_pos,
                'radius': half_radius + 0.2,
                'height': tower_h
            })

        return obstacle_specs

    def spawn_dynamic_obstacle(self, position, tower_height=2.2, half_radius=0.25, label="! OBSTACLE APPEARED !"):
        """
        Dynamically spawns an obstacle that suddenly appears in front of the moving drone.
        Returns the obstacle specification.
        """
        pos = np.array(position, dtype=np.float64)
        half_h = tower_height * 0.5
        box_center = [pos[0], pos[1], half_h]
        self.add_obstacle(position=box_center,
                          half_extents=(half_radius, half_radius, half_h),
                          color=(1.0, 0.1, 0.1, 0.95))

        # Flashing red/yellow alert label in 3D
        p.addUserDebugText(label,
                           [pos[0], pos[1], tower_height + 0.35],
                           textColorRGB=[1.0, 0.1, 0.0], textSize=1.35)

        return {
            'pos': np.array(box_center, dtype=np.float64),
            'radius': half_radius + 0.25,
            'height': tower_height
        }

    def scan_lidar(self, drone_pos, drone_orn, max_range=2.8, num_rays=15, fov_deg=100.0, visualize=False):
        """
        Simulates an onboard forward-looking LiDAR / depth sensor.
        Casts a fan of rays in the drone's heading direction.
        Returns:
            list of detected obstacle hit points: [{'hit_pos': [x,y,z], 'dist': d, 'dir': [dx,dy,dz]}, ...]
            Empty list if no obstacle is detected within sensor range.
        """
        q_pb = [drone_orn[1], drone_orn[2], drone_orn[3], drone_orn[0]]
        rot_mat = p.getMatrixFromQuaternion(q_pb)
        forward = np.array([rot_mat[0], rot_mat[3], rot_mat[6]], dtype=np.float64)
        right = np.array([rot_mat[1], rot_mat[4], rot_mat[7]], dtype=np.float64)

        angles = np.linspace(-np.deg2rad(fov_deg * 0.5), np.deg2rad(fov_deg * 0.5), num_rays)
        ray_froms = [drone_pos.tolist()] * num_rays
        ray_tos = []

        for ang in angles:
            # Ray direction in horizontal plane of drone
            ray_dir = np.cos(ang) * forward + np.sin(ang) * right
            ray_dir[2] = 0.0
            norm_d = np.linalg.norm(ray_dir)
            if norm_d > 1e-4:
                ray_dir = ray_dir / norm_d
            ray_to = drone_pos + ray_dir * max_range
            ray_tos.append(ray_to.tolist())

        results = p.rayTestBatch(ray_froms, ray_tos)
        detections = []

        for i, res in enumerate(results):
            hit_object_id = res[0]
            hit_fraction = res[2]
            hit_pos = res[3]

            # Ignore plane and drone's own body
            if hit_object_id > 0 and hit_object_id != self.plane_id and hit_object_id != self.drone_id:
                dist = hit_fraction * max_range
                if dist < max_range * 0.98:
                    dir_vec = np.array(hit_pos) - drone_pos
                    detections.append({
                        'hit_pos': np.array(hit_pos, dtype=np.float64),
                        'dist': float(dist),
                        'dir': dir_vec / (np.linalg.norm(dir_vec) + 1e-6)
                    })

        return detections

    def step(self):
        """Steps PyBullet simulation."""
        p.stepSimulation()

    def close(self):
        """Disconnects PyBullet."""
        p.disconnect(self.client_id)
