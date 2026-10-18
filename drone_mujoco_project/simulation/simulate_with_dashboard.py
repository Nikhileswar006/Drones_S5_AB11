import os
import sys
import time
import argparse
import numpy as np
import mujoco

# Add project root to sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from controllers.quaternion import quat_to_euler
from controllers.backstepping import BacksteppingController
from controllers.position_controller import PositionController
from controllers.motor_mixer import MotorMixer
from controllers.waypoint_manager import WaypointManager
from controllers.obstacle_avoidance import ObstacleAvoidancePlanner
from visualization.live_dashboard import LiveDashboard

MODEL_PATH = os.path.join(PROJECT_ROOT, "models", "quadrotor.xml")
RESULTS_DIR = os.path.join(PROJECT_ROOT, "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

DEFAULT_WAYPOINTS = [
    [3.0, 2.0, 2.5],   # WP1
    [6.0, 8.0, 3.5],   # WP2
    [10.0, 4.0, 4.0],  # WP3
    [14.0, 12.0, 5.0]  # Destination
]

def run_simulation_with_dashboard(start_pos: np.ndarray,
                                  waypoints: list,
                                  duration: float = 38.0,
                                  tolerance: float = 0.75,
                                  final_tolerance: float = 0.40,
                                  width: int = 1600,
                                  height: int = 900,
                                  save_snapshot_at: float = None,
                                  snapshot_path: str = None,
                                  max_steps: int = None):
    """
    Runs the closed-loop 500 Hz drone flight simulation integrated with
    a Full-Screen 3D UAV viewport and Floating HUD Telemetry Card.
    """
    print("=" * 80)
    print(" FULL-SCREEN 3D UAV SIMULATION + FLOATING LIVE RESULTS DASHBOARD ")
    print("=" * 80)
    print(f" Start Position P0        : {start_pos}")
    print(f" Intermediate Waypoints   : {waypoints[:-1]}")
    print(f" Final Destination Goal   : {waypoints[-1]}")
    print(f" Viewport Resolution      : {width} x {height} (Full Screen 3D)")
    print(f" Simulation Duration      : {duration} s")
    print("=" * 80)

    # 1. Instantiate MuJoCo Model & Data
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    dt = 0.002
    model.opt.timestep = dt

    # 2. Instantiate Controllers
    wp_manager = WaypointManager(waypoints=waypoints, tolerance=tolerance, final_tolerance=final_tolerance)
    obstacle_planner = ObstacleAvoidancePlanner(drone_radius=1.15, safety_margin=0.50, detect_dist=3.5)
    pos_controller = PositionController(mass=1.0, gravity=9.81, 
                                        Kp=np.array([1.2, 1.2, 2.0]), 
                                        Kv=np.array([2.6, 2.6, 2.5]))
    attitude_controller = BacksteppingController(J=np.diag([0.1, 0.1, 0.12]), k1=20.0, k2=2.0)
    mixer = MotorMixer(arm_length=0.85)

    # 3. Offscreen Full-Screen 3D Renderer
    renderer = mujoco.Renderer(model, height=height, width=width)

    # Camera setups matching reference video perspective
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera_modes = ["Flight Path", "Chase", "Overview", "Top-Down"]
    cam_mode_idx = 0

    def apply_camera(mode: str, drone_pos: np.ndarray):
        if mode == "Flight Path":
            # Perspective looking down the green trajectory corridor from Start
            camera.lookat = np.array([6.5, 5.5, 2.8])
            camera.distance = 18.0
            camera.azimuth = 135.0
            camera.elevation = -18.0
        elif mode == "Chase":
            # Dynamic 3rd person follow camera tracking the moving drone
            camera.lookat = drone_pos.copy()
            camera.distance = 5.0
            camera.azimuth = 135.0
            camera.elevation = -18.0
        elif mode == "Overview":
            # Wide arena overview of the entire flight volume
            camera.lookat = np.array([7.0, 6.0, 3.0])
            camera.distance = 23.0
            camera.azimuth = 135.0
            camera.elevation = -24.0
        elif mode == "Top-Down":
            # Tactical top-down view
            camera.lookat = np.array([7.0, 6.0, 0.0])
            camera.distance = 25.0
            camera.azimuth = 90.0
            camera.elevation = -89.0

    # 4. Instantiate Live Dashboard
    dashboard = LiveDashboard(width=width, height=height)
    dashboard.camera_mode = camera_modes[cam_mode_idx]

    # Initialize Drone State to start_pos
    def reset_state():
        mujoco.mj_resetData(model, data)
        data.qpos[0:3] = start_pos
        data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[:] = 0.0
        # Sync mocap positions
        mocap_names = ["Start_Point", "WP1", "WP2", "WP3", "Destination"]
        mocap_coords = [start_pos] + list(waypoints)
        for name, coord in zip(mocap_names, mocap_coords):
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid != -1:
                mid = model.body_mocapid[bid]
                if mid != -1:
                    data.mocap_pos[mid] = coord
        mujoco.mj_forward(model, data)
        wp_manager.reset()
        if 'obstacle_planner' in locals() or 'obstacle_planner' in globals():
            obstacle_planner.reset()
        if 'actual_path_history' in locals() or 'actual_path_history' in globals():
            actual_path_history.clear()
            actual_path_history.append(start_pos.copy())
            error_history.clear()

    # Trajectory path histories
    full_ref_path = [start_pos.copy()] + [np.array(wp, dtype=float) for wp in waypoints]
    actual_path_history = [start_pos.copy()]
    error_history = []

    reset_state()

    # Timing and rates
    physics_steps_per_render = 10 # 500 Hz physics / 10 = 50 Hz UI updates
    render_dt = dt * physics_steps_per_render
    fps_timer = time.time()
    fps_counter = 0
    current_fps = 50.0

    step_idx = 0
    t_sim = 0.0
    running = True
    mission_done = False
    snapshot_taken = False

    # Marker definitions for 3D tags (Waypoints + Physical Obstacles)
    marker_positions = {
        'Start_Point': (start_pos, (68, 138, 255), "START (WP0)"),
        'WP1': (waypoints[0], (255, 171, 0), "WP1"),
        'WP2': (waypoints[1], (255, 171, 0), "WP2"),
        'WP3': (waypoints[2], (255, 171, 0), "WP3"),
        'Destination': (waypoints[3], (0, 230, 118), "DESTINATION"),
        'Obs1_Pillar': (np.array([1.65, 1.10, 1.75]), (255, 90, 80), "OBS 1: PILLAR"),
        'Obs2_Tower': (np.array([4.5, 5.0, 2.05]), (80, 150, 255), "OBS 2: TOWER"),
        'Obs3_Sphere': (np.array([8.0, 6.0, 4.05]), (255, 190, 40), "OBS 3: SPHERE"),
        'Obs4_Pylon': (np.array([12.0, 8.0, 2.85]), (235, 80, 175), "OBS 4: PYLON")
    }

    # Initial controller & avoidance states
    T_desired = 9.81
    tau = np.zeros(3)
    q_err = np.array([1.0, 0.0, 0.0, 0.0])
    safe_target = np.array(waypoints[0], dtype=float)
    avoidance_telemetry = {
        'avoidance_active': False,
        'state': 'CLEAR',
        'active_obs_id': None,
        'active_obs_name': 'None',
        'nearest_obs_id': 'Obs1_Pillar',
        'nearest_obs_dist': float(np.linalg.norm(start_pos - np.array([1.65, 1.10, 1.25])) - 0.30),
        'detour_target': None,
        'detour_path': [],
        'collision_detected': False,
        'collision_count': 0,
        'min_clearance': 1.45
    }

    print("\n[Controls] [Space] Pause/Resume  |  [C] Switch Camera  |  [H] Toggle HUD  |  [R] Reset  |  [S] Snapshot  |  [Esc] Exit\n")

    try:
        while running and (max_steps is None or step_idx < max_steps):
            loop_start = time.time()

            # Process UI Events
            actions = dashboard.process_events()
            if actions["quit"]:
                break
            if actions["reset"]:
                reset_state()
                step_idx = 0
                t_sim = 0.0
                mission_done = False
                continue
            if actions["camera"]:
                cam_mode_idx = (cam_mode_idx + 1) % len(camera_modes)
                dashboard.camera_mode = camera_modes[cam_mode_idx]
            if actions["screenshot"]:
                snap_file = os.path.join(RESULTS_DIR, f"dashboard_snap_{int(time.time())}.png")
                dashboard.save_screenshot(snap_file)
                print(f"[Snapshot] Saved manually to: {snap_file}")

            # Check Pause
            if dashboard.paused:
                apply_camera(dashboard.camera_mode, data.qpos[0:3])
                renderer.update_scene(data, camera=camera)
                sim_pixels = renderer.render()
                c = renderer.scene.camera[0]
                cam_info = {'pos': np.array(c.pos), 'forward': np.array(c.forward), 'up': np.array(c.up), 'fovy': float(model.vis.global_.fovy)}

                dashboard.draw_scene(sim_pixels)
                dashboard.draw_paths(
                    cam_info, full_ref_path, actual_path_history,
                    detour_path=avoidance_telemetry.get('detour_path'),
                    detour_target=avoidance_telemetry.get('detour_target'),
                    avoidance_active=avoidance_telemetry.get('avoidance_active', False)
                )
                dashboard.draw_3d_marker_tags(cam_info, marker_positions)
                dashboard.draw_floating_hud(
                    t=t_sim, current_wp_idx=wp_manager.current_idx, total_wps=len(waypoints),
                    target_name=wp_manager.get_current_target_name(),
                    dist_err=float(np.linalg.norm(wp_manager.get_current_target() - data.qpos[0:3])),
                    actual_pos=data.qpos[0:3], target_pos=safe_target,
                    euler=quat_to_euler(data.qpos[3:7]), quat_err=q_err,
                    total_thrust=T_desired, torques=tau,
                    switch_history=wp_manager.switch_history, mission_completed=mission_done,
                    rmse=current_rmse if 'current_rmse' in locals() else None,
                    avoidance_info=avoidance_telemetry
                )
                dashboard.draw_top_right_legend()
                dashboard.draw_bottom_bar(t=t_sim, fps=current_fps, alt=data.qpos[2])
                dashboard.refresh()
                time.sleep(0.02)
                continue

            # Execute N Physics Steps (500 Hz)
            for _ in range(physics_steps_per_render):
                t_sim = step_idx * dt

                # State feedback
                pos = data.qpos[0:3].copy()
                vel = data.qvel[0:3].copy()
                quat = data.qpos[3:7].copy()
                omega = data.qvel[3:6].copy()

                # Step 1: Waypoint Manager -> Reference Target
                nominal_target, dist_to_tgt, switched, reached_final = wp_manager.update(pos, t_sim)
                if reached_final and not mission_done:
                    mission_done = True
                    print(f"\n[Mission Complete] Final Destination reached at t={t_sim:.2f}s!")

                # Step 2: Obstacle Avoidance Module -> Safe Target Reference
                safe_target, avoidance_telemetry = obstacle_planner.update(
                    current_pos=pos, current_vel=vel,
                    nominal_target=nominal_target,
                    current_waypoint_idx=wp_manager.current_idx,
                    current_time=t_sim
                )

                # MuJoCo Physics Contact Solver Collision Check
                obstacle_planner.check_mujoco_contacts(model, data)

                # Step 3: Position Controller -> Desired Attitude & Thrust
                is_final = (wp_manager.current_idx == wp_manager.num_waypoints - 1)
                T_desired, q_ref, e_p, a_cmd = pos_controller.compute(
                    current_pos=pos, current_vel=vel,
                    target_pos=safe_target, target_yaw=0.0,
                    is_final_target=is_final
                )

                # Step 4: Quaternion-based Backstepping Controller -> Motor Torques
                tau, q_err, w_err, delta = attitude_controller.compute(
                    quat=quat, omega=omega,
                    q_ref=q_ref
                )

                # Step 5: Motor Mixer -> Actuator Commands
                u_thrusts, yaw_torque = mixer.mix(T_desired, tau)

                # Apply Controls to MuJoCo actuators
                data.ctrl[0:4] = u_thrusts
                data.ctrl[4] = yaw_torque

                # Step 6: Advance MuJoCo Physics (dt=0.002s)
                mujoco.mj_step(model, data)
                step_idx += 1

                if step_idx * dt >= duration and not mission_done:
                    mission_done = True

            # Telemetry for Display
            euler = quat_to_euler(data.qpos[3:7])
            dist_err = float(np.linalg.norm(nominal_target - data.qpos[0:3]))

            # Record flight path history — stop after mission complete to
            # prevent long trailing blue lines from hover settling
            if not mission_done:
                actual_path_history.append(data.qpos[0:3].copy())
                error_history.append(dist_err)
            elif len(actual_path_history) > 0:
                # Record one final point at destination for a clean ending
                last_recorded = actual_path_history[-1]
                if float(np.linalg.norm(data.qpos[0:3] - last_recorded)) > 0.05:
                    actual_path_history.append(data.qpos[0:3].copy())
                    error_history.append(dist_err)
            current_rmse = float(np.sqrt(np.mean(np.array(error_history)**2))) if error_history else dist_err

            # Render Full-Screen 3D Viewport with MuJoCo
            apply_camera(dashboard.camera_mode, data.qpos[0:3])
            renderer.update_scene(data, camera=camera)
            sim_pixels = renderer.render()

            # Measure FPS
            fps_counter += 1
            if time.time() - fps_timer >= 1.0:
                current_fps = fps_counter / (time.time() - fps_timer)
                fps_counter = 0
                fps_timer = time.time()

            # 1. Full-Screen 3D Scene
            dashboard.draw_scene(sim_pixels)

            # 2. Reference Path (Planned) & Actual Path (Drone Movement) & Safe Detour Path
            c = renderer.scene.camera[0]
            cam_info = {'pos': np.array(c.pos), 'forward': np.array(c.forward), 'up': np.array(c.up), 'fovy': float(model.vis.global_.fovy)}
            dashboard.draw_paths(
                cam_info, full_ref_path, actual_path_history,
                detour_path=avoidance_telemetry.get('detour_path'),
                detour_target=avoidance_telemetry.get('detour_target'),
                avoidance_active=avoidance_telemetry.get('avoidance_active', False)
            )

            # 3. Dynamic 3D Marker Tags directly in the 3D scene (Waypoints + Obstacles)
            dynamic_markers = dict(marker_positions)
            active_obs_id = avoidance_telemetry.get('active_obs_id')
            if active_obs_id and active_obs_id in dynamic_markers:
                pt, col, _ = dynamic_markers[active_obs_id]
                dynamic_markers[active_obs_id] = (pt, (255, 170, 0), f"AVOIDING: {avoidance_telemetry.get('active_obs_name')}")

            dashboard.draw_3d_marker_tags(cam_info, dynamic_markers)

            # 4. Floating HUD Card (Top-Left)
            dashboard.draw_floating_hud(
                t=t_sim,
                current_wp_idx=wp_manager.current_idx,
                total_wps=len(waypoints),
                target_name=wp_manager.get_current_target_name(),
                dist_err=dist_err,
                actual_pos=data.qpos[0:3],
                target_pos=safe_target,
                euler=euler,
                quat_err=q_err,
                total_thrust=T_desired,
                torques=tau,
                switch_history=wp_manager.switch_history,
                mission_completed=mission_done,
                rmse=current_rmse,
                avoidance_info=avoidance_telemetry
            )

            # 5. Arena Trajectory & Obstacle Legend (Top-Right)
            dashboard.draw_top_right_legend()

            # 5. Bottom Status / Control Bar
            dashboard.draw_bottom_bar(t=t_sim, fps=current_fps, alt=data.qpos[2])

            # Flip Buffer
            dashboard.refresh()

            # Automatic snapshot for verification
            if save_snapshot_at is not None and not snapshot_taken and t_sim >= save_snapshot_at:
                out_path = snapshot_path or os.path.join(RESULTS_DIR, "live_dashboard_fullscreen_preview.png")
                dashboard.save_screenshot(out_path)
                print(f"\n[Snapshot] Captured live full-screen dashboard frame at t={t_sim:.2f}s to: {out_path}")
                snapshot_taken = True

            # Pacing
            elapsed = time.time() - loop_start
            sleep_time = render_dt - elapsed
            if sleep_time > 0.001:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[Dashboard] Simulation interrupted by user.")
    finally:
        dashboard.close()
        print("[Dashboard] Closed display cleanly.")

def main():
    parser = argparse.ArgumentParser(description="MuJoCo Full-Screen 3D UAV Flight Simulation with Floating HUD Dashboard")
    parser.add_argument("--duration", type=float, default=38.0, help="Simulation duration (s)")
    parser.add_argument("--tolerance", type=float, default=0.75, help="Waypoint tolerance (m)")
    parser.add_argument("--final-tolerance", type=float, default=0.40, help="Final destination tolerance (m)")
    parser.add_argument("--width", type=int, default=1600, help="Window width (px)")
    parser.add_argument("--height", type=int, default=900, help="Window height (px)")
    parser.add_argument("--snapshot-at", type=float, default=None, help="Capture screenshot at specific flight second")
    parser.add_argument("--snapshot-path", type=str, default=None, help="Target path for captured screenshot")
    parser.add_argument("--max-steps", type=int, default=None, help="Max physics steps (for automated test runs)")
    args = parser.parse_args()

    run_simulation_with_dashboard(
        start_pos=np.array([0.0, 0.0, 1.0]),
        waypoints=DEFAULT_WAYPOINTS,
        duration=args.duration,
        tolerance=args.tolerance,
        final_tolerance=args.final_tolerance,
        width=args.width,
        height=args.height,
        save_snapshot_at=args.snapshot_at,
        snapshot_path=args.snapshot_path,
        max_steps=args.max_steps
    )

if __name__ == "__main__":
    main()
