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

from uav_quaternion_backstepping import QuaternionMath


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
        if not self.gui:
            return

        # Start sphere (Green)
        start_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.15, rgbaColor=[0.1, 0.9, 0.2, 0.85])
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=start_vis, basePosition=start_pos.tolist())
        p.addUserDebugText("START", start_pos + np.array([0, 0, 0.25]), textColorRGB=[0.1, 0.9, 0.2], textSize=1.5)

        # Destination sphere (Red)
        goal_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.15, rgbaColor=[0.9, 0.1, 0.2, 0.85])
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=goal_vis, basePosition=goal_pos.tolist())
        p.addUserDebugText("DESTINATION", goal_pos + np.array([0, 0, 0.25]), textColorRGB=[0.9, 0.1, 0.2], textSize=1.5)

    def draw_reference_trajectory(self, waypoints: np.ndarray, color=[0.1, 0.9, 0.2], width=2.5):
        """Renders 3D trajectory line segments in PyBullet."""
        if not self.gui or len(waypoints) < 2:
            return

        for i in range(len(waypoints) - 1):
            p.addUserDebugLine(waypoints[i].tolist(),
                               waypoints[i + 1].tolist(),
                               lineColorRGB=color,
                               lineWidth=width)

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

    def step(self):
        """Steps PyBullet simulation."""
        p.stepSimulation()

    def close(self):
        """Disconnects PyBullet."""
        p.disconnect(self.client_id)
