#!/usr/bin/env python3
"""
UAV Controller Node (controller_node.py)
========================================
Implements the full 6-DOF Quadrotor Flight Control Architecture around the
exact Attitude Tracking Controller presented in:
    "Quaternion-Based Attitude Tracking Control Design for UAVs"
    Qain-Rong Lin, Jen-te Yu (CACS 2024)

Mathematical Pipeline:
1. Outer-Loop Translational Controller:
   - Position error: e_p = p - p_ref, Velocity error: e_v = v - v_ref
   - Desired inertial acceleration:
       a_des = a_ref - Kp * e_p - Kd * e_v - Ki * int(e_p) + [0, 0, g]^T
   - Total collective thrust:
       u_1 = m * a_des^T * (R(q) * e_z)
   - Desired attitude quaternion q_ref from a_des and psi_ref
   - Desired angular velocity omega_ref and angular acceleration omega_dot_ref
     via quaternion differential kinematics: [0; omega_ref] = 2 * q_ref^* (x) q_dot_ref

2. Inner-Loop Attitude Controller (Equations 10-23 of Lin & Yu, CACS 2024):
   - Eq. (10): q_err = q_ref^* (x) q
   - Eq. (11): omega_err = omega - omega_ref
   - Eq. (12): q_dot_err = 0.5 * q_err (x) [0; omega_err]
   - Eq. (13): [0; omega_err_d] = -k1 * q_err^* (x) (q_err - [1;0;0;0])
   - Eq. (16): delta = omega_err - omega_err_d
   - Eq. (20): [0; omega_dot_err_d] = d/dt { -k1 * q_err^* (x) (q_err - [1;0;0;0]) }
   - Eq. (21): tau_1 = omega x (J * omega) + J * omega_dot_ref + J * omega_dot_err_d
   - Eq. (22): tau_2 = -k2 * J * delta^(1/3)
   - Total torque: tau = tau_1 + tau_2

3. Quadrotor Control Allocation (Figure 1 of the Paper):
   - Converts [u_1, tau_1, tau_2, tau_3]^T -> Rotor forces [F_1, F_2, F_3, F_4]^T
"""

from __future__ import annotations
import os
import yaml
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import WrenchStamped
    from std_msgs.msg import Float64MultiArray
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object

from uav_simulation.quaternion_math import (
    PaperQuaternionBacksteppingController,
    quat_normalize,
    quat_conj,
    quat_multiply,
    quat_to_rotmat,
    desired_acc_to_quaternion,
)


class CascadedUAVQuaternionController:
    """
    Combines the outer-loop 3D trajectory tracker with the inner-loop
    Quaternion-Based Backstepping Fractional-Exponent Attitude Controller
    from Lin & Yu (CACS 2024).
    """

    def __init__(
        self,
        mass: float = 1.2,
        gravity: float = 9.81,
        J_diag=(0.1, 0.1, 0.12),
        k1: float = 20.0,
        k2: float = 2.0,
        exponent: float = 1.0 / 3.0,
        kp_pos=(4.8, 4.8, 6.5),
        kd_pos=(3.8, 3.8, 4.8),
        ki_pos=(0.15, 0.15, 0.25),
        arm_length: float = 0.225,
        c_tau: float = 0.016,
        max_tilt_deg: float = 35.0,
        max_torque=(4.0, 4.0, 2.5),
        dt: float = 0.005
    ):
        self.mass = float(mass)
        self.gravity = float(gravity)
        self.kp_pos = np.asarray(kp_pos, dtype=float)
        self.kd_pos = np.asarray(kd_pos, dtype=float)
        self.ki_pos = np.asarray(ki_pos, dtype=float)
        self.arm_length = float(arm_length)
        self.c_tau = float(c_tau)
        self.max_tilt_rad = float(np.radians(max_tilt_deg))
        self.max_torque = np.asarray(max_torque, dtype=float)
        self.dt = float(dt)

        # Paper's exact attitude controller (Equations 10-23)
        self.attitude_controller = PaperQuaternionBacksteppingController(
            J_diag=J_diag,
            k1=k1,
            k2=k2,
            exponent=exponent,
            enforce_shortest_path=True
        )

        # State memory for smooth finite-difference differentiation of q_ref(t) -> omega_ref(t)
        self.int_pos_err = np.zeros(3, dtype=float)
        self.prev_q_ref = None
        self.prev_omega_ref = np.zeros(3, dtype=float)

        # Control allocation matrix M mapping [F1, F2, F3, F4]^T -> [u1, tau_x, tau_y, tau_z]^T
        L = self.arm_length
        c = self.c_tau
        self.alloc_matrix = np.array([
            [1.0,  1.0,  1.0,  1.0],
            [0.0, -L,    0.0,  L],
            [-L,   0.0,  L,    0.0],
            [-c,   c,   -c,    c]
        ], dtype=float)
        self.alloc_matrix_inv = np.linalg.inv(self.alloc_matrix)

    def step(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
        q: np.ndarray,
        omega: np.ndarray,
        pos_ref: np.ndarray,
        vel_ref: np.ndarray,
        acc_ref: np.ndarray,
        yaw_ref: float = 0.0,
        dt: float = None
    ) -> dict:
        """
        Executes one control cycle at sampling step `dt`.
        """
        if dt is None or dt <= 1e-6:
            dt = self.dt

        pos = np.asarray(pos, dtype=float).flatten()
        vel = np.asarray(vel, dtype=float).flatten()
        q = quat_normalize(q)
        omega = np.asarray(omega, dtype=float).flatten()
        pos_ref = np.asarray(pos_ref, dtype=float).flatten()
        vel_ref = np.asarray(vel_ref, dtype=float).flatten()
        acc_ref = np.asarray(acc_ref, dtype=float).flatten()

        # 1. Position and velocity tracking errors
        pos_err = pos - pos_ref
        vel_err = vel - vel_ref

        self.int_pos_err = np.clip(self.int_pos_err + pos_err * dt, -1.5, 1.5)

        # 2. Desired 3D acceleration vector in inertial frame
        a_des = (
            acc_ref
            - self.kp_pos * pos_err
            - self.kd_pos * vel_err
            - self.ki_pos * self.int_pos_err
            + np.array([0.0, 0.0, self.gravity], dtype=float)
        )

        # 3. Desired unit quaternion q_ref from desired acceleration vector and yaw_ref
        q_ref = desired_acc_to_quaternion(a_des, yaw_des=yaw_ref, max_tilt_rad=self.max_tilt_rad)
        if np.dot(q_ref, q) < 0.0:
            q_ref = -q_ref

        # 4. Compute desired angular velocity omega_ref and angular acceleration omega_dot_ref
        #    using inverse of Equation (8): [0; omega_ref] = 2 * q_ref^* (x) q_dot_ref
        if self.prev_q_ref is None:
            self.prev_q_ref = q_ref.copy()
            omega_ref = np.zeros(3, dtype=float)
            omega_dot_ref = np.zeros(3, dtype=float)
        else:
            if np.dot(q_ref, self.prev_q_ref) < 0.0:
                self.prev_q_ref = -self.prev_q_ref
            q_dot_ref = (q_ref - self.prev_q_ref) / dt
            omega_ref_quat = 2.0 * quat_multiply(quat_conj(q_ref), q_dot_ref)
            raw_omega_ref = np.clip(omega_ref_quat[1:4], -4.0, 4.0)
            # Low-pass filter to prevent numerical differentiation noise
            alpha = 0.25
            omega_ref = (1.0 - alpha) * self.prev_omega_ref + alpha * raw_omega_ref
            omega_dot_ref = np.clip((omega_ref - self.prev_omega_ref) / dt, -15.0, 15.0)
            self.prev_q_ref = q_ref.copy()
            self.prev_omega_ref = omega_ref.copy()

        # 5. Total collective thrust u_1 projected onto current body Z axis b_z = R(q)*e_z
        R = quat_to_rotmat(q)
        b_z = R[:, 2]
        u1 = float(self.mass * np.dot(a_des, b_z))
        u1 = float(np.clip(u1, 0.1 * self.mass * self.gravity, 3.0 * self.mass * self.gravity))

        # 6. Inner-Loop Attitude Controller (Equations 10-23 of CACS 2024 Paper)
        att_out = self.attitude_controller.compute_control(
            q=q,
            omega=omega,
            q_ref=q_ref,
            omega_ref=omega_ref,
            omega_dot_ref=omega_dot_ref
        )

        tau = np.clip(att_out['tau'], -self.max_torque, self.max_torque)

        # 7. Quadrotor Motor Thrust Allocation (Figure 1: F1, F2, F3, F4)
        wrench_vec = np.array([u1, tau[0], tau[1], tau[2]], dtype=float)
        motor_forces = np.clip(self.alloc_matrix_inv @ wrench_vec, 0.0, 15.0)

        return {
            'u1': u1,
            'tau': tau,
            'tau1': att_out['tau1'],
            'tau2': att_out['tau2'],
            'motor_forces': motor_forces,
            'q_ref': att_out['q_ref_aligned'],
            'omega_ref': omega_ref,
            'omega_dot_ref': omega_dot_ref,
            'q_err': att_out['q_err'],
            'omega_err': att_out['omega_err'],
            'omega_err_d': att_out['omega_err_d'],
            'delta': att_out['delta'],
            'pos_err': pos_err,
            'vel_err': vel_err,
        }


class ControllerNode(Node):
    """
    ROS 2 Node executing the Quaternion-Based Backstepping Fractional-Exponent
    UAV Controller at `control_frequency` Hz.
    """

    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('config_file', '')
        self.declare_parameter('uav_mass', 1.20)
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('inertia', [0.1, 0.1, 0.12])
        self.declare_parameter('k1', 20.0)
        self.declare_parameter('k2', 2.0)
        self.declare_parameter('fractional_exponent', 0.3333333333)
        self.declare_parameter('kp_pos', [4.8, 4.8, 6.5])
        self.declare_parameter('kd_pos', [3.8, 3.8, 4.8])
        self.declare_parameter('ki_pos', [0.15, 0.15, 0.25])
        self.declare_parameter('arm_length', 0.225)
        self.declare_parameter('thrust_to_torque_coeff', 0.016)
        self.declare_parameter('max_tilt_angle_deg', 35.0)
        self.declare_parameter('max_torque', [4.0, 4.0, 2.5])
        self.declare_parameter('control_frequency', 200.0)
        self.declare_parameter('initial_position', [0.0, 0.0, 2.0])

        config_file = self.get_parameter('config_file').value
        if config_file and os.path.exists(config_file):
            with open(config_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            p = data.get('uav_simulation', {}).get('ros__parameters', data)
            mass = float(p.get('uav_mass', 1.20))
            gravity = float(p.get('gravity', 9.81))
            inertia = list(p.get('inertia', [0.1, 0.1, 0.12]))
            k1 = float(p.get('k1', 20.0))
            k2 = float(p.get('k2', 2.0))
            exp_val = float(p.get('fractional_exponent', 1.0 / 3.0))
            kp_pos = list(p.get('kp_pos', [4.8, 4.8, 6.5]))
            kd_pos = list(p.get('kd_pos', [3.8, 3.8, 4.8]))
            ki_pos = list(p.get('ki_pos', [0.15, 0.15, 0.25]))
            arm_len = float(p.get('arm_length', 0.225))
            c_tau = float(p.get('thrust_to_torque_coeff', 0.016))
            max_tilt = float(p.get('max_tilt_angle_deg', 35.0))
            max_torque = list(p.get('max_torque', [4.0, 4.0, 2.5]))
            freq = float(p.get('control_frequency', 200.0))
            init_pos = list(p.get('initial_position', [0.0, 0.0, 2.0]))
        else:
            mass = float(self.get_parameter('uav_mass').value)
            gravity = float(self.get_parameter('gravity').value)
            inertia = list(self.get_parameter('inertia').value)
            k1 = float(self.get_parameter('k1').value)
            k2 = float(self.get_parameter('k2').value)
            exp_val = float(self.get_parameter('fractional_exponent').value)
            kp_pos = list(self.get_parameter('kp_pos').value)
            kd_pos = list(self.get_parameter('kd_pos').value)
            ki_pos = list(self.get_parameter('ki_pos').value)
            arm_len = float(self.get_parameter('arm_length').value)
            c_tau = float(self.get_parameter('thrust_to_torque_coeff').value)
            max_tilt = float(self.get_parameter('max_tilt_angle_deg').value)
            max_torque = list(self.get_parameter('max_torque').value)
            freq = float(self.get_parameter('control_frequency').value)
            init_pos = list(self.get_parameter('initial_position').value)

        self.dt = 1.0 / max(freq, 20.0)
        self.controller = CascadedUAVQuaternionController(
            mass=mass,
            gravity=gravity,
            J_diag=inertia,
            k1=k1,
            k2=k2,
            exponent=exp_val,
            kp_pos=kp_pos,
            kd_pos=kd_pos,
            ki_pos=ki_pos,
            arm_length=arm_len,
            c_tau=c_tau,
            max_tilt_deg=max_tilt,
            max_torque=max_torque,
            dt=self.dt
        )

        # Current UAV states
        self.pos = np.array(init_pos, dtype=float)
        self.vel = np.zeros(3, dtype=float)
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.omega = np.zeros(3, dtype=float)

        # Reference states
        self.pos_ref = np.array(init_pos, dtype=float)
        self.vel_ref = np.zeros(3, dtype=float)
        self.acc_ref = np.zeros(3, dtype=float)

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/uav/odometry', self.odom_callback, 20
        )
        self.ref_sub = self.create_subscription(
            Float64MultiArray, '/uav/reference_state', self.ref_callback, 20
        )

        # Publishers
        self.wrench_pub = self.create_publisher(
            WrenchStamped, '/uav/control_wrench', 20
        )
        self.diag_pub = self.create_publisher(
            Float64MultiArray, '/uav/controller_diagnostics', 20
        )

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            f"CACS 2024 Quaternion Backstepping Controller started | "
            f"J=diag{inertia}, k1={k1}, k2={k2}, exp={exp_val:.3f}, rate={freq} Hz"
        )

    def odom_callback(self, msg: Odometry):
        self.pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ], dtype=float)
        self.vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ], dtype=float)
        # Convert ROS Quaternion (x, y, z, w) -> Paper Quaternion [q0(w), q1(x), q2(y), q3(z)]
        self.q = quat_normalize(np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ], dtype=float))
        self.omega = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ], dtype=float)

    def ref_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 9:
            self.pos_ref = np.array(msg.data[0:3], dtype=float)
            self.vel_ref = np.array(msg.data[3:6], dtype=float)
            self.acc_ref = np.array(msg.data[6:9], dtype=float)

    def control_loop(self):
        out = self.controller.step(
            pos=self.pos,
            vel=self.vel,
            q=self.q,
            omega=self.omega,
            pos_ref=self.pos_ref,
            vel_ref=self.vel_ref,
            acc_ref=self.acc_ref,
            yaw_ref=0.0,
            dt=self.dt
        )

        # Publish WrenchStamped to Gazebo UAV dynamics bridge
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = 'base_link'
        wrench_msg.wrench.force.x = 0.0
        wrench_msg.wrench.force.y = 0.0
        wrench_msg.wrench.force.z = float(out['u1'])
        wrench_msg.wrench.torque.x = float(out['tau'][0])
        wrench_msg.wrench.torque.y = float(out['tau'][1])
        wrench_msg.wrench.torque.z = float(out['tau'][2])
        self.wrench_pub.publish(wrench_msg)

        # Publish comprehensive diagnostics for Data Logger & Live Right-Side Panel
        # Layout:
        # [0..3]: q_ref (q0,q1,q2,q3)
        # [4..7]: q_err (q0,q1,q2,q3)
        # [8..10]: omega_ref (wx,wy,wz)
        # [11..13]: omega_err (wx,wy,wz)
        # [14..16]: delta (dx,dy,dz)
        # [17..19]: tau (tx,ty,tz)
        # [20]: u1 (total thrust)
        # [21..24]: motor_forces (F1,F2,F3,F4)
        diag_msg = Float64MultiArray()
        diag_msg.data = [
            *out['q_ref'].tolist(),
            *out['q_err'].tolist(),
            *out['omega_ref'].tolist(),
            *out['omega_err'].tolist(),
            *out['delta'].tolist(),
            *out['tau'].tolist(),
            float(out['u1']),
            *out['motor_forces'].tolist()
        ]
        self.diag_pub.publish(diag_msg)


def main(args=None):
    if not ROS2_AVAILABLE:
        print("ROS 2 (rclpy) is not sourced in this shell.")
        return
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
