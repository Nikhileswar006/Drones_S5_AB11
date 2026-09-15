"""
"Quaternion-Based Attitude Tracking Control Design for UAVs"
Figures 3 - 12 directly through numerical simulation of:
Eq. (8): dot(q) = 0.5 * q (x) [0; omega]
Eq. (9): dot(omega) = J^(-1) * [tau - omega x (J * omega)]
Eq. (10) - (23): Backstepping control law with fractional term delta^(1/3)
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from uav_quaternion_backstepping import QuaternionMath, QuaternionBacksteppingController


def run_paper_attitude_benchmark(duration=10.0, dt=0.001, save_plots=True):
    print("=" * 70)
    print(" Attitude Tracking ")
    print("=" * 70)

    # Parameters from Section V
    J = np.diag([0.1, 0.1, 0.12])
    k1 = 20.0
    k2 = 2.0
    controller = QuaternionBacksteppingController(J=J, k1=k1, k2=k2)

    # Initial reference states
    q_ref_0 = np.array([0.94628, -0.1541, 0.19051, -0.21098], dtype=np.float64)
    q_ref_0 = QuaternionMath.normalize(q_ref_0)

    # Target reference angular velocity trajectory (three sine waves matching Figs 8, 9, 10)
    def get_w_ref(t):
        w1 = 0.10 * np.sin(0.75 * t - np.pi / 2.0)
        w2 = 0.45 * np.sin(0.90 * t - 0.45)
        w3 = 0.35 * np.sin(0.70 * t + 0.60)
        return np.array([w1, w2, w3], dtype=np.float64)

    def get_dw_ref(t):
        dw1 = 0.10 * 0.75 * np.cos(0.75 * t - np.pi / 2.0)
        dw2 = 0.45 * 0.90 * np.cos(0.90 * t - 0.45)
        dw3 = 0.35 * 0.70 * np.cos(0.70 * t + 0.60)
        return np.array([dw1, dw2, dw3], dtype=np.float64)

    # Initial drone state with perturbation to reproduce the transient in Figs 3, 7, 11
    # As observed in Figs 3, 7, 11: q_err(0) != [1, 0, 0, 0] and omega_err(0) != [0, 0, 0]
    q_curr = np.array([0.80, 0.25, 0.15, -0.52], dtype=np.float64)
    q_curr = QuaternionMath.normalize(q_curr)
    omega_curr = np.array([4.2, -2.8, 1.8], dtype=np.float64)

    q_ref_curr = q_ref_0.copy()

    # Telemetry records
    t_arr = np.arange(0, duration, dt)
    n_steps = len(t_arr)

    log = {
        'time': t_arr,
        'q_err': np.zeros((n_steps, 4)),
        'euler': np.zeros((n_steps, 3)),
        'euler_ref': np.zeros((n_steps, 3)),
        'omega': np.zeros((n_steps, 3)),
        'omega_ref': np.zeros((n_steps, 3)),
        'omega_err': np.zeros((n_steps, 3)),
        'delta': np.zeros((n_steps, 3)),
        'tau': np.zeros((n_steps, 3))
    }

    # RK4 integration loop
    for i, t in enumerate(t_arr):
        w_ref = get_w_ref(t)
        dw_ref = get_dw_ref(t)

        # Compute control torque using paper Eqs. (10) - (23)
        res = controller.compute_control(
            q=q_curr,
            omega=omega_curr,
            q_ref=q_ref_curr,
            omega_ref=w_ref,
            d_omega_ref=dw_ref,
            dt=dt
        )
        tau = res['tau']

        # Log
        log['q_err'][i] = res['q_err']
        log['euler'][i] = QuaternionMath.to_euler_deg(q_curr)
        log['euler_ref'][i] = QuaternionMath.to_euler_deg(q_ref_curr)
        log['omega'][i] = omega_curr
        log['omega_ref'][i] = w_ref
        log['omega_err'][i] = res['omega_err']
        log['delta'][i] = res['delta']
        log['tau'][i] = tau

        # Dynamics derivatives
        def dynamics(q, w, torque):
            # Eq. (8): dot(q) = 0.5 * q (x) [0; w]
            q_dot = 0.5 * QuaternionMath.multiply(q, np.array([0.0, w[0], w[1], w[2]]))
            # Eq. (9): dot(w) = J^(-1) * [tau - w x (J * w)]
            w_dot = controller.J_inv @ (torque - np.cross(w, controller.J @ w))
            return q_dot, w_dot

        # RK4 step for drone
        k1_q, k1_w = dynamics(q_curr, omega_curr, tau)
        k2_q, k2_w = dynamics(q_curr + 0.5 * dt * k1_q, omega_curr + 0.5 * dt * k1_w, tau)
        k3_q, k3_w = dynamics(q_curr + 0.5 * dt * k2_q, omega_curr + 0.5 * dt * k2_w, tau)
        k4_q, k4_w = dynamics(q_curr + dt * k3_q, omega_curr + dt * k3_w, tau)

        q_curr = q_curr + (dt / 6.0) * (k1_q + 2.0 * k2_q + 2.0 * k3_q + k4_q)
        q_curr = QuaternionMath.normalize(q_curr)
        omega_curr = omega_curr + (dt / 6.0) * (k1_w + 2.0 * k2_w + 2.0 * k3_w + k4_w)

        # RK4 step for reference quaternion
        ref_q_dot = 0.5 * QuaternionMath.multiply(q_ref_curr, np.array([0.0, w_ref[0], w_ref[1], w_ref[2]]))
        q_ref_curr = q_ref_curr + dt * ref_q_dot
        q_ref_curr = QuaternionMath.normalize(q_ref_curr)

    print("Numerical simulation completed successfully!")

    if save_plots:
        plot_paper_benchmark(log)

    return log


def plot_paper_benchmark(log, output_dir=None):
    if output_dir is None:
        output_dir = os.path.dirname(os.path.abspath(__file__))

    t = log['time']
    q_err = log['q_err']
    euler = log['euler']
    euler_ref = log['euler_ref']
    omega = log['omega']
    omega_ref = log['omega_ref']
    omega_err = log['omega_err']
    delta = log['delta']
    tau = log['tau']

    fig, axes = plt.subplots(5, 2, figsize=(16, 18))

    # Fig 3
    axes[0, 0].plot(t, q_err[:, 0], label=r'$q_{err0}$', color='#1f77b4')
    axes[0, 0].plot(t, q_err[:, 1], label=r'$q_{err1}$', color='#2ca02c')
    axes[0, 0].plot(t, q_err[:, 2], label=r'$q_{err2}$', color='#d62728')
    axes[0, 0].plot(t, q_err[:, 3], label=r'$q_{err3}$', color='#9467bd')
    axes[0, 0].set_title('Fig. 3. Trajectory of quaternion error')
    axes[0, 0].set_xlabel('Time (sec)')
    axes[0, 0].set_ylabel('Quaternion Error')
    axes[0, 0].grid(True)
    axes[0, 0].legend()
    axes[0, 0].set_xlim([0, 1])

    # Fig 7
    axes[0, 1].plot(t, omega_err[:, 0], label=r'$\omega_{err1}$', color='#1f77b4')
    axes[0, 1].plot(t, omega_err[:, 1], label=r'$\omega_{err2}$', color='#d62728')
    axes[0, 1].plot(t, omega_err[:, 2], label=r'$\omega_{err3}$', color='#2ca02c')
    axes[0, 1].set_title('Fig. 7. Trajectory of angular velocity error')
    axes[0, 1].set_xlabel('Time (seconds)')
    axes[0, 1].set_ylabel(r'$\omega_{err}$ (rad/sec)')
    axes[0, 1].grid(True)
    axes[0, 1].legend()
    axes[0, 1].set_xlim([0, 10])

    # Fig 4: Yaw
    axes[1, 0].plot(t, euler[:, 0], label='actual', color='#1f77b4')
    axes[1, 0].plot(t, euler_ref[:, 0], label='reference', linestyle='--', color='#ff7f0e')
    axes[1, 0].set_title('Fig. 4. Trajectory of yaw angle tracking')
    axes[1, 0].set_xlabel('Time (sec)')
    axes[1, 0].set_ylabel('Yaw angle (degree)')
    axes[1, 0].grid(True)
    axes[1, 0].legend()

    # Fig 8: Roll rate
    axes[1, 1].plot(t, omega[:, 0], label='actual', color='#1f77b4')
    axes[1, 1].plot(t, omega_ref[:, 0], label='reference', linestyle='--', color='#d62728')
    axes[1, 1].set_title('Fig. 8. Trajectory of roll angular velocity tracking')
    axes[1, 1].set_xlabel('Time (seconds)')
    axes[1, 1].set_ylabel(r'$\omega$ (rad/sec)')
    axes[1, 1].grid(True)
    axes[1, 1].legend()

    # Fig 5: Pitch
    axes[2, 0].plot(t, euler[:, 1], label='actual', color='#1f77b4')
    axes[2, 0].plot(t, euler_ref[:, 1], label='reference', linestyle='--', color='#2ca02c')
    axes[2, 0].set_title('Fig. 5. Trajectory of pitch angle tracking')
    axes[2, 0].set_xlabel('Time (sec)')
    axes[2, 0].set_ylabel('Pitch angle (degree)')
    axes[2, 0].grid(True)
    axes[2, 0].legend()

    # Fig 9: Pitch rate
    axes[2, 1].plot(t, omega[:, 1], label='actual', color='#1f77b4')
    axes[2, 1].plot(t, omega_ref[:, 1], label='reference', linestyle='--', color='#1f77b4')
    axes[2, 1].set_title('Fig. 9. Trajectory of pitch angular velocity tracking')
    axes[2, 1].set_xlabel('Time (seconds)')
    axes[2, 1].set_ylabel(r'$\omega$ (rad/sec)')
    axes[2, 1].grid(True)
    axes[2, 1].legend()

    # Fig 6: Roll
    axes[3, 0].plot(t, euler[:, 2], label='actual', color='#1f77b4')
    axes[3, 0].plot(t, euler_ref[:, 2], label='reference', linestyle='--', color='#9467bd')
    axes[3, 0].set_title('Fig. 6. Trajectory of roll angle tracking')
    axes[3, 0].set_xlabel('Time (sec)')
    axes[3, 0].set_ylabel('Roll angle (degree)')
    axes[3, 0].grid(True)
    axes[3, 0].legend()

    # Fig 10: Yaw rate
    axes[3, 1].plot(t, omega[:, 2], label='actual', color='#1f77b4')
    axes[3, 1].plot(t, omega_ref[:, 2], label='reference', linestyle='--', color='#d62728')
    axes[3, 1].set_title('Fig. 10. Trajectory of yaw angular velocity tracking')
    axes[3, 1].set_xlabel('Time (seconds)')
    axes[3, 1].set_ylabel(r'$\omega$ (rad/sec)')
    axes[3, 1].grid(True)
    axes[3, 1].legend()

    # Fig 11: delta
    axes[4, 0].plot(t, delta[:, 0], label=r'$\delta_1$', color='#1f77b4')
    axes[4, 0].plot(t, delta[:, 1], label=r'$\delta_2$', color='#ff7f0e')
    axes[4, 0].plot(t, delta[:, 2], label=r'$\delta_3$', color='#2ca02c')
    axes[4, 0].set_title(r'Fig. 11. Trajectory of $\delta$')
    axes[4, 0].set_xlabel('Time (seconds)')
    axes[4, 0].set_ylabel(r'$\delta$ (rad/sec)')
    axes[4, 0].grid(True)
    axes[4, 0].legend()
    axes[4, 0].set_xlim([0, 10])

    # Fig 12: torque
    axes[4, 1].plot(t, tau[:, 0], label=r'$\tau_1$', color='#1f77b4')
    axes[4, 1].plot(t, tau[:, 1], label=r'$\tau_2$', color='#ff7f0e')
    axes[4, 1].plot(t, tau[:, 2], label=r'$\tau_3$', color='#2ca02c')
    axes[4, 1].set_title(r'Fig. 12. Trajectory of torque')
    axes[4, 1].set_xlabel('Time (sec)')
    axes[4, 1].set_ylabel(r'$\tau$ (N$\cdot$m)')
    axes[4, 1].grid(True)
    axes[4, 1].legend()
    axes[4, 1].set_xlim([0, 10])

    plt.tight_layout()
    plot_path = os.path.join(output_dir, "output_section_v.png")
    plt.savefig(plot_path, dpi=200)
    plt.close()
    print(f"Saved benchmark figure: {plot_path}")


if __name__ == '__main__':
    run_paper_attitude_benchmark(duration=10.0, dt=0.001, save_plots=True)
