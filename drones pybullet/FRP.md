# First Review Progress Report (FRP)

## Project Details
* **Project Title:** Quaternion-Based Attitude and 3D Trajectory Tracking Control for Quadrotor UAVs in PyBullet
* **Review Milestone:** First Review Update (Review 1)
* **Domain:** Robotics, Autonomous Aerial Systems, Nonlinear Control Theory, Physics Simulation
* **Platform & Environment:** Python 3.x, PyBullet Physics Engine, NumPy, Matplotlib, SciPy

---

## 1. Executive Summary & Objective

This project focuses on the modeling, replication, and physical simulation of advanced quaternion-based nonlinear control for quadrotor Unmanned Aerial Vehicles (UAVs). 

Following the initial guidance to shift from numerical computing in MATLAB to realistic rigid-body physics simulation, this project replicates and extends the research published in:
> **"Quaternion-Based Attitude Tracking Control Design for UAVs"**  
> *Authors:* Qain-Rong Lin and Jen-te Yu  
> *Conference:* 2024 International Automatic Control Conference (CACS 2024), IEEE  
> *DOI:* 10.1109/CACS63404.2024.10773309  

### Objectives for Review 1 (Completed):
1. **Mathematical Rigor:** Implement the full quaternion kinematics, error dynamics, and backstepping attitude control law featuring a fractional-power exponent ($\delta^{1/3}$) from the base paper.
2. **Attitude Control Verification:** Benchmark and validate the controller against Section V of the paper, successfully reproducing Figures 3 through 12.
3. **3D Flight Extension:** Extend the paper's attitude-only formulation to full 3D spatial navigation from an arbitrary start point to a destination along smooth reference trajectories (minimum-jerk quintic polynomials and 3D curved paths).
4. **PyBullet Physics Simulation:** Develop an interactive 3D simulation environment in PyBullet featuring real-time visual trajectory rendering, coordinate frame indicators, camera tracking, and coordinate export.

---

## 2. Theoretical Framework & Mathematical Modeling

The mathematical formulation directly maps the governing equations established by Lin & Yu (2024):

### 2.1 Quaternion Algebra & Kinematics
To eliminate singularities and gimbal lock inherent in Euler angle representations, unit quaternions are utilized with the scalar-first convention:
$$\mathbf{q} = [q_0, \mathbf{q}_v^T]^T = [q_0, q_1, q_2, q_3]^T \in \mathbb{R}^4, \quad \|\mathbf{q}\| = 1 \tag{1-2}$$

* **Conjugate:** $\mathbf{q}^* = [q_0, -q_1, -q_2, -q_3]^T$ (Eq. 3)
* **Quaternion Multiplication (Hamilton Product):**
$$\mathbf{p} \otimes \mathbf{q} = \begin{bmatrix}
p_0 q_0 - \mathbf{p}_v \cdot \mathbf{q}_v \\
p_0 \mathbf{q}_v + q_0 \mathbf{p}_v + \mathbf{p}_v \times \mathbf{q}_v
\end{bmatrix} \tag{7}$$
* **Attitude Kinematics:**
$$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix} \tag{8}$$
where $\boldsymbol{\omega} = [\omega_1, \omega_2, \omega_3]^T$ is the body angular velocity vector.
* **Rotational Dynamics:**
$$\dot{\boldsymbol{\omega}} = \mathbf{J}^{-1} \left[ \boldsymbol{\tau} - \boldsymbol{\omega} \times (\mathbf{J}\boldsymbol{\omega}) \right] \tag{9}$$
where $\mathbf{J} = \text{diag}([0.1, 0.1, 0.12])\,\text{kg}\cdot\text{m}^2$ is the quadrotor moment of inertia matrix, and $\boldsymbol{\tau} = [\tau_1, \tau_2, \tau_3]^T$ is the control torque.

---

### 2.2 Nonlinear Backstepping Attitude Controller with Fractional Exponent
The error states between the current and reference orientation are formulated as:
$$\mathbf{q}_{err} = \mathbf{q}_{ref}^* \otimes \mathbf{q} \tag{10}$$
$$\boldsymbol{\omega}_{err} = \boldsymbol{\omega} - \boldsymbol{\omega}_{ref} \tag{11}$$
$$\dot{\mathbf{q}}_{err} = \frac{1}{2} \mathbf{q}_{err} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega}_{err} \end{bmatrix} \tag{12}$$

1. **Virtual Control Law ($\boldsymbol{\omega}_{err,d}$):**
   $$\begin{bmatrix} 0 \\ \boldsymbol{\omega}_{err,d} \end{bmatrix} = -k_1 \mathbf{q}_{err}^* \otimes \left( \mathbf{q}_{err} - \begin{bmatrix} 1 \\ 0 \\ 0 \\ 0 \end{bmatrix} \right) \tag{13}$$
   yielding the vector form $\boldsymbol{\omega}_{err,d} = -k_1 \mathbf{q}_{err,v}$ with $k_1 = 20.0$.
2. **Auxiliary Error Variable ($\boldsymbol{\delta}$):**
   $$\boldsymbol{\delta} = \boldsymbol{\omega}_{err} - \boldsymbol{\omega}_{err,d} \tag{16}$$
3. **Control Torque Decomposition ($\boldsymbol{\tau} = \boldsymbol{\tau}_1 + \boldsymbol{\tau}_2$):**
   $$\boldsymbol{\tau}_1 = \boldsymbol{\omega} \times (\mathbf{J}\boldsymbol{\omega}) + \mathbf{J}\dot{\boldsymbol{\omega}}_{ref} + \mathbf{J}\dot{\boldsymbol{\omega}}_{err,d} \tag{21}$$
   $$\boldsymbol{\tau}_2 = -k_2 \mathbf{J} \boldsymbol{\delta}^{1/3} \tag{22}$$
   where $\boldsymbol{\delta}^{1/3} = \text{sign}(\boldsymbol{\delta})|\boldsymbol{\delta}|^{1/3}$ is evaluated element-wise with gain $k_2 = 2.0$.
4. **Finite-Time Convergence:**
   Substituting $\boldsymbol{\tau}$ into the error dynamics yields:
   $$\dot{\boldsymbol{\delta}} = -k_2 \boldsymbol{\delta}^{1/3} \tag{23}$$
   ensuring finite-time convergence of the tracking error to the origin.

---

### 2.3 Outer-Loop Cascaded Position Controller on SE(3)
To enable full 3D waypoint navigation, an outer-loop controller generates the required collective thrust $T$ and reference attitude $\mathbf{q}_{ref}$:

$$\mathbf{a}_{des} = \ddot{\mathbf{p}}_d + K_p (\mathbf{p}_d - \mathbf{p}) + K_d (\dot{\mathbf{p}}_d - \dot{\mathbf{p}}) + g\hat{\mathbf{z}}_W$$
* **Total Collective Thrust:** $T = m (\mathbf{a}_{des} \cdot \mathbf{z}_B)$
* **Target Body Axes:**
  $$\mathbf{z}_{B,des} = \frac{\mathbf{a}_{des}}{\|\mathbf{a}_{des}\|}, \quad \mathbf{y}_{B,des} = \frac{\mathbf{z}_{B,des} \times \mathbf{x}_{\psi}}{\|\mathbf{z}_{B,des} \times \mathbf{x}_{\psi}\|}, \quad \mathbf{x}_{B,des} = \mathbf{y}_{B,des} \times \mathbf{z}_{B,des}$$
* **Attitude Reference ($\mathbf{q}_{ref}$):** Extracted from $\mathbf{R}_{des} = [\mathbf{x}_{B,des}, \mathbf{y}_{B,des}, \mathbf{z}_{B,des}]$.

---

## 3. System Architecture & Project Organization

```
drones pybullet/
├── uav_quaternion_backstepping.py  # Eqs. (1)–(23): Quaternion algebra & backstepping controller
├── uav_trajectory.py               # Minimum-jerk (quintic) and 3D curved trajectory generators
├── uav_position_controller.py      # Outer-loop SE(3) position controller (Thrust & q_ref)
├── uav_pybullet_env.py             # PyBullet 3D physics environment, multi-body drone & visuals
├── run_3d_mission.py               # Main 3D simulation script with interactive GUI & telemetry
├── verify_paper_standalone.py      # Numerical benchmark replicating Section V of the paper
├── run_mission.bat                 # Single-click Windows launcher
├── flight_coordinates.csv          # High-frequency logged coordinates (240 Hz)
├── requirements.txt                # Dependencies (pybullet, numpy, matplotlib, scipy)
├── FRP.md                          # First Review Progress Report (This document)
└── README.md                       # Execution guide & CLI documentation
```

---

## 4. Summary of Results for Review 1

### 4.1 Base Paper Reproduction (Section V Verification)
* Simulated using `verify_paper_standalone.py` with the exact parameters:
  $J = \text{diag}([0.1, 0.1, 0.12])\,\text{kg}\cdot\text{m}^2$, $k_1 = 20$, $k_2 = 2$.
* **Convergence verified:**
  * $\boldsymbol{\delta}$ converges to zero in $\approx 0.4\,\text{s}$ (matching Fig. 11).
  * $\boldsymbol{\omega}_{err}$ converges to zero in $\approx 0.8\,\text{s}$ (matching Fig. 7).
  * $\mathbf{q}_{err}$ converges to $[1, 0, 0, 0]^T$ (matching Fig. 3).
  * Attitude angles (yaw, pitch, roll) smoothly track target trajectories (Figs. 4, 5, 6).
* Saved verification plot: `paper_section_v_reproduced.png`.

### 4.2 3D Flight Simulation in PyBullet
* Developed in `run_3d_mission.py`.
* **Visual Trajectory Feedback:**
  * 🟢 **Green line:** Smooth minimum-jerk reference trajectory.
  * 🔵 **Cyan line:** Real-time physical flight path recorded from PyBullet.
  * 🟢 **Green sphere:** User-defined Start coordinate.
  * 🔴 **Red sphere:** User-defined Destination coordinate.
* **Flight Trajectory Modes Implemented:**
  1. **Straight Smooth Navigation:** 5th-order minimum jerk trajectory between any start and goal points.
  2. **3D Curved Flight:** Spatial trajectory with both vertical altitude arching ($\Delta z$) and lateral swinging ($\Delta x, \Delta y$).
* **Data Export:** All flight coordinates, errors, and velocities are logged and exported to `flight_coordinates.csv`.
* Saved mission plots: `uav_3d_flight_trajectory.png` and `uav_paper_metrics_reproduction.png`.

---

## 5. Work Completed vs. Remaining Milestones

| Task / Milestone | Status | Target Phase |
|---|:---:|:---:|
| Mathematical formulation of Quaternion Backstepping with $\delta^{1/3}$ | **Completed** | Review 1 |
| Reproduction of Base Paper Section V (Figs. 3–12) | **Completed** | Review 1 |
| Outer-loop position controller design on SE(3) | **Completed** | Review 1 |
| 3D Physics Simulation environment in PyBullet | **Completed** | Review 1 |
| Real-time visual path rendering & dynamic camera tracking | **Completed** | Review 1 |
| Parameterized start/destination and 3D curved trajectory generation | **Completed** | Review 1 |
| Telemetry logging and CSV coordinate export | **Completed** | Review 1 |
| Robustness evaluation under external wind disturbances & sensor noise | *Planned* | Review 2 |
| Dynamic obstacle avoidance along the 3D trajectory | *Planned* | Review 2 |
| Comparative analysis (Quaternion Backstepping vs. Standard PID / SMC) | *Planned* | Review 3 / Final |

---

## 6. References
1. Q.-R. Lin and J.-t. Yu, "Quaternion-Based Attitude Tracking Control Design for UAVs," *2024 International Automatic Control Conference (CACS)*, Taiwan, 2024, pp. 1–5.
2. E. Fresk and G. Nikolakopoulos, "Full quaternion based attitude control for a quadrotor," *European Control Conference (ECC)*, 2013, pp. 3864–3869.
3. M. S. Esmail et al., "Attitude and Altitude Tracking Controller for Quadcopter Dynamical Systems," *IEEE Access*, vol. 10, pp. 53344–53358, 2022.
4. J.-t. Yu, "A Unified SO(3) Approach to the Attitude Control Design for Quadrotors," *IEEE Access*, vol. 9, pp. 64871–64879, 2021.
5. E. Coumans and Y. Bai, "PyBullet, a Python module for physics simulation for games, robotics and machine learning," *GitHub Repository*, 2016–2024.
