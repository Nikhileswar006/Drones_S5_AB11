# Quaternion-Based Attitude & 3D Trajectory Tracking for Quadrotor UAVs in PyBullet



---

## 🚀 Key Features

1. **Exact CACS 2024 Paper Implementation**:
   - Quaternion algebra (Equations 1–7) with scalar-first convention $q = [q_0, q_1, q_2, q_3]^T$.
   - Quaternion kinematics $\dot{q} = \frac{1}{2} q \otimes [0, \omega]^T$ (Eq. 8).
   - Quaternion tracking error $q_{err} = q_{ref}^* \otimes q$ (Eq. 10).
   - Backstepping virtual control $\omega_{err,d} = -k_1 q_{err,v}$ (Eq. 13).
   - Finite-time convergence auxiliary error $\delta = \omega_{err} - \omega_{err,d}$ (Eq. 16).
   - Nonlinear fractional exponent torque control law $\tau = \tau_1 + \tau_2$ with $\tau_2 = -k_2 J \delta^{1/3}$ (Eqs. 21–23).
   - Parameters from Section V: $J = \text{diag}(0.1, 0.1, 0.12)$, $k_1 = 20.0$, $k_2 = 2.0$.

2. **3D Flight Extension (Start to Destination Navigation)**:
   - Cascaded SE(3) outer-loop position controller converting smooth 3D trajectory tracking into collective thrust $T$, attitude reference $q_{ref}$, and feedforward angular velocity $\omega_{ref}$.
   - 5th-order (quintic polynomial) minimum-jerk trajectory connecting any defined Start point $\mathbf{p}_{\text{start}}$ to Destination $\mathbf{p}_{\text{goal}}$ with smooth $C^2$ position, velocity, and acceleration profiles.

3. **Realistic PyBullet 3D Simulation**:
   - Custom quadrotor multi-body with realistic visual chassis and rotor discs (red front, blue rear).
   - Live 3D debug trajectory rendering:
     - **Green Path**: Reference trajectory in 3D space.
     - **Cyan/Blue Line**: Real-time flown flight trail.
     - **Green 3D Sphere**: Start marker.
     - **Red 3D Sphere**: Destination marker.
   - Real-time smooth camera tracking following the drone in 3D space.

4. **Telemetry & Figure Reproduction**:
   - Replicates **Figures 3 through 12** from the paper ($q_{err}$, Euler angles, $\omega_{err}$, $\omega$ vs $\omega_{ref}$, $\delta$, and control torques $\tau$).

---

## 📦 Requirements & Installation

Only standard Python packages are needed:

```bash
pip install pybullet numpy matplotlib
```

---

## 💻 How to Run

### 1. Run the Full 3D UAV Mission in PyBullet (Start to Destination)
Flies the UAV following a smooth 3D trajectory with real-time visual rendering and camera tracking.

**Default run** (from `[0, 0, 0.5]` to `[4, 4, 2.5]`):
```bash
python run_3d_mission.py
```

**Custom Destination Point via CLI**:
You can pass any destination or start coordinates directly:
```bash
# Fly to destination (x=6.0m, y=3.0m, z=4.0m):
python run_3d_mission.py --goal 6.0 3.0 4.0

# With custom start point, destination, and flight duration:
python run_3d_mission.py --start 1.0 1.0 0.5 --goal 5.0 5.0 3.0 --duration 12.0
```

- When the flight finishes, it automatically generates and saves:
  - `uav_3d_flight_trajectory.png`: 3D spatial flight path and position tracking errors $(e_x, e_y, e_z)$.
  - `uav_paper_metrics_reproduction.png`: All 10 telemetry plots corresponding to Figures 3–12 of the paper under full 3D flight!

### 2. Run the Standalone Paper Benchmark (Exact Section V Replication)
Replicates the pure attitude tracking scenario from Section V (Figures 3–12) with identical parameters, sine wave commands, and transient convergence:

```bash
python verify_paper_standalone.py
```

- Generates `paper_section_v_reproduced.png` comparing all paper curves.

---

## 📁 File Structure

| File | Description |
|---|---|
| `uav_quaternion_backstepping.py` | Quaternion math and backstepping attitude control law (Eqs. 1–23). |
| `uav_trajectory.py` | Quintic minimum-jerk smooth 3D trajectory generator. |
| `uav_position_controller.py` | Outer-loop position-to-attitude cascaded controller on SE(3). |
| `uav_pybullet_env.py` | PyBullet 3D simulation environment, drone multi-body, visual paths & markers. |
| `run_3d_mission.py` | Main script executing 3D flight from start to goal in PyBullet GUI. |
| `verify_paper_standalone.py` | Benchmark script reproducing the paper's exact numerical simulation (Section V). |
