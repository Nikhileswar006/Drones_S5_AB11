# ROS 2 & Gazebo UAV Simulation: Quaternion-Based Attitude Tracking Control with Backstepping & Fractional Exponent

Complete ROS 2 and Gazebo 3D Quadrotor UAV Simulation implementing the control algorithm from:

> **Qain-Rong Lin and Jen-te Yu**, *"Quaternion-Based Attitude Tracking Control Design for UAVs,"* 2024 International Automatic Control Conference (CACS 2024), National Taiwan University of Science and Technology, Taipei, Taiwan, Nov. 2024.

---

## 1. Package Directory Structure

```text
uav_simulation/
├── launch/
│   └── uav_simulation.launch.py          # Master ROS 2 + Gazebo launch file
├── config/
│   └── uav_params.yaml                   # Start point, waypoints, tolerance, inertia, gains
├── worlds/
│   └── uav_waypoint_world.sdf            # Gazebo 3D world with UAV, waypoints, & reference path
├── urdf/
│   └── quadrotor.urdf                    # Quadrotor URDF model (J = diag[0.1, 0.1, 0.12])
├── meshes/
│   └── README_meshes.txt                 # Mesh assets directory
├── models/
│   └── quadrotor/
│       ├── model.config                  # Gazebo model metadata
│       └── model.sdf                     # 3D Quadrotor SDF model matching Fig. 1 of paper
├── src/
│   ├── controller_node.py                # CACS 2024 Quaternion Backstepping controller entrypoint
│   ├── trajectory_node.py                # Quintic minimum-jerk 3D trajectory generator entrypoint
│   ├── waypoint_manager.py               # Sequential waypoint manager & switching entrypoint
│   └── data_logger.py                    # CSV logger & RMSE performance evaluator entrypoint
├── uav_simulation/
│   ├── __init__.py
│   ├── quaternion_math.py                # Exact Equations (1)-(23) from the CACS 2024 paper
│   ├── controller_node.py                # Outer-loop 3D tracker + Inner-loop Paper Controller
│   ├── trajectory_node.py                # Smooth 3D reference path generator
│   ├── waypoint_manager.py               # Euclidean waypoint switching logic
│   ├── gazebo_uav_node.py                # 6-DOF RK4 UAV dynamics & live Gazebo 3D bridge
│   ├── info_panel_node.py                # Docked right-side real-time coordinate HUD panel
│   └── data_logger.py                    # 20 Hz CSV logger & performance metrics calculator
├── scripts/
│   ├── generate_gazebo_world.py          # Auto-builds Gazebo SDF world from uav_params.yaml
│   ├── plot_results.py                   # Generates all 2D & 3D trajectory/error/attitude plots
│   └── run_standalone_verification.py    # Reproduces Paper Figs. 3-12 + 3D Waypoint Simulation
├── rviz/
│   └── uav_visualization.rviz            # Pre-configured RViz2 layout
├── setup.py
├── setup.cfg
└── package.xml
```

---

## 2. Installation & Build Commands (Ubuntu 22.04 / 24.04 + ROS 2 Humble / Jazzy + Gazebo)

### 2.1 Install Required ROS 2 & Gazebo Packages
```bash
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-desktop \
  ros-${ROS_DISTRO}-ros-gz \
  ros-${ROS_DISTRO}-ros-gz-sim \
  ros-${ROS_DISTRO}-ros-gz-bridge \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-tf2-ros \
  ros-${ROS_DISTRO}-rviz2 \
  python3-colcon-common-extensions \
  python3-numpy \
  python3-yaml \
  python3-matplotlib \
  python3-tk
```

### 2.2 Create Workspace & Build the Package
```bash
mkdir -p ~/ros2_ws/src
cp -r uav_simulation ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select uav_simulation --symlink-install
source ~/ros2_ws/install/setup.bash
```

---

## 3. Starting the Gazebo Simulation

### 3.1 Launch Full 3D Gazebo Simulation + Right-Side Coordinate Panel + CSV Logger
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch uav_simulation uav_simulation.launch.py
```

### 3.2 Launch Gazebo + RViz2 Simultaneously (Optional)
```bash
ros2 launch uav_simulation uav_simulation.launch.py use_rviz:=true
```

### 3.3 Generate All Plots After Simulation
```bash
python3 ~/ros2_ws/src/uav_simulation/scripts/plot_results.py \
  --csv uav_trajectory_log.csv \
  --out plots
```

### 3.4 Run Standalone Verification (Paper Benchmark Figs. 3–12 + 3D Waypoint Mission)
```bash
python3 scripts/run_standalone_verification.py
```

---

## 4. Changing the Starting Position & Waypoints

Open `config/uav_params.yaml` and edit `initial_position`, `waypoints`, or `waypoint_tolerance`:

```yaml
uav_simulation:
  ros__parameters:
    initial_position: [0.0, 0.0, 2.0]
    waypoints:
      - [5.0, 3.0, 4.0]
      - [10.0, -2.0, 5.0]
      - [15.0, 4.0, 6.0]
      - [20.0, 0.0, 3.0]   # <-- Add as many waypoints as needed without changing code!
    waypoint_tolerance: 0.20
```

When you run `ros2 launch uav_simulation uav_simulation.launch.py`, the launch file automatically regenerates the Gazebo SDF world (`uav_waypoint_world.sdf`) with the updated Start marker, Waypoint spheres, and 3D Reference Trajectory curve.
