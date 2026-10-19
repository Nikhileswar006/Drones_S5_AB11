#!/usr/bin/env bash
# ==============================================================================
# Automated Gazebo Harmonic + ROS 2 Setup & Live UAV Launcher for Ubuntu WSL2
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export DISPLAY="${DISPLAY:-:0}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}"

echo "=================================================================="
echo "   UAV 3D GAZEBO SIMULATION LAUNCHER (Ubuntu 24.04 WSL2)"
echo "   Paper: Quaternion-Based Attitude Tracking Control (CACS 2024)"
echo "=================================================================="

# 1. Check if Gazebo ('gz') is installed; if not, install Gazebo Harmonic
if ! command -v gz >/dev/null 2>&1; then
    echo "[Step 1/3] Gazebo is not installed yet in WSL Ubuntu."
    echo "Installing Gazebo Harmonic (gz-harmonic) & Python dependencies..."
    echo "(Please enter your Ubuntu sudo password if prompted)"
    sudo apt-get update
    sudo apt-get install -y curl lsb-release gnupg python3-numpy python3-yaml python3-matplotlib python3-tk

    sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

    sudo apt-get update
    sudo apt-get install -y gz-harmonic
    echo "Gazebo Harmonic installed successfully!"
else
    echo "[Step 1/3] Gazebo found: $(gz sim --versions | head -n 1)"
fi

# Ensure Python packages for controller & HUD panel are present
if ! python3 -c "import numpy, yaml, tkinter" >/dev/null 2>&1; then
    echo "Installing Python3 numpy, yaml, and tkinter..."
    sudo apt-get update
    sudo apt-get install -y python3-numpy python3-yaml python3-tk python3-matplotlib
fi

# 2. If ROS 2 is installed, offer or run ros2 launch; otherwise run direct live Gazebo bridge
if [ -f "/opt/ros/jazzy/setup.bash" ] || [ -f "/opt/ros/humble/setup.bash" ]; then
    ROS_SETUP="/opt/ros/jazzy/setup.bash"
    [ -f "/opt/ros/humble/setup.bash" ] && ROS_SETUP="/opt/ros/humble/setup.bash"
    echo "[Step 2/3] Found ROS 2 installation ($ROS_SETUP). Building uav_simulation..."
    source "$ROS_SETUP"
    mkdir -p ~/ros2_ws/src
    ln -sfn "$SCRIPT_DIR" ~/ros2_ws/src/uav_simulation
    (cd ~/ros2_ws && colcon build --packages-select uav_simulation --symlink-install)
    source ~/ros2_ws/install/setup.bash
    echo "[Step 3/3] Launching ROS 2 + Gazebo Simulation..."
    ros2 launch uav_simulation uav_simulation.launch.py
else
    echo "[Step 2/3] Regenerating Gazebo World SDF from config/uav_params.yaml..."
    python3 scripts/generate_gazebo_world.py
    echo "[Step 3/3] Starting Live Gazebo 3D Simulator + Quaternion Controller + Right-Side Panel..."
    python3 scripts/run_live_in_gazebo.py
fi
