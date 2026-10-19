@echo off
title UAV Gazebo 3D Simulation Launcher
cd /d "%~dp0"

echo ==================================================================
echo    UAV 3D GAZEBO SIMULATION LAUNCHER (WSL2 Ubuntu + Gazebo Sim)
echo    Paper: Quaternion-Based Attitude Tracking Control (CACS 2024)
echo ==================================================================
echo.

REM 1. Ensure Gazebo Harmonic and Python dependencies are installed in WSL Ubuntu (runs as root so no password needed)
echo [1/3] Checking Gazebo installation in WSL Ubuntu...
wsl -d Ubuntu -u root bash -c "if ! command -v gz >/dev/null 2>&1; then echo 'Installing Gazebo Harmonic (one-time setup, please wait 1-2 minutes)...'; apt-get update && apt-get install -y curl lsb-release gnupg python3-numpy python3-yaml python3-tk python3-matplotlib && curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable \$(lsb_release -cs) main\" > /etc/apt/sources.list.d/gazebo-stable.list && apt-get update && apt-get install -y gz-harmonic; else echo 'Gazebo Harmonic is already installed.'; fi"

REM 2. Ensure Python dependencies exist
wsl -d Ubuntu -u root bash -c "python3 -c 'import numpy, yaml, tkinter' 2>/dev/null || (apt-get update && apt-get install -y python3-numpy python3-yaml python3-tk python3-matplotlib)"

REM 3. Launch Live Gazebo 3D Simulation + Right-Side Coordinate Panel
echo.
echo [2/3] Regenerating Gazebo 3D World from config/uav_params.yaml...
python scripts/generate_gazebo_world.py

echo.
echo [3/3] Launching Gazebo 3D Simulator + Quaternion Controller + Live Coordinate Panel...
wsl -d Ubuntu bash -c "export DISPLAY=:0; cd '/mnt/c/Users/hp/Downloads/Sem 5/uav_simulation' && python3 scripts/run_live_in_gazebo.py"

pause
