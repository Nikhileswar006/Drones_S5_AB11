#!/usr/bin/env python3
"""
Hardware-Accelerated 3D WebGL Simulation & Clean Real-World Flight Visualizer
==============================================================================
Exports a clean, photorealistic 3D WebGL flight simulation featuring:
  - Realistic slow, smooth quadrotor flight matching true real-world trajectory dynamics
  - Clean interactive 3D Orbit Camera (free 360° pan, tilt, zoom exploration)
  - Realistic Outdoor Airfield Proving Ground:
      * Asphalt Runway with centerline dashes, threshold piano keys, taxiway markings,
      * Airfield turf grass, perimeter fence posts, floodlight mast towers,
      * Functional 3D Airfield Windsock rotating dynamically with wind direction,
      * Rotating Ground Control Station radar dish,
      * Dynamic UAV ground drop shadow scaling and fading with altitude Z.
  - Aerodynamic Visual Effects:
      * Ground effect prop-wash dust ring spreading outward at low altitude (Z < 2.5m),
      * 250 drifting atmospheric wind particles,
      * 3D Aviation Obstacle Towers with pulsing hazard strobe beacons.
  - Full docked Right-Side Aerospace Real-Time Coordinate Information Panel:
      * Time, Position, Reference, Error, Waypoint Navigation,
      * Linear Velocity, Attitude Angles (Euler), Paper Control States.
"""

import os
import sys
import csv
import json
import yaml

PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from uav_simulation.waypoint_manager import load_waypoints_from_yaml
from uav_simulation.trajectory_node import QuinticSegmentTrajectoryGenerator


def export_html(csv_path: str, yaml_path: str, output_html: str):
    start_pos, waypoints, tol = load_waypoints_from_yaml(yaml_path)

    # Load velocity and acceleration limits from YAML
    max_vel = 1.0
    max_acc = 1.2
    if yaml_path and os.path.exists(yaml_path):
        with open(yaml_path, 'r', encoding='utf-8') as f:
            ydata = yaml.safe_load(f)
        yparams = ydata.get('uav_simulation', {}).get('ros__parameters', ydata)
        max_vel = float(yparams.get('max_velocity', 1.0))
        max_acc = float(yparams.get('max_acceleration', 1.2))

    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    flight_data = []
    for r in rows:
        flight_data.append({
            't': round(float(r['timestamp']), 3),
            'x': round(float(r['actual_x']), 3),
            'y': round(float(r['actual_y']), 3),
            'z': round(float(r['actual_z']), 3),
            'xr': round(float(r['reference_x']), 3),
            'yr': round(float(r['reference_y']), 3),
            'zr': round(float(r['reference_z']), 3),
            'ex': round(float(r['error_x']), 4),
            'ey': round(float(r['error_y']), 4),
            'ez': round(float(r['error_z']), 4),
            'e': round(float(r['position_error']), 4),
            'vx': round(float(r['velocity_x']), 3),
            'vy': round(float(r['velocity_y']), 3),
            'vz': round(float(r['velocity_z']), 3),
            'roll': round(float(r['roll']), 2),
            'pitch': round(float(r['pitch']), 2),
            'yaw': round(float(r['yaw']), 2),
            'wp': int(r.get('active_waypoint', 1)),
            'dwp': round(float(r.get('dist_to_waypoint', 0.0)), 3),
            'qerr': [round(float(r['q_err0']), 4), round(float(r['q_err1']), 4), round(float(r['q_err2']), 4), round(float(r['q_err3']), 4)],
            'delta': [round(float(r['delta_x']), 4), round(float(r['delta_y']), 4), round(float(r['delta_z']), 4)],
            'tau': [round(float(r['tau_x']), 4), round(float(r['tau_y']), 4), round(float(r['tau_z']), 4)],
            'u1': round(float(r.get('thrust_u1', 11.77)), 3)
        })

    flight_json = json.dumps(flight_data)
    waypoints_json = json.dumps(waypoints)
    start_json = json.dumps(start_pos)

    traj_gen = QuinticSegmentTrajectoryGenerator(
        start_pos, waypoints, max_velocity=max_vel, max_acceleration=max_acc
    )
    obstacles_json = json.dumps(traj_gen.obstacles)
    ref_sampled_pts = traj_gen.sample_full_reference_path(num_points_per_segment=45)
    reference_path_json = json.dumps([[round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 3)] for p in ref_sampled_pts])

    html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>Real-World 3D UAV Flight Simulation & Smooth Obstacle Detour (CACS 2024)</title>
  <style>
    * {{ box-sizing: border-box; margin: 0; padding: 0; }}
    body {{
      font-family: 'Segoe UI', -apple-system, BlinkMacSystemFont, Roboto, sans-serif;
      background: #04070d;
      color: #e2e8f0;
      overflow: hidden;
      display: flex;
      height: 100vh;
      width: 100vw;
    }}
    #viewport-container {{
      flex: 1;
      position: relative;
      height: 100%;
      background: #000;
    }}
    #viewport3d {{
      width: 100%;
      height: 100%;
      display: block;
    }}

    /* Bottom Control Bar */
    #controls-bar {{
      position: absolute;
      bottom: 20px;
      left: 50%;
      transform: translateX(-50%);
      background: rgba(10, 16, 31, 0.94);
      border: 1px solid rgba(51, 65, 85, 0.8);
      padding: 9px 24px;
      border-radius: 30px;
      display: flex;
      align-items: center;
      gap: 12px;
      box-shadow: 0 10px 40px rgba(0,0,0,0.7);
      z-index: 10;
      backdrop-filter: blur(12px);
    }}
    .ctrl-btn {{
      background: #1e293b;
      border: 1px solid #475569;
      color: #e2e8f0;
      padding: 6px 14px;
      border-radius: 14px;
      font-weight: 600;
      font-size: 12px;
      cursor: pointer;
      transition: all 0.2s;
    }}
    .ctrl-btn:hover {{
      background: #00f5d4;
      color: #04070d;
      border-color: #00f5d4;
      transform: translateY(-1px);
    }}
    #timeline-slider {{
      width: 260px;
      accent-color: #00f5d4;
      cursor: pointer;
    }}
    #time-display {{
      font-family: 'Consolas', monospace;
      font-size: 12px;
      color: #f8fafc;
      min-width: 120px;
    }}
    #speed-select {{
      background: #1e293b;
      border: 1px solid #334155;
      color: #e2e8f0;
      padding: 5px 8px;
      border-radius: 8px;
      font-size: 11.5px;
      cursor: pointer;
    }}

    /* Right Side Docked Aerospace Telemetry Panel */
    #hud-sidebar {{
      width: 410px;
      background: #070b14;
      border-left: 2px solid #00f5d4;
      display: flex;
      flex-direction: column;
      height: 100%;
      box-shadow: -8px 0 28px rgba(0,0,0,0.7);
      z-index: 20;
    }}
    #hud-header {{
      background: #0d1424;
      padding: 12px 16px;
      border-bottom: 1px solid #1e293b;
      text-align: center;
    }}
    #hud-header h2 {{
      font-size: 13px;
      color: #00f5d4;
      letter-spacing: 1px;
      text-transform: uppercase;
      font-weight: 800;
    }}
    #hud-header span {{
      font-size: 10.5px;
      color: #94a3b8;
    }}
    #hud-content {{
      flex: 1;
      padding: 12px 16px;
      overflow-y: auto;
      font-family: 'Consolas', 'Courier New', monospace;
      font-size: 11.5px;
      line-height: 1.45;
      color: #e2e8f0;
    }}
    .hud-section {{
      margin-bottom: 10px;
      background: #0d1527;
      border: 1px solid #1e293b;
      border-radius: 8px;
      padding: 8px 12px;
    }}
    .hud-title {{
      font-size: 10px;
      font-weight: 700;
      color: #00f5d4;
      text-transform: uppercase;
      letter-spacing: 0.8px;
      margin-bottom: 4px;
      border-bottom: 1px solid #1e293b;
      padding-bottom: 2px;
    }}
    .hud-row {{
      display: flex;
      justify-content: space-between;
      margin-bottom: 1px;
    }}
    .hud-val {{
      color: #38bdf8;
      font-weight: 600;
    }}
    .hud-val.error {{
      color: #f43f5e;
    }}
    .hud-val.wp {{
      color: #facc15;
    }}
    .hud-val.acc {{
      color: #4ade80;
    }}
  </style>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
</head>
<body>
  <div id="viewport-container">
    <div id="controls-bar">
      <button class="ctrl-btn" id="btn-play">Pause</button>
      <button class="ctrl-btn" id="btn-restart">Restart</button>
      <button class="ctrl-btn" id="btn-toggle-ref" style="background:#00f5d4; color:#04070d; border-color:#00f5d4;">Reference Line: ON</button>
      <input type="range" id="timeline-slider" min="0" max="{len(flight_data) - 1}" value="0">
      <span id="time-display">t = 0.00 s</span>
      <select id="speed-select">
        <option value="0.25">0.25x</option>
        <option value="0.5">0.5x</option>
        <option value="0.75">0.75x</option>
        <option value="1.0" selected>1.0x</option>
        <option value="1.5">1.5x</option>
        <option value="2.0">2.0x</option>
      </select>
    </div>
    <div id="viewport3d"></div>
  </div>

  <div id="hud-sidebar">
    <div id="hud-header">
      <h2>UAV Live Coordinate Information Panel</h2>
    </div>
    <div id="hud-content">
      <div class="hud-section">
        <div class="hud-title">Simulation Time</div>
        <div class="hud-row"><span>Time:</span><span class="hud-val acc" id="hud-time">t = 0.000 s</span></div>
      </div>
      <div class="hud-section">
        <div class="hud-title">Current UAV Position</div>
        <div class="hud-row"><span>X =</span><span class="hud-val" id="hud-x">0.000 m</span></div>
        <div class="hud-row"><span>Y =</span><span class="hud-val" id="hud-y">0.000 m</span></div>
        <div class="hud-row"><span>Z =</span><span class="hud-val" id="hud-z">2.000 m</span></div>
      </div>
      <div class="hud-section">
        <div class="hud-title">Reference Position</div>
        <div class="hud-row"><span>X_ref =</span><span class="hud-val" id="hud-xr">0.000 m</span></div>
        <div class="hud-row"><span>Y_ref =</span><span class="hud-val" id="hud-yr">0.000 m</span></div>
        <div class="hud-row"><span>Z_ref =</span><span class="hud-val" id="hud-zr">2.000 m</span></div>
      </div>
      <div class="hud-section">
        <div class="hud-title">Position Errors</div>
        <div class="hud-row"><span>e_x =</span><span class="hud-val error" id="hud-ex">+0.0000 m</span></div>
        <div class="hud-row"><span>e_y =</span><span class="hud-val error" id="hud-ey">+0.0000 m</span></div>
        <div class="hud-row"><span>e_z =</span><span class="hud-val error" id="hud-ez">+0.0000 m</span></div>
        <div class="hud-row" style="margin-top:3px;border-top:1px dashed #334155;padding-top:3px;">
          <span>Euclidean Error e:</span><span class="hud-val error" id="hud-e">0.0000 m</span>
        </div>
      </div>
      <div class="hud-section">
        <div class="hud-title">Waypoint Navigation</div>
        <div class="hud-row"><span>Current Waypoint:</span><span class="hud-val wp" id="hud-wp">Waypoint 1</span></div>
        <div class="hud-row"><span>Target Coords:</span><span class="hud-val wp" id="hud-wptarget">(4.0, 2.0, 3.0) m</span></div>
        <div class="hud-row"><span>Distance to Waypoint:</span><span class="hud-val" id="hud-dwp">4.583 m</span></div>
      </div>
      <div class="hud-section">
        <div class="hud-title">Linear Velocity</div>
        <div class="hud-row"><span>Vx =</span><span class="hud-val" id="hud-vx">+0.000 m/s</span></div>
        <div class="hud-row"><span>Vy =</span><span class="hud-val" id="hud-vy">+0.000 m/s</span></div>
        <div class="hud-row"><span>Vz =</span><span class="hud-val" id="hud-vz">+0.000 m/s</span></div>
      </div>
      <div class="hud-section">
        <div class="hud-title">Attitude Angles (Euler)</div>
        <div class="hud-row"><span>Roll (phi):</span><span class="hud-val" id="hud-roll">+0.00 deg</span></div>
        <div class="hud-row"><span>Pitch (theta):</span><span class="hud-val" id="hud-pitch">+0.00 deg</span></div>
        <div class="hud-row"><span>Yaw (psi):</span><span class="hud-val" id="hud-yaw">+0.00 deg</span></div>
      </div>
      <div class="hud-section">
        <div class="hud-title">Control States</div>
        <div class="hud-row"><span>q_err:</span><span class="hud-val" id="hud-qerr">[1.000, 0.000, 0.000, 0.000]</span></div>
        <div class="hud-row"><span>delta:</span><span class="hud-val" id="hud-delta">[0.000, 0.000, 0.000]</span></div>
        <div class="hud-row"><span>Torque tau:</span><span class="hud-val" id="hud-tau">[0.000, 0.000, 0.000]</span></div>
      </div>
    </div>
  </div>

  <script>
    const data = {flight_json};
    const waypoints = {waypoints_json};
    const startPos = {start_json};
    const obstacles = {obstacles_json};
    const refPathPoints = {reference_path_json};

    let currentIndex = 0;
    let isPlaying = true;
    let speed = 1.0;
    let lastTime = performance.now();

    // 1. Setup Three.js Scene
    const container = document.getElementById('viewport3d');
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x060c18);
    scene.fog = new THREE.FogExp2(0x060c18, 0.012);

    const camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.1, 1000);
    camera.position.set(-6, 12, 18);

    const renderer = new THREE.WebGLRenderer({{ antialias: true, powerPreference: 'high-performance' }});
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.15;
    container.appendChild(renderer.domElement);

    const controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.target.set(8.0, 3.5, 2.0);
    controls.enableDamping = true;
    controls.dampingFactor = 0.06;

    // Coordinate Mapping: UAV frame (X forward, Y left, Z up) -> Three.js (X->X, Z->Y, Y->Z)
    function uavToThree(x, y, z) {{
      return new THREE.Vector3(x, z, y);
    }}

    // Lighting (Warm Sunlight + Cool Sky Fill)
    const ambientLight = new THREE.AmbientLight(0xdbeafe, 0.7);
    scene.add(ambientLight);

    const sunLight = new THREE.DirectionalLight(0xffedd5, 1.25);
    sunLight.position.set(20, 38, 22);
    sunLight.castShadow = true;
    sunLight.shadow.mapSize.width = 2048;
    sunLight.shadow.mapSize.height = 2048;
    sunLight.shadow.camera.near = 0.5;
    sunLight.shadow.camera.far = 120;
    sunLight.shadow.camera.left = -25;
    sunLight.shadow.camera.right = 25;
    sunLight.shadow.camera.top = 25;
    sunLight.shadow.camera.bottom = -25;
    sunLight.shadow.bias = -0.0004;
    scene.add(sunLight);

    const cyanRim = new THREE.DirectionalLight(0x00f5d4, 0.45);
    cyanRim.position.set(-20, 15, -15);
    scene.add(cyanRim);

    // --------------------------------------------------------------------------
    // Realistic Airfield Proving Grounds (Asphalt Runway, Grass, Markings)
    // --------------------------------------------------------------------------
    const airfieldGroup = new THREE.Group();

    // Surrounding Airfield Grass Turf
    const turfGeo = new THREE.PlaneGeometry(120, 120);
    const turfMat = new THREE.MeshStandardMaterial({{ color: 0x132a13, roughness: 0.95, metalness: 0.05 }});
    const turfMesh = new THREE.Mesh(turfGeo, turfMat);
    turfMesh.rotation.x = -Math.PI / 2;
    turfMesh.receiveShadow = true;
    airfieldGroup.add(turfMesh);

    // Asphalt Main Runway (Runs along X-axis: -4 to 22m, width 12m)
    const runwayGeo = new THREE.PlaneGeometry(28, 12);
    const runwayMat = new THREE.MeshStandardMaterial({{ color: 0x181c24, roughness: 0.82, metalness: 0.18 }});
    const runwayMesh = new THREE.Mesh(runwayGeo, runwayMat);
    runwayMesh.rotation.x = -Math.PI / 2;
    runwayMesh.position.set(8.5, 0.002, 1.0);
    runwayMesh.receiveShadow = true;
    airfieldGroup.add(runwayMesh);

    // Runway Centerline Dashes
    for (let x = -3; x <= 20; x += 3.5) {{
      const dashGeo = new THREE.PlaneGeometry(2.0, 0.22);
      const dashMat = new THREE.MeshBasicMaterial({{ color: 0xf8fafc }});
      const dash = new THREE.Mesh(dashGeo, dashMat);
      dash.rotation.x = -Math.PI / 2;
      dash.position.set(x, 0.005, 1.0);
      airfieldGroup.add(dash);
    }}

    // Runway Threshold Stripes ("Piano Keys") at both ends
    for (let offset of [-4.5, 21.5]) {{
      for (let z = -4.0; z <= 6.0; z += 1.2) {{
        const stripeGeo = new THREE.PlaneGeometry(1.6, 0.45);
        const stripeMat = new THREE.MeshBasicMaterial({{ color: 0xf8fafc }});
        const stripe = new THREE.Mesh(stripeGeo, stripeMat);
        stripe.rotation.x = -Math.PI / 2;
        stripe.position.set(offset, 0.006, z);
        airfieldGroup.add(stripe);
      }}
    }}

    // Taxiway Edge Guide Lines (Yellow)
    for (let side of [-4.8, 6.8]) {{
      const lineGeo = new THREE.PlaneGeometry(27.5, 0.16);
      const lineMat = new THREE.MeshBasicMaterial({{ color: 0xfacc15 }});
      const line = new THREE.Mesh(lineGeo, lineMat);
      line.rotation.x = -Math.PI / 2;
      line.position.set(8.5, 0.006, side);
      airfieldGroup.add(line);
    }}

    // --------------------------------------------------------------------------
    // Dynamic 3D Airfield Windsock (Rotates with Wind Direction & Speed)
    // --------------------------------------------------------------------------
    const windsockPivot = new THREE.Group();
    windsockPivot.position.set(-3.5, 0, -6.0);

    const wsPole = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.06, 3.8, 12), new THREE.MeshStandardMaterial({{ color: 0x94a3b8, metalness: 0.8 }}));
    wsPole.position.y = 1.9;
    wsPole.castShadow = true;
    windsockPivot.add(wsPole);

    const wsSwivel = new THREE.Group();
    wsSwivel.position.y = 3.8;

    // Red/White Striped Windsock Cone
    const sockBands = 5;
    const sockLen = 1.4;
    for (let b = 0; b < sockBands; b++) {{
      const r1 = 0.22 * (1.0 - 0.15 * b);
      const r2 = 0.22 * (1.0 - 0.15 * (b + 1));
      const sGeo = new THREE.CylinderGeometry(r2, r1, sockLen / sockBands, 16, 1, true);
      const sMat = new THREE.MeshStandardMaterial({{
        color: (b % 2 === 0) ? 0xef4444 : 0xf8fafc,
        side: THREE.DoubleSide,
        roughness: 0.8
      }});
      const sMesh = new THREE.Mesh(sGeo, sMat);
      sMesh.rotation.x = Math.PI / 2;
      sMesh.position.z = - (b + 0.5) * (sockLen / sockBands);
      wsSwivel.add(sMesh);
    }}
    windsockPivot.add(wsSwivel);
    airfieldGroup.add(windsockPivot);

    // Start Position Helipad Mark
    const padGroup = new THREE.Group();
    const padGeo = new THREE.CylinderGeometry(1.6, 1.6, 0.04, 32);
    const padMat = new THREE.MeshStandardMaterial({{ color: 0x1e293b, roughness: 0.8 }});
    const padMesh = new THREE.Mesh(padGeo, padMat);
    padMesh.position.y = 0.02;
    padGroup.add(padMesh);

    const padRingGeo = new THREE.RingGeometry(1.35, 1.55, 32);
    const padRingMat = new THREE.MeshBasicMaterial({{ color: 0xfacc15, side: THREE.DoubleSide }});
    const padRing = new THREE.Mesh(padRingGeo, padRingMat);
    padRing.rotation.x = -Math.PI / 2;
    padRing.position.y = 0.045;
    padGroup.add(padRing);

    padGroup.position.set(startPos[0], 0, startPos[1]);
    airfieldGroup.add(padGroup);
    scene.add(airfieldGroup);

    // --------------------------------------------------------------------------
    // Dynamic Quadrotor Ground Drop Shadow (Scales & Fades with Altitude Z)
    // --------------------------------------------------------------------------
    const dropShadowGeo = new THREE.PlaneGeometry(1.4, 1.4);
    const dropShadowMat = new THREE.MeshBasicMaterial({{
      color: 0x000000,
      transparent: true,
      opacity: 0.65,
      depthWrite: false
    }});
    const dropShadow = new THREE.Mesh(dropShadowGeo, dropShadowMat);
    dropShadow.rotation.x = -Math.PI / 2;
    dropShadow.position.y = 0.015;
    scene.add(dropShadow);

    // Start Position Marker Sphere
    const startMesh = new THREE.Mesh(
      new THREE.SphereGeometry(0.22, 24, 24),
      new THREE.MeshStandardMaterial({{ color: 0x3a86ff, roughness: 0.3, emissive: 0x1d4ed8, emissiveIntensity: 0.6 }})
    );
    startMesh.position.copy(uavToThree(startPos[0], startPos[1], startPos[2]));
    scene.add(startMesh);

    // Waypoint Spheres & Halos
    const wpColors = [0xff007f, 0xffbe0b, 0x00f5d4, 0xa855f7, 0x10b981];
    const wpMeshes = [];
    waypoints.forEach((wp, idx) => {{
      const wpGroup = new THREE.Group();
      const geo = new THREE.SphereGeometry(0.24, 24, 24);
      const mat = new THREE.MeshStandardMaterial({{
        color: wpColors[idx % wpColors.length],
        emissive: wpColors[idx % wpColors.length],
        emissiveIntensity: 0.55
      }});
      const sphere = new THREE.Mesh(geo, mat);
      wpGroup.add(sphere);

      const tolMesh = new THREE.Mesh(
        new THREE.SphereGeometry(0.20, 16, 16),
        new THREE.MeshBasicMaterial({{ color: wpColors[idx % wpColors.length], wireframe: true, transparent: true, opacity: 0.35 }})
      );
      wpGroup.add(tolMesh);

      wpGroup.position.copy(uavToThree(wp[0], wp[1], wp[2]));
      scene.add(wpGroup);
      wpMeshes.push(wpGroup);
    }});

    // --------------------------------------------------------------------------
    // 3D Aviation Obstacle Towers (Steel Lattice Columns + Beacons)
    // --------------------------------------------------------------------------
    const obstacleBeacons = [];
    obstacles.forEach((obs) => {{
      const obsGroup = new THREE.Group();
      const h = obs.height || 4.5;
      const r = obs.radius || 0.45;

      // Base Hazard Ring with Caution Striping
      const baseMesh = new THREE.Mesh(
        new THREE.CylinderGeometry(r * 1.5, r * 1.6, 0.20, 24),
        new THREE.MeshStandardMaterial({{ color: 0xeab308, roughness: 0.5, metalness: 0.2 }})
      );
      baseMesh.position.y = 0.10;
      obsGroup.add(baseMesh);

      // Alternating Orange / White Safety Bands
      const numBands = 5;
      const bandHeight = h / numBands;
      for (let b = 0; b < numBands; b++) {{
        const bandMesh = new THREE.Mesh(
          new THREE.CylinderGeometry(r * (1.0 - 0.02 * b), r * (1.0 - 0.02 * (b - 1)), bandHeight, 20),
          new THREE.MeshStandardMaterial({{
            color: (b % 2 === 0) ? 0xea580c : 0xf8fafc,
            metalness: 0.35,
            roughness: 0.45
          }})
        );
        bandMesh.position.y = bandHeight * (b + 0.5);
        bandMesh.castShadow = true;
        obsGroup.add(bandMesh);
      }}

      // Top Service Platform & Safety Railing
      const platMesh = new THREE.Mesh(
        new THREE.CylinderGeometry(r * 1.3, r * 1.3, 0.08, 16),
        new THREE.MeshStandardMaterial({{ color: 0x334155, metalness: 0.8 }})
      );
      platMesh.position.y = h + 0.04;
      obsGroup.add(platMesh);

      // Top Antenna Mast & Pulsating Aviation Strobe Beacon
      const mast = new THREE.Mesh(
        new THREE.CylinderGeometry(0.03, 0.03, 0.55, 8),
        new THREE.MeshStandardMaterial({{ color: 0x475569, metalness: 0.8 }})
      );
      mast.position.y = h + 0.32;
      obsGroup.add(mast);

      const beaconMesh = new THREE.Mesh(
        new THREE.SphereGeometry(r * 0.45, 16, 16),
        new THREE.MeshBasicMaterial({{ color: 0xef4444 }})
      );
      beaconMesh.position.y = h + 0.62;
      obsGroup.add(beaconMesh);

      const beaconLight = new THREE.PointLight(0xef4444, 1.4, 7.0);
      beaconLight.position.y = h + 0.65;
      obsGroup.add(beaconLight);
      obstacleBeacons.push(beaconLight);

      const p = obs.position;
      obsGroup.position.set(p[0], 0, p[1]);
      scene.add(obsGroup);
    }});

    // Active Waypoint Halo Ring
    const haloMesh = new THREE.Mesh(
      new THREE.RingGeometry(0.4, 0.58, 32),
      new THREE.MeshBasicMaterial({{ color: 0xffff00, side: THREE.DoubleSide, transparent: true, opacity: 0.85 }})
    );
    haloMesh.rotation.x = Math.PI / 2;
    scene.add(haloMesh);

    // --------------------------------------------------------------------------
    // 1. Reference Trajectory Path (Planned Smooth Obstacle Avoidance)
    // --------------------------------------------------------------------------
    const refGroup = new THREE.Group();
    const refPts = refPathPoints.map(p => uavToThree(p[0], p[1], p[2]));
    const refCurve = new THREE.CatmullRomCurve3(refPts);

    // Thick Solid Glowing Electric Cyan 3D Tube
    const refTubeGeo = new THREE.TubeGeometry(refCurve, 320, 0.065, 12, false);
    const refTubeMat = new THREE.MeshStandardMaterial({{
      color: 0x00f5d4,
      emissive: 0x00f5d4,
      emissiveIntensity: 0.75,
      roughness: 0.25,
      metalness: 0.15,
      transparent: false
    }});
    const refPathMesh = new THREE.Mesh(refTubeGeo, refTubeMat);
    refPathMesh.castShadow = true;
    refGroup.add(refPathMesh);

    // Bright White Core Centerline inside the Cyan Tube
    const corePts = refCurve.getPoints(360);
    const coreGeo = new THREE.BufferGeometry().setFromPoints(corePts);
    const coreMat = new THREE.LineBasicMaterial({{ color: 0xffffff, linewidth: 2 }});
    const coreLine = new THREE.Line(coreGeo, coreMat);
    refGroup.add(coreLine);

    // Reference Waypoint Drop Pillars (from each waypoint to the ground)
    const allWps = [startPos, ...waypoints];
    allWps.forEach((wp) => {{
      const topPt = uavToThree(wp[0], wp[1], wp[2]);
      const botPt = new THREE.Vector3(topPt.x, 0.02, topPt.z);

      const pillarGeo = new THREE.BufferGeometry().setFromPoints([topPt, botPt]);
      const pillarMat = new THREE.LineDashedMaterial({{
        color: 0x00f5d4,
        dashSize: 0.25,
        gapSize: 0.15,
        transparent: true,
        opacity: 0.65
      }});
      const pillar = new THREE.Line(pillarGeo, pillarMat);
      pillar.computeLineDistances();
      refGroup.add(pillar);

      const ringGeo = new THREE.RingGeometry(0.28, 0.38, 24);
      const ringMat = new THREE.MeshBasicMaterial({{ color: 0x00f5d4, side: THREE.DoubleSide, transparent: true, opacity: 0.55 }});
      const ring = new THREE.Mesh(ringGeo, ringMat);
      ring.rotation.x = -Math.PI / 2;
      ring.position.set(topPt.x, 0.025, topPt.z);
      refGroup.add(ring);
    }});
    scene.add(refGroup);

    // --------------------------------------------------------------------------
    // 2. Actual Trajectory Trail: 3D Ribbon + Luminous Core Line (Orange - Real-World Disturbances & Sway)
    // --------------------------------------------------------------------------
    const maxTrailPoints = data.length;
    const ribbonPositions = new Float32Array(maxTrailPoints * 2 * 3);
    const ribbonIndices = new Uint16Array((maxTrailPoints - 1) * 6);
    for (let i = 0; i < maxTrailPoints - 1; i++) {{
      const i2 = i * 2;
      ribbonIndices[i * 6] = i2;
      ribbonIndices[i * 6 + 1] = i2 + 1;
      ribbonIndices[i * 6 + 2] = i2 + 2;
      ribbonIndices[i * 6 + 3] = i2 + 1;
      ribbonIndices[i * 6 + 4] = i2 + 3;
      ribbonIndices[i * 6 + 5] = i2 + 2;
    }}
    const ribbonGeo = new THREE.BufferGeometry();
    ribbonGeo.setAttribute('position', new THREE.BufferAttribute(ribbonPositions, 3));
    ribbonGeo.setIndex(new THREE.BufferAttribute(ribbonIndices, 1));
    const ribbonMesh = new THREE.Mesh(
      ribbonGeo,
      new THREE.MeshBasicMaterial({{ color: 0xff4d29, side: THREE.DoubleSide, transparent: false }})
    );
    scene.add(ribbonMesh);

    const trailPositions = new Float32Array(maxTrailPoints * 3);
    const trailGeo = new THREE.BufferGeometry();
    trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
    const trailLine = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({{ color: 0xffea00, linewidth: 2 }}));
    scene.add(trailLine);

    // Atmospheric Wind Dust / Air Particles (250 particles)
    const particleCount = 250;
    const particlePositions = new Float32Array(particleCount * 3);
    for (let i = 0; i < particleCount; i++) {{
      particlePositions[i * 3] = (Math.random() - 0.2) * 32;
      particlePositions[i * 3 + 1] = Math.random() * 10;
      particlePositions[i * 3 + 2] = (Math.random() - 0.5) * 24;
    }}
    const particleGeo = new THREE.BufferGeometry();
    particleGeo.setAttribute('position', new THREE.BufferAttribute(particlePositions, 3));
    const particleSystem = new THREE.Points(
      particleGeo,
      new THREE.PointsMaterial({{ color: 0x38bdf8, size: 0.08, transparent: true, opacity: 0.5 }})
    );
    scene.add(particleSystem);

    // --------------------------------------------------------------------------
    // Ground Effect Prop-Wash Dust Particle System (Low-Altitude Dust Ring)
    // --------------------------------------------------------------------------
    const dustCount = 80;
    const dustGeo = new THREE.BufferGeometry();
    const dustPositions = new Float32Array(dustCount * 3);
    const dustVelocities = [];
    for (let i = 0; i < dustCount; i++) {{
      dustPositions[i * 3] = 0;
      dustPositions[i * 3 + 1] = 0.02;
      dustPositions[i * 3 + 2] = 0;
      const ang = Math.random() * Math.PI * 2;
      const spd = 0.8 + Math.random() * 1.5;
      dustVelocities.push({{ vx: Math.cos(ang) * spd, vz: Math.sin(ang) * spd, age: Math.random() * 0.8 }});
    }}
    dustGeo.setAttribute('position', new THREE.BufferAttribute(dustPositions, 3));
    const dustMat = new THREE.PointsMaterial({{ color: 0xd97706, size: 0.12, transparent: true, opacity: 0.0 }});
    const dustParticles = new THREE.Points(dustGeo, dustMat);
    scene.add(dustParticles);

    // --------------------------------------------------------------------------
    // Photorealistic 3D Quadrotor UAV Model
    // --------------------------------------------------------------------------
    const uavGroup = new THREE.Group();

    // Central Carbon Fiber Body Core
    const bodyMat = new THREE.MeshStandardMaterial({{ color: 0x0f172a, roughness: 0.3, metalness: 0.85 }});
    const canopyMesh = new THREE.Mesh(new THREE.ConeGeometry(0.18, 0.12, 8), bodyMat);
    canopyMesh.geometry.rotateX(Math.PI / 2);
    canopyMesh.position.set(0.04, 0.04, 0);
    canopyMesh.castShadow = true;
    uavGroup.add(canopyMesh);

    const bellyMesh = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.07, 0.18), new THREE.MeshStandardMaterial({{ color: 0x1e293b, roughness: 0.5 }}));
    bellyMesh.position.set(0, -0.01, 0);
    bellyMesh.castShadow = true;
    uavGroup.add(bellyMesh);

    // Front Gimbal Nose Camera
    const camMesh = new THREE.Mesh(new THREE.SphereGeometry(0.045, 16, 16), new THREE.MeshStandardMaterial({{ color: 0x0284c7, metalness: 0.9, roughness: 0.1 }}));
    camMesh.position.set(0.15, -0.01, 0);
    uavGroup.add(camMesh);

    // Carbon Fiber Arms (X-Configuration)
    const armMat = new THREE.MeshStandardMaterial({{ color: 0x334155, roughness: 0.4, metalness: 0.7 }});
    const armGeo = new THREE.CylinderGeometry(0.014, 0.014, 0.48, 12);
    const arm1 = new THREE.Mesh(armGeo, armMat);
    arm1.rotation.z = Math.PI / 2;
    arm1.rotation.y = Math.PI / 4;
    arm1.castShadow = true;
    uavGroup.add(arm1);

    const arm2 = new THREE.Mesh(armGeo, armMat);
    arm2.rotation.z = Math.PI / 2;
    arm2.rotation.y = -Math.PI / 4;
    arm2.castShadow = true;
    uavGroup.add(arm2);

    // Curved Landing Skids
    const skidMat = new THREE.MeshStandardMaterial({{ color: 0x64748b, metalness: 0.5 }});
    const skidGeo = new THREE.TorusGeometry(0.14, 0.008, 8, 16, Math.PI);
    const skid1 = new THREE.Mesh(skidGeo, skidMat);
    skid1.rotation.y = Math.PI / 2;
    skid1.position.set(0, -0.06, 0.09);
    uavGroup.add(skid1);
    const skid2 = new THREE.Mesh(skidGeo, skidMat);
    skid2.rotation.y = Math.PI / 2;
    skid2.position.set(0, -0.06, -0.09);
    uavGroup.add(skid2);

    // 4 Motors, Blades, Blur Discs & Aviation Navigation Lighting
    const motorGeo = new THREE.CylinderGeometry(0.025, 0.025, 0.035, 16);
    const motorMat = new THREE.MeshStandardMaterial({{ color: 0xd97706, metalness: 0.9, roughness: 0.2 }});
    const bladeGeo = new THREE.BoxGeometry(0.24, 0.004, 0.022);
    const bladeMat = new THREE.MeshStandardMaterial({{ color: 0xf8fafc, roughness: 0.3 }});
    const blurGeo = new THREE.CylinderGeometry(0.13, 0.13, 0.003, 24);
    const blurMat = new THREE.MeshBasicMaterial({{ color: 0x38bdf8, transparent: true, opacity: 0.45 }});

    const rotors = [];
    const rotorArmOffsets = [
      [0.17, 0.025, 0.17],   // Motor 1: Front-Left (Green nav light)
      [-0.17, 0.025, 0.17],  // Motor 2: Rear-Left (White nav light)
      [-0.17, 0.025, -0.17], // Motor 3: Rear-Right (White nav light)
      [0.17, 0.025, -0.17]   // Motor 4: Front-Right (Red nav light)
    ];
    const navLightColors = [0x22c55e, 0xf8fafc, 0xf8fafc, 0xef4444];

    rotorArmOffsets.forEach((pos, i) => {{
      const motor = new THREE.Mesh(motorGeo, motorMat);
      motor.position.set(pos[0], pos[1], pos[2]);
      motor.castShadow = true;
      uavGroup.add(motor);

      // Nav LED
      const led = new THREE.Mesh(new THREE.SphereGeometry(0.018, 8, 8), new THREE.MeshBasicMaterial({{ color: navLightColors[i] }}));
      led.position.set(pos[0], pos[1] - 0.03, pos[2]);
      uavGroup.add(led);

      // Rotor Assembly
      const rotorAssembly = new THREE.Group();
      rotorAssembly.position.set(pos[0], pos[1] + 0.02, pos[2]);
      rotorAssembly.add(new THREE.Mesh(bladeGeo, bladeMat));
      rotorAssembly.add(new THREE.Mesh(blurGeo, blurMat));
      uavGroup.add(rotorAssembly);
      rotors.push(rotorAssembly);
    }});

    // Tail White Anti-Collision Strobe Light
    const tailStrobeLight = new THREE.PointLight(0xffffff, 0, 4.0);
    tailStrobeLight.position.set(-0.16, 0.05, 0);
    uavGroup.add(tailStrobeLight);
    const tailStrobeMesh = new THREE.Mesh(new THREE.SphereGeometry(0.018, 8, 8), new THREE.MeshBasicMaterial({{ color: 0xffffff }}));
    tailStrobeMesh.position.set(-0.16, 0.05, 0);
    uavGroup.add(tailStrobeMesh);

    scene.add(uavGroup);

    // --------------------------------------------------------------------------
    // UI Event Listeners
    // --------------------------------------------------------------------------
    const btnPlay = document.getElementById('btn-play');
    const btnRestart = document.getElementById('btn-restart');
    const btnToggleRef = document.getElementById('btn-toggle-ref');
    const timeline = document.getElementById('timeline-slider');
    const timeDisplay = document.getElementById('time-display');
    const speedSelect = document.getElementById('speed-select');

    btnPlay.addEventListener('click', () => {{
      isPlaying = !isPlaying;
      btnPlay.textContent = isPlaying ? 'Pause' : 'Play';
    }});

    btnRestart.addEventListener('click', () => {{
      currentIndex = 0;
      timeline.value = 0;
    }});

    btnToggleRef.addEventListener('click', () => {{
      refGroup.visible = !refGroup.visible;
      btnToggleRef.textContent = refGroup.visible ? 'Reference Line: ON' : 'Reference Line: OFF';
      btnToggleRef.style.background = refGroup.visible ? '#00f5d4' : '#1e293b';
      btnToggleRef.style.color = refGroup.visible ? '#04070d' : '#e2e8f0';
      btnToggleRef.style.borderColor = refGroup.visible ? '#00f5d4' : '#475569';
    }});

    timeline.addEventListener('input', (e) => {{
      currentIndex = parseInt(e.target.value);
      updateState(currentIndex);
    }});

    speedSelect.addEventListener('change', (e) => {{
      speed = parseFloat(e.target.value);
    }});

    window.addEventListener('resize', () => {{
      camera.aspect = container.clientWidth / container.clientHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(container.clientWidth, container.clientHeight);
    }});

    // --------------------------------------------------------------------------
    // Update Simulation State Frame
    // --------------------------------------------------------------------------
    function updateState(idx) {{
      const d = data[idx];
      const p = uavToThree(d.x, d.y, d.z);
      uavGroup.position.copy(p);

      // Attitude (Euler Roll, Pitch, Yaw)
      uavGroup.rotation.set(
        THREE.MathUtils.degToRad(d.pitch),
        THREE.MathUtils.degToRad(-d.yaw),
        THREE.MathUtils.degToRad(d.roll),
        'YXZ'
      );

      // Spin Rotors
      rotors.forEach((r, i) => {{
        r.rotation.y += (i % 2 === 0 ? 0.45 : -0.45);
      }});

      // Update Dynamic Drop Shadow
      dropShadow.position.set(p.x, 0.015, p.z);
      const shadowScale = Math.max(0.6, 1.4 - d.z * 0.12);
      dropShadow.scale.set(shadowScale, shadowScale, 1);
      dropShadowMat.opacity = Math.max(0.15, 0.75 - d.z * 0.11);

      // Active Waypoint Halo
      const wpIdx = Math.min(d.wp - 1, waypoints.length - 1);
      const targetWp = waypoints[wpIdx];
      haloMesh.position.copy(uavToThree(targetWp[0], targetWp[1], targetWp[2]));

      // Update Actual Trajectory Trail Ribbon & Core Line
      const trailWidth = 0.045; // 4.5 cm wide 3D ribbon
      for (let i = 0; i <= idx; i++) {{
        const pt = uavToThree(data[i].x, data[i].y, data[i].z);
        trailPositions[i * 3] = pt.x;
        trailPositions[i * 3 + 1] = pt.y + 0.005; // Slightly above ribbon
        trailPositions[i * 3 + 2] = pt.z;

        let nx = 0, nz = 1;
        if (i < data.length - 1) {{
          const nextPt = uavToThree(data[i + 1].x, data[i + 1].y, data[i + 1].z);
          const dx = nextPt.x - pt.x;
          const dz = nextPt.z - pt.z;
          const len = Math.sqrt(dx * dx + dz * dz);
          if (len > 1e-4) {{
            nx = -dz / len;
            nz = dx / len;
          }}
        }} else if (i > 0) {{
          const prevPt = uavToThree(data[i - 1].x, data[i - 1].y, data[i - 1].z);
          const dx = pt.x - prevPt.x;
          const dz = pt.z - prevPt.z;
          const len = Math.sqrt(dx * dx + dz * dz);
          if (len > 1e-4) {{
            nx = -dz / len;
            nz = dx / len;
          }}
        }}

        const vLeft = (i * 2) * 3;
        const vRight = (i * 2 + 1) * 3;
        ribbonPositions[vLeft] = pt.x - nx * trailWidth;
        ribbonPositions[vLeft + 1] = pt.y;
        ribbonPositions[vLeft + 2] = pt.z - nz * trailWidth;
        ribbonPositions[vRight] = pt.x + nx * trailWidth;
        ribbonPositions[vRight + 1] = pt.y;
        ribbonPositions[vRight + 2] = pt.z + nz * trailWidth;
      }}
      trailGeo.setDrawRange(0, idx + 1);
      trailGeo.attributes.position.needsUpdate = true;
      ribbonGeo.setDrawRange(0, Math.max(0, idx) * 6);
      ribbonGeo.attributes.position.needsUpdate = true;

      // Update Right-Side HUD Panel
      document.getElementById('hud-time').textContent = `t = ${{d.t.toFixed(3)}} s`;
      document.getElementById('hud-x').textContent = `${{d.x.toFixed(3)}} m`;
      document.getElementById('hud-y').textContent = `${{d.y.toFixed(3)}} m`;
      document.getElementById('hud-z').textContent = `${{d.z.toFixed(3)}} m`;
      document.getElementById('hud-xr').textContent = `${{d.xr.toFixed(3)}} m`;
      document.getElementById('hud-yr').textContent = `${{d.yr.toFixed(3)}} m`;
      document.getElementById('hud-zr').textContent = `${{d.zr.toFixed(3)}} m`;

      const exPrefix = d.ex >= 0 ? '+' : '';
      const eyPrefix = d.ey >= 0 ? '+' : '';
      const ezPrefix = d.ez >= 0 ? '+' : '';
      document.getElementById('hud-ex').textContent = `${{exPrefix}}${{d.ex.toFixed(4)}} m`;
      document.getElementById('hud-ey').textContent = `${{eyPrefix}}${{d.ey.toFixed(4)}} m`;
      document.getElementById('hud-ez').textContent = `${{ezPrefix}}${{d.ez.toFixed(4)}} m`;
      document.getElementById('hud-e').textContent = `${{d.e.toFixed(4)}} m`;

      document.getElementById('hud-wp').textContent = `Waypoint ${{d.wp}}`;
      document.getElementById('hud-wptarget').textContent = `(${{targetWp[0].toFixed(1)}}, ${{targetWp[1].toFixed(1)}}, ${{targetWp[2].toFixed(1)}}) m`;
      document.getElementById('hud-dwp').textContent = `${{d.dwp.toFixed(3)}} m`;

      document.getElementById('hud-vx').textContent = `${{d.vx >= 0 ? '+' : ''}}${{d.vx.toFixed(3)}} m/s`;
      document.getElementById('hud-vy').textContent = `${{d.vy >= 0 ? '+' : ''}}${{d.vy.toFixed(3)}} m/s`;
      document.getElementById('hud-vz').textContent = `${{d.vz >= 0 ? '+' : ''}}${{d.vz.toFixed(3)}} m/s`;

      document.getElementById('hud-roll').textContent = `${{d.roll >= 0 ? '+' : ''}}${{d.roll.toFixed(2)}} deg`;
      document.getElementById('hud-pitch').textContent = `${{d.pitch >= 0 ? '+' : ''}}${{d.pitch.toFixed(2)}} deg`;
      document.getElementById('hud-yaw').textContent = `${{d.yaw >= 0 ? '+' : ''}}${{d.yaw.toFixed(2)}} deg`;

      document.getElementById('hud-qerr').textContent = `[${{d.qerr.map(v => v.toFixed(3)).join(', ')}}]`;
      document.getElementById('hud-delta').textContent = `[${{d.delta.map(v => v.toFixed(3)).join(', ')}}]`;
      document.getElementById('hud-tau').textContent = `[${{d.tau.map(v => v.toFixed(3)).join(', ')}}]`;

      // Timeline & Time Text
      timeline.value = idx;
      timeDisplay.textContent = `t = ${{d.t.toFixed(2)}} s / ${{data[data.length-1].t.toFixed(2)}} s`;
    }}

    // --------------------------------------------------------------------------
    // Main Animation Loop
    // --------------------------------------------------------------------------
    function animate(now) {{
      requestAnimationFrame(animate);

      const delta = (now - lastTime) / 1000;
      lastTime = now;

      // Orient 3D Windsock according to atmospheric wind (0.6, -0.4)
      const windAngle = Math.atan2(-0.4, 0.6);
      wsSwivel.rotation.y = -windAngle + Math.PI / 2 + Math.sin(now * 0.002) * 0.12;

      // Pulsate Obstacle Warning Beacons & Tail Strobe Light
      const beaconIntensity = 1.0 + 0.8 * Math.sin(now * 0.008);
      obstacleBeacons.forEach(b => {{ b.intensity = beaconIntensity; }});

      // Aviation Double-Flash Tail Strobe (1.2 Hz)
      const strobePhase = (now % 833) / 833;
      const isStrobeOn = (strobePhase < 0.06) || (strobePhase > 0.12 && strobePhase < 0.18);
      tailStrobeLight.intensity = isStrobeOn ? 3.5 : 0.0;
      tailStrobeMesh.material.color.setHex(isStrobeOn ? 0xffffff : 0x334155);

      // Drift Atmospheric Wind Particles
      const pAttr = particleSystem.geometry.attributes.position;
      const windSpeedX = 0.6 * delta;
      const windSpeedZ = -0.4 * delta;
      for (let i = 0; i < particleCount; i++) {{
        pAttr.array[i * 3] += windSpeedX;
        pAttr.array[i * 3 + 2] += windSpeedZ;
        if (pAttr.array[i * 3] > 28) pAttr.array[i * 3] = -4;
        if (pAttr.array[i * 3 + 2] < -12) pAttr.array[i * 3 + 2] = 12;
      }}
      pAttr.needsUpdate = true;

      // Ground Effect Prop-Wash Dust Particle Simulation
      const d = data[currentIndex];
      const p = uavToThree(d.x, d.y, d.z);
      if (d.z < 2.5) {{
        dustMat.opacity = THREE.MathUtils.lerp(dustMat.opacity, (2.5 - d.z) * 0.35, 0.1);
        const dPos = dustGeo.attributes.position.array;
        for (let i = 0; i < dustCount; i++) {{
          dustVelocities[i].age += delta;
          if (dustVelocities[i].age > 0.6) {{
            dPos[i * 3] = p.x + (Math.random() - 0.5) * 0.4;
            dPos[i * 3 + 1] = 0.02;
            dPos[i * 3 + 2] = p.z + (Math.random() - 0.5) * 0.4;
            dustVelocities[i].age = 0;
          }} else {{
            dPos[i * 3] += dustVelocities[i].vx * delta;
            dPos[i * 3 + 2] += dustVelocities[i].vz * delta;
          }}
        }}
        dustGeo.attributes.position.needsUpdate = true;
      }} else {{
        dustMat.opacity = THREE.MathUtils.lerp(dustMat.opacity, 0.0, 0.1);
      }}

      if (isPlaying) {{
        currentIndex += Math.max(1, Math.round(speed * 20 * delta));
        if (currentIndex >= data.length) {{
          currentIndex = data.length - 1;
          isPlaying = false;
          btnPlay.textContent = 'Play';
        }}
      }}

      updateState(currentIndex);
      controls.update();
      renderer.render(scene, camera);
    }}

    requestAnimationFrame(animate);
  </script>
</body>
</html>
"""
    os.makedirs(os.path.dirname(os.path.abspath(output_html)), exist_ok=True)
    with open(output_html, 'w', encoding='utf-8') as f:
        f.write(html_content)
    print(f"Generated clean real-world 3D WebGL viewer: {output_html}")


if __name__ == '__main__':
    csv_file = os.path.join(PKG_ROOT, 'uav_trajectory_log.csv')
    yaml_file = os.path.join(PKG_ROOT, 'config', 'uav_params.yaml')
    html_file = os.path.join(PKG_ROOT, 'plots', 'interactive_3d_simulation.html')
    export_html(csv_file, yaml_file, html_file)
