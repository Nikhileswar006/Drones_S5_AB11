import time
import math
import heapq
import traceback
import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil

# ============================================================
# SETTINGS
# ============================================================

CONNECTION = "udpin:0.0.0.0:14550"

TAKEOFF_ALTITUDE = 5.0
WAYPOINT_TOLERANCE = 0.65
SEND_PERIOD = 0.2
LOOP_PERIOD = 0.05

# Gaussian sensor noise standard deviations
POSITION_NOISE_STD = 0.15       # metres
VELOCITY_NOISE_STD = 0.08       # metres/second
QUATERNION_NOISE_STD = 0.025

RANDOM_SEED = None
if RANDOM_SEED is not None:
    np.random.seed(RANDOM_SEED)

# Backstepping parameters from the reference paper
J = np.diag([0.1, 0.1, 0.12])
K1 = 20.0
K2 = 2.0

# A -> B -> C -> D -> A in local NED coordinates
WAYPOINTS = [
    np.array([0., 0., -5.]),
    np.array([8., 0., -5.]),
    np.array([8., 8., -5.]),
    np.array([0., 8., -5.]),
    np.array([0., 0., -5.])
]

WAYPOINT_NAMES = ["A", "B", "C", "D", "A"]

# Static obstacle map: X, Y coordinates in metres
OBSTACLES = [
    {"name": "O1", "x": 4., "y": 0., "radius": 0.55},
    {"name": "O2", "x": 8., "y": 4., "radius": 0.55},
    {"name": "O3", "x": 4., "y": 8., "radius": 0.55},
    {"name": "O4", "x": 0., "y": 4., "radius": 0.55}
]

CLEARANCE = 0.9
GRID = 0.25
MIN_X, MAX_X = -2., 10.
MIN_Y, MAX_Y = -2., 10.

# ============================================================
# QUATERNION FUNCTIONS
# Quaternion format: [w, x, y, z]
# ============================================================

def normalize_q(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1., 0., 0., 0.])
    return q / n


def q_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_multiply(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b

    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


def euler_to_q(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)

    return normalize_q(np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy
    ]))


def add_noise(value, std):
    value = np.asarray(value, dtype=float)
    noise = np.random.normal(0.0, std, size=value.shape)
    return value + noise, noise


def noisy_quaternion(q):
    noisy, noise = add_noise(q, QUATERNION_NOISE_STD)
    return normalize_q(noisy), noise


# ============================================================
# A* OBSTACLE AVOIDANCE
# ============================================================

def free_point(x, y):
    if not (MIN_X <= x <= MAX_X and MIN_Y <= y <= MAX_Y):
        return False

    for obs in OBSTACLES:
        distance = math.hypot(x - obs["x"], y - obs["y"])
        if distance < obs["radius"] + CLEARANCE:
            return False

    return True


def to_cell(x, y):
    return (round(x / GRID), round(y / GRID))


def to_xy(cell):
    return (cell[0] * GRID, cell[1] * GRID)


def line_free(a, b):
    distance = math.hypot(b[0] - a[0], b[1] - a[1])
    steps = max(2, int(distance / (GRID * 0.35)))

    for i in range(steps + 1):
        t = i / steps
        x = a[0] + t * (b[0] - a[0])
        y = a[1] + t * (b[1] - a[1])
        if not free_point(x, y):
            return False

    return True


def smooth_path(path):
    if len(path) <= 2:
        return path

    result = [path[0]]
    index = 0

    while index < len(path) - 1:
        farthest = index + 1

        for candidate in range(len(path) - 1, index, -1):
            if line_free(path[index], path[candidate]):
                farthest = candidate
                break

        result.append(path[farthest])
        index = farthest

    return result


def astar(start_xy, goal_xy):
    start = to_cell(*start_xy)
    goal = to_cell(*goal_xy)

    if not free_point(*start_xy) or not free_point(*goal_xy):
        raise ValueError("A waypoint is inside an obstacle clearance zone.")

    directions = [
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1), (0, 1),
        (1, -1), (1, 0), (1, 1)
    ]

    def heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    queue = [(heuristic(start, goal), 0.0, start)]
    parent = {}
    cost = {start: 0.0}
    visited = set()

    while queue:
        _, _, current = heapq.heappop(queue)

        if current in visited:
            continue

        if current == goal:
            cells = [current]
            while current in parent:
                current = parent[current]
                cells.append(current)

            cells.reverse()
            path = [tuple(start_xy)]
            path += [to_xy(c) for c in cells[1:-1]]
            path.append(tuple(goal_xy))
            return smooth_path(path)

        visited.add(current)

        for dx, dy in directions:
            nxt = (current[0] + dx, current[1] + dy)
            x, y = to_xy(nxt)

            if not free_point(x, y):
                continue

            if dx != 0 and dy != 0:
                if (not free_point(*to_xy((current[0]+dx, current[1]))
                    ) or not free_point(*to_xy((current[0], current[1]+dy)))):
                    continue

            step = math.sqrt(2) if dx and dy else 1.0
            new_cost = cost[current] + step

            if new_cost < cost.get(nxt, float("inf")):
                cost[nxt] = new_cost
                parent[nxt] = current
                priority = new_cost + heuristic(nxt, goal)
                heapq.heappush(queue, (priority, new_cost, nxt))

    raise RuntimeError("A* could not find a path around the obstacles.")


def build_route():
    route = [WAYPOINTS[0].copy()]
    labels = ["A (start)"]

    for leg in range(len(WAYPOINTS) - 1):
        start = WAYPOINTS[leg]
        goal = WAYPOINTS[leg + 1]

        path = astar(start[:2], goal[:2])

        for xy in path[1:]:
            point = np.array([xy[0], xy[1], goal[2]], dtype=float)
            route.append(point)

            if np.linalg.norm(np.asarray(xy) - goal[:2]) < 1e-5:
                labels.append(WAYPOINT_NAMES[leg + 1])
            else:
                labels.append(f"Detour {leg+1}")

    clean_route = []
    clean_labels = []

    for point, label in zip(route, labels):
        if clean_route and np.linalg.norm(point-clean_route[-1]) < 1e-6:
            clean_labels[-1] = label
        else:
            clean_route.append(point)
            clean_labels.append(label)

    return clean_route, clean_labels


# ============================================================
# MAVLINK CONNECTION AND TELEMETRY
# ============================================================

connection = None

position_latest = np.array([0., 0., -5.])
velocity_latest = np.zeros(3)
q_latest = np.array([1., 0., 0., 0.])
omega_latest = np.zeros(3)


def connect():
    global connection

    print("Connecting to ArduPilot...")
    connection = mavutil.mavlink_connection(CONNECTION)
    connection.wait_heartbeat(timeout=60)
    print("Heartbeat received.")


def set_guided():
    modes = connection.mode_mapping()
    if "GUIDED" not in modes:
        raise RuntimeError("GUIDED mode not available.")

    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        modes["GUIDED"]
    )
    time.sleep(1)


def arm():
    print("Requesting arm...")

    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )

    start = time.time()

    while time.time() - start < 15:
        msg = connection.recv_match(
            type="HEARTBEAT", blocking=True, timeout=1
        )

        if msg and (
            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        ):
            print("Vehicle armed.")
            return

    raise RuntimeError(
        "Arm failed. Check the ArduPilot console for pre-arm errors."
    )


def takeoff(altitude):
    print(f"Taking off to {altitude:.1f} m...")

    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )

    start = time.time()

    while time.time() - start < 35:
        msg = connection.recv_match(
            type="GLOBAL_POSITION_INT", blocking=True, timeout=1
        )

        if msg is None:
            continue

        current_alt = msg.relative_alt / 1000.0
        print(f"\rAltitude: {current_alt:.2f} m", end="", flush=True)

        if current_alt >= altitude * 0.85:
            print("\nTakeoff altitude reached.")
            return

    print("\nWarning: takeoff altitude was not confirmed.")


def send_target(target):
    x, y, z = map(float, target)

    # Position only. Velocity, acceleration, yaw and yaw-rate ignored.
    type_mask = 0b0000111111111000

    connection.mav.set_position_target_local_ned_send(
        0,
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )


def get_telemetry():
    global position_latest, velocity_latest, q_latest, omega_latest

    for _ in range(100):
        msg = connection.recv_match(
            type=["LOCAL_POSITION_NED", "ATTITUDE"],
            blocking=False
        )

        if msg is None:
            break

        if msg.get_type() == "LOCAL_POSITION_NED":
            position_latest = np.array([msg.x, msg.y, msg.z], dtype=float)
            velocity_latest = np.array([msg.vx, msg.vy, msg.vz], dtype=float)

        elif msg.get_type() == "ATTITUDE":
            q_latest = euler_to_q(msg.roll, msg.pitch, msg.yaw)
            omega_latest = np.array([
                msg.rollspeed, msg.pitchspeed, msg.yawspeed
            ], dtype=float)

    return (
        position_latest.copy(),
        velocity_latest.copy(),
        q_latest.copy(),
        omega_latest.copy()
    )


# ============================================================
# BACKSTEPPING CALCULATIONS
# These values are displayed, not sent as motor torque commands.
# ============================================================

previous_omega_desired = np.zeros(3)
previous_time = None


def backstepping(q, omega, now):
    global previous_omega_desired, previous_time

    q = normalize_q(q)
    identity = np.array([1., 0., 0., 0.])

    q_error = normalize_q(q_multiply(q_conjugate(identity), q))
    omega_error = omega.copy()

    q_difference = q_error - identity
    omega_desired_quaternion = -K1 * q_multiply(
        q_conjugate(q_error), q_difference
    )
    omega_desired = omega_desired_quaternion[1:4]

    if previous_time is None:
        omega_desired_dot = np.zeros(3)
    else:
        dt = max(1e-3, min(now - previous_time, 0.2))
        omega_desired_dot = (
            omega_desired - previous_omega_desired
        ) / dt

    delta = omega_error - omega_desired

    tau1 = np.cross(omega, J @ omega) + J @ omega_desired_dot

    signed_cube_root = np.sign(delta) * np.abs(delta) ** (1.0 / 3.0)
    tau2 = -K2 * (J @ signed_cube_root)

    tau = tau1 + tau2

    previous_omega_desired = omega_desired.copy()
    previous_time = now

    return {
        "q_error": q_error,
        "omega_error": omega_error,
        "omega_desired": omega_desired,
        "delta": delta,
        "tau1": tau1,
        "tau2": tau2,
        "tau": tau
    }


# ============================================================
# DASHBOARD 1: TRAJECTORY
# DASHBOARD 2: NOISY READINGS + CALCULATIONS
# ============================================================

plt.ion()

route, route_labels = build_route()

fig1, ax1 = plt.subplots(figsize=(9, 7))
ax1.set_title("Drone trajectory and obstacle avoidance")
ax1.set_xlabel("Local X (m)")
ax1.set_ylabel("Local Y (m)")
ax1.set_xlim(MIN_X, MAX_X)
ax1.set_ylim(MIN_Y, MAX_Y)
ax1.set_aspect("equal", adjustable="box")
ax1.grid(True, alpha=0.3)

for obs in OBSTACLES:
    ax1.add_patch(plt.Circle(
        (obs["x"], obs["y"]), obs["radius"],
        color="red", alpha=0.75
    ))
    ax1.add_patch(plt.Circle(
        (obs["x"], obs["y"]), obs["radius"] + CLEARANCE,
        fill=False, color="orange", linestyle="--"
    ))
    ax1.text(obs["x"], obs["y"], obs["name"],
             ha="center", va="center", color="white")

route_xy = np.array([[p[0], p[1]] for p in route])
ax1.plot(route_xy[:, 0], route_xy[:, 1], "b--", label="Planned route")

for point, name in zip(WAYPOINTS, WAYPOINT_NAMES):
    ax1.scatter(point[0], point[1], marker="s", color="black")
    ax1.text(point[0]+0.1, point[1]+0.1, name)

actual_line, = ax1.plot([], [], "g-", linewidth=2, label="Actual path")
drone_dot, = ax1.plot([], [], "ko", label="Drone")
target_dot, = ax1.plot([], [], "mo", label="Target")
ax1.legend()

status = ax1.text(
    0.02, 0.98, "Waiting for telemetry",
    transform=ax1.transAxes, va="top",
    bbox=dict(facecolor="white", alpha=0.8)
)

fig2, ax2 = plt.subplots(figsize=(13, 9))
ax2.axis("off")
fig2.subplots_adjust(left=0.02, right=0.98, top=0.98, bottom=0.02)

numbers = ax2.text(
    0.01, 0.99, "Waiting for telemetry...",
    transform=ax2.transAxes, va="top", ha="left",
    family="monospace", fontsize=9
)

plt.show(block=False)


def format_vector(v):
    return "[" + ", ".join(f"{x:+.4f}" for x in v) + "]"


def update_dashboard(position, velocity, q, omega, noisy, calc,
                     target, route_index, path_history, complete):

    actual_line.set_data(path_history[:, 0], path_history[:, 1])
    drone_dot.set_data([position[0]], [position[1]])
    target_dot.set_data([target[0]], [target[1]])

    status.set_text(
        f"Target {route_index+1}/{len(route)}\n"
        f"Position: {format_vector(position)}\n"
        f"Target: {format_vector(target)}\n"
        f"{'MISSION COMPLETE' if complete else 'Mission running'}"
    )

    pn, pnoise = noisy["position"]
    vn, vnoise = noisy["velocity"]
    qn, qnoise = noisy["quaternion"]

    text_lines = [
        "LIVE SENSOR NOISE + BACKSTEPPING CALCULATIONS",
        "=" * 88,
        "Gaussian noise enabled. No filter or noise recovery.",
        f"Noise standard deviations: position={POSITION_NOISE_STD} m, "
        f"velocity={VELOCITY_NOISE_STD} m/s, quaternion={QUATERNION_NOISE_STD}",
        "",
        "POSITION (LOCAL NED, metres)",
        f" True:  {format_vector(position)}",
        f" Noise: {format_vector(pnoise)}",
        f" Noisy: {format_vector(pn)}",
        "",
        "VELOCITY (LOCAL NED, m/s)",
        f" True:  {format_vector(velocity)}",
        f" Noise: {format_vector(vnoise)}",
        f" Noisy: {format_vector(vn)}",
        "",
        "QUATERNION [w, x, y, z]",
        f" True:  {format_vector(q)}",
        f" Noise: {format_vector(qnoise)}",
        f" Noisy normalized: {format_vector(qn)}",
        "",
        "ANGULAR VELOCITY (rad/s)",
        f" True telemetry: {format_vector(omega)}",
        "",
        "BACKSTEPPING VALUES (from original attitude telemetry)",
        f" q_error:             {format_vector(calc['q_error'])}",
        f" omega_error:         {format_vector(calc['omega_error'])}",
        f" omega_error_desired: {format_vector(calc['omega_desired'])}",
        f" delta:               {format_vector(calc['delta'])}",
        f" tau1:                {format_vector(calc['tau1'])}",
        f" tau2:                {format_vector(calc['tau2'])}",
        f" total tau:           {format_vector(calc['tau'])}",
        "",
        f"Current target: {format_vector(target)}",
        "Flight and obstacle avoidance use original telemetry.",
        "Torque is displayed for analysis; it is not sent to motors.",
        "Obstacle circles are a planning map, not physical Gazebo objects."
    ]

    numbers.set_text("\n".join(text_lines))

    fig1.canvas.draw_idle()
    fig2.canvas.draw_idle()
    plt.pause(0.001)


# ============================================================
# MAIN
# ============================================================

def main():
    connect()
    set_guided()
    arm()
    takeoff(TAKEOFF_ALTITUDE)

    print("\nStarting A -> B -> C -> D -> A mission.")
    print("Close dashboard windows or press Ctrl+C to stop.")

    route_index = 1
    path_history = []
    last_send = 0.0
    last_print = 0.0
    complete = False

    while plt.fignum_exists(fig1.number) and plt.fignum_exists(fig2.number):
        start_loop = time.time()

        position, velocity, q, omega = get_telemetry()

        # Add noise only to DISPLAY COPIES of the readings.
        noisy_position, position_noise = add_noise(
            position, POSITION_NOISE_STD
        )
        noisy_velocity, velocity_noise = add_noise(
            velocity, VELOCITY_NOISE_STD
        )
        noisy_q, quaternion_noise = noisy_quaternion(q)

        noisy = {
            "position": (noisy_position, position_noise),
            "velocity": (noisy_velocity, velocity_noise),
            "quaternion": (noisy_q, quaternion_noise)
        }

        now = time.time()
        calc = backstepping(q, omega, now)

        if not path_history or np.linalg.norm(
            position[:2] - path_history[-1][:2]
        ) > 0.03:
            path_history.append(position.copy())

        path_array = np.array(path_history)

        if not complete:
            target = route[route_index]

            if now - last_send >= SEND_PERIOD:
                send_target(target)
                last_send = now

            # Use original telemetry for waypoint completion.
            if np.linalg.norm(position - target) <= WAYPOINT_TOLERANCE:
                print(
                    f"\nReached {route_labels[route_index]} "
                    f"({route_index+1}/{len(route)})"
                )

                if route_index >= len(route) - 1:
                    complete = True
                    print("A -> B -> C -> D -> A mission complete.")
                else:
                    route_index += 1
                    last_send = 0.0

            display_target = target
        else:
            display_target = route[-1]

        update_dashboard(
            position, velocity, q, omega, noisy, calc,
            display_target, route_index, path_array, complete
        )

        if now - last_print >= 2:
            print(
                f"\rPosition={format_vector(position)} | "
                f"|tau|={np.linalg.norm(calc['tau']):.3f}",
                end="", flush=True
            )
            last_print = now

        time.sleep(max(0, LOOP_PERIOD - (time.time() - start_loop)))

    print("\nDashboard closed. Script stopped.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception as exc:
        print(f"\nERROR: {exc}")
        traceback.print_exc()
    finally:
        plt.ioff()
        plt.close("all")
