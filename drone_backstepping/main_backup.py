import time
import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil


# ============================================================
# CONNECTION
# ============================================================

connection = mavutil.mavlink_connection(
    "udpin:0.0.0.0:14550"
)

print("Waiting for ArduPilot heartbeat...")
connection.wait_heartbeat()

print(
    "Connected! System ID:",
    connection.target_system,
    "Component ID:",
    connection.target_component
)


# ============================================================
# CONTROLLER PARAMETERS
# ============================================================

J = np.diag([0.1, 0.1, 0.12])

k1 = 20.0
k2 = 2.0

TAKEOFF_ALTITUDE = 5.0

WAYPOINTS = np.array([
    [0.0, 0.0, -5.0],   # A - START
    [8.0, 0.0, -5.0],   # B
    [8.0, 8.0, -5.0],   # C
    [0.0, 8.0, -5.0],   # D
    [0.0, 0.0, -5.0]    # A - FINISH
])

WAYPOINT_NAMES = [
    "A - START",
    "B",
    "C",
    "D",
    "A - FINISH"
]


# ============================================================
# QUATERNION FUNCTIONS
# Quaternion format: [w, x, y, z]
# ============================================================

def normalize_quaternion(q):
    norm = np.linalg.norm(q)

    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])

    return q / norm


def quaternion_conjugate(q):
    return np.array([
        q[0],
        -q[1],
        -q[2],
        -q[3]
    ])


def quaternion_multiply(q1, q2):

    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,

        w1*x2 + x1*w2 + y1*z2 - z1*y2,

        w1*y2 - x1*z2 + y1*w2 + z1*x2,

        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


def euler_to_quaternion(roll, pitch, yaw):

    cr = np.cos(roll / 2.0)
    sr = np.sin(roll / 2.0)

    cp = np.cos(pitch / 2.0)
    sp = np.sin(pitch / 2.0)

    cy = np.cos(yaw / 2.0)
    sy = np.sin(yaw / 2.0)

    q = np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy
    ])

    return normalize_quaternion(q)


# ============================================================
# MAVLINK FUNCTIONS
# ============================================================

def set_guided_mode():

    print("Setting GUIDED mode...")

    connection.set_mode_apm("GUIDED")

    time.sleep(2)


def arm_vehicle():

    print("Arming...")

    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0
    )

    start = time.time()

    while time.time() - start < 10:

        msg = connection.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=1
        )

        if msg is None:
            continue

        armed = (
            msg.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

        if armed:
            print("Vehicle armed.")
            return True

    print("WARNING: Vehicle did not arm.")
    return False


def takeoff(altitude):

    print(f"Taking off to {altitude:.1f} meters...")

    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        altitude
    )

    start = time.time()

    while time.time() - start < 30:

        msg = connection.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=True,
            timeout=1
        )

        if msg is None:
            continue

        current_altitude = msg.relative_alt / 1000.0

        print(
            f"\rAltitude: {current_altitude:.2f} m",
            end=""
        )

        if current_altitude >= altitude * 0.90:
            print()
            print("Takeoff reached.")
            return

    print()
    print("Takeoff timeout.")


def send_position_target(position):

    x = float(position[0])
    y = float(position[1])
    z = float(position[2])

    type_mask = 0b0000111111000111

    connection.mav.set_position_target_local_ned_send(
        0,
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x,
        y,
        z,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    )


# ============================================================
# TELEMETRY
# ============================================================

latest_position = np.array([0.0, 0.0, 0.0])
latest_q = np.array([1.0, 0.0, 0.0, 0.0])
latest_omega = np.zeros(3)


def get_telemetry():

    global latest_position
    global latest_q
    global latest_omega

    for _ in range(20):

        msg = connection.recv_match(
            type=[
                "LOCAL_POSITION_NED",
                "ATTITUDE"
            ],
            blocking=False
        )

        if msg is None:
            break

        if msg.get_type() == "LOCAL_POSITION_NED":

            latest_position = np.array([
                msg.x,
                msg.y,
                msg.z
            ])

        elif msg.get_type() == "ATTITUDE":

            roll = msg.roll
            pitch = msg.pitch
            yaw = msg.yaw

            latest_q = euler_to_quaternion(
                roll,
                pitch,
                yaw
            )

            latest_omega = np.array([
                msg.rollspeed,
                msg.pitchspeed,
                msg.yawspeed
            ])

    return (
        latest_position.copy(),
        latest_q.copy(),
        latest_omega.copy()
    )


# ============================================================
# BACKSTEPPING CONTROLLER
# ============================================================

def calculate_backstepping(
    q,
    omega,
    q_ref,
    omega_ref,
    omega_ref_dot
):

    # --------------------------------------------------------
    # Quaternion error
    #
    # q_err = q_ref* ⊗ q
    # --------------------------------------------------------

    q_ref_conj = quaternion_conjugate(q_ref)

    q_err = quaternion_multiply(
        q_ref_conj,
        q
    )

    q_err = normalize_quaternion(q_err)

    # --------------------------------------------------------
    # Angular velocity error
    #
    # omega_err = omega - omega_ref
    # --------------------------------------------------------

    omega_err = omega - omega_ref

    # --------------------------------------------------------
    # Virtual control
    #
    # omega_err,d = -k1 * epsilon
    #
    # epsilon = vector part of q_err
    # --------------------------------------------------------

    epsilon = q_err[1:4]

    omega_err_d = -k1 * epsilon

    # --------------------------------------------------------
    # Derivative of desired angular velocity error
    #
    # Current implementation uses zero reference derivative
    # --------------------------------------------------------

    omega_err_d_dot = np.zeros(3)

    # --------------------------------------------------------
    # Backstepping error
    #
    # delta = omega_err - omega_err,d
    # --------------------------------------------------------

    delta = omega_err - omega_err_d

    # --------------------------------------------------------
    # Tau 1
    #
    # tau1 =
    # omega x (J omega)
    # + J omega_ref_dot
    # + J omega_err_d_dot
    # --------------------------------------------------------

    tau1 = (
        np.cross(
            omega,
            J @ omega
        )
        + J @ omega_ref_dot
        + J @ omega_err_d_dot
    )

    # --------------------------------------------------------
    # Tau 2
    #
    # tau2 = -k2 J delta^(1/3)
    #
    # sign(delta)*|delta|^(1/3)
    # is used for real-valued fractional power.
    # --------------------------------------------------------

    delta_fraction = (
        np.sign(delta)
        * np.abs(delta) ** (1.0 / 3.0)
    )

    tau2 = -k2 * (J @ delta_fraction)

    # --------------------------------------------------------
    # Total torque
    #
    # tau = tau1 + tau2
    # --------------------------------------------------------

    tau = tau1 + tau2

    return (
        q_err,
        omega_err,
        omega_err_d,
        delta,
        tau1,
        tau2,
        tau
    )


# ============================================================
# DASHBOARD 1
# FLIGHT / TRAJECTORY
# ============================================================

plt.ion()

fig1 = plt.figure(
    figsize=(15, 9),
    num="Dashboard 1 - Flight and Trajectory"
)

gs1 = fig1.add_gridspec(
    2,
    2,
    hspace=0.35,
    wspace=0.25
)

ax3d = fig1.add_subplot(
    gs1[0, 0],
    projection="3d"
)

ax_pos = fig1.add_subplot(gs1[0, 1])

ax_quat = fig1.add_subplot(gs1[1, 0])

ax_omega = fig1.add_subplot(gs1[1, 1])


# ============================================================
# 3D WAYPOINT PATH
# ============================================================

path = WAYPOINTS

ax3d.plot(
    path[:, 0],
    path[:, 1],
    path[:, 2],
    linestyle="--",
    linewidth=2,
    label="Desired path"
)

# Waypoint markers

for i, waypoint in enumerate(WAYPOINTS):

    x, y, z = waypoint

    if i == 0:
        ax3d.scatter(
            x, y, z,
            s=120,
            marker="o",
            label="START / FINISH"
        )

        ax3d.text(
            x,
            y,
            z - 0.5,
            "A\nSTART / FINISH",
            fontsize=9
        )

    elif i == len(WAYPOINTS) - 1:
        pass

    else:

        ax3d.scatter(
            x,
            y,
            z,
            s=100,
            marker="o"
        )

        ax3d.text(
            x,
            y,
            z - 0.5,
            WAYPOINT_NAMES[i],
            fontsize=10
        )


# Actual drone path

actual_path_line, = ax3d.plot(
    [],
    [],
    [],
    linewidth=2,
    label="Actual drone path"
)

drone_point, = ax3d.plot(
    [],
    [],
    [],
    marker="o",
    markersize=8,
    linestyle=""
)

target_point, = ax3d.plot(
    [],
    [],
    [],
    marker="x",
    markersize=12,
    markeredgewidth=3,
    linestyle=""
)

ax3d.set_xlabel("X - North (m)")
ax3d.set_ylabel("Y - East (m)")
ax3d.set_zlabel("Z - Down (m)")

ax3d.set_title(
    "3D Waypoint Trajectory\nA → B → C → D → A"
)

ax3d.set_xlim(-2, 10)
ax3d.set_ylim(-2, 10)
ax3d.set_zlim(-7, 1)

ax3d.view_init(
    elev=25,
    azim=-60
)

ax3d.legend(
    loc="upper left",
    fontsize=8
)


# ============================================================
# POSITION ERROR PLOT
# ============================================================

ax_pos.set_title("Position Tracking Error")

line_ex, = ax_pos.plot(
    [],
    [],
    label="X error"
)

line_ey, = ax_pos.plot(
    [],
    [],
    label="Y error"
)

line_ez, = ax_pos.plot(
    [],
    [],
    label="Z error"
)

ax_pos.set_xlabel("Time (s)")
ax_pos.set_ylabel("Error (m)")
ax_pos.grid(True)
ax_pos.legend()


# ============================================================
# QUATERNION PLOT
# ============================================================

ax_quat.set_title("Actual Quaternion")

quat_lines = []

for i, label in enumerate(
    ["qw", "qx", "qy", "qz"]
):

    line, = ax_quat.plot(
        [],
        [],
        label=label
    )

    quat_lines.append(line)

ax_quat.set_xlabel("Time (s)")
ax_quat.set_ylabel("Quaternion")
ax_quat.grid(True)
ax_quat.legend()


# ============================================================
# ANGULAR VELOCITY
# ============================================================

ax_omega.set_title("Angular Velocity")

omega_lines = []

for i, label in enumerate(
    ["ωx", "ωy", "ωz"]
):

    line, = ax_omega.plot(
        [],
        [],
        label=label
    )

    omega_lines.append(line)

ax_omega.set_xlabel("Time (s)")
ax_omega.set_ylabel("rad/s")
ax_omega.grid(True)
ax_omega.legend()


fig1.suptitle(
    "UAV Flight / Trajectory Dashboard",
    fontsize=16
)


# ============================================================
# DASHBOARD 2
# BACKSTEPPING CONTROLLER
# ============================================================

fig2 = plt.figure(
    figsize=(16, 10),
    num="Dashboard 2 - Backstepping Controller"
)

gs2 = fig2.add_gridspec(
    3,
    3,
    height_ratios=[1, 1, 1.4],
    hspace=0.45,
    wspace=0.3
)

ax_qerr = fig2.add_subplot(gs2[0, 0])
ax_omerr = fig2.add_subplot(gs2[0, 1])
ax_omerrd = fig2.add_subplot(gs2[0, 2])

ax_delta = fig2.add_subplot(gs2[1, 0])
ax_tau1 = fig2.add_subplot(gs2[1, 1])
ax_tau2 = fig2.add_subplot(gs2[1, 2])

ax_tau = fig2.add_subplot(
    gs2[2, 0:2]
)

ax_info = fig2.add_subplot(
    gs2[2, 2]
)

ax_info.axis("off")


# ============================================================
# CONTROLLER PLOT HELPER
# ============================================================

def setup_vector_plot(ax, title, ylabel):

    ax.set_title(title)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True)


setup_vector_plot(
    ax_qerr,
    "Quaternion Error",
    "q error"
)

setup_vector_plot(
    ax_omerr,
    "Angular Velocity Error",
    "rad/s"
)

setup_vector_plot(
    ax_omerrd,
    "Desired Angular Velocity Error",
    "rad/s"
)

setup_vector_plot(
    ax_delta,
    "Backstepping Error δ",
    "δ"
)

setup_vector_plot(
    ax_tau1,
    "τ1",
    "Torque"
)

setup_vector_plot(
    ax_tau2,
    "τ2",
    "Torque"
)

setup_vector_plot(
    ax_tau,
    "Final Control Torque τ = τ1 + τ2",
    "Torque"
)


# ============================================================
# LIVE CONTROLLER LINES
# ============================================================

qerr_lines = []

for label in ["ex", "ey", "ez"]:

    line, = ax_qerr.plot(
        [],
        [],
        label=label
    )

    qerr_lines.append(line)

ax_qerr.legend(fontsize=8)


omega_error_lines = []

for label in ["x", "y", "z"]:

    line, = ax_omerr.plot(
        [],
        [],
        label=label
    )

    omega_error_lines.append(line)

ax_omerr.legend(fontsize=8)


omega_error_d_lines = []

for label in ["x", "y", "z"]:

    line, = ax_omerrd.plot(
        [],
        [],
        label=label
    )

    omega_error_d_lines.append(line)

ax_omerrd.legend(fontsize=8)


delta_lines = []

for label in ["x", "y", "z"]:

    line, = ax_delta.plot(
        [],
        [],
        label=label
    )

    delta_lines.append(line)

ax_delta.legend(fontsize=8)


tau1_lines = []

for label in ["x", "y", "z"]:

    line, = ax_tau1.plot(
        [],
        [],
        label=label
    )

    tau1_lines.append(line)

ax_tau1.legend(fontsize=8)


tau2_lines = []

for label in ["x", "y", "z"]:

    line, = ax_tau2.plot(
        [],
        [],
        label=label
    )

    tau2_lines.append(line)

ax_tau2.legend(fontsize=8)


tau_lines = []

for label in ["x", "y", "z"]:

    line, = ax_tau.plot(
        [],
        [],
        label=label
    )

    tau_lines.append(line)

ax_tau.legend()


fig2.suptitle(
    "Quaternion Backstepping Controller - Live Calculations",
    fontsize=16
)


# ============================================================
# HISTORY ARRAYS
# ============================================================

time_history = []

position_history = []

target_history = []

position_error_history = []

q_history = []

omega_history = []

qerr_history = []

omega_error_history = []

omega_error_d_history = []

delta_history = []

tau1_history = []

tau2_history = []

tau_history = []


# ============================================================
# NUMERICAL INFORMATION
# ============================================================

def create_numeric_text(
    elapsed,
    position,
    target,
    position_error,
    q,
    q_err,
    omega,
    omega_err,
    omega_err_d,
    delta,
    tau1,
    tau2,
    tau
):

    text = (
        f"TIME\n"
        f"{elapsed:8.2f} s\n\n"

        f"POSITION\n"
        f"X = {position[0]:8.3f}\n"
        f"Y = {position[1]:8.3f}\n"
        f"Z = {position[2]:8.3f}\n\n"

        f"TARGET\n"
        f"X = {target[0]:8.3f}\n"
        f"Y = {target[1]:8.3f}\n"
        f"Z = {target[2]:8.3f}\n\n"

        f"POSITION ERROR\n"
        f"X = {position_error[0]:8.3f}\n"
        f"Y = {position_error[1]:8.3f}\n"
        f"Z = {position_error[2]:8.3f}\n\n"

        f"q = [\n"
        f" {q[0]:8.4f},\n"
        f" {q[1]:8.4f},\n"
        f" {q[2]:8.4f},\n"
        f" {q[3]:8.4f}\n"
        f"]\n\n"

        f"q_err = [\n"
        f" {q_err[0]:8.4f},\n"
        f" {q_err[1]:8.4f},\n"
        f" {q_err[2]:8.4f},\n"
        f" {q_err[3]:8.4f}\n"
        f"]\n\n"

        f"omega\n"
        f"[{omega[0]:7.3f}, "
        f"{omega[1]:7.3f}, "
        f"{omega[2]:7.3f}]\n\n"

        f"omega_err\n"
        f"[{omega_err[0]:7.3f}, "
        f"{omega_err[1]:7.3f}, "
        f"{omega_err[2]:7.3f}]\n\n"

        f"omega_err,d\n"
        f"[{omega_err_d[0]:7.3f}, "
        f"{omega_err_d[1]:7.3f}, "
        f"{omega_err_d[2]:7.3f}]\n\n"

        f"delta\n"
        f"[{delta[0]:7.3f}, "
        f"{delta[1]:7.3f}, "
        f"{delta[2]:7.3f}]\n\n"

        f"tau1\n"
        f"[{tau1[0]:7.3f}, "
        f"{tau1[1]:7.3f}, "
        f"{tau1[2]:7.3f}]\n\n"

        f"tau2\n"
        f"[{tau2[0]:7.3f}, "
        f"{tau2[1]:7.3f}, "
        f"{tau2[2]:7.3f}]\n\n"

        f"tau\n"
        f"[{tau[0]:7.3f}, "
        f"{tau[1]:7.3f}, "
        f"{tau[2]:7.3f}]"
    )

    return text


info_text = ax_info.text(
    0.02,
    0.99,
    "",
    transform=ax_info.transAxes,
    verticalalignment="top",
    horizontalalignment="left",
    fontsize=8,
    family="monospace"
)


# ============================================================
# WAYPOINT STATE
# ============================================================

current_waypoint_index = 1

waypoint_reach_distance = 0.7


# ============================================================
# INITIAL FLIGHT
# ============================================================

set_guided_mode()

if not arm_vehicle():

    print("Stopping because vehicle did not arm.")
    raise SystemExit


takeoff(TAKEOFF_ALTITUDE)


# ============================================================
# MAIN LOOP
# ============================================================

print()
print("Starting waypoint tracking...")
print()
print("Path:")
print("A → B → C → D → A")
print()

start_time = time.time()

last_plot_update = 0.0

running = True


while running:

    elapsed = time.time() - start_time

    # --------------------------------------------------------
    # TELEMETRY
    # --------------------------------------------------------

    position, q, omega = get_telemetry()

    # --------------------------------------------------------
    # CURRENT TARGET
    # --------------------------------------------------------

    target = WAYPOINTS[current_waypoint_index]

    position_error = target - position

    distance_to_target = np.linalg.norm(
        position_error
    )

    # --------------------------------------------------------
    # WAYPOINT REACHED
    # --------------------------------------------------------

    if distance_to_target < waypoint_reach_distance:

        if current_waypoint_index < len(WAYPOINTS) - 1:

            print(
                f"Reached {WAYPOINT_NAMES[current_waypoint_index]}"
            )

            current_waypoint_index += 1

            target = WAYPOINTS[current_waypoint_index]

            print(
                f"Moving to "
                f"{WAYPOINT_NAMES[current_waypoint_index]}"
            )

        else:

            print("Final waypoint reached.")
            print("Trajectory complete.")

            running = False
            continue

    # --------------------------------------------------------
    # SEND POSITION TARGET
    # --------------------------------------------------------

    send_position_target(target)

    # --------------------------------------------------------
    # BACKSTEPPING REFERENCE
    # --------------------------------------------------------

    q_ref = np.array([
        1.0,
        0.0,
        0.0,
        0.0
    ])

    omega_ref = np.zeros(3)

    omega_ref_dot = np.zeros(3)

    # --------------------------------------------------------
    # BACKSTEPPING CALCULATIONS
    # --------------------------------------------------------

    (
        q_err,
        omega_err,
        omega_err_d,
        delta,
        tau1,
        tau2,
        tau
    ) = calculate_backstepping(
        q,
        omega,
        q_ref,
        omega_ref,
        omega_ref_dot
    )

    # --------------------------------------------------------
    # STORE HISTORY
    # --------------------------------------------------------

    time_history.append(elapsed)

    position_history.append(
        position.copy()
    )

    target_history.append(
        target.copy()
    )

    position_error_history.append(
        position_error.copy()
    )

    q_history.append(
        q.copy()
    )

    omega_history.append(
        omega.copy()
    )

    qerr_history.append(
        q_err.copy()
    )

    omega_error_history.append(
        omega_err.copy()
    )

    omega_error_d_history.append(
        omega_err_d.copy()
    )

    delta_history.append(
        delta.copy()
    )

    tau1_history.append(
        tau1.copy()
    )

    tau2_history.append(
        tau2.copy()
    )

    tau_history.append(
        tau.copy()
    )

    # --------------------------------------------------------
    # LIMIT HISTORY SIZE
    # --------------------------------------------------------

    max_history = 1500

    if len(time_history) > max_history:

        time_history.pop(0)
        position_history.pop(0)
        target_history.pop(0)
        position_error_history.pop(0)
        q_history.pop(0)
        omega_history.pop(0)
        qerr_history.pop(0)
        omega_error_history.pop(0)
        omega_error_d_history.pop(0)
        delta_history.pop(0)
        tau1_history.pop(0)
        tau2_history.pop(0)
        tau_history.pop(0)

    # --------------------------------------------------------
    # UPDATE DASHBOARDS
    # --------------------------------------------------------

    if elapsed - last_plot_update > 0.05:

        last_plot_update = elapsed

        t = np.array(time_history)

        pos_hist = np.array(
            position_history
        )

        target_hist = np.array(
            target_history
        )

        pos_error_hist = np.array(
            position_error_history
        )

        q_hist = np.array(
            q_history
        )

        omega_hist = np.array(
            omega_history
        )

        qerr_hist = np.array(
            qerr_history
        )

        omega_err_hist = np.array(
            omega_error_history
        )

        omega_err_d_hist = np.array(
            omega_error_d_history
        )

        delta_hist = np.array(
            delta_history
        )

        tau1_hist = np.array(
            tau1_history
        )

        tau2_hist = np.array(
            tau2_history
        )

        tau_hist = np.array(
            tau_history
        )

        # ====================================================
        # DASHBOARD 1
        # ====================================================

        # Actual trajectory

        if len(pos_hist) > 0:

            actual_path_line.set_data(
                pos_hist[:, 0],
                pos_hist[:, 1]
            )

            actual_path_line.set_3d_properties(
                pos_hist[:, 2]
            )

            drone_point.set_data(
                [position[0]],
                [position[1]]
            )

            drone_point.set_3d_properties(
                [position[2]]
            )

            target_point.set_data(
                [target[0]],
                [target[1]]
            )

            target_point.set_3d_properties(
                [target[2]]
            )

        # Position error

        line_ex.set_data(
            t,
            pos_error_hist[:, 0]
        )

        line_ey.set_data(
            t,
            pos_error_hist[:, 1]
        )

        line_ez.set_data(
            t,
            pos_error_hist[:, 2]
        )

        ax_pos.relim()
        ax_pos.autoscale_view()

        # Quaternion

        for i in range(4):

            quat_lines[i].set_data(
                t,
                q_hist[:, i]
            )

        ax_quat.relim()
        ax_quat.autoscale_view()

        # Angular velocity

        for i in range(3):

            omega_lines[i].set_data(
                t,
                omega_hist[:, i]
            )

        ax_omega.relim()
        ax_omega.autoscale_view()

        # ====================================================
        # DASHBOARD 2
        # ====================================================

        # q error

        for i in range(3):

            qerr_lines[i].set_data(
                t,
                qerr_hist[:, i + 1]
            )

        ax_qerr.relim()
        ax_qerr.autoscale_view()

        # omega error

        for i in range(3):

            omega_error_lines[i].set_data(
                t,
                omega_err_hist[:, i]
            )

        ax_omerr.relim()
        ax_omerr.autoscale_view()

        # omega error desired

        for i in range(3):

            omega_error_d_lines[i].set_data(
                t,
                omega_err_d_hist[:, i]
            )

        ax_omerrd.relim()
        ax_omerrd.autoscale_view()

        # delta

        for i in range(3):

            delta_lines[i].set_data(
                t,
                delta_hist[:, i]
            )

        ax_delta.relim()
        ax_delta.autoscale_view()

        # tau1

        for i in range(3):

            tau1_lines[i].set_data(
                t,
                tau1_hist[:, i]
            )

        ax_tau1.relim()
        ax_tau1.autoscale_view()

        # tau2

        for i in range(3):

            tau2_lines[i].set_data(
                t,
                tau2_hist[:, i]
            )

        ax_tau2.relim()
        ax_tau2.autoscale_view()

        # tau

        for i in range(3):

            tau_lines[i].set_data(
                t,
                tau_hist[:, i]
            )

        ax_tau.relim()
        ax_tau.autoscale_view()

        # ====================================================
        # NUMERICAL PANEL
        # ====================================================

        numeric_text = create_numeric_text(
            elapsed,
            position,
            target,
            position_error,
            q,
            q_err,
            omega,
            omega_err,
            omega_err_d,
            delta,
            tau1,
            tau2,
            tau
        )

        info_text.set_text(
            numeric_text
        )

        # ====================================================
        # CURRENT WAYPOINT TITLE
        # ====================================================

        fig1.suptitle(
            "UAV Flight / Trajectory Dashboard\n"
            f"Current Target: "
            f"{WAYPOINT_NAMES[current_waypoint_index]}",
            fontsize=15
        )

        # ====================================================
        # REDRAW
        # ====================================================

        fig1.canvas.draw_idle()
        fig1.canvas.flush_events()

        fig2.canvas.draw_idle()
        fig2.canvas.flush_events()

        plt.pause(0.001)


# ============================================================
# FINISHED
# ============================================================

print()
print("Waypoint trajectory completed.")
print("A → B → C → D → A")

plt.ioff()

plt.show()
