import numpy as np
from scipy.spatial.transform import Rotation as R

def takeoff_trajectory(dt, init_pose):
    steps_takeoff = 4 * 120
    steps_hover = 1 * 120

    # Takeoff from initial pose z to hover height 1.2m above
    time_space_takeoff = np.linspace(0, steps_takeoff * dt, steps_takeoff)
    x_traj_takeoff = np.ones_like(time_space_takeoff) * init_pose[0]
    y_traj_takeoff = np.ones_like(time_space_takeoff) * init_pose[1]
    z_traj_takeoff = np.linspace(init_pose[2], init_pose[2] + 1.2, steps_takeoff)

    # Small hover to smooth transition
    time_space_hover = np.linspace(0, steps_hover * dt, steps_hover)
    x_traj_hover = np.ones_like(time_space_hover) * init_pose[0]
    y_traj_hover = np.ones_like(time_space_hover) * init_pose[1]
    z_traj_hover = np.ones_like(time_space_hover) * (init_pose[2] + 1.2)

    x_traj = np.concatenate((x_traj_takeoff, x_traj_hover))
    y_traj = np.concatenate((y_traj_takeoff, y_traj_hover))
    z_traj = np.concatenate((z_traj_takeoff, z_traj_hover))
    time_space_total = np.concatenate((time_space_takeoff, time_space_hover))

    # Orientation (flat)
    roll_traj = np.zeros_like(time_space_total)
    pitch_traj = np.zeros_like(time_space_total)
    yaw_traj = np.zeros_like(time_space_total)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]

    # Velocities
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space_total)
    ay_traj = np.zeros_like(time_space_total)
    az_traj = np.zeros_like(time_space_total)

    # Control inputs placeholders
    u1 = np.zeros_like(time_space_total)
    u2 = np.zeros_like(time_space_total)
    u3 = np.zeros_like(time_space_total)
    u4 = np.zeros_like(time_space_total)

    traj = np.array([
        x_traj, y_traj, z_traj,
        qw_traj, qx_traj, qy_traj, qz_traj,
        vx_traj, vy_traj, vz_traj,
        ax_traj, ay_traj, az_traj,
        u1, u2, u3, u4
    ])
    return traj, "takeoff"


def land_trajectory(dt, init_pose):
    steps_descend = 3 * 120
    time_space_descend = np.linspace(0, steps_descend * dt, steps_descend)

    x_traj = np.ones_like(time_space_descend) * init_pose[0]
    y_traj = np.ones_like(time_space_descend) * init_pose[1]
    z_traj = np.linspace(init_pose[2], 0.1, steps_descend)  # descend to near ground

    roll_traj = np.zeros_like(time_space_descend)
    pitch_traj = np.zeros_like(time_space_descend)
    yaw_traj = np.zeros_like(time_space_descend)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]

    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space_descend)
    ay_traj = np.zeros_like(time_space_descend)
    az_traj = np.zeros_like(time_space_descend)

    u1 = np.zeros_like(time_space_descend)
    u2 = np.zeros_like(time_space_descend)
    u3 = np.zeros_like(time_space_descend)
    u4 = np.zeros_like(time_space_descend)

    traj = np.array([
        x_traj, y_traj, z_traj,
        qw_traj, qx_traj, qy_traj, qz_traj,
        vx_traj, vy_traj, vz_traj,
        ax_traj, ay_traj, az_traj,
        u1, u2, u3, u4
    ])
    return traj, "land"


def hover_trajectory(dt, init_pose):
    # Takeoff first
    takeoff_traj, _ = takeoff_trajectory(dt, init_pose)

    steps_hover = 10 * 120
    time_space = np.linspace(0, steps_hover * dt, steps_hover)

    x_traj = np.ones_like(time_space) * init_pose[0]
    y_traj = np.ones_like(time_space) * init_pose[1]
    z_traj = np.ones_like(time_space) * (init_pose[2] + 1.2)

    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]

    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space)
    ay_traj = np.zeros_like(time_space)
    az_traj = np.zeros_like(time_space)

    u1 = np.zeros_like(time_space)
    u2 = np.zeros_like(time_space)
    u3 = np.zeros_like(time_space)
    u4 = np.zeros_like(time_space)

    hover_traj = np.array([
        x_traj, y_traj, z_traj,
        qw_traj, qx_traj, qy_traj, qz_traj,
        vx_traj, vy_traj, vz_traj,
        ax_traj, ay_traj, az_traj,
        u1, u2, u3, u4
    ])

    # Land after hover
    land_traj, _ = land_trajectory(dt, hover_traj[:, -1])

    # Concatenate full trajectory: takeoff -> hover -> land
    traj = np.concatenate((takeoff_traj, hover_traj, land_traj), axis=1)
    return traj, "hover"

def circle_trajectory(dt, init_pose, center_offset_x=0.0, center_offset_y=0.0):
    # Takeoff first
    takeoff_traj, _ = takeoff_trajectory(dt, init_pose)

    # =========================
    # Circle parameters
    # =========================
    R_circle = 0.5          # radius (m)
    T_circle = 20.0         # period (s)
    n_loops = 1             # number of full circles

    # Ensure exact discretisation
    steps_per_loop = int(T_circle / dt)
    steps_circle = n_loops * steps_per_loop
    time_space = np.arange(steps_circle) * dt

    omega = 2 * np.pi / T_circle

    x0 = init_pose[0]
    y0 = init_pose[1]
    z0 = init_pose[2] + 1.2

    x_traj = x0 + R_circle * (np.cos(omega * time_space) - 1)
    y_traj = y0 + R_circle * np.sin(omega * time_space)
    z_traj = np.ones_like(time_space) * z0

    vx_traj = -R_circle * omega * np.sin(omega * time_space)
    vy_traj =  R_circle * omega * np.cos(omega * time_space)
    vz_traj = np.zeros_like(time_space)

    ax_traj = -R_circle * omega**2 * np.cos(omega * time_space)
    ay_traj = -R_circle * omega**2 * np.sin(omega * time_space)
    az_traj = np.zeros_like(time_space)

    #yaw_traj = np.arctan2(vy_traj, vx_traj)
    yaw_traj = np.zeros_like(time_space)
    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)

    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()

    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]

    u1 = np.zeros_like(time_space)
    u2 = np.zeros_like(time_space)
    u3 = np.zeros_like(time_space)
    u4 = np.zeros_like(time_space)

    circle_traj = np.array([
        x_traj, y_traj, z_traj,
        qw_traj, qx_traj, qy_traj, qz_traj,
        vx_traj, vy_traj, vz_traj,
        ax_traj, ay_traj, az_traj,
        u1, u2, u3, u4
    ])

    hover_time = 3.0  # seconds
    steps_hover = int(hover_time / dt)

    hover_traj = np.tile(circle_traj[:, -1].reshape(-1, 1), (1, steps_hover))

    land_traj, _ = land_trajectory(dt, hover_traj[:, -1])

    traj = np.concatenate((takeoff_traj, circle_traj, hover_traj, land_traj), axis=1)

    return traj, "circle"



