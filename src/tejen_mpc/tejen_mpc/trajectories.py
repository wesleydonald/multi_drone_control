import numpy as np
from scipy.spatial.transform import Rotation as R

def zeng_simplified_payload_trajectory(
    dt,
    waypoints=None,
    total_time=15.0,
    n_nodes=180,
    cable_length=0.5,
    payload_mass=0.2,
    alpha_max_deg=50.0,
    waypoint_tolerance=0.05,
    plot_debug=False,
):
    """
    Simplified Zeng-style flatness/collocation trajectory generator.

    Returns:
        traj: np.ndarray, shape (17, N_samples)
              [x, y, z, qw, qx, qy, qz,
               vx, vy, vz, ax, ay, az,
               u1, u2, u3, u4]
        name: str

    Notes:
        - This optimises payload trajectory x_L, cable direction q,
          cable tension T, and cable length l.
        - Obstacle avoidance is intentionally omitted.
        - Full attitude recovery using differential flatness is NOT implemented yet.
          For now, attitude is kept level so it can plug into your existing MPC format.
    """
    import numpy as np
    import casadi as ca
    from scipy.spatial.transform import Rotation as R

    take_off_traj, _ = takeoff_trajectory(dt)

    g = 9.81
    e3 = np.array([0.0, 0.0, 1.0])

    if waypoints is None:
        # Payload waypoints, not quadrotor waypoints
        waypoints = np.array([
            [0.0, 0.0, 0.5],
            [1.5, 0.0, 0.8],
            [1.5, 1.0, 0.8],
            [0.0, 1.0, 0.5],
        ], dtype=float)
    else:
        waypoints = np.array(waypoints, dtype=float)

    if waypoints.ndim != 2 or waypoints.shape[1] != 3:
        raise ValueError("waypoints must have shape (M, 3)")

    M = waypoints.shape[0]
    N = int(n_nodes)
    h = total_time / (N - 1)

    alpha_max = np.deg2rad(alpha_max_deg)

    # -------------------------
    # Decision variables
    # -------------------------
    opti = ca.Opti()

    # Payload flat-output derivatives
    x0 = opti.variable(3, N)  # payload position
    x1 = opti.variable(3, N)  # velocity
    x2 = opti.variable(3, N)  # acceleration
    x3 = opti.variable(3, N)  # jerk
    x4 = opti.variable(3, N)  # snap
    x5 = opti.variable(3, N)  # 5th derivative
    x6 = opti.variable(3, N)  # 6th derivative

    q = opti.variable(3, N)   # unit vector from quadrotor to payload
    T = opti.variable(1, N)   # cable tension
    l = opti.variable(1, N)   # cable length

    # -------------------------
    # Initial guess
    # -------------------------
    # Piecewise linear initial guess through waypoints
    waypoint_node_ids = np.linspace(0, N - 1, M).astype(int)

    x_guess = np.zeros((3, N))
    for seg in range(M - 1):
        i0 = waypoint_node_ids[seg]
        i1 = waypoint_node_ids[seg + 1]
        for j in range(i0, i1 + 1):
            s = 0.0 if i1 == i0 else (j - i0) / (i1 - i0)
            x_guess[:, j] = (1 - s) * waypoints[seg] + s * waypoints[seg + 1]

    opti.set_initial(x0, x_guess)
    opti.set_initial(x1, np.gradient(x_guess, h, axis=1))
    opti.set_initial(x2, 0)
    opti.set_initial(x3, 0)
    opti.set_initial(x4, 0)
    opti.set_initial(x5, 0)
    opti.set_initial(x6, 0)

    # Cable points downward from quadrotor to payload.
    # Since x_Q = x_L - l*q, choose q = [0, 0, -1] so quadrotor is above payload.
    q_guess = np.tile(np.array([[0.0], [0.0], [-1.0]]), (1, N))
    opti.set_initial(q, q_guess)
    opti.set_initial(l, cable_length)
    opti.set_initial(T, payload_mass * g)

    # -------------------------
    # Constraints
    # -------------------------
    for k in range(N):
        # Unit cable direction
        opti.subject_to(ca.sumsqr(q[:, k]) == 1.0)

        # Tension and cable length
        opti.subject_to(T[0, k] >= 0.0)
        opti.subject_to(T[0, k] <= 20.0)
        opti.subject_to(l[0, k] >= 0.0)
        opti.subject_to(l[0, k] <= cable_length)

        # Swing angle limit:
        # q points from quadrotor to payload, so vertical-down is q = -e3.
        opti.subject_to(-q[2, k] >= np.cos(alpha_max))

        # Payload dynamics:
        # m*xddot = -T*q - m*g*e3
        opti.subject_to(payload_mass * x2[:, k] == -T[0, k] * q[:, k] - payload_mass * g * ca.DM(e3))

        # Complementarity:
        # T * (l - l0) = 0.
        # This is exact but nonconvex. IPOPT may need good guesses.
        opti.subject_to(T[0, k] * (l[0, k] - cable_length) == 0.0)

    # Direct trapezoidal collocation for derivatives
    derivative_vars = [x0, x1, x2, x3, x4, x5]
    next_derivative_vars = [x1, x2, x3, x4, x5, x6]

    for k in range(N - 1):
        for xi, xidot in zip(derivative_vars, next_derivative_vars):
            opti.subject_to(
                xi[:, k + 1] - xi[:, k]
                == 0.5 * h * (xidot[:, k + 1] + xidot[:, k])
            )

    # Boundary conditions: start/end payload rest
    opti.subject_to(x0[:, 0] == waypoints[0])
    opti.subject_to(x0[:, -1] == waypoints[-1])
    opti.subject_to(x1[:, 0] == 0)
    opti.subject_to(x1[:, -1] == 0)
    opti.subject_to(x2[:, 0] == 0)
    opti.subject_to(x2[:, -1] == 0)

    # Waypoint constraints with tolerance
    for wp, node_id in zip(waypoints, waypoint_node_ids):
        opti.subject_to(x0[:, node_id] >= wp - waypoint_tolerance)
        opti.subject_to(x0[:, node_id] <= wp + waypoint_tolerance)

    # -------------------------
    # Cost function
    # -------------------------
    cost = 0

    # Smoothness / energy-like term from Zeng-style sixth derivative penalty
    for k in range(N):
        cost += 1e-3 * ca.sumsqr(x6[:, k])

    # Encourage taut cable, but do not hard-force it beyond complementarity
    for k in range(N):
        cost += 1e-2 * ca.sumsqr(l[0, k] - cable_length)

    # Penalise cable length jumps
    for k in range(N - 1):
        cost += 1e-1 * ca.sumsqr(l[0, k + 1] - l[0, k])

    # Penalise aggressive payload swing
    for k in range(N):
        cost += 1e-2 * ca.sumsqr(q[:, k] - ca.DM([0.0, 0.0, -1.0]))

    opti.minimize(cost)

    # -------------------------
    # Solver
    # -------------------------
    opts = {
        "expand": True,
        "ipopt.print_level": 0,
        "print_time": False,
        "ipopt.max_iter": 2000,
        "ipopt.tol": 1e-5,
    }
    opti.solver("ipopt", opts)

    sol = opti.solve()

    xL = np.array(sol.value(x0))
    vL = np.array(sol.value(x1))
    aL = np.array(sol.value(x2))
    q_sol = np.array(sol.value(q))
    l_sol = np.array(sol.value(l)).reshape(-1)

    # Recover quadrotor position:
    # xQ = xL - l*q
    xQ_nodes = xL - q_sol * l_sol.reshape(1, -1)

    # Resample from collocation nodes to controller dt
    t_nodes = np.linspace(0.0, total_time, N)
    t_samples = np.arange(0.0, total_time, dt)

    xQ = np.vstack([
        np.interp(t_samples, t_nodes, xQ_nodes[0, :]),
        np.interp(t_samples, t_nodes, xQ_nodes[1, :]),
        np.interp(t_samples, t_nodes, xQ_nodes[2, :]),
    ])

    vx = np.gradient(xQ, dt, axis=1)
    ax = np.gradient(vx, dt, axis=1)

    # Temporary attitude reference: level attitude.
    # Later: replace with differential-flatness attitude recovery.
    yaw = np.zeros_like(t_samples)
    roll = np.zeros_like(t_samples)
    pitch = np.zeros_like(t_samples)
    rpy = np.vstack((roll, pitch, yaw)).T
    quats = R.from_euler("xyz", rpy).as_quat()
    qx = quats[:, 0]
    qy = quats[:, 1]
    qz = quats[:, 2]
    qw = quats[:, 3]

    u1 = np.zeros_like(t_samples)
    u2 = np.zeros_like(t_samples)
    u3 = np.zeros_like(t_samples)
    u4 = np.zeros_like(t_samples)

    traj_core = np.array([
        xQ[0, :], xQ[1, :], xQ[2, :],
        qw, qx, qy, qz,
        vx[0, :], vx[1, :], vx[2, :],
        ax[0, :], ax[1, :], ax[2, :],
        u1, u2, u3, u4
    ])

    if plot_debug:
        import matplotlib.pyplot as plt

        fig = plt.figure()
        ax3d = fig.add_subplot(111, projection="3d")
        ax3d.plot(xL[0, :], xL[1, :], xL[2, :], "o-", label="payload")
        ax3d.plot(xQ_nodes[0, :], xQ_nodes[1, :], xQ_nodes[2, :], "o-", label="quadrotor")

        for k in range(0, N, max(1, N // 15)):
            ax3d.plot(
                [xQ_nodes[0, k], xL[0, k]],
                [xQ_nodes[1, k], xL[1, k]],
                [xQ_nodes[2, k], xL[2, k]],
                "k-",
                linewidth=0.7,
            )

        ax3d.set_xlabel("x [m]")
        ax3d.set_ylabel("y [m]")
        ax3d.set_zlabel("z [m]")
        ax3d.legend()
        plt.show()

    land_traj, _ = land_trajectory(dt, take_off_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

    
    traj = np.concatenate((take_off_traj,traj_core, land_traj,zeros), axis=1)

    return traj, "zeng_simplified_payload"

def takeoff_trajectory(dt, z0=0.63, zf=1.0):

    steps_takeoff = 4 * 30  # 1 second of takeoff at 30 Hz
    steps_hover = 4 * 30    # 2 seconds of hover at 30 Hz

    # Takeoff phase
    time_space_takeoff = np.linspace(0, steps_takeoff * dt, steps_takeoff)
    x_traj_takeoff = np.zeros_like(time_space_takeoff)
    y_traj_takeoff = np.zeros_like(time_space_takeoff)
    # z_traj_takeoff = np.linspace(0.0, 1.0, steps_takeoff)
    z_traj_takeoff = np.linspace(z0, zf, steps_takeoff)

    # Hover phase
    time_space_hover = np.linspace(0, steps_hover * dt, steps_hover)
    x_traj_hover = np.zeros_like(time_space_hover)
    y_traj_hover = np.zeros_like(time_space_hover)
    z_traj_hover = np.ones_like(time_space_hover) * 1.0

    # Concatenate phases
    x_traj = np.concatenate((x_traj_takeoff, x_traj_hover))
    y_traj = np.concatenate((y_traj_takeoff, y_traj_hover))
    z_traj = np.concatenate((z_traj_takeoff, z_traj_hover))
    time_space_total = np.concatenate((time_space_takeoff, time_space_hover))

    roll_traj = np.zeros_like(time_space_total)
    pitch_traj = np.zeros_like(time_space_total)
    yaw_traj = np.zeros_like(time_space_total)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space_total)
    ay_traj = np.zeros_like(time_space_total)
    az_traj = np.zeros_like(time_space_total)
    u1 = np.zeros_like(time_space_total)
    u2 = np.zeros_like(time_space_total)
    u3 = np.zeros_like(time_space_total)
    u4 = np.zeros_like(time_space_total)
    traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                     vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])
    return traj, "takeoff"
    

def land_trajectory(dt, init_pose):
    steps_move_back = 3 * 30  # 5 seconds of takeoff at 30 Hz
    steps_descend = 3 * 30  # 5 seconds of takeoff at 30 Hz
    time_space_move_back = np.linspace(0, steps_move_back * dt, steps_move_back)
    time_space_descend = np.linspace(0, steps_descend * dt, steps_descend)

    time_space_total = np.concatenate((time_space_move_back, time_space_descend))

    x_traj_move_back = np.linspace(init_pose[0], 0, steps_move_back)  # Move back 1 meter
    x_traj_descend = np.linspace(0, 0, steps_descend)  # Move back 1 meter
    x_traj = np.concatenate((x_traj_move_back, x_traj_descend))

    y_traj_move_back = np.linspace(init_pose[1], 0, steps_move_back)  # Move back 1 meter
    y_traj_descend = np.linspace(0, 0, steps_descend)  # Move back 1 meter
    y_traj = np.concatenate((y_traj_move_back, y_traj_descend))

    z_traj_move_back = np.linspace(init_pose[2], 1.0, steps_move_back)  # Move back 1 meter
    z_traj_descend  = np.linspace(1.0, 0.1, steps_descend)  # Move back 1 meter
    z_traj = np.concatenate((z_traj_move_back, z_traj_descend))

    roll_traj = np.zeros_like(time_space_total)
    pitch_traj = np.zeros_like(time_space_total)
    yaw_traj = np.zeros_like(time_space_total)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space_total)
    ay_traj = np.zeros_like(time_space_total)
    az_traj = np.zeros_like(time_space_total)
    u1 = np.zeros_like(time_space_total)
    u2 = np.zeros_like(time_space_total)
    u3 = np.zeros_like(time_space_total)
    u4 = np.zeros_like(time_space_total)
    traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                     vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])
    return traj, "land"



def hover_trajectory(dt):
    # take_off_traj, _ = takeoff_trajectory(dt)
    take_off_traj, _ = takeoff_trajectory(dt, z0=0.63)

    steps = 5 * 30  # 10 seconds of hover at 30 Hz
    time_space = np.linspace(0, steps * dt, steps)
    x_traj = np.zeros_like(time_space) #np.zeros_like(time_space)
    y_traj = np.zeros_like(time_space) #np.zeros_like(time_space) # 0.0 + 0.5 * np.sin(1.0 * np.pi * time_space / 5.0)
    z_traj = 1.5 * np.ones_like(time_space)#1.2 * np.ones_like(time_space) #1.5 + 0.5 * np.sin(1.5 * np.pi * time_space / 5.0)    #

    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
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
    hover_traj =  np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                 vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])

    land_traj, _ = land_trajectory(dt, take_off_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

    traj = np.concatenate((take_off_traj,hover_traj, land_traj,zeros), axis=1)
    return traj, "hover"




def z_sin_trajectory(dt):
    trajectory_name = "ZSINE"
    take_off_traj, _ = takeoff_trajectory(dt)

    # Phase 1: Hover for 20 seconds
    steps_hover = 20 * 30  # 20 seconds of hover at 30 Hz
    time_space_hover = np.linspace(0, steps_hover * dt, steps_hover)
    x_traj_hover = np.zeros_like(time_space_hover)
    y_traj_hover = np.zeros_like(time_space_hover)
    z_traj_hover = 1.5 * np.ones_like(time_space_hover)  # Constant hover at 1.5m
    
    # Phase 2: Variable frequency sinusoidal motion for 40 seconds
    steps_sin = 40 * 30  # 40 seconds of sinusoidal motion at 30 Hz
    time_space_sin = np.linspace(0, steps_sin * dt, steps_sin)
    
    # Frequency increases linearly from 1.0 to 3.0 Hz over 40 seconds
    freq_start = 0.1  # Hz
    freq_end = 0.5    # Hz
    frequencies = np.linspace(freq_start, freq_end, steps_sin)
    
    # Calculate the instantaneous phase by integrating frequency
    # phase(t) = 2π * ∫frequency(τ)dτ from 0 to t
    phase = 2 * np.pi * np.cumsum(frequencies) * dt
    
    x_traj_sin = np.zeros_like(time_space_sin)
    y_traj_sin = np.zeros_like(time_space_sin)
    z_traj_sin = 1.5 + 1.0 * np.sin(phase)  # 1m amplitude around 1.5m center
    
    # Combine both phases
    x_traj = np.concatenate((x_traj_hover, x_traj_sin))
    y_traj = np.concatenate((y_traj_hover, y_traj_sin))
    z_traj = np.concatenate((z_traj_hover, z_traj_sin))
    time_space = np.concatenate((time_space_hover, time_space_sin))

    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
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
    hover_traj =  np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                 vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])

    land_traj, _ = land_trajectory(dt, hover_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

    traj = np.concatenate((take_off_traj,hover_traj, land_traj,zeros), axis=1)
    return traj, "z_sin"




def xyz_sine_trajectory(dt):
    take_off_traj, _ = takeoff_trajectory(dt)

    steps = 20 * 30  # 10 seconds of hover at 30 Hz
    time_space = np.linspace(0, steps * dt, steps)
    x_traj = 0.0 + 1.0 * np.sin(1.0 * np.pi * time_space / 5.0) #np.zeros_like(time_space)
    y_traj = 0.0 + 1.0 * np.sin(0.5 * np.pi * time_space / 5.0) #np.zeros_like(time_space) # 0.0 + 0.5 * np.sin(1.0 * np.pi * time_space / 5.0)
    z_traj = 1.5 + 0.5 * np.sin(2.0 * np.pi * time_space / 5.0)#1.2 * np.ones_like(time_space) #1.5 + 0.5 * np.sin(1.5 * np.pi * time_space / 5.0)    #

    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
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
    hover_traj =  np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                 vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])

    land_traj, _ = land_trajectory(dt, take_off_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

    traj = np.concatenate((take_off_traj,hover_traj, land_traj,zeros), axis=1)
    return traj, "xyz_sine"


def fast_xyz_sine_trajectory(dt):
    take_off_traj, _ = takeoff_trajectory(dt)

    steps = 20 * 30  # 10 seconds of hover at 30 Hz
    time_space = np.linspace(0, steps * dt, steps)
    y_traj = 0.0 + 0.75 * np.sin(4.0 * np.pi * time_space / 5.0) #np.zeros_like(time_space)
    x_traj = np.zeros_like(time_space)
    z_traj = 1.5 + 0.75 * np.sin(1.0 * np.pi * time_space / 5.0)#1.2 * np.ones_like(time_space) #1.5 + 0.5 * np.sin(1.5 * np.pi * time_space / 5.0)    #

    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
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
    hover_traj =  np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                 vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])

    land_traj, _ = land_trajectory(dt, take_off_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

    traj = np.concatenate((take_off_traj,hover_traj, land_traj,zeros), axis=1)
    return traj, "fast_xyz_sine"





def fence_trajectory(dt):
    take_off_traj, _ = takeoff_trajectory(dt)

    steps = 20 * 30  # 10 seconds of hover at 30 Hz
    time_space = np.linspace(0, steps * dt, steps)
    x_traj = 0.0 + 1.0 * np.sin(1.0 * np.pi * time_space / 5.0) #np.zeros_like(time_space)
    y_traj = 0.0 + 1.0 * np.sin(1.0 * np.pi * time_space / 5.0) #np.zeros_like(time_space) # 0.0 + 0.5 * np.sin(1.0 * np.pi * time_space / 5.0)
    z_traj = 1.5 + 0.5 * np.sin(6.0 * np.pi * time_space / 5.0)#1.2 * np.ones_like(time_space) #1.5 + 0.5 * np.sin(1.5 * np.pi * time_space / 5.0)    #

    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
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
    hover_traj =  np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                 vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])

    land_traj, _ = land_trajectory(dt, take_off_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

    traj = np.concatenate((take_off_traj,hover_traj, land_traj,zeros), axis=1)
    return traj, "fence"



def circle_trajectory(dt):
    take_off_traj, _ = takeoff_trajectory(dt)

    # Transition from hover to circle start (3 seconds)
    steps_transition = 5 * 30  # 3 seconds at 30 Hz
    time_space_transition = np.linspace(0, steps_transition * dt, steps_transition)
    
    # Circle parameters
    radius = 1.25  # 1 meter
    period = 4  # seconds per rotation
    omega = 2 * np.pi / period  # angular velocity (rad/s)
    
    # Get the final position from takeoff (should be 0, 0, 1.0)
    takeoff_end_x = take_off_traj[0, -1]  # x position
    takeoff_end_y = take_off_traj[1, -1]  # y position
    takeoff_end_z = take_off_traj[2, -1]  # z position
    
    # Transition trajectory from takeoff end to circle start
    x_traj_transition = np.linspace(takeoff_end_x, radius, steps_transition)  # Move to circle start
    y_traj_transition = np.linspace(takeoff_end_y, 0.0, steps_transition)     # Move to y=0
    z_traj_transition = np.linspace(takeoff_end_z, 1.5, steps_transition)     # Move to circle altitude

    steps = 30 * 30  # 40 seconds at 30 Hz
    time_space = np.linspace(0, steps * dt, steps)

    x_traj = radius * np.cos(omega * time_space)
    y_traj = radius * np.sin(omega * time_space)
    z_traj = 1.5 * np.ones_like(time_space)  # constant altitude

    # Combine transition and circle time spaces for trajectory calculations
    time_space_combined = np.concatenate((time_space_transition, time_space))
    x_traj_combined = np.concatenate((x_traj_transition, x_traj))
    y_traj_combined = np.concatenate((y_traj_transition, y_traj))
    z_traj_combined = np.concatenate((z_traj_transition, z_traj))

    roll_traj = np.zeros_like(time_space_combined)
    pitch_traj = np.zeros_like(time_space_combined)
    yaw_traj = np.zeros_like(time_space_combined)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj_combined, dt)
    vy_traj = np.gradient(y_traj_combined, dt)
    vz_traj = np.gradient(z_traj_combined, dt)
    ax_traj = np.zeros_like(time_space_combined)
    ay_traj = np.zeros_like(time_space_combined)
    az_traj = np.zeros_like(time_space_combined)
    u1 = np.zeros_like(time_space_combined)
    u2 = np.zeros_like(time_space_combined)
    u3 = np.zeros_like(time_space_combined)
    u4 = np.zeros_like(time_space_combined)
    hover_traj = np.array([x_traj_combined, y_traj_combined, z_traj_combined, qw_traj, qx_traj, qy_traj, qz_traj,
                           vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])

    land_traj, _ = land_trajectory(dt, hover_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

    traj = np.concatenate((take_off_traj, hover_traj, land_traj, zeros), axis=1)
    return traj, "circle"






def figure8_zsine_trajectory(dt, duration=10.0, repeats=3, z_amplitude=0.5, z_offset=1.5):
    """
    Generates a repeated figure-8 trajectory (each loop duration, repeated N times) on a zsine slope.
    Args:
        dt: timestep (s)
        duration: duration of one figure-8 (s)
        repeats: number of times to repeat the figure-8
        z_amplitude: amplitude of zsine (m)
        z_offset: base z height (m)
    Returns:
        traj: np.ndarray shape (16, N)
    """
    take_off_traj, _ = takeoff_trajectory(dt)

    # Build one figure-8 segment
    steps = int(duration / dt)
    t = np.linspace(0, duration, steps)
    R_ = 1.0  # 1m radius
    y_seg = R_ * np.sin(2 * np.pi * t / duration)
    x_seg = R_ * np.sin(2 * np.pi * t / duration) * np.cos(2 * np.pi * t / duration)
    z_seg = z_offset + z_amplitude * np.sin(2 * np.pi * t / duration)

    vx = np.gradient(x_seg, dt)
    vy = np.gradient(y_seg, dt)
    vz = np.gradient(z_seg, dt)
    yaw_seg = np.zeros_like(vx)
    roll_seg = np.zeros_like(yaw_seg)
    pitch_seg = np.zeros_like(yaw_seg)
    rpy_seg = np.vstack((roll_seg, pitch_seg, yaw_seg)).T
    quats = R.from_euler('xyz', rpy_seg).as_quat()
    qx_seg = quats[:, 0]
    qy_seg = quats[:, 1]
    qz_seg = quats[:, 2]
    qw_seg = quats[:, 3]
    ax_seg = np.zeros_like(x_seg)
    ay_seg = np.zeros_like(y_seg)
    az_seg = np.zeros_like(z_seg)
    u1 = np.zeros_like(x_seg)
    u2 = np.zeros_like(x_seg)
    u3 = np.zeros_like(x_seg)
    u4 = np.zeros_like(x_seg)

    # Repeat the segment
    x_traj = np.tile(x_seg, repeats)
    y_traj = np.tile(y_seg, repeats)
    z_traj = np.tile(z_seg, repeats)
    qw_traj = np.tile(qw_seg, repeats)
    qx_traj = np.tile(qx_seg, repeats)
    qy_traj = np.tile(qy_seg, repeats)
    qz_traj = np.tile(qz_seg, repeats)
    vx_traj = np.tile(vx, repeats)
    vy_traj = np.tile(vy, repeats)
    vz_traj = np.tile(vz, repeats)
    ax_traj = np.tile(ax_seg, repeats)
    ay_traj = np.tile(ay_seg, repeats)
    az_traj = np.tile(az_seg, repeats)
    u1 = np.tile(u1, repeats)
    u2 = np.tile(u2, repeats)
    u3 = np.tile(u3, repeats)
    u4 = np.tile(u4, repeats)

    hover_traj = np.array([
        x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
        vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4
    ])

    land_traj, _ = land_trajectory(dt, hover_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

    traj = np.concatenate((take_off_traj, hover_traj, land_traj, zeros), axis=1)
    return traj, "figure8_zsine"




def power_loop_trajectory(dt):
    take_off_traj, _ = takeoff_trajectory(dt)

    # Phase lengths (Hz assumed 30 via your code)
    hz = 30
    steps_runup        = 1 * hz   # 1 s run-up (comment said 3 s previously; leaving as code dictates)
    steps_loop         = 3 * hz   # 3 s loop
    steps_rundown      = 3 * hz   # 3 s run-down
    steps_final_hover  = 2 * hz   # 2 s hover

    time_space_runup        = np.linspace(0, steps_runup * dt, steps_runup, endpoint=False)
    time_space_loop         = np.linspace(0, steps_loop * dt, steps_loop, endpoint=False)
    time_space_rundown      = np.linspace(0, steps_rundown * dt, steps_rundown, endpoint=False)
    time_space_final_hover  = np.linspace(0, steps_final_hover * dt, steps_final_hover, endpoint=False)

    # Takeoff end position
    takeoff_end_x = take_off_traj[0, -1]
    takeoff_end_y = take_off_traj[1, -1]
    takeoff_end_z = take_off_traj[2, -1]

    # Loop geometry
    loop_radius   = 1.1
    loop_center_x = 1.0
    loop_center_z = takeoff_end_z + loop_radius

    # --- KEYFRAMES (θ in DEGREES from bottom) ---
    # Feel free to tweak these. They aim to keep your original “feel”:
    # bottom ~ level, right ~ +30° nose up, top ~ -90° (center-pointing),
    # left ~ -30°, back to bottom ~ level.
    keyframes = [
        {'theta_deg':   0.0, 'pitch_deg':  30.0},
        {'theta_deg':  90.0, 'pitch_deg':  0.0},
        {'theta_deg': 180.0, 'pitch_deg': -100.0},
        {'theta_deg': 270.0, 'pitch_deg': -300.0},
        {'theta_deg': 360.0, 'pitch_deg':   -400.0},
    ]

    # --- Phase 1: Run-up to bottom entry point ---
    x_runup_start = takeoff_end_x
    x_runup_end   = loop_center_x
    z_runup_start = takeoff_end_z
    z_runup_end   = loop_center_z - loop_radius  # bottom of loop

    x_traj_runup = np.linspace(x_runup_start, x_runup_end, steps_runup, endpoint=False)
    y_traj_runup = np.full_like(time_space_runup, takeoff_end_y)
    z_traj_runup = np.linspace(z_runup_start, z_runup_end, steps_runup, endpoint=False)

    # --- Phase 2: Power loop (θ from 0° -> 360°) ---
    theta_deg_loop = np.linspace(0.0, 360.0, steps_loop, endpoint=False)

    # Convert bottom-based degrees (0° at bottom) to standard circle param φ (0° at +x/right).
    # Bottom corresponds to φ=270°, so φ = θ + 270 (mod 360).
    phi_deg = (theta_deg_loop + 270.0) % 360.0
    phi_rad = np.deg2rad(phi_deg)

    x_traj_loop = loop_center_x + loop_radius * np.cos(phi_rad)
    y_traj_loop = np.full_like(time_space_loop, takeoff_end_y)
    z_traj_loop = loop_center_z + loop_radius * np.sin(phi_rad)

    # --- Phase 3: Run-down back to hover position ---
    x_rundown_start = loop_center_x
    x_rundown_end   = takeoff_end_x
    z_rundown_start = loop_center_z - loop_radius
    z_rundown_end   = takeoff_end_z

    x_traj_rundown = np.linspace(x_rundown_start, x_rundown_end, steps_rundown, endpoint=False)
    y_traj_rundown = np.full_like(time_space_rundown, takeoff_end_y)
    z_traj_rundown = np.linspace(z_rundown_start, z_rundown_end, steps_rundown, endpoint=False)

    # --- Phase 4: Final hover ---
    x_traj_final_hover = np.full_like(time_space_final_hover, takeoff_end_x)
    y_traj_final_hover = np.full_like(time_space_final_hover, takeoff_end_y)
    z_traj_final_hover = np.full_like(time_space_final_hover, takeoff_end_z)

    # Combine position trajectories
    x_traj_combined = np.concatenate((x_traj_runup, x_traj_loop, x_traj_rundown, x_traj_final_hover))
    y_traj_combined = np.concatenate((y_traj_runup, y_traj_loop, y_traj_rundown, y_traj_final_hover))
    z_traj_combined = np.concatenate((z_traj_runup, z_traj_loop, z_traj_rundown, z_traj_final_hover))
    time_space_combined = np.concatenate((time_space_runup, time_space_loop, time_space_rundown, time_space_final_hover))

    # --- Orientation (roll=0, yaw=0; pitch from keyframes) ---
    roll_traj = np.zeros_like(time_space_combined)
    yaw_traj  = np.zeros_like(time_space_combined)
    pitch_traj = np.zeros_like(time_space_combined)

    # Phase 1: pitch up slightly for entry
    pitch_start = 0.0
    pitch_end   = np.deg2rad(20.0)
    pitch_traj[:steps_runup] = np.linspace(pitch_start, pitch_end, steps_runup, endpoint=False)

    # Phase 2: pitch from keyframes along θ
    loop_start_idx = steps_runup
    loop_end_idx   = steps_runup + steps_loop

    # Sort and unwrap keyframes in degrees [0,360], allow wrap-around interpolation
    kf_thetas = np.array([kf['theta_deg'] for kf in keyframes], dtype=float)
    kf_pitchs = np.deg2rad(np.array([kf['pitch_deg'] for kf in keyframes], dtype=float))
    sort_idx = np.argsort(kf_thetas)
    kf_thetas = kf_thetas[sort_idx]
    kf_pitchs = kf_pitchs[sort_idx]

    # For fast lookup, pre-build arrays for piecewise-linear interpolation with wrap:
    # Extend by one keyframe at +360° for continuous interpolation at the end.
    kf_thetas_ext = np.concatenate((kf_thetas, [kf_thetas[0] + 360.0]))
    kf_pitchs_ext = np.concatenate((kf_pitchs, [kf_pitchs[0]]))

    def interp_pitch_deg_from_bottom(theta_deg):
        """Linear interpolate pitch (radians) from keyframes at θ in degrees [0,360)."""
        # Find segment
        idx = np.searchsorted(kf_thetas_ext, theta_deg, side='right') - 1
        idx = np.clip(idx, 0, len(kf_thetas_ext) - 2)
        t1, p1 = kf_thetas_ext[idx], kf_pitchs_ext[idx]
        t2, p2 = kf_thetas_ext[idx + 1], kf_pitchs_ext[idx + 1]
        if t2 == t1:
            return p1
        w = (theta_deg - t1) / (t2 - t1)
        return p1 + w * (p2 - p1)

    for i, th in enumerate(theta_deg_loop):
        pitch_traj[loop_start_idx + i] = interp_pitch_deg_from_bottom(th)

    # Phase 3: pitch based on path slope (keeps a “natural” rundown)
    vx_runup_rundown = np.concatenate((
        np.gradient(x_traj_runup, dt),
        np.gradient(x_traj_rundown, dt),
        np.gradient(x_traj_final_hover, dt)
    ))
    vz_runup_rundown = np.concatenate((
        np.gradient(z_traj_runup, dt),
        np.gradient(z_traj_rundown, dt),
        np.gradient(z_traj_final_hover, dt)
    ))

    rundown_start_idx = loop_end_idx
    rundown_end_idx   = rundown_start_idx + steps_rundown
    eps = 1e-2
    pitch_traj[rundown_start_idx:rundown_end_idx] = np.arctan2(
        vz_runup_rundown[steps_runup:steps_runup + steps_rundown],
        np.clip(np.abs(vx_runup_rundown[steps_runup:steps_runup + steps_rundown]), eps, None)
    )

    # Phase 4: level
    pitch_traj[rundown_end_idx:] = 0.0

    # Convert RPY to quaternions
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quats = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj, qy_traj, qz_traj, qw_traj = quats[:,0], quats[:,1], quats[:,2], quats[:,3]

    # Velocities
    vx_traj = np.gradient(x_traj_combined, dt)
    vy_traj = np.gradient(y_traj_combined, dt)
    vz_traj = np.gradient(z_traj_combined, dt)

    # Angular rates (time-derivatives of RPY)
    roll_rate  = np.gradient(roll_traj, dt)
    pitch_rate = np.gradient(pitch_traj, dt)
    yaw_rate   = np.gradient(yaw_traj, dt)

    ax_traj = roll_rate
    ay_traj = pitch_rate
    az_traj = yaw_rate

    # Placeholders for thrust/inputs
    u1 = np.zeros_like(time_space_combined)
    u2 = np.zeros_like(time_space_combined)
    u3 = np.zeros_like(time_space_combined)
    u4 = np.zeros_like(time_space_combined)

    power_loop_traj = np.array([
        x_traj_combined, y_traj_combined, z_traj_combined,
        qw_traj, qx_traj, qy_traj, qz_traj,
        vx_traj, vy_traj, vz_traj,
        ax_traj, ay_traj, az_traj,
        u1, u2, u3, u4
    ])

    land_traj, _ = land_trajectory(dt, power_loop_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * hz))

    traj = np.concatenate((take_off_traj, power_loop_traj, land_traj, zeros), axis=1)
    return traj, "power_loop"

