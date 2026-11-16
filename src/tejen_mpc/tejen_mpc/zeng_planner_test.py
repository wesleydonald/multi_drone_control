def zeng_simplified_payload_trajectory(
    dt,
    waypoints=None,
    total_time=8.0,
    n_nodes=60,
    cable_length=0.6,
    payload_mass=0.1,
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

    return traj_core, "zeng_simplified_payload"
    
    
if __name__ == "__main__":
    DT = 1.0 / 30.0

    traj, name = zeng_simplified_payload_trajectory(
        DT,
        waypoints=[
            [0.0, 0.0, 0.5],
            [0.5, 0.0, 0.7],
            [1.0, 0.0, 0.7],
            [1.0, 0.5, 0.7],
        ],
        total_time=10.0,
        n_nodes=50,
        cable_length=0.6,
        payload_mass=0.1,
        alpha_max_deg=40.0,
        waypoint_tolerance=0.05,
        plot_debug=True,
    )

    print(f"Generated trajectory: {name}")
    print(f"Trajectory shape: {traj.shape}")

    # Extra plot of x, y, z versus time
    import numpy as np
    import matplotlib.pyplot as plt

    t = np.arange(traj.shape[1]) * DT

    plt.figure()
    plt.plot(t, traj[0, :], label="x")
    plt.plot(t, traj[1, :], label="y")
    plt.plot(t, traj[2, :], label="z")
    plt.xlabel("Time [s]")
    plt.ylabel("Quadrotor position [m]")
    plt.title("Generated Quadrotor Trajectory")
    plt.legend()
    plt.grid(True)
    plt.show()
