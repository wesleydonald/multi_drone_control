"""
geometry.py
-----------
Small, stateless geometry helpers shared by the planner node and the OCP solver
wrapper: quaternion->rotation, a Rodrigues "align a onto b" rotation, the attach
ring / nominal cable directions built from the fleet size, and the azimuth-based
drone<->slot assignment. All pure functions of their arguments (numpy only), so
they carry no ROS or solver state and are trivially testable.
"""
import itertools
import re

import numpy as np


def quat_to_rot_np(q):
    """Rotation matrix from quaternion q = [w, x, y, z] (numpy)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def yaw_from_quat(q):
    """Yaw (rotation about world z) of quaternion q = [w, x, y, z]. The attach ring
    and the load attitude reference are both defined about world z, so this is the
    only component of the load's measured attitude they need."""
    w, x, y, z = q
    return float(np.arctan2(2.0 * (w * z + x * y),
                            1.0 - 2.0 * (y * y + z * z)))


def quat_same_hemisphere(q, q_prev):
    """q, sign-flipped if needed so that q . q_prev >= 0. The rig mocap sends w >= 0,
    so a yaw crossing 180 deg flips the sign of the whole quaternion (same rotation)."""
    q = np.asarray(q, float)
    if q_prev is not None and float(np.dot(q, q_prev)) < 0.0:
        return -q
    return q


def yaw_quat(psi):
    """Quaternion [w, x, y, z] of a pure yaw about world z."""
    return np.array([np.cos(0.5 * psi), 0.0, 0.0, np.sin(0.5 * psi)])


def rot_z(psi):
    """Rotation matrix of a pure yaw about world z."""
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_align(a, b):
    """Rotation matrix mapping unit vector a onto unit vector b (Rodrigues). Used to
    tilt the nominal cable formation toward the effective-gravity direction."""
    a = a / max(np.linalg.norm(a), 1e-12)
    b = b / max(np.linalg.norm(b), 1e-12)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    s = float(np.linalg.norm(v))
    if s < 1e-9:                         # already aligned (c~+1) or opposite (c~-1)
        return np.eye(3) if c > 0 else -np.eye(3)
    vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))


def thrust_attitude(acc, yaw=0.0):
    """Rotation of a drone whose body z points along the thrust acceleration `acc`,
    at heading `yaw` (the attitude a tracker flies to produce that thrust)."""
    a = np.asarray(acc, float)
    na = float(np.linalg.norm(a))
    zb = a / na if na > 1e-9 else np.array([0.0, 0.0, 1.0])
    xc = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    yb = np.cross(zb, xc)
    ny = float(np.linalg.norm(yb))
    if ny < 1e-9:                        # thrust horizontal along the heading
        return rot_z(yaw)
    yb /= ny
    return np.column_stack([np.cross(yb, zb), yb, zb])


def centre_from_pivot(pivot, acc, yaw, pivot_offset):
    """Drone-centre position for a rod pivot at `pivot`: the pivot sits at
    centre + R @ pivot_offset, with R the attitude that flies thrust `acc`."""
    b = np.asarray(pivot_offset, float)
    if not b.any():
        return np.asarray(pivot, float)
    return np.asarray(pivot, float) - thrust_attitude(acc, yaw) @ b


def parse_azimuths_deg(spec):
    """'0,90,180' / [0, 90, 180] / '' -> list of azimuths (deg) or None (= even ring).
    The rig's rim magnets sit where they were placed, not on an even ring; this is how
    a launch says where (clock face: 3 o'clock = 0, 12 = 90, 9 = 180, 6 = 270)."""
    if spec is None:
        return None
    if isinstance(spec, str):
        # 'even' / 'none' are explicit spellings of the even ring, for launch args and
        # configs where an empty string cannot be passed (ros2 launch k:= with no value).
        if spec.strip().lower() in ('even', 'none', 'auto'):
            return None
        spec = [t for t in re.split(r'[,\s]+', spec.strip()) if t]
    vals = [float(v) for v in spec]
    return vals or None


def attach_points(n, attach_radius, attach_z, azimuths_deg=None):
    """The n cable attach points on the payload body: a ring of radius attach_radius
    at height attach_z above the load CoG. Azimuths (load frame) are the given list
    (deg, must have n entries) or the even ring 2*pi*k/n."""
    az = parse_azimuths_deg(azimuths_deg)
    if az is not None:
        if len(az) != n:
            raise ValueError(f'attach azimuths {az} do not match n={n}')
        th = [np.deg2rad(a) for a in az]
    else:
        th = [2 * np.pi * k / n for k in range(n)]
    return [np.array([attach_radius * np.cos(t), attach_radius * np.sin(t), attach_z])
            for t in th]


def nominal_cable_dirs(rho, elev_deg=45.0):
    """Nominal drone->load cable directions at hover: one per attach point, pointing
    down-and-inward at elev_deg above horizontal (the flatness s_i reference, tilted
    per node by the load acceleration in the OCP reference)."""
    phi = np.deg2rad(elev_deg)
    dirs = []
    for r in rho:
        th = np.arctan2(r[1], r[0])
        dirs.append(np.array([-np.cos(th) * np.cos(phi),
                              -np.sin(th) * np.cos(phi),
                              -np.sin(phi)]))
    return dirs


def apex_direction(rho_ring, s_ring, rho_new):
    """Nominal cable direction for a cable joining an existing ring at rho_new.

    With every cable fixed in direction, a static balance can load a cable only if its
    line of action passes through the point where the others' lines meet (their apex;
    for 45-deg cables on a 0.25 m ring it sits 0.25 m below the attach plane). A newcomer
    given "45 deg from its own attach point" misses it whenever it welds off the ring
    radius, and the balance then assigns it zero tension (R0575/R0577: welded at r 0.28,
    planned tension 0.1 N, incumbents kept the three-drone split). Returns the unit
    drone->load direction whose line passes through the least-squares apex of the ring."""
    A = np.zeros((3, 3))
    b = np.zeros(3)
    for r, d in zip(rho_ring, s_ring):
        d = np.asarray(d, float) / np.linalg.norm(d)
        P = np.eye(3) - np.outer(d, d)
        A += P
        b += P @ np.asarray(r, float)
    apex = np.linalg.solve(A, b)
    v = apex - np.asarray(rho_new, float)
    return v / np.linalg.norm(v)


def balanced_tensions(rho, s_dirs, load_mass, g=9.81, t_min=0.1, rod_mass=0.0, rod_lam=0.0):
    """Nominal cable tensions that hold the load LEVEL at hover for this attach layout.

    Equal tensions balance an even ring only. On an uneven ring (30/90/150/270: the layout
    that leaves a level triple after a detach) equal references carry a net moment, and
    the OCP either flies the load tilted (SIL R0498, 12-27 deg) or flips between vertex
    tension solutions and never lifts (R0497). Solve the static balance
        sum_i t_i s_i = -m g z_hat,   sum_i rho_i x (t_i s_i) = 0
    for the tensions closest (least squares) to the equal split, then floor at t_min.
    Even rings return the equal split unchanged."""
    n = len(rho)
    s = [np.asarray(v, float).reshape(3) for v in s_dirs]
    r = [np.asarray(v, float).reshape(3) for v in rho]
    A = np.zeros((6, n))
    for i in range(n):
        A[0:3, i] = s[i]
        A[3:6, i] = np.cross(r[i], s[i])
    b = np.array([0.0, 0.0, -float(load_mass) * g, 0.0, 0.0, 0.0])
    if rod_mass > 0.0:
        # massive rods (option F): each hands the load end its weight share, which the
        # tensions must also carry (LoadCableDynamics.rod_on_load)
        G = np.array([0.0, 0.0, -g])
        for i in range(n):
            R_i = rod_mass * (rod_lam * float(G @ s[i]) * s[i] + (1.0 - rod_lam) * G)
            b[0:3] += R_i
            b[3:6] += np.cross(r[i], R_i)
    sz = -np.mean([v[2] for v in s])
    t_eq = np.full(n, float(load_mass) * g / max(n * sz, 1e-9))
    t = t_eq + np.linalg.pinv(A) @ (b - A @ t_eq)
    return [float(max(v, t_min)) for v in t]


def azimuth_slot_assignment(drone_pos, load_xy, n, load_yaw=0.0, slot_az=None):
    """Match each physical drone to the nearest nominal azimuth slot around the load,
    so the drones can be placed in the ring in any order. The slots are equally
    spaced (slot i at 2*pi*i/n), so the optimal assignment is a cyclic rotation of
    the drones sorted by their measured azimuth; we pick the shift that minimises the
    total angular error. Returns the slot->drone permutation (relabels I/O only).

    The slots are the attach points rho_i, which live in the LOAD frame -- slot i
    sits at world azimuth 2*pi*i/n + load_yaw. So the measured azimuths must be
    de-rotated by load_yaw before matching, or a payload placed at any yaw past
    half a slot pitch (60 deg for 3 drones) matches every drone to its NEIGHBOUR's
    attach point. That is not a cosmetic relabel: build_x_init then reads each
    cable's direction from the wrong attach point, the implied cable length comes
    out ~25% longer than the modelled cable_len, and the first solve goes QP
    infeasible (acados status 4) -- the fleet never leaves the ground. Defaults to
    0.0 so a caller with a world-aligned payload is unaffected."""
    lp = load_xy
    az = np.array([np.arctan2(drone_pos[j][1] - lp[1],
                              drone_pos[j][0] - lp[0]) - load_yaw
                   for j in range(n)])
    az = (az + np.pi) % (2.0 * np.pi) - np.pi        # keep the sort well defined
    order = list(np.argsort(az))                       # drones CCW by azimuth
    if slot_az is None:
        slot_az = np.array([2.0 * np.pi * i / n for i in range(n)])
    slot_az = np.asarray(slot_az, float)

    def _cost(perm):
        return sum(((az[perm[i]] - slot_az[i] + np.pi) % (2.0 * np.pi) - np.pi) ** 2
                   for i in range(n))
    # Uneven slots (rim magnets placed by hand) break the cyclic-rotation shortcut, so
    # for the fleet sizes we fly just try every permutation.
    cands = (itertools.permutations(range(n)) if n <= 6
             else [[order[(k + s) % n] for k in range(n)] for s in range(n)])
    best, best_cost = list(range(n)), np.inf
    for perm in cands:
        perm = list(perm)
        c = _cost(perm)
        if c < best_cost:
            best_cost, best = c, perm
    return best


PLATE_PITCH_DEG = 30.0      # M2A ring: 12 magnet plates, plate 0 on the ring body's +x
SLOT_OFFSET_WARN_DEG = 10.0


def slot_azimuth_errors(drone_pos, load_xy, load_yaw, slot2drone, slot_az):
    """Per OCP slot i: (drone, error_deg, plate). error_deg is the drone's azimuth about
    the load, in the load frame, minus slot i's modelled azimuth, wrapped to +-180."""
    out = []
    for i, d in enumerate(slot2drone):
        az = np.arctan2(drone_pos[d][1] - load_xy[1], drone_pos[d][0] - load_xy[0]) - load_yaw
        err = (np.degrees(az - slot_az[i]) + 180.0) % 360.0 - 180.0
        plate = int(round(np.degrees(slot_az[i]) / PLATE_PITCH_DEG)) % int(360 / PLATE_PITCH_DEG)
        out.append((int(d), float(err), plate))
    return out


def slot_offset_warnings(errors, tol_deg=SLOT_OFFSET_WARN_DEG):
    """Operator lines for the slots whose |error| exceeds tol_deg (see slot_azimuth_errors)."""
    return [f'drone {d + 1} sits {err:+.0f} deg from plate {plate}: check the ring rigid body '
            f'(+x toward plate 0) or the magnet plates'
            for d, err, plate in errors if abs(err) > tol_deg]
