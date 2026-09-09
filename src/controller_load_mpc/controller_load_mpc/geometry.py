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
