"""
preflight_geometry.py -- the pure geometry behind tools/preflight.py (numpy only, no ROS).

Drones placed radially out from their rim magnets with equal rods lie on a circle
centred on the disc centre; the fit gives the centre (payload origin check), the radius
(rim + rod) and each drone's bearing, which is compared with the typed attach azimuths.
"""
import itertools
import math

import numpy as np


def fit_circle(points):
    """Least-squares (Kasa) circle through n >= 3 xy points: x^2 + y^2 + D x + E y + F = 0.
    Exact circumcircle for n == 3. Returns (cx, cy, radius, rms residual) or None when the
    points are collinear or too few."""
    P = np.asarray(points, float).reshape(-1, 2)
    if len(P) < 3:
        return None
    A = np.column_stack([P[:, 0], P[:, 1], np.ones(len(P))])
    b = -(P[:, 0] ** 2 + P[:, 1] ** 2)
    if np.linalg.matrix_rank(A, tol=1e-9) < 3:
        return None
    (D, E, F), *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = -D / 2.0, -E / 2.0
    r2 = cx * cx + cy * cy - F
    if r2 <= 0.0:
        return None
    r = math.sqrt(r2)
    res = np.hypot(P[:, 0] - cx, P[:, 1] - cy) - r
    return float(cx), float(cy), float(r), float(np.sqrt(np.mean(res ** 2)))


def bearings_deg(points, centre, yaw_deg=0.0):
    """Bearing of each xy point about `centre` in the LOAD frame (world bearing minus the
    payload yaw), in [0, 360)."""
    P = np.asarray(points, float).reshape(-1, 2)
    return [float((math.degrees(math.atan2(y - centre[1], x - centre[0])) - yaw_deg) % 360.0)
            for x, y in P]


def ang_diff_deg(a, b):
    """Signed a - b wrapped to (-180, 180]."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


def spacing_deg(bearings):
    """Gaps between neighbouring bearings going CCW round the ring (sum 360)."""
    s = sorted(bearings)
    return [s[k + 1] - s[k] for k in range(len(s) - 1)] + [s[0] + 360.0 - s[-1]]


def match_azimuths(measured_deg, typed_deg):
    """Match drones to typed slots as the planner's auto slot assignment does (any
    placement order; the permutation with the least squared angle error). Returns
    (slot2drone, errors) with errors[slot] = measured[slot2drone[slot]] - typed[slot]
    in deg. Brute force: n <= 6 on this rig."""
    n = len(typed_deg)
    if len(measured_deg) != n:
        raise ValueError(f'{len(measured_deg)} bearings for {n} typed azimuths')
    best, best_cost = None, math.inf
    for perm in itertools.permutations(range(n)):
        err = [ang_diff_deg(measured_deg[perm[s]], typed_deg[s]) for s in range(n)]
        c = sum(e * e for e in err)
        if c < best_cost:
            best, best_cost = (list(perm), err), c
    return best


def slot_offset_detail(drone, err_deg, typed_deg, tol_deg=10.0, plate_pitch_deg=30.0):
    """The preflight line for one matched drone; names the plate and what to check on a miss.
    Plate 0 is on the ring rigid body's +x, one plate every 30 deg (M2A ring)."""
    plate = int(round(typed_deg / plate_pitch_deg)) % int(round(360.0 / plate_pitch_deg))
    if abs(err_deg) <= tol_deg:
        return f'{err_deg:+.1f} deg from plate {plate}'
    return (f'drone {drone} sits {err_deg:+.0f} deg from plate {plate}: check the ring rigid body '
            f'(+x toward plate 0) or the magnet plates')


def typed_azimuths(spec, n):
    """The planner's attach_azimuths_deg read-back ('30,150,270', a list, or '' = even)."""
    if spec is None or (isinstance(spec, str) and spec.strip().lower() in ('', 'even', 'none', 'auto')):
        return [360.0 * i / n for i in range(n)]
    vals = spec if not isinstance(spec, str) else [t for t in spec.replace(' ', ',').split(',') if t]
    vals = [float(v) for v in vals]
    if not vals:
        return [360.0 * i / n for i in range(n)]
    if len(vals) != n:
        raise ValueError(f'attach_azimuths_deg {vals} has {len(vals)} entries for {n} drones')
    return vals


def yaw_deg_from_quat(w, x, y, z):
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
