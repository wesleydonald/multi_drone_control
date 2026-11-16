"""
position_limit.py -- the fleet's position limit (Wesley 22 Oct; replaces the 4 Aug "no geofence" in safety.py).

Any quad's x or y past +-pos_limit_m (from the mocap and world origin) while airborne makes the fleet LAND: an envelope
LAND, never a disarm (the fleet manager). Nothing is commanded or landed outside it: every drone reference is kept
POS_LIMIT_REF_MARGIN_M inside (the planner, and the trackers on receipt), and a freed or dropped drone's landing point
POS_LIMIT_LAND_MARGIN_M inside (the detach mode). pos_limit_m 0 is off, the code default until the project's critic
has looked at it (card docs/experiments/2026-10-22_position_limit.md).

Pure functions, no ROS: the nodes call them and the tests run them as they are.
"""
import numpy as np

POS_LIMIT_REF_MARGIN_M = 0.05     # m inside the limit: every drone reference
POS_LIMIT_LAND_MARGIN_M = 0.10    # m inside the limit: a freed or dropped drone's landing point
POS_LIMIT_SAMPLES = 3             # poses in a row past the limit before the LAND (one bad mocap frame is not a breach)


def limit_xy(p, bound):
    """A copy of point p (x, y, ...) with x and y clamped to +-bound; bound <= 0 leaves it as it is."""
    q = np.array(p, float)
    if bound > 0.0:
        q[:2] = np.clip(q[:2], -bound, bound)
    return q


def limit_ray(c, p, bound):
    """A copy of point p moved back along the ray from c to where it meets +-bound in x and y, so a step out from c
    keeps its bearing (limit_xy turns it sideways; critic m1); limit_xy when c itself is outside, p when bound <= 0."""
    q = np.array(p, float)
    if bound <= 0.0 or max(abs(q[0]), abs(q[1])) <= bound:
        return q
    c = np.asarray(c, float)[:2]
    if max(abs(c[0]), abs(c[1])) > bound:
        return limit_xy(q, bound)
    d = q[:2] - c
    t = min((np.sign(q[i]) * bound - c[i]) / d[i] for i in (0, 1) if abs(q[i]) > bound)
    q[:2] = c + t * d
    return q


def clear_point(c, u, r, bound):
    """The xy point of the circle of radius r about c that is inside +-bound in x and y and whose bearing from c is
    nearest u's, or None when no point of the circle is inside."""
    c = np.asarray(c, float)[:2]
    phi0 = float(np.arctan2(u[1], u[0]))

    def at(phi):
        return c + r * np.array([np.cos(phi), np.sin(phi)])

    def inside(phi):
        return float(np.max(np.abs(at(phi)))) <= bound + 1e-9

    if inside(phi0):
        return at(phi0)
    # u's bearing is outside: the nearest inside point ends an arc, where the circle crosses a side of the square
    ends = []
    for i in (0, 1):
        for s in (-1.0, 1.0):
            k = (s * bound - c[i]) / r
            if abs(k) <= 1.0:
                a = float(np.arccos(k)) if i == 0 else float(np.arcsin(k))
                ends += [phi for phi in ((a, -a) if i == 0 else (a, np.pi - a)) if inside(phi)]
    if not ends:
        return None
    return at(min(ends, key=lambda phi: abs((phi - phi0 + np.pi) % (2.0 * np.pi) - np.pi)))


def limit_horizon(pos, vel, bound):
    """(positions, velocities, nodes clamped) of a reference horizon kept inside +-bound in x and y: each node clamped,
    and a velocity carrying a clamped node further out zeroed, so the drone is asked to stop at the limit rather than
    to run on into it (copies; bound <= 0 leaves both as they are)."""
    pos, vel = np.array(pos, float), np.array(vel, float)
    if bound <= 0.0:
        return pos, vel, 0
    xy = pos[:, :2]
    over = np.abs(xy) > bound
    if not over.any():
        return pos, vel, 0
    onward = over & (np.sign(vel[:, :2]) == np.sign(xy))
    pos[:, :2] = np.clip(xy, -bound, bound)
    vel[:, :2][onward] = 0.0
    return pos, vel, int(over.any(axis=1).sum())


def breach(p, bound):
    """True when point p's x or y is past +-bound (bound > 0)."""
    return bound > 0.0 and max(abs(float(p[0])), abs(float(p[1]))) > bound
