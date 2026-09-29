#!/usr/bin/env python3
"""tools/headroom.py -- predicted carry throttle per drone before a tethered rig flight (readiness review 2026-09-29, K1).

    python3 tools/headroom.py --masses 0.82,0.80,0.85 --az 30,150,270 --u-free 0.45,0.44,0.46
    python3 tools/headroom.py --masses 0.82,0.80,0.85,0.83 --az 30,90,150,270 --kt 22

Static balance at hover with the rods at --elev (the planner's 45 deg): the ring's weight is split by the planner's
own geometry.balanced_tensions (level ring, uneven layouts included); each drone's thrust carries its own weight plus
its rod's pull, |F_i| = |m_i g z + t_i u_i| (u_i the unit vector from the drone down the rod to its plate). With a
linear thrust map the throttle is u_i = |F_i| / (m_i kT_i), or u_free_i |F_i| / (m_i g) from a measured free hover
(rods on, same pack). The tracker caps throttle at 0.6 (acados.py:135): a drone at or above --stop has no margin
to hold the ring. Assumptions, not measurements: linear thrust map, rods straight at --elev, no ring swing."""
import argparse
import math
import sys

import numpy as np

G = 9.81


def floats(s):
    return [float(v) for v in str(s).split(',') if v.strip()]


def predict(masses, az_deg, ring_mass, elev_deg=45.0, rho=0.25, kt=None, u_free=None):
    """Per drone (tension N, thrust ratio |F|/(m g), predicted throttle)."""
    from controller_load_mpc.geometry import balanced_tensions
    n = len(masses)
    e = math.radians(elev_deg)
    rho_v, s_v = [], []
    for a in az_deg:
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        rho_v.append(np.array([rho * c, rho * s, 0.0]))
        s_v.append(-np.array([math.cos(e) * c, math.cos(e) * s, math.sin(e)]))   # drone -> plate
    t = balanced_tensions(rho_v, s_v, ring_mass, g=G)
    out = []
    for i in range(n):
        f = masses[i] * G * np.array([0.0, 0.0, 1.0]) - t[i] * s_v[i]
        ratio = float(np.linalg.norm(f)) / (masses[i] * G)
        if u_free is not None:
            u = u_free[i] * ratio
        else:
            u = float(np.linalg.norm(f)) / (masses[i] * kt[i])
        out.append((t[i], ratio, u))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--masses', required=True, help='kg per drone with pack, rod and magnet, slot order')
    ap.add_argument('--az', required=True, help='attach azimuths (deg), same order (plate k = 30 k)')
    ap.add_argument('--ring', type=float, default=0.86, help='ring mass kg (weigh it)')
    ap.add_argument('--elev', type=float, default=45.0, help='rod elevation (deg)')
    ap.add_argument('--rho', type=float, default=0.25, help='plate radius (m)')
    grp = ap.add_mutually_exclusive_group(required=True)
    grp.add_argument('--u-free', help='measured free-hover throttle per drone (one value = all)')
    grp.add_argument('--kt', help='kT per drone (one value = all)')
    ap.add_argument('--cap', type=float, default=0.6)
    ap.add_argument('--stop', type=float, default=0.55, help="Wesley's stop value for the HOLD throttle")
    a = ap.parse_args(argv)
    m, az = floats(a.masses), floats(a.az)
    if len(m) != len(az):
        sys.exit('--masses and --az need the same number of entries')
    per = lambda v: (floats(v) * len(m))[:len(m)] if len(floats(v)) == 1 else floats(v)
    uf = per(a.u_free) if a.u_free else None
    kt = per(a.kt) if a.kt else None
    if (uf and len(uf) != len(m)) or (kt and len(kt) != len(m)):
        sys.exit('--u-free / --kt: one value or one per drone')
    rows = predict(m, az, a.ring, a.elev, a.rho, kt=kt, u_free=uf)
    print(f'ring {a.ring:.3f} kg, rods at {a.elev:g} deg, {len(m)} drones; cap {a.cap:.2f}, stop {a.stop:.2f}')
    print(f'{"drone":>5} {"az":>6} {"mass":>6} {"tension N":>10} {"x free":>7} {"u pred":>7}  verdict')
    worst = 0.0
    for i, (t, ratio, u) in enumerate(rows):
        worst = max(worst, u)
        v = ('OVER CAP' if u >= a.cap else 'above stop' if u >= a.stop else 'ok')
        print(f'{i:>5} {az[i]:>6.0f} {m[i]:>6.3f} {t:>10.2f} {ratio:>7.3f} {u:>7.3f}  {v}')
    print(f'worst {worst:.3f}: ' + ('NO tethered flight on this layout at this cap' if worst >= a.cap else
                                  'R3a as a measurement only (LAND if u sits at the cap)' if worst >= a.stop else
                                  'go for R3a/R3b0 on the prediction (R3a measures it)'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
