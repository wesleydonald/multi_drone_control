#!/usr/bin/env python3
"""tools/tilt_statics.py -- can three rods hold the ring when the survivors leave a gap >= 180 deg, if the
ring is allowed to tilt? (Wesley 8 Oct: accept a non-zero steady ring tilt.)

    python3 tools/tilt_statics.py --azimuths 0,180,270            # drone 2 on plate 0, drone 4 left from 3
    python3 tools/tilt_statics.py --azimuths 330,180,270 --min-elev 20

Statics of the ring (mass M at its centre, anchors on the rim at radius r in the ring plane) hung on three
rods. The ring is tilted by phi about the line through the two anchors that bound the gap, the gap side
down. For each phi the rod forces f_i (tension along the rod, pulling toward its drone) are found that
minimise sum |f_i|^2 subject to: force and moment balance; every rod at least --min-elev above the
horizontal (a rod near flat pulls the magnet sideways: r200006 slipped at ~30 deg of ring tilt); every
drone's thrust |m_d g z + f_i| below --max-thrust; every rod taut (>= --min-tension). Prints per phi: feasible, the tensions, the rods'
elevations and the drones' thrust as a fraction of hover-with-nothing. Statics only: no dynamics, no
stability (about the hinge line gravity gives no restoring moment when the CoG is on it).
"""
import argparse
import math

import numpy as np
from scipy.optimize import minimize

G = 9.81


def rot_axis(axis, ang):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def hinge(az_deg):
    """The two anchors bounding the largest gap, and the unit direction toward the gap (ring frame)."""
    a = sorted(float(x) % 360.0 for x in az_deg)
    gaps = [((a[(i + 1) % len(a)] - a[i]) % 360.0, a[i], a[(i + 1) % len(a)]) for i in range(len(a))]
    gap, lo, hi = max(gaps)
    mid = math.radians(lo + gap / 2.0)
    return gap, lo, hi, np.array([math.cos(mid), math.sin(mid), 0.0])


def solve(az_deg, phi_deg, M=0.86, r=0.225, m_d=0.55, min_elev_deg=20.0, max_thrust=2.0, min_tension=0.5):
    gap, lo, hi, toward_gap = hinge(az_deg)
    p_lo = r * np.array([math.cos(math.radians(lo)), math.sin(math.radians(lo)), 0.0])
    p_hi = r * np.array([math.cos(math.radians(hi)), math.sin(math.radians(hi)), 0.0])
    axis = p_hi - p_lo
    # tilt so the gap side goes DOWN: rotate about the hinge axis, sign chosen by the gap direction
    Rm = rot_axis(axis, math.radians(phi_deg))
    if (Rm @ toward_gap)[2] > 0:
        Rm = rot_axis(axis, -math.radians(phi_deg))
    P = [Rm @ (r * np.array([math.cos(math.radians(a)), math.sin(math.radians(a)), 0.0])) for a in az_deg]
    se = math.sin(math.radians(min_elev_deg))
    W = np.array([0.0, 0.0, M * G])

    def unpack(x):
        return [x[3 * i:3 * i + 3] for i in range(3)]

    cons = [{'type': 'eq', 'fun': lambda x: sum(unpack(x)) - W},
            {'type': 'eq', 'fun': lambda x: sum(np.cross(P[i], f) for i, f in enumerate(unpack(x)))}]
    for i in range(3):
        cons.append({'type': 'ineq', 'fun': lambda x, i=i: unpack(x)[i][2] - se * np.linalg.norm(unpack(x)[i])})
        # a slack rod holds nothing: without it the ring hangs on the hinge pair alone, free to roll
        cons.append({'type': 'ineq', 'fun': lambda x, i=i: np.linalg.norm(unpack(x)[i]) - min_tension})
        cons.append({'type': 'ineq', 'fun': lambda x, i=i: max_thrust * m_d * G
                     - np.linalg.norm(unpack(x)[i] + np.array([0.0, 0.0, m_d * G]))})
    x0 = np.tile(W / 3.0, 3)
    res = minimize(lambda x: float(x @ x), x0, constraints=cons, method='SLSQP',
                   options={'maxiter': 500, 'ftol': 1e-10})
    F = unpack(res.x)
    ok = res.success and all(c['fun'](res.x) >= -1e-6 for c in cons if c['type'] == 'ineq') and \
        np.allclose(cons[0]['fun'](res.x), 0, atol=1e-4) and np.allclose(cons[1]['fun'](res.x), 0, atol=1e-4)
    t = [float(np.linalg.norm(f)) for f in F]
    elev = [math.degrees(math.asin(max(-1.0, min(1.0, f[2] / max(n, 1e-9))))) for f, n in zip(F, t)]
    thrust = [float(np.linalg.norm(f + np.array([0.0, 0.0, m_d * G])) / (m_d * G)) for f in F]
    return gap, ok, t, elev, thrust


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--azimuths', default='0,180,270', help='survivors\' anchor azimuths, deg (ring frame)')
    ap.add_argument('--min-elev', type=float, default=20.0, help='minimum rod elevation above horizontal, deg')
    ap.add_argument('--max-thrust', type=float, default=2.0, help='max drone thrust / its own weight')
    ap.add_argument('--phi-max', type=float, default=45.0)
    ap.add_argument('--min-tension', type=float, default=0.5, help='every rod keeps at least this tension, N')
    a = ap.parse_args(argv)
    az = [float(v) for v in a.azimuths.split(',')]
    gap = hinge(az)[0]
    print(f'survivors {az} deg: largest gap {gap:.0f} deg; rods >= {a.min_elev:.0f} deg; thrust <= {a.max_thrust:.1f} x weight')
    print(' tilt  feasible  tensions (N)          rod elevations (deg)   drone thrust / weight')
    first = None
    for phi in np.arange(0.0, a.phi_max + 1e-9, 2.5):
        g, ok, t, el, th = solve(az, phi, min_elev_deg=a.min_elev, max_thrust=a.max_thrust,
                                 min_tension=a.min_tension)
        if ok and first is None:
            first = phi
        print(f'{phi:5.1f}  {"yes" if ok else " no":>8}  ' + ' '.join(f'{v:5.2f}' for v in t) + '      '
              + ' '.join(f'{v:5.1f}' for v in el) + '        ' + ' '.join(f'{v:4.2f}' for v in th))
    print(f'smallest feasible tilt: {first if first is not None else "none"} deg')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
