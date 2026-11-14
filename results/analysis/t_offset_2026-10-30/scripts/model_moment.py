from load import *
import numpy as np
RHO_R, RHO_Z, M, G = 0.225, 0.02, 0.86, 9.81
CW = {0: 0.0, 1: 270.0, 2: 180.0, 3: 90.0}   # typed clockwise azimuth per drone
def rho_body(d):
    th = np.radians(-CW[d])                   # maths counter-clockwise
    return np.array([RHO_R*np.cos(th), RHO_R*np.sin(th), RHO_Z])
def model_terms(x, lo, hi):
    p = x.at(lo, hi, x.pl)
    p = p[p.n == 3] if (p.n == 3).any() else p
    out = []
    for _, row in p.iterrows():
        q = [row.qw, row.qx, row.qy, row.qz]
        Rm = quat_R(*q)
        ids = [int(v) for v in str(row.slot2drone).split()]
        F = np.zeros(3); Mo = np.zeros(3)
        for sl, d in enumerate(ids):
            t = float(row[f't{sl}']); tsz = float(row[f'tsz{sl}']); az = float(row[f'az1_{sl}'])
            sz = tsz / t if abs(t) > 1e-6 else -0.88
            sh = np.sqrt(max(1 - sz**2, 0))
            s = np.array([sh*np.cos(az), sh*np.sin(az), sz])
            f = -t * s
            r = Rm @ rho_body(d)
            F += f; Mo += np.cross(r, f)
        out.append((row.t - x.te, F, Mo, row.tilt_deg))
    return out
if __name__ == '__main__':
    for r in (2965, 2972, 2967, 2968, 2969, 2977, 2966):
        x = Run(r)
        o = model_terms(x, x.te + 15, x.te + 20)
        F = np.mean([v[1] for v in o], 0); Mo = np.mean([v[2] for v in o], 0); tl = np.mean([v[3] for v in o])
        # restoring check: gravity acts at CoG (no moment); int_f acts at CoG
        print(r, x.arm, 'tilt', round(tl, 2), 'F_cables', F.round(3), 'W', -M*G, 'M_cables (N m)', Mo.round(4))
