from load import *
from model_moment import model_terms
import numpy as np
from scipy.optimize import brentq
def kin_tilt(px, py, pz, cx, cy, cz):
    # tilt a (deg, + = plate-9 side low, plate 3 (-y) up) that puts plate 3's attach 1.0 m from drone 3's pivot,
    # ring centre (cx,cy,cz), rotation about the world x axis only
    piv = np.array([px, py, pz - 0.04])
    def f(a):
        a = np.radians(a)
        att = np.array([cx, cy - 0.225*np.cos(a) + 0.02*np.sin(a), cz + 0.225*np.sin(a) + 0.02*np.cos(a)])
        return np.linalg.norm(piv - att) - 1.0
    try:
        return brentq(f, -40, 60)
    except ValueError:
        return np.nan
def series(x, lo=-1, hi=20, step=0.1):
    g = np.arange(lo, hi, step)
    d = x.df; t = d.t.values - x.te
    I = lambda c: np.interp(g, t, d[c].values)
    out = dict(g=g, tilt=np.interp(g, t, np.degrees(np.arcsin(np.clip(d.ny.values, -1, 1)))))
    for k in range(4):
        for c in ('x', 'y', 'z', 'ref_x', 'ref_y', 'ref_z'):
            out[f'd{k}_{c}'] = I(f'd{k}_{c}')
    for c in ('payload_x', 'payload_y', 'payload_z'):
        out[c] = I(c)
    out['kin'] = np.array([kin_tilt(out['d3_x'][i], out['d3_y'][i], out['d3_z'][i], out['payload_x'][i], out['payload_y'][i], out['payload_z'][i]) for i in range(len(g))])
    out['kin_ref'] = np.array([kin_tilt(out['d3_ref_x'][i], out['d3_ref_y'][i], out['d3_ref_z'][i], out['payload_x'][i], out['payload_y'][i], out['payload_z'][i]) for i in range(len(g))])
    p = x.pl; tp = p.t.values - x.te
    # planned tension of drone 3's slot
    t3 = []
    for _, row in p.iterrows():
        ids = [int(v) for v in str(row.slot2drone).split()]
        sl = ids.index(3)
        t3.append(float(row[f't{sl}']))
    out['T3'] = np.interp(g, tp, np.array(t3))
    for c in ('int_fx', 'int_fy', 'int_fz', 'int_by', 'int_bz'):
        out[c] = np.interp(g, tp, pd.to_numeric(p[c], errors='coerce').values)
    out['n'] = np.interp(g, tp, p.n.values)
    mt = model_terms(x, x.te + lo, x.te + hi)
    out['Mx'] = np.interp(g, [v[0] for v in mt], [v[2][0] for v in mt])
    out['Fy'] = np.interp(g, [v[0] for v in mt], [v[1][1] for v in mt])
    tr = x.tr[3]; tt = tr.t.values - x.te
    for c in ('acm_x', 'acm_y', 'acm_z', 'acx', 'acy', 'acz'):
        out['tr3_' + c] = np.interp(g, tt, pd.to_numeric(tr[c], errors='coerce').values)
    return out
