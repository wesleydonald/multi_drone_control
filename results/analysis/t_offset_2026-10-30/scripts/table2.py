import sys; sys.path.insert(0,'.')
from load import *
from model_moment import model_terms
import numpy as np
W = 0.86*9.81; MD = 0.55; ROD_W = 0.075*9.81; C = 0.2667
def rod_tau(Fd, u):
    # force of the rod on the drone, the tracker's drone mass including the rod: Fd = (c-1) W_rod - tau u ; least squares tau
    v = np.array([0, 0, (1 - C) * ROD_W]) - Fd   # = tau u
    return float(v @ u)
rows = []
for arm, rr in RUNS.items():
    for r in rr:
        x = Run(r)
        if r in (2970, 2971, 2978):
            continue
        a, b = x.te + 12, x.te + 18
        d = x.at(a, b); p = x.at(a, b, x.pl)
        ids = [int(v) for v in str(p.slot2drone.iloc[-1]).split()]
        row = dict(run=r, arm=arm, n=int(p.n.iloc[-1]))
        row['tilt_x'] = float(np.degrees(np.arcsin(d.ny.mean())))     # about the 0-6 (x) axis
        row['tilt_y'] = float(np.degrees(np.arcsin(d.nx.mean())))
        row['ring'] = d[['payload_x', 'payload_y', 'payload_z']].mean().values
        row['ring_ref'] = d[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].mean().values
        for k in (0, 2, 3):
            sl = ids.index(k)
            row[f'T{k}'] = pd.to_numeric(p[f't{sl}']).mean(); row[f'Tz{k}'] = -pd.to_numeric(p[f'tsz{sl}']).mean()
            row[f'el{k}'] = pd.to_numeric(p[f'elev{sl}']).mean()
            tr = x.at(a, b, x.tr[k])
            R = quat_R(tr.pose_qw, tr.pose_qx, tr.pose_qy, tr.pose_qz)
            est = np.array([0, 0, 9.81]) - (tr.kt_mpc.values * (tr.thr_out.values - tr.thr_off.values))[:, None] * R[:, :, 2]
            Fd = MD * est.mean(0)
            row[f'Fd{k}'] = Fd
            row[f'ac{k}'] = MD * tr[['acx', 'acy', 'acz']].mean().values
            # rod direction from ring attach to drone pivot (measured)
            dp = d[[f'd{k}_x', f'd{k}_y', f'd{k}_z']].mean().values - np.array([0, 0, 0.04])
            Rr = quat_R(d.payload_qw.mean(), d.payload_qx.mean(), d.payload_qy.mean(), d.payload_qz.mean())
            th = np.radians(-{0: 0, 1: 270, 2: 180, 3: 90}[k])
            att = row['ring'] + Rr @ np.array([0.225*np.cos(th), 0.225*np.sin(th), 0.02])
            u = (dp - att) / np.linalg.norm(dp - att)
            row[f'tau{k}'] = rod_tau(Fd, u)
            row[f'ringz_from_rod{k}'] = row[f'tau{k}'] * u[2] - (1 - C) * ROD_W   # net vertical the rod puts on its plate
            row[f'd{k}'] = d[[f'd{k}_x', f'd{k}_y', f'd{k}_z']].mean().values
            row[f'd{k}ref'] = d[[f'd{k}_ref_x', f'd{k}_ref_y', f'd{k}_ref_z']].mean().values
            row[f'err{k}'] = d[f'd{k}_track_err'].mean()
        row['int_f'] = p[['int_fx', 'int_fy', 'int_fz']].apply(pd.to_numeric).mean().values
        row['int_b'] = p[['int_bx', 'int_by', 'int_bz']].apply(pd.to_numeric).mean().values
        mt = model_terms(x, a, b)
        row['Mx'] = np.mean([v[2][0] for v in mt]); row['Fpl'] = np.mean([v[1] for v in mt], 0)
        row['share3_plan'] = row['Tz3'] / (row['Tz0'] + row['Tz2'] + row['Tz3'] + (0 if row['n'] == 3 else -pd.to_numeric(p['tsz1']).mean()))
        rows.append(row)
