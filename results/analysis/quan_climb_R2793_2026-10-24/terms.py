"""Rebuild the law's per-term vertical command from R2793's run.csv (the planner's own view, re-computed)."""
import sys
import numpy as np, pandas as pd
sys.path.insert(0, '/home/wesley/multi_drone_control/src/dissipative_planner')
from dissipative_planner.quan_law import QuanLaw, QuanParams, hor, k_d

RUN = '/home/wesley/multi_drone_control/results/2026-10-23/R2793_sim_gz_c_net_quan_hubcom_hover_L100/logs/run.csv'
d = pd.read_csv(RUN)
t = d.t.values
L = d[['payload_x', 'payload_y', 'payload_z']].values
Ld = np.gradient(L, t, axis=0)
X = np.stack([d[[f'd{i}_x', f'd{i}_y', f'd{i}_z']].values for i in range(4)], 1)
XD = np.stack([d[[f'd{i}_vx', f'd{i}_vy', f'd{i}_vz']].values for i in range(4)], 1)
rod = float(sys.argv[1]) if len(sys.argv) > 1 else 0.075
P = QuanParams(k_ij=0.5, c_ij=0.25, k_i=1.0, c_i=1.0, f_c=0.5, hc_m=3.41, rod_mass=rod)
m, mL, g = 0.55, 0.86, 9.81
T_HO = 27.209
k0 = int(np.searchsorted(t, T_HO))
law = QuanLaw(4, m, mL, P)
x0 = np.array([0.0, 0.0, 1.0])
law.start(X[k0], XD[k0], L[k0], x0, 1.0 + P.hc_m, xd_L=Ld[k0])
print('l_i(0)', np.round(law.li0, 3), 'node plane', 1.0 + P.hc_m)


def split(k, hold_z):
    h_c = hold_z + P.hc_m
    x, xd, xL, xdL = X[k], XD[k], L[k], Ld[k]
    q, qd = law.nodes_now(x, xd, xL, xdL, h_c)
    rows = []
    for i in range(4):
        sp, dp = law._node_terms(i, x, q, qd, x0, np.zeros(3), h_c, 0.0)
        kd = k_d(x[i][2], xL[2], h_c)
        e = q[i] - x[i]; li = np.linalg.norm(e)
        kspr = P.k_i * (1 - law.li0[i] / li) * e
        cdmp = P.c_i / li ** 2 * float(e @ (qd[i] - xd[i])) * e
        kij = -kd * P.k_ij * hor(sp); cij = -kd * P.c_ij * hor(dp)
        fr = -kd * P.f_c * hor(qd[i])
        h = kspr + cdmp + kij + cij + fr
        a_cable = law.cable_pull(i, x, xL) / m
        a_ff = np.array([0, 0, g]) + h / m - a_cable
        rows.append(dict(i=i, l_i=li, k_i_z=kspr[2], c_i_z=cdmp[2], kij_hor=np.linalg.norm(kij), cij_hor=np.linalg.norm(cij),
                         fric_hor=np.linalg.norm(fr), k_i_radial=float(hor(kspr) @ hor(x[i] - xL) / max(np.linalg.norm(hor(x[i] - xL)), 1e-6)),
                         a_cmd_z=h[2] / m, pull_z_N=-a_cable[2] * m, a_ff_z=a_ff[2], a_ff_h=np.linalg.norm(a_ff[:2])))
    return pd.DataFrame(rows)


for tt, hz, lab in [(T_HO, 1.0, 'hand-over'), (T_HO + 0.5, 1.0, '+0.5 s'), (T_HO + 1.5, 1.0, '+1.5 s'), (T_HO + 3.0, 1.0, '+3.0 s (top)'),
                    (60.0, 1.0, 'held 60 s'), (85.0, 1.0, 'held 85 s'), (130.0, 0.06, 'after LAND 130 s')]:
    k = int(np.searchsorted(t, tt))
    df = split(k, hz)
    print(f'\n{lab}: t {t[k]:.2f} ring z {L[k][2]:.3f} quad z {X[k][:,2].mean():.3f}')
    print(df.round(3).to_string(index=False))
    print('mean a_cmd_z %.3f  sum k_i_z %.3f N  sum c_i_z %.3f N' % (df.a_cmd_z.mean(), df.k_i_z.sum(), df.c_i_z.sum()))

# held-window means (45-90 s)
w = (t > 45) & (t < 90)
ks = np.where(w)[0][::10]
acc = np.array([split(k, 1.0)[['k_i_z', 'c_i_z', 'a_cmd_z', 'pull_z_N', 'a_ff_z']].mean().values for k in ks])
print('\nheld 45-90 s mean: k_i_z %.3f N, c_i_z %.3f N, a_cmd_z %.3f m/s2, pull_z %.3f N, a_ff_z %.3f m/s2' % tuple(acc.mean(0)))
print('needed per quad (0.55+0.215)g = %.3f N; law+pull feed-forward vertical (no spring) = %.3f N' % ((m + mL / 4) * g, m * g + (mL / 4 + rod) * g))
