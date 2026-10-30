"""After: the split (R6c, rod 0.055, k_att 8) on the twin-rods plant, and the overshoot's one-variable arms.
Each arm: hover 40 s, line 0.2 (4 cycles), line 0.4 (8 cycles)."""
import sys, json, math, numpy as np
from multiprocessing import Pool
sys.path.insert(0, '/home/wesley/multi_drone_control/tools')
import network_tune as nt
from dissipative_planner import quan_law as QL

BASE = dict(quan_k_ij=0.5, quan_c_ij=0.25, quan_fc=0.5, quan_k_i=1.0, quan_c_i=1.0, vel_kv=3.0, vel_k_att=8.0,
            quan_rod_mass=0.055)
DAMP, SPRING = ('quan_c_ij', 'quan_c_i', 'quan_fc'), ('quan_k_ij', 'quan_k_i')
sc = lambda keys, f: {k: BASE[k] * f for k in keys}  # noqa: E731
ARMS = {
    'split': {},
    'a: a_expected zero': dict(quan_aexp_zero=1.0),
    'b: centre dynamic (2 kg, kp 1, kd 5)': dict(quan_centre_dyn=1.0),
    'c: damping x2': sc(DAMP, 2.0), 'c: damping x0.5': sc(DAMP, 0.5),
    'c: springs x2': sc(SPRING, 2.0), 'c: springs x0.5': sc(SPRING, 0.5),
    'd: vel_kv 1.5': dict(vel_kv=1.5), 'd: vel_kv 6': dict(vel_kv=6.0),
    'diag: a_expected / (1 + kv dt)': dict(_aexp_scale=1.0 / 1.3),
    'diag: a_expected / 1.15': dict(_aexp_scale=1.0 / 1.15),
    'diag: v_d = v (a_cmd in a_ff only)': dict(_vd_plain=1.0),
    'c: damping x1.5': sc(DAMP, 1.5),
    'c: c_i x2 alone': dict(quan_c_i=2.0),
    'c: c_ij, f_c x2': dict(quan_c_ij=0.5, quan_fc=1.0),
    'R7c: v_d = v (quan_vcmd feedforward)': dict(_structure='quan_r7c'),
}


def patch(knobs):
    s = knobs.pop('_aexp_scale', None)
    if s is not None:
        cmd = QL.QuanLaw.command

        def scaled(self, i, x, xd, x_L, xd_L, x_0, xd_0, a_exp, *a, **k):
            return cmd(self, i, x, xd, x_L, xd_L, x_0, xd_0, s * np.asarray(a_exp, float), *a, **k)
        QL.QuanLaw.command = scaled
    if knobs.pop('_vd_plain', None):
        cmd = QL.QuanLaw.command

        def plain(self, i, x, xd, *a, **k):
            v_d, a_ff, a_cable, a_cmd = cmd(self, i, x, xd, *a, **k)
            self.v_d[i] = np.asarray(xd[i], float).copy()
            return self.v_d[i].copy(), a_ff, a_cable, a_cmd
        QL.QuanLaw.command = plain
    return knobs


def one(job):
    arm, case, speed = job
    knobs = patch({**BASE, **ARMS[arm]})
    structure = knobs.pop('_structure', 'quan')
    r = nt.fly(case, speed, knobs, structure, hover_s=40.0, twin_rods=True)
    if 'diverged' in r:
        return arm, case, speed, {'diverged': r['diverged']}
    s = nt.score(r)
    t, e = r['t'], r['act'] - r['p_des']
    out = {k: v for k, v in s.items() if isinstance(v, float)}
    if case == 'hover':
        w = (t >= 5) & (t <= 40)
        out.update(dz_held_mm=1000 * float(e[w, 2].mean()), dxy_rms_mm=1000 * float(np.sqrt((e[w, :2] ** 2).sum(1).mean())))
    else:
        win = (t >= nt.HOLD_S) & (t <= nt.HOLD_S + r['T_path'])
        out['dz_mean_mm'] = 1000 * float(e[win, 2].mean())
    return arm, case, speed, out


if __name__ == '__main__':
    arms = sys.argv[2].split('|') if len(sys.argv) > 2 else list(ARMS)
    jobs = [(a, c, v) for a in arms for c, v in (('hover', 0.0), ('line_x', 0.2), ('line_x', 0.4))]
    with Pool(3, maxtasksperchild=1) as p:
        res = p.map(one, jobs)
    for a, c, v, d in res:
        print(f'{a:38s} {c:6s} {v:.1f}', json.dumps({k: round(x, 3) for k, x in d.items() if k in (
            'err_rms_mm', 'dz_held_mm', 'dxy_rms_mm', 'band_max_mm', 'amp_ratio', 'reach_mean_mm', 'headline_mm',
            'cf_rms_mm', 'lag_s', 'dz_mean_mm', 'quad_tilt_peak_deg', 'thr_peak', 'diverged')}))
    json.dump([[a, c, v, d] for a, c, v, d in res], open(sys.argv[1], 'w'), indent=1)
