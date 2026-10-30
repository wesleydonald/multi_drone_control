"""Before: the seeded constant (R6b at fz_ki 0, rod 0.055, k_att 8: R2805/R2808/R2809's reading) at 413ec0e's code,
twin rods. Hover 40 s, line 0.2 (4 cycles), line 0.4 (8 cycles)."""
import sys, json, math, numpy as np
from multiprocessing import Pool
sys.path.insert(0, '/home/wesley/multi_drone_control/tools')
import network_tune as nt
import metrics as M
from dissipative_planner.quan_law import QuanLaw, QuanParams

BEST = dict(quan_k_ij=0.5, quan_c_ij=0.25, quan_fc=0.5, quan_k_i=1.0, quan_c_i=1.0, vel_kv=3.0, vel_k_att=8.0,
            quan_rod_mass=0.055)


def law_const(k, structure='quan'):
    nodes, vcmd = nt.QUAN_READINGS[structure]
    return QuanLaw(nt.N, nt.M_QUAD, nt.M_LOAD, QuanParams(
        k_ij=k['quan_k_ij'], c_ij=k['quan_c_ij'], k_i=k['quan_k_i'], c_i=k['quan_c_i'], f_c=k['quan_fc'],
        hc_m=k['quan_hc_m'], range_m=nt.QUAN_RANGE_M, nodes=nodes, vcmd=vcmd, node_mass=k['quan_node_mass'],
        rod_mass=k['quan_rod_mass'], fz='autopilot', fz_ki=0.0))


nt.quan_law = law_const


def summary(case, speed):
    r = nt.fly(case, speed, BEST, 'quan', hover_s=40.0, twin_rods=True)
    if 'diverged' in r:
        return case, speed, {'diverged': r['diverged']}
    s = nt.score(r)
    t, e = r['t'], r['act'] - r['p_des']
    out = {k: v for k, v in s.items() if isinstance(v, float)}
    if case == 'hover':
        w = (t >= 5) & (t <= 40)
        out.update(dz_held_mm=1000 * float(e[w, 2].mean()), dxy_rms_mm=1000 * float(np.sqrt((e[w, :2] ** 2).sum(1).mean())))
    else:
        T = math.pi * nt.LINE_M / speed
        t0, t1 = nt.HOLD_S, nt.HOLD_S + r['T_path']
        win = (t >= t0) & (t <= t1)
        x0 = float(r['p_des'][0, 0])
        sched = x0 + 0.5 * nt.LINE_M * (1 - np.cos(2 * speed / nt.LINE_M * np.clip(t - t0, 0, None)))
        out['amp_ratio'] = float(M.radius_ratio(r['act'][:, :2], np.c_[sched, r['p_des'][:, 1]], win))
        out['dz_mean_mm'] = 1000 * float(e[win, 2].mean())
    return case, speed, out


if __name__ == '__main__':
    with Pool(3) as p:
        res = p.starmap(summary, [('hover', 0.0), ('line_x', 0.2), ('line_x', 0.4)])
    for c, v, d in res:
        print(c, v, json.dumps({k: round(x, 3) for k, x in d.items()}))
    json.dump([[c, v, d] for c, v, d in res], open(sys.argv[1], 'w'), indent=1)
