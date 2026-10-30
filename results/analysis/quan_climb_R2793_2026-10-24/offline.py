"""R2793's climb offline: the harness at c_net_quan.yaml's point, the pull model's rod (R6) x R6b, hover then LAND."""
import sys, json, numpy as np
from multiprocessing import Pool
sys.path.insert(0, '/home/wesley/multi_drone_control/tools')
import network_tune as nt
BEST = dict(quan_k_ij=0.5, quan_c_ij=0.25, quan_fc=0.5, quan_k_i=1.0, quan_c_i=1.0, vel_kv=3.0, vel_k_att=16.0)
ARMS = [('R6 rod 0.075 (R2793)', 0.075, 0.0), ('R6 rod 0.075 + R6b ki 2', 0.075, 2.0), ('R6 rod 0.055 + R6b ki 2', 0.055, 2.0),
        ('R6 rod 0.055 + R6b ki 5', 0.055, 5.0), ('R6 rod 0', 0.0, 0.0), ('R6 rod 0 + R6b ki 2', 0.0, 2.0)]
TWIN = len(sys.argv) > 2 and sys.argv[2] == 'twin'
LAND_AT, T_END = 40.0, 52.0


def hover(arm):
    name, rod, ki = arm
    r = nt.fly('hover', 0.0, dict(BEST, quan_rod_mass=rod, quan_fz_ki=ki), 'quan', hover_s=T_END - nt.TAIL_S, land_at=LAND_AT, twin_rods=TWIN)
    if 'diverged' in r:
        return name, {'diverged': r['diverged']}
    t, e, z = r['t'], r['act'] - r['p_des'], r['act'][:, 2]
    w = lambda a, b: (t >= a) & (t < b)
    landed = t >= LAND_AT + (1.0 - nt.QUAN_LAND_Z) / nt.QUAN_LAND_VEL + 3.0
    return name, dict(climb_peak_mm=1000 * float(e[w(0, 15), 2].max()), dz_0_5_mm=1000 * float(e[w(0, 5), 2].mean()),
                      dz_held_mm=1000 * float(e[w(20, LAND_AT), 2].mean()),
                      dxy_rms_mm=1000 * float(np.sqrt((e[w(5, LAND_AT), :2] ** 2).sum(1).mean())),
                      ring_z_after_land_m=float(z[landed].mean()), load_down=bool(z[landed].mean() <= nt.QUAN_LAND_Z + 0.10),
                      quad_tilt_peak=float(r['quad_tilt'][w(0, LAND_AT)].max()), thr_peak=float(r['thr'][w(0, LAND_AT)].max()))


def line(arm):
    name, rod, ki = arm
    r = nt.fly('line_x', 0.4, dict(BEST, quan_rod_mass=rod, quan_fz_ki=ki), 'quan', twin_rods=TWIN)
    if 'diverged' in r:
        return name, {'diverged': r['diverged']}
    s = nt.score(r)
    e = r['act'] - r['p_des']
    return name, dict(headline_mm=s['headline_mm'], cf_rms_mm=s['cf_rms_mm'], band_max_mm=s['band_max_mm'],
                      dz_mean_mm=1000 * float(e[r['t'] > nt.HOLD_S, 2].mean()), quad_tilt_peak=s['quad_tilt_peak_deg'],
                      thr_peak=s['thr_peak'])


if __name__ == '__main__':
    which = sys.argv[1]
    arms = ARMS
    with Pool(4) as p:
        res = p.map(hover if which == 'hover' else line, arms)
    for n, d in res:
        print(f'{n:28s}', json.dumps({k: round(v, 3) if isinstance(v, float) else v for k, v in d.items()}))
    json.dump(dict(res), open(f'/tmp/claude-1000/-home-wesley-multi-drone-control/d021810e-9d5d-427c-be70-40b127760327/scratchpad/climb/offline_{which}_{"twin" if TWIN else "massless"}.json', 'w'), indent=1)
