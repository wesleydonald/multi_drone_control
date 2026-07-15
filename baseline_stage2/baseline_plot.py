#!/usr/bin/env python3
"""Stage 2 baseline: open-loop (kinematic) load-position error, hover + slow circle.

Reads the drone-0 tracker logs (payload_x/y/z = actual load, payload_ref_x/y/z =
commanded load) and produces one figure + a stats summary. The stats over the
settled window are 'the number to beat' for the Stage 3 closed loop.
"""
import csv, sys, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

RUNS = [
    ("hover",  sys.argv[1], (30, None)),   # settle window start ~30s (post-lift)
    ("circle", sys.argv[2], (30, 90)),     # circle-tracking window (before descent)
]
OUT = sys.argv[3]


def load(p):
    rows = list(csv.DictReader(open(p)))
    c = lambda n: np.array([float(r[n]) for r in rows])
    t = c('sim_time'); t = t - t[0]
    return dict(t=t,
                px=c('payload_x'), py=c('payload_y'), pz=c('payload_z'),
                rx=c('payload_ref_x'), ry=c('payload_ref_y'), rz=c('payload_ref_z'))


fig, axes = plt.subplots(2, 2, figsize=(13, 9))
print("=" * 68)
print("STAGE 2 — OPEN-LOOP (kinematic) LOAD-POSITION BASELINE")
print("=" * 68)

for r, (name, path, (t0, t1)) in enumerate(RUNS):
    d = load(path)
    t = d['t']
    ex, ey, ez = d['px'] - d['rx'], d['py'] - d['ry'], d['pz'] - d['rz']
    exy = np.sqrt(ex**2 + ey**2)          # LATERAL error — the meaningful metric
    en = np.sqrt(ex**2 + ey**2 + ez**2)

    m = t >= t0
    if t1 is not None:
        m &= t <= t1
    def stat(a):
        aw = a[m]
        return aw.mean(), np.sqrt((aw**2).mean()), np.abs(aw).max()
    print(f"\n[{name}]  file={path.split('/')[-2]}  window t∈[{t0},{t1 or 'end'}]s")
    for lbl, e in (("x", ex), ("y", ey), ("z", ez)):
        mu, rms, mx = stat(e)
        tag = "  (z: thrust-ratio sim artifact, exclude)" if lbl == "z" else ""
        print(f"   err_{lbl}:  mean={mu:+.3f}  rms={rms:.3f}  max|·|={mx:.3f}  m{tag}")
    mu, rms, mx = stat(exy)
    print(f"   err_xy:  mean={mu: .3f}  rms={rms:.3f}  max   ={mx:.3f}  m  <== NUMBER TO BEAT")

    # left col: error vs time
    ax = axes[r][0]
    ax.plot(t, ex, label='err x'); ax.plot(t, ey, label='err y')
    ax.plot(t, ez, color='tab:red', alpha=.4, label='err z (artifact)')
    ax.plot(t, exy, 'k-', lw=1.8, label='err xy (lateral)')
    ax.axvspan(t0, t1 or t[-1], color='green', alpha=0.06)
    ax.set_title(f"{name}: load position error (actual − desired)")
    ax.set_xlabel("t [s]"); ax.set_ylabel("error [m]"); ax.legend(fontsize=8); ax.grid(alpha=.3)

    # right col: XY path actual vs desired
    ax = axes[r][1]
    ax.plot(d['rx'], d['ry'], ':', color='tab:blue', lw=2, label='desired')
    ax.plot(d['px'], d['py'], '-', color='k', lw=1.5, label='actual')
    ax.scatter(d['px'][0], d['py'][0], c='g', marker='o', s=40, zorder=5, label='start')
    ax.scatter(d['px'][-1], d['py'][-1], c='r', marker='X', s=50, zorder=5, label='end')
    ax.set_title(f"{name}: load XY path"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.axis('equal'); ax.legend(fontsize=8); ax.grid(alpha=.3)

fig.suptitle("Stage 2 baseline — open-loop kinematic (three_soft_paper.sdf): "
             "stable but large steady offset", fontsize=13)
fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig(OUT, dpi=120)
print(f"\nSaved figure: {OUT}")
