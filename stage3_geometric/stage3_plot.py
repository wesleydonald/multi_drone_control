#!/usr/bin/env python3
"""Stage 3 A/B: open-loop kinematic vs closed-loop geometric load-position control.

The persistent model mismatch (sim thrust-ratio, cable sag, tracker bias) acts as
a sustained disturbance force on the suspended load. Open-loop kinematic has no
load feedback so it DRIFTS to a ~1.1 m lateral steady offset and stays there;
closed-loop geometric (PD+I on the measured load pose) REJECTS it to ~0.02 m.
"""
import csv, sys, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

KIN, GEO, OUT = sys.argv[1], sys.argv[2], sys.argv[3]

def load(p):
    r = list(csv.DictReader(open(p)))
    c = lambda n: np.array([float(x[n]) for x in r])
    t = c('sim_time'); t -= t[0]
    exy = np.hypot(c('payload_x')-c('payload_ref_x'), c('payload_y')-c('payload_ref_y'))
    return t, c('payload_x'), c('payload_y'), c('payload_ref_x'), c('payload_ref_y'), exy

tk, kx, ky, krx, kry, ek = load(KIN)
tg, gx, gy, grx, gry, eg = load(GEO)

def settle(t, e, t0=40):
    m = t >= t0
    return e[m].mean(), np.sqrt((e[m]**2).mean()), e[m].max()

mk = settle(tk, ek); mg = settle(tg, eg)
print("STAGE 3 — closed-loop geometric vs open-loop kinematic (hover, three_soft_paper)")
print(f"  kinematic (open-loop):  settled e_xy  mean={mk[0]:.3f}  rms={mk[1]:.3f}  max={mk[2]:.3f} m")
print(f"  geometric (closed-loop): settled e_xy  mean={mg[0]:.3f}  rms={mg[1]:.3f}  max={mg[2]:.3f} m")
print(f"  improvement: {mk[0]/max(mg[0],1e-6):.1f}x lower steady lateral error")

fig, ax = plt.subplots(1, 2, figsize=(13, 5))
ax[0].plot(tk, ek, color='tab:red', lw=1.6, label=f'kinematic (open-loop)  →{mk[0]:.2f} m')
ax[0].plot(tg, eg, color='tab:green', lw=1.6, label=f'geometric (closed-loop) →{mg[0]:.02f} m')
ax[0].axhline(0, color='k', lw=.5)
ax[0].set_title('Lateral load-position error vs time')
ax[0].set_xlabel('t [s]'); ax[0].set_ylabel('|e_xy| [m]'); ax[0].legend(); ax[0].grid(alpha=.3)

ax[1].plot(kx, ky, color='tab:red', lw=1.2, alpha=.8, label='kinematic load path')
ax[1].plot(gx, gy, color='tab:green', lw=1.2, alpha=.8, label='geometric load path')
ax[1].scatter([0], [0], color='k', marker='*', s=140, zorder=5, label='setpoint (0,0)')
ax[1].set_title('Load XY path (setpoint at origin)')
ax[1].set_xlabel('x [m]'); ax[1].set_ylabel('y [m]'); ax[1].axis('equal'); ax[1].legend(); ax[1].grid(alpha=.3)

fig.suptitle('Stage 3: closed-loop geometric controller rejects the load offset that open-loop kinematic leaves uncorrected',
             fontsize=12)
fig.tight_layout(rect=[0, 0, 1, 0.95])
fig.savefig(OUT, dpi=120)
print(f"Saved {OUT}")
