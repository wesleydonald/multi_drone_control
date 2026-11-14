#!/usr/bin/env python3
"""tools/dropoff_summary.py -- the supervisor's drop-off (start -> ring -> start, 13 Nov 2026), one twin run per row:
his C++ transit time (TRANSIT_TO_DROP_POINT -> SETTLE), QUAD4's highest body z from his TAKEOFF to the drop, the closest
QUAD4 {body, cable, object} came to a carrier {body, cable} from his TAKEOFF to the drop (centre lines; the object's
fall into the net excluded), the ring's speed when his transit started and its fastest during it (the pause), the ball
in the net, QUAD4 back at its start (AT_START), and whether the carriers stayed armed through QUAD4's landing.
    python3 tools/dropoff_summary.py R4423 R4424 ...
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import attach_clearance as A  # noqa: E402
import m1_metrics as M  # noqa: E402

print('| run | his transit (s) | QUAD4 z max to the drop (m) | closest to a carrier in his transit (m, pair) | closest before the drop (m, pair) | ring speed at transit start / max in transit (m/s) | ball in net | AT_START | carriers armed through QUAD4 landing | end |')
print('|---|---|---|---|---|---|---|---|---|---|')
for rid in sys.argv[1:]:
    r = M.Run(rid)
    t = r.tt
    a = r.t('PHASE:TAKEOFF')
    tr = r.t('PHASE:TRANSIT_TO_DROP_POINT', after=a)
    st = r.t('PHASE:SETTLE_ABOVE_DROP_POINT', after=tr) if tr else None
    dr = r.t('PHASE:DROP_OBJECT', after=a)
    at = r.t('PHASE:AT_START', after=a)
    land = r.t('LAND', after=a)
    end = dr or land or float(t[-1])
    mask = (t >= a) & (t < end)
    z = r.xyz(r.k)[:, 2]
    c = A.clearance(r, mask)
    ct = A.clearance(r, (t >= tr) & (t < (st or end))) if tr else None
    ring = r.xyz('payload_')
    v = np.r_[0.0, np.linalg.norm(np.diff(ring[:, :2], axis=0), axis=1) / np.maximum(np.diff(t), 1e-6)]
    k = max(1, int(round(0.2 / max(np.median(np.diff(t)), 1e-3))))
    v = np.convolve(v, np.ones(k) / k, mode='same')
    vs = float(v[np.argmin(np.abs(t - tr))]) if tr else float('nan')
    vm = float(v[(t >= tr) & (t <= (st or end))].max()) if tr else float('nan')
    armed = [r.col(f'd{i}_armed') for i in range(len(r.az))]
    win = (t >= (at or end)) & (t < (land or t[-1]))
    carriers_ok = all(np.nanmin(x[win]) > 0.5 for x in armed) if win.any() else None
    o = A.score(rid)
    endev = [e for e in ('FLEET_ABORT', 'LANDED') if r.t(e, after=a)]
    print(f'| {rid} | {(st - tr) if tr and st else float("nan"):.1f} | {z[mask].max():.2f} | '
          + (f'{ct["min_m"]:.2f} ({ct["pair"]}, carrier {ct["carrier"]})' if ct else '-') + ' | '
          + (f'{c["min_m"]:.2f} ({c["pair"]}, carrier {c["carrier"]}, {c["phase"]})' if c else '-')
          + f' | {vs:.3f} / {vm:.3f} | {o.get("ball_in_net_at_land")} | {"yes" if at else "NO"} | {carriers_ok} | {",".join(endev)} |')
