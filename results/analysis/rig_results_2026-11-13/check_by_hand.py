#!/usr/bin/env python3
"""Independent check of drops.py on fm9_5: the tilt peak, swing and dip recomputed from the raw ~90 Hz ring poses
(fm9_5_full/ring.csv, made by results/analysis/hero_2026-11-13/extract.py) and the log's phase lines, sharing no code
with drops.py / paper_rig (no 50 Hz grid, no set-point cut of the before window: fm9_5's set-point was still)."""
import os
import re

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
log = open(os.path.expanduser('~/multi_drone_control/results/rig/2026-11-12/fm9_5.log'), errors='replace').read()
te = float(re.search(r'\[(\d+\.\d+)\] \[online_join_planner\]: Planner phase: \w+ -> DROP_OBJECT', log).group(1))
desc = float(re.search(r'\[(\d+\.\d+)\] \[online_join_planner\]: Planner phase: \w+ -> DESCEND_TO_ATTACHMENT',
                       log).group(1))
r = pd.read_csv(os.path.join(HERE, 'fm9_5_full', 'ring.csv'))
w, x, y, z = r.qw.values, r.qx.values, r.qy.values, r.qz.values
nz, nx, ny = 1 - 2 * (x * x + y * y), 2 * (x * z + w * y), 2 * (y * z - w * x)
ang = np.degrees(np.arctan2(np.hypot(nx, ny), nz))
tv = np.stack([nx, ny], 1) * (ang / np.maximum(np.hypot(nx, ny), 1e-12))[:, None]
t = r.t.values
pre, post = (t >= te - 5) & (t < te), (t >= te) & (t < desc)
d = pd.read_csv(os.path.join(HERE, 'drop_disturbance.csv'))
row = d[d.flight == 'fm9_5'].iloc[0]
for name, mine, by_hand in (
        ('window s', row.window_s, desc - te),
        ('tilt before, mean (deg)', row.tilt_pre_deg, ang[pre].mean()),
        ('tilt before, max (deg)', row.tilt_pre_max_deg, ang[pre].max()),
        ('tilt peak (deg)', row.tilt_peak_deg, ang[post].max()),
        ('swing (deg)', row.swing_deg, np.linalg.norm(tv[post] - tv[pre].mean(0), axis=1).max()),
        ('dip (mm)', row.dip_mm, 1000 * (r.z.values[pre].mean() - r.z.values[post].min())),
        ('dip before (mm)', row.dip_pre_mm, 1000 * (r.z.values[pre].mean() - r.z.values[pre].min()))):
    print(f'{name:26s} drops.py {mine:8.3f}   by hand {by_hand:8.3f}')
