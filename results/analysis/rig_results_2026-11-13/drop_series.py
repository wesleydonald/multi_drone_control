#!/usr/bin/env python3
"""The ring's response around every drop of drop_disturbance.csv (the 13 in the net, 13 Nov's fm9hc that missed, and the
two ring losses k1ob_2 / k1zo), aligned at the release (his DROP_OBJECT), -5 to +10 s on the 50 Hz grid of drops.py.

Columns: flight, day, t_from_release_s, ring_tilt_deg (the ring's tilt magnitude), ring_dz_mm (height minus the mean over
the 5 s before the release), xy_err_cm (the ring's distance to the path its reference traces, drops.py's clock-free
path_err), ref_speed_mps (the reference's horizontal speed). Output: drop_series.csv.gz.
    python3 drop_series.py      (after drops.py; uses its bag cache)"""
import os
import warnings

import numpy as np
import pandas as pd

import common as C
import drops as D

PRE, POST = 5.0, 10.0


def series(fid):
    F = C.flight(fid)
    te = [p for p in F.J['phases'] if p[2] == 'DROP_OBJECT'][0][0]
    G = F.G
    t = G.t.to_numpy(float)
    sel = np.nonzero((t >= te - PRE - 1e-9) & (t <= te + POST + 1e-9))[0]
    th = G.attrs['th']
    z = G.payload_z.to_numpy(float)
    xy = G[['payload_x', 'payload_y']].to_numpy(float)
    ref = G[['payload_ref_x', 'payload_ref_y']].to_numpy(float)
    pre = (t >= te - PRE) & (t < te)
    vref = np.full(len(t), np.nan)
    dt = np.gradient(t)
    vref[:] = np.linalg.norm(np.gradient(ref, axis=0), axis=1) / dt
    return pd.DataFrame({
        'flight': F.name, 'day': F.day,
        't_from_release_s': np.round(t[sel] - te, 3),
        'ring_tilt_deg': np.linalg.norm(th[sel], axis=1),
        'ring_dz_mm': 1000.0 * (z[sel] - np.nanmean(z[pre])),
        'xy_err_cm': 100.0 * D.path_err(t, xy, ref, sel),
        'ref_speed_mps': vref[sel],
    })


def main():
    warnings.simplefilter('ignore', RuntimeWarning)
    here = os.path.dirname(os.path.abspath(__file__))
    tab = pd.read_csv(os.path.join(here, 'drop_disturbance.csv'))
    keep = tab[(tab.in_net == 'yes') | tab.flight.isin(['k1ob_2', 'k1zo'])
               | ((tab.flight == 'fm9hc') & (tab.day == '2026-11-13'))]
    out = [series(f'{r.day}/{r.flight}') for r in keep.itertuples()]
    s = pd.concat(out, ignore_index=True)
    s.to_csv(os.path.join(here, 'drop_series.csv.gz'), index=False, float_format='%.4f', compression='gzip')
    print(f'{s.groupby(["flight", "day"]).ngroups} drops, {len(s)} rows')
    print(s.groupby(['flight', 'day']).t_from_release_s.agg(['min', 'max', 'count']).to_string())


if __name__ == '__main__':
    main()
