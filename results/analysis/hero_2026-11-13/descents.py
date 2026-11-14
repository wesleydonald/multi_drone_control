#!/usr/bin/env python3
"""How low QUAD4's magnet went in each pickup and re-attach descent of a hero run.
usage: descents.py <extract_dir> <t2_log>"""
import sys, re
import numpy as np, pandas as pd
from scipy.spatial.transform import Rotation as R

d, log = sys.argv[1], sys.argv[2]
L = lambda k: pd.read_csv(f'{d}/{k}.csv')
tip, obj, ring, q4, ref = L('tip'), L('obj'), L('ring'), L('q4'), L('ref')
ph = []
for line in open(log, errors='replace'):
    m = re.search(r'\[(\d+\.\d+)\] \[online_join_planner\]: Planner phase: (\S+) -> (\S+)\. Reason: (.*)', line)
    if m: ph.append((float(m[1]), m[2], m[3], m[4].strip()))
ph = pd.DataFrame(ph, columns=['t', 'a', 'b', 'why'])
at = lambda df, t: df.iloc[np.clip(np.searchsorted(df.t.values, t), 0, len(df) - 1)]
interp = lambda df, c, t: np.interp(t, df.t.values, df[c].values)
CH = 0.011  # tip marker over the object marker with the magnet resting flat on it (pickup_contact_height)
t0 = ph.t.iloc[0]

def window(a, b):
    return pd.DataFrame({'t': tip.t[(tip.t >= a) & (tip.t <= b)]})

print('== pickups (gap = tip-object height - contact height 0.011; negative = pressed past contact)')
for i, r0 in ph[ph.b == 'DESCEND_TO_PICKUP'].iterrows():
    nxt = ph[ph.t > r0.t]
    t_end = nxt.t.iloc[0]
    lift = nxt[nxt.a == 'LIFT_OBJECT'].iloc[0] if (nxt.a == 'LIFT_OBJECT').any() else None
    tb = nxt[nxt.a == 'MAGNET_ATTACH_WAIT'].t.iloc[0] if (nxt.a == 'MAGNET_ATTACH_WAIT').any() else t_end
    w = window(r0.t, tb + 0.0)
    T = w.t.values
    gap = interp(tip, 'z', T) - interp(obj, 'z', T) - CH
    dxy = np.hypot(interp(tip, 'x', T) - interp(obj, 'x', T), interp(tip, 'y', T) - interp(obj, 'y', T))
    rq = interp(ref, 'z', T); qz = interp(q4, 'z', T)
    ref_tip_gap = rq - (qz - interp(tip, 'z', T)) - interp(obj, 'z', T) - CH  # reference's tip, same convention
    k = np.argmin(gap)
    near = gap < 0.01
    print(f'descent at +{r0.t - t0:.1f} s, {T[-1] - T[0]:.1f} s long -> {nxt.b.iloc[0]} ({nxt.why.iloc[0][:60]})')
    print(f'   lowest tip gap {gap[k]*100:+.1f} cm (xy off {dxy[k]*100:.1f} cm) | ref tip gap at its lowest {ref_tip_gap.min()*100:+.1f} cm'
          f' | time within 1 cm of contact {near.sum() / max(len(T),1) * (T[-1]-T[0]):.2f} s | xy at end {dxy[-1]*100:.1f} cm')
    if lift is not None:
        tl, tl2 = lift.t, nxt[nxt.t > lift.t].t.iloc[0] if (nxt.t > lift.t).any() else lift.t + 3
        wl = window(tl, tl2); TL = wl.t.values
        sep = interp(tip, 'z', TL) - interp(obj, 'z', TL) - CH
        rise = interp(obj, 'z', TL) - interp(obj, 'z', [tl])[0]
        print(f'   lift {tl2 - tl:.1f} s -> {lift.b}: tip-object separation max {sep.max()*100:.1f} cm, object rose {rise.max()*100:.1f} cm'
              f' ({lift.why[:50]})')

print('\n== re-attach on plate 9 (plate point = ring pose + R (0, 0.225, 0.018); gap = tip z - plate z)')
for i, r0 in ph[ph.b == 'DESCEND_TO_ATTACHMENT'].iterrows():
    nxt = ph[ph.t > r0.t]; t_end = nxt.t.iloc[0] if len(nxt) else tip.t.iloc[-1]
    T = window(r0.t, t_end + 3.0).t.values
    rp = np.c_[interp(ring, 'x', T), interp(ring, 'y', T), interp(ring, 'z', T)]
    rq = np.c_[[interp(ring, c, T) for c in ('qx', 'qy', 'qz', 'qw')]].T
    plate = rp + R.from_quat(rq).apply([0.0, 0.225, 0.018])
    tp = np.c_[interp(tip, 'x', T), interp(tip, 'y', T), interp(tip, 'z', T)]
    gap = tp[:, 2] - plate[:, 2]; dxy = np.hypot(*(tp[:, :2] - plate[:, :2]).T)
    k = np.argmin(gap)
    print(f'descent at +{r0.t - t0:.1f} s -> {nxt.b.iloc[0] if len(nxt) else "end"} ({nxt.why.iloc[0][:60] if len(nxt) else ""})')
    print(f'   lowest tip-plate {gap[k]*100:+.1f} cm with xy {dxy[k]*100:.1f} cm; tip-plate at the phase end {gap[np.searchsorted(T, t_end)-1]*100:+.1f} cm')
pr = L('proof')
for _, p in pr.drop_duplicates('text').iterrows():
    print(f'proof +{p.t - t0:.1f}: {p.text[:200]}')
b = L('bool')
for k, g in b.groupby('topic'):
    ch = g[g.v.ne(g.v.shift())]
    print(k, ' '.join(f'+{r.t - t0:.1f}:{r.v}' for r in ch.itertuples())[:400])
