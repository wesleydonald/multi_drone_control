#!/usr/bin/env python3
"""Te-Jen's re-attach bands (match_xy, match_dwell, descent_abort_xy) against the real-world plate descents, for the funnels.

    source ~/ros2_humble/install/setup.bash && source ~/multi_drone_control/install/setup.bash
    BAND_CACHE=<dir> nice -n 10 python3 results/analysis/attach_band_2026-11-10/bands.py
    -> descents.csv, gate_check.csv, wait.csv, abort_replay.csv, abort_check_sq5_2.csv, horizon.csv, hold_sinks.csv,
       hold_estimate.csv, recommend.csv (this folder); then tables.py -> tables.md

Exploratory reading of recorded flights (no bars). The gates are modelled as his planner computes them
(online_join_planner.py, tracking_errors / MATCH_VELOCITY / DESCEND_TO_ATTACHMENT with contact_attach):
  xy_err  = horizontal (world) distance from his measured tip (/tejen/magnet_tip_pose, raw) to the plate point
            (ring pose x the typed contact target, full attitude: ring_bridge full_pose_attachment)
  speed   = horizontal norm of QUAD4's BODY velocity (/drone_3/motion_capture_state twist) minus the plate's
            (/fake_attachment_point/twist; zero with attach_twist:=zero)
  MATCH   = xy_err < match_xy and speed < match_speed, held unbroken for match_dwell (condition_true_for)
  descent = descent_hold true (4-5 Nov): xy_err > descent_abort_xy only HOLDS the descent's height; it restarts only past
            descent_gross_xy (0.20) or descent_abort_speed; descent_debounce is not used on this path.
            descent_hold false (29 Oct): xy_err > descent_abort_xy for descent_debounce (0 = one tick) restarts it.
The funnel scoring uses the plate frame of rig_joins_2026-11-04_05 (funnel_catch.py): offsets of the tip from the TRUE weld
point in the ring's plane (r radial + outward, s sideways + anticlockwise), height n along the ring normal.
"""
import ast
import csv
import math
import os
import re
import sys

import numpy as np
from scipy.spatial.transform import Rotation as Rot

HERE = os.path.dirname(os.path.abspath(__file__))
PREV = os.path.join(HERE, '..', 'rig_joins_2026-11-04_05')
sys.path.insert(0, HERE)
sys.path.insert(0, PREV)
import joins  # noqa: E402  parse_log, descents_from_phases
from read_flights import FLIGHTS, RIG, load  # noqa: E402

DISC_R = 32.0                      # mm, the holder disc's radius
DESCEND_RATE = 0.08                # m/s, his descend_rate (not typed on the rig)
TILT_MAX = 10.0                    # deg, hover samples with the ring leaning more are left out
MATCH_XY = (0.02, 0.03, 0.04, 0.05, 0.06, 0.08)
DWELLS = (0.3, 0.5, 1.0)
ABORT_XY = (0.04, 0.05, 0.06, 0.07, 0.08, 0.10)
HORIZON = (0.0, 0.1, 0.2, 0.3, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0)


def funnel_variants():
    """P, VARIANTS (with the later VARIANTS['v4'] = ... lines) and mouth() from cad/funnel/funnel.py without CadQuery
    (funnel_catch.py's reader keeps only the first VARIANTS literal)."""
    src = os.path.expanduser('~/multi_drone_control/cad/funnel/funnel.py')
    tree = ast.parse(open(src).read())

    def named(t):
        return getattr(t, 'id', '') in ('P', 'VARIANTS') or getattr(getattr(t, 'value', None), 'id', '') == 'VARIANTS'
    keep = [n for n in tree.body if (isinstance(n, ast.Assign) and any(named(t) for t in n.targets))
            or (isinstance(n, ast.FunctionDef) and n.name == 'mouth')]
    ns = {'math': math}
    exec(compile(ast.Module(body=keep, type_ignores=[]), src, 'exec'), ns)
    out = {}
    for name, p in ns['VARIANTS'].items():
        a, b, h = ns['mouth'](p)
        disc_bottom = -p['steel_below_top'] + p['magnet_below_disc']
        out[name] = {'a': a, 'b': b, 'H': p['bore_top'] + h, 'plane_mm': p['bore_top'] + h - disc_bottom}
    return out


def funnels():
    """name -> plane (m, tip height above the weld point when the disc's bottom is at the mouth), mouth and fully-inside
    half-axes (m, radial, sideways)."""
    V = funnel_variants()
    out = {}
    for n in ('v4',):
        v = V[n]
        out[n] = dict(plane=v['plane_mm'] / 1000, mouth=(v['a'] / 1000, v['b'] / 1000),
                      inside=((v['a'] - DISC_R) / 1000, (v['b'] - DISC_R) / 1000), label=f"v4 (mouth {v['H']:.0f} mm up)")
    # the supervisor's cone: 105 mm mouth diameter, mouth height not known: 30 / 35 / 40 mm above the ring top assumed
    for h in (30, 35, 40):
        out[f'sup{h}'] = dict(plane=(h - 17.0) / 1000, mouth=(0.0525, 0.0525), inside=(0.0205, 0.0205),
                              label=f'supervisor cone (mouth {h} mm up, assumed)')
    v = V['v6']
    out['v6'] = dict(plane=v['plane_mm'] / 1000, mouth=(v['a'] / 1000, v['b'] / 1000),
                     inside=((v['a'] - DISC_R) / 1000, (v['b'] - DISC_R) / 1000), label=f"v6 (mouth {v['H']:.0f} mm up)")
    return out


def inside(r, s, ab):
    return (r / ab[0]) ** 2 + (s / ab[1]) ** 2 <= 1.0


# ------------------------------------------------------------------------------------------------- launch arguments
def launch_args(day, name):
    with open(os.path.join(RIG, day, 'launches.csv')) as f:
        for r in csv.DictReader(f):
            if r['name'] == name:
                a = {}
                for k, v in re.findall(r'(\S+?):=(\S+)', r['launch'].replace('\n', ' ')):
                    a.setdefault(k, v)
                return a
    raise KeyError(name)


def gates(a):
    """His contact gates as the rig launch sets them (partner_mission.launch.py, the real path)."""
    f = lambda k, d: float(a[k]) if k in a else d  # noqa: E731
    return dict(match_xy=f('match_xy', 0.05), match_speed=f('match_speed', 0.15), dwell=f('match_dwell', 1.0),
                abort_xy=f('descent_abort_xy', 0.08), abort_speed=f('descent_abort_speed', 0.25),
                hold=a.get('descent_hold', '').lower() in ('1', 'true', 'yes'), gross=f('descent_gross_xy', 0.20),
                debounce=f('descent_debounce', 0.0), approach_z=f('approach_offset_z', math.nan),
                contact_dz=f('contact_dz', -0.01))


def true_weld(day, name, typed):
    """The weld point each flight is scored against (rig_joins flights.csv; 29 Oct: the typed one)."""
    with open(os.path.join(PREV, 'flights.csv')) as f:
        for r in csv.DictReader(f):
            if r['day'] == day and r['flight'] == name:
                return np.array([float(x) for x in r['target_xy'].split(',')] + [float(r['target_z'])])
    return np.array(typed)


# --------------------------------------------------------------------------------------------------- the time series
def series(day, name):
    A = load(day, name)
    log = joins.parse_log(os.path.join(RIG, day, name + '.log'))
    a = launch_args(day, name)
    typed = np.array(log['contact_target'])
    weld = true_weld(day, name, typed)
    tip, body, ring, attv = A['tip'], A['body'], A['ring'], A['att_v']
    T = tip[:, 0]
    k = np.clip(np.searchsorted(ring[:, 0], T, side='right') - 1, 0, len(ring) - 1)
    rr = ring[k]
    rot = Rot.from_quat(rr[:, 4:8])
    plate_typed = rr[:, 1:4] + rot.apply(typed)
    xy_err = np.hypot(tip[:, 1] - plate_typed[:, 0], tip[:, 2] - plate_typed[:, 1])
    rel = rot.inv().apply(tip[:, 1:4] - rr[:, 1:4]) - weld
    u = weld[:2] / np.hypot(*weld[:2])
    r_ = rel[:, :2] @ u
    s_ = rel[:, :2] @ np.array([-u[1], u[0]])
    kb = np.clip(np.searchsorted(body[:, 0], T, side='right') - 1, 0, len(body) - 1)
    # the plate's velocity is left out: zero on 4-5 Nov (attach_twist:=zero); on 29 Oct sq2_5 the bag's
    # /fake_attachment_point/twist is not zero, but his MATCH time is reproduced only without it (12.09 vs 12.30 s)
    speed = np.hypot(body[kb, 4], body[kb, 5])
    tilt = np.degrees(np.arccos(np.clip(rot.apply([0, 0, 1])[:, 2], -1, 1)))
    # his phase at each sample (log transitions)
    ph = log['phases']
    pt = np.array([p[0] for p in ph]) if ph else np.zeros(0)
    pi = np.searchsorted(pt, T, side='right') - 1
    names = np.array(['PRE'] + [p[2] for p in ph])
    phase = names[pi + 1]
    return dict(day=day, name=name, T=T, xy=xy_err, r=r_, s=s_, n=rel[:, 2], speed=speed, tilt=tilt, phase=phase,
                log=log, gates=gates(a), args=a, weld=weld, typed=typed)


def first_pass(T, cond, dwell, t_start):
    """His condition_true_for from t_start: the first time cond has held unbroken for dwell (None if never)."""
    start = None
    for t, c in zip(T, cond):
        if t < t_start:
            continue
        if c:
            if start is None:
                start = t
            if t - start >= dwell:
                return t
        else:
            start = None
    return None


# ------------------------------------------------------------------------------------------------------- descents
def descents(S, F):
    """One row per DESCEND_TO_ATTACHMENT entry (rig_joins numbering)."""
    log, g = S['log'], S['gates']
    T = S['T']
    rows = []
    jr = {}
    with open(os.path.join(PREV, 'joins.csv')) as f:
        for r in csv.DictReader(f):
            if r['day'] == S['day'] and r['flight'] == S['name'] and r['kind'] in ('re-attach', 'attach from the mat'):
                jr[int(r['n'])] = r
    takeback = [p[0] for p in log['phases'] if p[2] in ('TAKEN_BACK', 'LANDING')]
    t_end_flight = min(takeback) if takeback else T[-1]
    for n, (t0, t1, frm, endp, why) in enumerate(joins.descents_from_phases(log['phases'], 'DESCEND_TO_ATTACHMENT',
                                                                             ('ATTACH_READY',)), 1):
        t1 = t1 if t1 is not None else T[-1]
        j = jr.get(n, {})
        row = dict(day=S['day'], flight=S['name'], n=n, kind=j.get('kind', 'attach from the mat' if S['args'].get(
            'partner_attached') != 'true' else 're-attach'), plate=S['args'].get('reattach_plate', S['args'].get('plate', '9')),
            formation=j.get('formation', 'triangle'), motion=S['args'].get('load_traj', 'hover'), frm=frm,
            outcome=j.get('outcome') or ('aborted' if 'aborted' in why else why), end_reason=why,
            t0=t0, t1=t1, dur=t1 - t0, held=sum(1 for h in log['held'] if t0 <= h <= t1))
        w = (T >= t0) & (T <= t1)
        if w.sum() < 5:
            continue
        i0 = int(np.flatnonzero(w)[0])
        row.update(start_xy=S['xy'][i0], start_r=S['r'][i0], start_s=S['s'][i0], start_n=S['n'][i0])
        wg = (T >= t0 - g['dwell']) & (T <= t0)
        row['gate_max_xy'] = float(S['xy'][wg].max()) if wg.any() else math.nan
        if frm == 'MATCH_VELOCITY':
            tm = [p[0] for p in log['phases'] if p[2] == 'MATCH_VELOCITY' and p[0] <= t0][-1]
            im = int(np.searchsorted(T, tm))
            row.update(match_t=tm, match_dur=t0 - tm, match_xy=S['xy'][im], match_r=S['r'][im], match_s=S['s'][im],
                       match_n=S['n'][im])
            ta = [p[0] for p in log['phases'] if p[2] == 'APPROACH_ABOVE_TARGET' and p[0] <= tm]
            row['approach_dur'] = tm - ta[-1] if ta else math.nan
        # the plate-height reading of rig_joins (where the tip first reached the plate)
        for k in ('eval', 'tip_rad_cm', 'tip_tan_cm', 'tip_xy_cm', 'tip_min_normal_cm'):
            row['joins_' + k] = j.get(k, '')
        idx = np.flatnonzero(w)
        for name, f in F.items():
            if S['n'][i0] <= f['plane']:
                row[name + '_status'] = 'started below'
                continue
            hit = idx[S['n'][idx] <= f['plane']]
            if not len(hit):
                row[name + '_status'] = 'never reached'
                continue
            k = int(hit[0])
            row.update({name + '_status': 'scored', name + '_t': T[k] - t0, name + '_r': S['r'][k], name + '_s': S['s'][k],
                        name + '_xy': S['xy'][k], name + '_mouth': inside(S['r'][k], S['s'][k], f['mouth']),
                        name + '_inside': inside(S['r'][k], S['s'][k], f['inside']), name + '_tilt': S['tilt'][k],
                        name + '_held_before': sum(1 for h in log['held'] if t0 <= h <= T[k])})
        rows.append(row)
    return rows, t_end_flight


# --------------------------------------------------------------------------------------- 1. the gate reconstruction
def gate_check(S):
    """For every MATCH -> DESCEND of the flight: his MATCH time vs the reconstructed first pass; every logged hold vs
    the reconstructed xy_err."""
    g, log, T = S['gates'], S['log'], S['T']
    cond = (S['xy'] < g['match_xy']) & (S['speed'] < g['match_speed'])
    out = []
    ph = log['phases']
    for i, (t, a, b, _) in enumerate(ph):
        if a == 'MATCH_VELOCITY':
            tm = [p[0] for p in ph[:i] if p[2] == 'MATCH_VELOCITY'][-1]
            fp = first_pass(T, cond, g['dwell'], tm)
            out.append(dict(day=S['day'], flight=S['name'], kind='match', his_s=t - tm, to=b,
                            recon_s=(fp - tm) if fp is not None else math.nan))
    for t, kind, v in held_lines(S):
        w = (T > t - 0.15) & (T <= t)
        if w.any():
            out.append(dict(day=S['day'], flight=S['name'], kind=kind, his_s=v,
                            recon_s=100 * float(S['xy'][w].max() if kind == 'held' else S['xy'][w].min())))
    return out


def held_lines(S):
    out = []
    with open(os.path.join(RIG, S['day'], S['name'] + '.log'), errors='replace') as f:
        for ln in f:
            m = re.search(r'\[(\d{9,}\.\d+)\].*Contact descent (held|resumed): tip ([\d.]+) cm', ln)
            if m:
                out.append((float(m.group(1)), m.group(2), float(m.group(3))))
    return out


# ------------------------------------------------------------------------------------------------ 3. the MATCH wait
POOLS = {
    'hover': 'his APPROACH_ABOVE_TARGET / MATCH_VELOCITY, tip >= 0.20 m above the plate and inside his 0.15 m approach band',
    'descent': 'DESCEND_TO_ATTACHMENT from 1.0 s after its start, tip >= 0.10 m above the plate',
}


def segments(S, t_end, pool, descents_rows=()):
    """Contiguous stretches of samples (gaps < 0.1 s) of one pool (POOLS), ring tilt <= TILT_MAX, before the take-back."""
    T, ph = S['T'], S['phase']
    base = (T < t_end) & (S['tilt'] <= TILT_MAX) & np.isfinite(S['xy'])
    if pool == 'hover':
        m = base & np.isin(ph, ('APPROACH_ABOVE_TARGET', 'MATCH_VELOCITY')) & (S['xy'] < 0.15) & (S['n'] >= 0.20)
    else:
        late = np.zeros(len(T), bool)
        for r in descents_rows:
            if r['day'] == S['day'] and r['flight'] == S['name'] and r['frm'] == 'MATCH_VELOCITY':
                late |= (T >= r['t0'] + 1.0) & (T <= r['t1'])
        m = base & late & (ph == 'DESCEND_TO_ATTACHMENT') & (S['n'] >= 0.10)
    idx = np.flatnonzero(m)
    if not len(idx):
        return []
    cut = np.flatnonzero((np.diff(T[idx]) > 0.1) | (np.diff(idx) > 1)) + 1
    return [part for part in np.split(idx, cut) if len(part) > 5 and T[part[-1]] - T[part[0]] > 0.2]


def wait_stats(pool_segs, B, vmax, dwell):
    """The MATCH wait from the pooled segments: qualifying windows (the gate condition unbroken for `dwell`, his
    condition_true_for) per second of pooled time. The segments are short (the as-flown MATCH ends at the 0.08 m pass),
    so a run cut by its segment's end before reaching the dwell is unknown: lam_low counts only the windows seen,
    lam_high adds every such cut run. Expected wait ~ dwell + 1/lam; no window within T s ~ exp(-lam (T - dwell))
    (Poisson). The longest unbroken run seen is given as a check."""
    tot, t_in, q, cut, longest = 0.0, 0.0, 0, 0, 0.0
    for S, part in pool_segs:
        T = S['T'][part]
        cond = (S['xy'][part] < B) & (S['speed'][part] < vmax)
        dt = np.diff(T, append=T[-1])
        tot += T[-1] - T[0]
        t_in += float(dt[cond].sum())
        i, N = 0, len(T)
        while i < N:
            if not cond[i]:
                i += 1
                continue
            j = i
            while j + 1 < N and cond[j + 1]:
                j += 1
            length = T[j] - T[i]
            longest = max(longest, length)
            if length >= dwell:
                q += 1                                  # one window per run (the planner leaves MATCH)
            elif j == N - 1:
                cut += 1
            i = j + 1
    lo, hi = q / tot, (q + cut) / tot

    def wait(lam):
        return (dwell + 1 / lam) if lam > 0 else math.inf

    def none(lam, T):
        return math.exp(-lam * max(0.0, T - dwell)) if lam > 0 else 1.0
    return dict(B=B, vmax=vmax, dwell=dwell, pooled_s=tot, segments=len(pool_segs), frac_in=t_in / tot if tot else math.nan,
                windows=q, cut_short=cut, longest_run_s=longest, windows_per_min_lo=60 * lo, windows_per_min_hi=60 * hi,
                wait_s_lo=wait(hi), wait_s_hi=wait(lo), p_none_30_lo=none(hi, 30), p_none_30_hi=none(lo, 30),
                p_none_60_lo=none(hi, 60), p_none_60_hi=none(lo, 60))


# --------------------------------------------------------------------------------------- 4. the abort and hold replay
def abort_replay(S, row, F, B, debounce):
    """Would a descent (from its start to each funnel's mouth plane) have been restarted by xy_err > B held for
    `debounce` s (descent_hold false; his contact_position_lost)? Returns {funnel: (aborted, t_abort)}."""
    T = S['T']
    w = (T >= row['t0']) & (T <= row['t1'])
    idx = np.flatnonzero(w)
    out = {}
    for name, f in F.items():
        if row.get(name + '_status') != 'scored':
            continue
        tc = row['t0'] + row[name + '_t']
        sub = idx[T[idx] <= tc]
        over = S['xy'][sub] > B
        start, ta = None, None
        for t, o in zip(T[sub], over):
            if o:
                if start is None:
                    start = t
                if t - start >= debounce:
                    ta = t
                    break
            else:
                start = None
        out[name] = (ta is not None, (ta - row['t0']) if ta is not None else math.nan)
    return out


def hold_estimate(rows, allS, F, name, B, lag):
    """descent_hold true with band B: his reference moves down only while xy_err <= B; the tip follows it LAG s behind
    (the sink after his real holds, 4-5 Nov: median 2.3 cm at 0.08 m/s = ~0.3 s). The tip therefore reaches the mouth
    plane ~LAG after an instant the tip was inside B: P(the offset LAG later is inside the window | xy_err <= B now),
    over the descent samples (4-5 Nov, from MATCH) at least 1 cm above the plane whose LAG-later sample is still at or
    above it. The samples conditioned on are those 1-4 cm above the plane (where his reference passes it, the tip lagging
    ~2.3 cm): over the whole descent the estimate misses the measured as-flown arrivals at the supervisor's plane
    (10 % vs 21 % inside), near the plane it reproduces them (bands.py check in hold_estimate.csv, B 0.10). The time the
    descent needs grows as 1 / (the fraction of descent time inside B)."""
    f = F[name]
    num = mouth = den = 0
    contrib = set()
    tin = ttot = 0.0
    for r in rows:
        if r.get(name + '_status') != 'scored':
            continue
        S = allS[(r['day'], r['flight'])]
        T = S['T']
        tc = r['t0'] + r[name + '_t']
        idx = np.flatnonzero((T >= r['t0']) & (T <= tc))
        if len(idx) < 5:
            continue
        dt = np.diff(T[idx], append=T[idx[-1]])
        tin += float(dt[S['xy'][idx] <= B].sum())
        ttot += float(dt.sum())
        j = np.searchsorted(T, T[idx] + lag)
        ok = (j < len(T)) & (S['n'][idx] >= f['plane'] + 0.01) & (S['n'][idx] <= f['plane'] + 0.04)
        ok[ok] &= T[j[ok]] <= tc + 0.02
        sel = ok & (S['xy'][idx] <= B)
        jj = j[sel]
        den += int(sel.sum())
        num += int(inside(S['r'][jj], S['s'][jj], f['inside']).sum())
        mouth += int(inside(S['r'][jj], S['s'][jj], f['mouth']).sum())
        if sel.any():
            contrib.add((r['day'], r['flight'], r['n']))
    return dict(funnel=name, B=B, lag=lag, samples=den, descents=len(contrib), p_inside=num / den if den else math.nan,
                p_mouth=mouth / den if den else math.nan, frac_time_in_band=tin / ttot if ttot else math.nan)


def main():
    F = funnels()
    allS, rows, gc, ends = {}, [], [], {}
    for day, name in FLIGHTS:
        S = series(day, name)
        allS[(day, name)] = S
        r, ends[(day, name)] = descents(S, F)
        rows += r
        gc += gate_check(S)
    pools = {p: [] for p in POOLS}
    for key, S in allS.items():
        for p in pools:
            pools[p] += [(S, seg) for seg in segments(S, ends[key], p, rows)]

    # ---- descents.csv, gate_check.csv
    cols = ['day', 'flight', 'n', 'kind', 'plate', 'formation', 'motion', 'frm', 'outcome', 'end_reason', 'dur', 'held',
            'approach_dur', 'match_dur', 'match_xy', 'match_r', 'match_s', 'match_n', 'gate_max_xy',
            'start_xy', 'start_r', 'start_s', 'start_n', 'cycle_s']
    main_rows = [r for r in rows if r['day'] != '2026-10-29' and r['frm'] == 'MATCH_VELOCITY']
    for r in main_rows:      # descent start to the next descent start from MATCH in the same flight (one attempt)
        nxt = [q['t0'] for q in main_rows if q['day'] == r['day'] and q['flight'] == r['flight'] and q['t0'] > r['t0']]
        r['cycle_s'] = (min(nxt) - r['t0']) if nxt else math.nan
    for name in F:
        cols += [name + k for k in ('_status', '_t', '_r', '_s', '_xy', '_mouth', '_inside', '_tilt', '_held_before')]
    cols += ['joins_eval', 'joins_tip_rad_cm', 'joins_tip_tan_cm', 'joins_tip_xy_cm', 'joins_tip_min_normal_cm']
    write_csv('descents.csv', rows, cols)
    write_csv('gate_check.csv', gc, ['day', 'flight', 'kind', 'to', 'his_s', 'recon_s'])

    # ---- the MATCH wait per pool, and all pools together
    wrows = []
    groups = [(f'{p} {lab}', [(S, sg) for S, sg in segs if S['day'] in days])
              for p, segs in pools.items() for lab, days in (('4-5 Nov', ('2026-11-04', '2026-11-05')), ('29 Oct', ('2026-10-29',)))]
    groups.append(('all', [x for segs in pools.values() for x in segs]))
    for label, ps in groups:
        if not ps:
            continue
        for B in MATCH_XY:
            for d in DWELLS:
                w = wait_stats(ps, B, 0.20, d)
                w['pool'] = label
                wrows.append(w)
    write_csv('wait.csv', wrows, ['pool', 'B', 'vmax', 'dwell', 'pooled_s', 'segments', 'frac_in', 'windows', 'cut_short',
                                  'longest_run_s', 'windows_per_min_lo', 'windows_per_min_hi', 'wait_s_lo', 'wait_s_hi',
                                  'p_none_30_lo', 'p_none_30_hi', 'p_none_60_lo', 'p_none_60_hi'])

    # ---- the abort replay (descent_hold false), 4-5 Nov descents from MATCH, up to each mouth plane
    arows = []
    for B in ABORT_XY:
        for deb in (0.0, 0.5, 1.0):
            res = [(r, abort_replay(allS[(r['day'], r['flight'])], r, F, B, deb)) for r in main_rows]
            for name in F:
                sc = [(r, a[name]) for r, a in res if name in a]
                ab = [r for r, (x, _) in sc if x]
                kept = [r for r, (x, _) in sc if not x]
                arows.append(dict(B=B, debounce=deb, funnel=name, scored=len(sc), aborted=len(ab),
                                  aborted_inside=sum(bool(r[name + '_inside']) for r in ab),
                                  aborted_mouth=sum(bool(r[name + '_mouth']) for r in ab),
                                  kept_inside=sum(bool(r[name + '_inside']) for r in kept),
                                  kept_mouth=sum(bool(r[name + '_mouth']) for r in kept), kept=len(kept),
                                  abort_after_s_median=float(np.median([t for _, (x, t) in sc if x])) if ab else math.nan))
    write_csv('abort_replay.csv', arows, ['B', 'debounce', 'funnel', 'scored', 'aborted', 'aborted_inside', 'aborted_mouth',
                                          'kept', 'kept_inside', 'kept_mouth', 'abort_after_s_median'])
    # check of the replay on the one flight that flew the restart (29 Oct sq5_2: 0.08 m, one tick): his abort times
    chk = []
    S = allS[('2026-10-29', 'sq5_2')]
    for r in [q for q in rows if q['flight'] == 'sq5_2']:
        T = S['T']
        w = np.flatnonzero((T >= r['t0']) & (T <= r['t1'] + 0.05))
        over = np.flatnonzero(S['xy'][w] > S['gates']['abort_xy'])
        chk.append(dict(n=r['n'], his_abort_s=r['dur'], replay_abort_s=(T[w[over[0]]] - r['t0']) if len(over) else math.nan))
    write_csv('abort_check_sq5_2.csv', chk, ['n', 'his_abort_s', 'replay_abort_s'])

    # ---- the horizon: the tip at the plane vs the tip dt before (4-5 Nov, descents from MATCH)
    hrows = []
    for name in ('v4', 'sup35'):
        pairs = {h: [] for h in HORIZON}
        for r in main_rows:
            if r.get(name + '_status') != 'scored':
                continue
            S = allS[(r['day'], r['flight'])]
            tc = r['t0'] + r[name + '_t']
            kc = int(np.searchsorted(S['T'], tc))
            for h in HORIZON:
                k = int(np.clip(np.searchsorted(S['T'], tc - h, side='right') - 1, 0, len(S['T']) - 1))
                pairs[h].append((S['r'][k], S['s'][k], S['r'][kc], S['s'][kc], tc - h >= r['t0']))
        for h, p in pairs.items():
            p = np.array(p, dtype=float)
            hrows.append(dict(funnel=name, dt=h, n=len(p), in_descent=int(p[:, 4].sum()),
                              corr_r=np.corrcoef(p[:, 0], p[:, 2])[0, 1], corr_s=np.corrcoef(p[:, 1], p[:, 3])[0, 1],
                              rms_change_r_cm=100 * np.sqrt(np.mean((p[:, 2] - p[:, 0]) ** 2)),
                              rms_change_s_cm=100 * np.sqrt(np.mean((p[:, 3] - p[:, 1]) ** 2))))
    write_csv('horizon.csv', hrows, ['funnel', 'dt', 'n', 'in_descent', 'corr_r', 'corr_s', 'rms_change_r_cm',
                                     'rms_change_s_cm'])

    # ---- the hold band (descent_hold true, as flown); LAG from the sink after his real holds
    sinks = hold_sinks(allS)
    write_csv('hold_sinks.csv', sinks, ['day', 'flight', 't_held', 'dur', 'n_at_hold_cm', 'sink_cm'])
    hi = [h['sink_cm'] for h in sinks if h['n_at_hold_cm'] > 8.0]
    lag = float(np.median(hi)) / 100.0 / DESCEND_RATE
    est = []
    for name in ('v4', 'sup30', 'sup35', 'sup40'):
        for B in (0.02, 0.03) + ABORT_XY:
            for lg in (lag / 2, lag, 1.5 * lag):
                e = hold_estimate(main_rows, allS, F, name, B, lg)
                e['lag_case'] = 'main' if lg == lag else ('half' if lg < lag else 'x1.5')
                est.append(e)
    write_csv('hold_estimate.csv', est, ['funnel', 'B', 'lag', 'lag_case', 'samples', 'descents', 'p_inside', 'p_mouth',
                                         'frac_time_in_band'])
    recommend(main_rows, est, wrows, F)
    print(f'{len(rows)} descents; holds with the tip > 8 cm up: {len(hi)}, sink median {np.median(hi):.1f} cm '
          f'-> lag {lag:.2f} s; pools: ' + ', '.join(f'{p} {len(s)} segments' for p, s in pools.items()))
    return 0


def recommend(main_rows, est, wrows, F):
    """Per funnel and (match_xy, match_dwell, descent_abort_xy): attempts per minute and the share of attempts whose
    disc arrives inside the window. His code raises descent_abort_xy to at least match_xy (contact_abort_xy_threshold =
    max(contact_match_xy_threshold, typed)), so the hold band used is max(match_xy, descent_abort_xy). One attempt =
    APPROACH (his dwell_time_s = match_dwell, the tip already inside 0.15 m) + the MATCH wait (all pools, the middle of
    lo..hi) + the descent to the plane (as flown x frac_in(0.10) / frac_in(band)) + the rest of the as-flown attempt
    (below the plane, the bottom, the climb). Arrival shares: hold_estimate (main lag)."""
    out = []
    W = {(w['B'], w['dwell']): w for w in wrows if w['pool'] == 'all'}
    E = {(e['funnel'], e['B']): e for e in est if e['lag_case'] == 'main'}
    for name in ('v4', 'sup30', 'sup35', 'sup40'):
        sc = [r for r in main_rows if r.get(name + '_status') == 'scored']
        t_plane = float(np.median([r[name + '_t'] for r in sc]))
        rest = float(np.nanmedian([r['cycle_s'] - r['approach_dur'] - r['match_dur'] - r[name + '_t'] for r in sc]))
        f10 = E[(name, 0.10)]['frac_time_in_band']
        for Bm in (0.08, 0.06, 0.05, 0.04, 0.03):
            for d in (0.3, 0.5):
                w = W[(Bm, d)]
                lam = (w['windows_per_min_lo'] + w['windows_per_min_hi']) / 2 / 60
                wait = d + 1 / lam if lam > 0 else math.inf
                for Bh in (0.10, 0.08, 0.06, 0.05, 0.04, 0.03):
                    if Bh < Bm:
                        continue
                    e = E[(name, Bh)]
                    tp = t_plane * f10 / e['frac_time_in_band']
                    cyc = d + wait + tp + rest
                    out.append(dict(funnel=name, match_xy=Bm, match_dwell=d, descent_abort_xy=Bh, approach_s=d, match_wait_s=wait,
                                    descent_to_plane_s=tp, rest_s=rest, attempt_s=cyc, attempts_per_min=60 / cyc,
                                    p_inside=e['p_inside'], p_mouth=e['p_mouth'],
                                    inside_per_min=60 / cyc * e['p_inside'], mouth_per_min=60 / cyc * e['p_mouth'],
                                    p_no_attempt_30=math.exp(-lam * max(0.0, 30 - d)) if lam > 0 else 1.0,
                                    p_no_attempt_60=math.exp(-lam * max(0.0, 60 - d)) if lam > 0 else 1.0))
    write_csv('recommend.csv', out, ['funnel', 'match_xy', 'match_dwell', 'descent_abort_xy', 'approach_s', 'match_wait_s',
                                     'descent_to_plane_s', 'rest_s', 'attempt_s', 'attempts_per_min', 'p_inside', 'p_mouth',
                                     'inside_per_min', 'mouth_per_min', 'p_no_attempt_30', 'p_no_attempt_60'])


def hold_sinks(allS):
    """Every logged 'Contact descent held' (4-5 Nov): the tip's height then and how much lower it went before the
    'resumed' (his reference is frozen in between)."""
    out = []
    for (day, name), S in allS.items():
        if day == '2026-10-29':
            continue
        T, n = S['T'], S['n']
        H = held_lines(S)
        for i, (t, kind, _) in enumerate(H):
            if kind != 'held':
                continue
            nxt = [h for h in H[i + 1:] if h[1] == 'resumed']
            te = nxt[0][0] if nxt else t + 3.0
            w = (T >= t) & (T <= te)
            if not w.any():
                continue
            k0 = int(np.searchsorted(T, t))
            out.append(dict(day=day, flight=name, t_held=t - T[0], dur=te - t, n_at_hold_cm=100 * n[k0],
                            sink_cm=100 * (n[k0] - n[w].min())))
    return out


def write_csv(fn, rows, cols):
    def fmt(v):
        if isinstance(v, (bool, np.bool_)):
            return 'Y' if v else ''
        if isinstance(v, (float, np.floating)):
            return '' if not math.isfinite(v) else f'{v:.4f}'
        return '' if v is None else v
    with open(os.path.join(HERE, fn), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore', lineterminator='\n')
        w.writeheader()
        for r in rows:
            w.writerow({k: fmt(r.get(k)) for k in cols})


if __name__ == '__main__':
    sys.exit(main())
