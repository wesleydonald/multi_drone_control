"""analyze.py [run ...]: one row per replay run (runs/<run>/) -> summary.csv (and printed).

Times are seconds after the replay's CLEAR_DROP_ZONE entry (s0); 'h' columns after the clearance end (t0, the 7 s
budget's start). Plans from his replans.csv, split at the backend's grant; the exit counters from his last status
JSON (they count only solves whose incumbent was the bound prefix, i.e. before the grant). Reference joins: the
step between a publication's sample 0 and the previous publication evaluated at the new stamp (ticks.csv), at the
latch (prefix 1), the refreshes (prefix 2..), the grant, the switch to C++ (RETURN_TO_START), the first new C++
window after it, and the AT_START switch; 'tick p95/max' = all other ticks of the run. Geometry along the executed
exit (q4.csv, QUAD4 as flown in the replay, i.e. reference + recorded error): body z max, |x|/|y| max of the
reference while C++ holds authority, the body-box gap to each carrier (his boxes 0.105/0.105/0.06 half, no tracking
inflation), QUAD4's cable (body-0.04 to the magnet 0.05 below the 1.034 m tip) to each carrier tether (body-0.04 to
its plate on the ring at 0.225 m) as centre lines, and that cable to his basket cylinder (0.56 m diameter, ring z-0.27
to ring z)."""
import csv
import glob
import json
import os
import sys

import sqlite3

import numpy as np

RIG = '/home/wesley/multi_drone_control/results/rig/2026-11-13'

HERE = os.path.dirname(os.path.abspath(__file__))
ROD, PIVOT, MAG = 1.034, 0.04, 0.05
BOX = np.array([0.21, 0.21, 0.12])          # two 0.105/0.105/0.06 half boxes side by side
BASKET_R, BASKET_BOTTOM = 0.28, -0.27


def seg_seg(p1, q1, p2, q2):
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    c, b = d1 @ r, d1 @ d2
    den = a * e - b * b
    s = min(max((b * f - c * e) / den, 0.0), 1.0) if den > 1e-12 else 0.0
    t = (b * s + f) / e
    if t < 0.0:
        t, s = 0.0, min(max(-c / a, 0.0), 1.0)
    elif t > 1.0:
        t, s = 1.0, min(max((b - c) / a, 0.0), 1.0)
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


def cyl_dist(pts, centre):
    """Distance from points to a finite vertical cylinder (radius BASKET_R, z from centre+BOTTOM to centre)."""
    dxy = np.linalg.norm(pts[:, :2] - centre[:2], axis=1) - BASKET_R
    dz = np.maximum(pts[:, 2] - centre[2], (centre[2] + BASKET_BOTTOM) - pts[:, 2])
    out = np.where((dxy > 0) & (dz > 0), np.hypot(dxy, dz), np.maximum(dxy, dz))
    return float(np.min(out))


def carriers(flight, ta, tb):
    """The three carriers' recorded positions (bag time) over [ta, tb]: list of (t, xyz) arrays."""
    from rclpy.serialization import deserialize_message
    from interfaces.msg import MotionCaptureState
    con = sqlite3.connect(f'file:{RIG}/{flight}/{flight}_0.db3?mode=ro', uri=True)
    out = []
    for i in range(3):
        tid = con.execute('select id from topics where name=?', (f'/drone_{i}/motion_capture_state',)).fetchone()[0]
        rows = con.execute('select timestamp, data from messages where topic_id=? and timestamp>=? and timestamp<=? '
                           'order by timestamp', (tid, int((ta - 0.5) * 1e9), int((tb + 0.5) * 1e9))).fetchall()
        t = np.array([r[0] * 1e-9 for r in rows])
        P = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z]
                      for m in (deserialize_message(r[1], MotionCaptureState) for r in rows)])
        out.append((t, P))
    return out


def events(run):
    ev = {}
    s0 = None
    for line in open(os.path.join(run, 'events.txt')):
        parts = line.split(' ', 2)
        t, rest = float(parts[0]), parts[2].strip()
        if s0 is None:
            s0 = t
        def mark(k):
            ev.setdefault(k, t)
        if rest.startswith('clearance complete'):
            mark('t0')
        elif rest.startswith('prefix 1 published'):
            mark('prefix1')
        elif rest.startswith('request seedtraj exit 1 '):
            mark('seedtraj1')
        elif rest.startswith('backend status seed_prefix'):
            mark('seed_accepted')
        elif rest.startswith('backend status prepared'):
            mark('prepared')
        elif rest.startswith('request grant'):
            mark('grant')
        elif rest.startswith('backend authority ack'):
            mark('ack')
        elif rest.startswith('backend status granted'):
            mark('granted')
        elif 'phase CLEAR_DROP_ZONE -> RETURN_TO_START' in rest:
            mark('return')
        elif '-> AT_START' in rest:
            mark('at_start')
        elif '-> LANDING' in rest:
            mark('landing')
        elif 'handoff preparing -> failed' in rest or 'handoff preparing -> landed' in rest:
            mark('failed')
            ev['fail_reason'] = rest.split('(', 1)[1].rstrip(')') if '(' in rest else rest
        elif rest.startswith('backend exit_reject_reason'):
            ev.setdefault('reject_reasons', []).append(rest.split()[-1])
        elif rest.startswith('EXCEPTION'):
            ev['exceptions'] = ev.get('exceptions', 0) + 1
            ev.setdefault('exception_text', rest[:300])
    return s0, ev


def analyze(run):
    name = os.path.basename(run.rstrip('/'))
    cfg = open(os.path.join(run, 'run.txt')).readline().strip()
    s0, ev = events(run)
    r = {'run': name, 'cfg': cfg}
    rel = lambda k: round(ev[k] - s0, 3) if k in ev else ''
    for k in ('t0', 'prefix1', 'seedtraj1', 'seed_accepted', 'prepared', 'grant', 'ack', 'granted', 'return',
              'at_start', 'landing', 'failed'):
        r[k] = rel(k)
    t0 = ev.get('t0')
    r['h_return'] = round(ev['return'] - t0, 3) if 'return' in ev and t0 else ''
    r['h_at_start'] = round(ev['at_start'] - t0, 3) if 'at_start' in ev and t0 else ''
    r['return_to_at_start'] = round(ev['at_start'] - ev['return'], 3) if 'at_start' in ev and 'return' in ev else ''
    r['fail_reason'] = ev.get('fail_reason', '')
    r['reject_reasons'] = '|'.join(ev.get('reject_reasons', []))
    r['exceptions'] = ev.get('exceptions', 0)
    r['exception_text'] = ev.get('exception_text', '')
    # his backend statuses (latest_status changes) in the hand-over window
    bs = [l.split(' ', 2)[2].strip() for l in open(os.path.join(run, 'backend_status.txt'))
          if not l.startswith('#') and t0 and float(l.split()[0]) >= t0 - 0.05]
    r['backend_statuses'] = '|'.join(dict.fromkeys(bs))
    st = [json.loads(l) for l in open(os.path.join(run, 'status.jsonl'))]
    last = next((s for s in reversed(st) if 'candidates_attempted' in s), {})
    for k in ('candidates_attempted', 'candidates_accepted', 'candidates_late', 'candidates_invalidated'):
        r['st_' + k.split('_')[1]] = last.get(k, '')
    # the join: windows the backend published with authority that the mission could still join (stamp <= the first
    # post-grant splice), and the first granted window's stamp minus that splice
    g = [x for x in st if x.get('granted') is True]
    if g:
        sp = g[0].get('splice_time_ns', 0)
        r['granted_first_window_minus_splice_ms'] = round((g[0]['reference_stamp_ns'] - sp) * 1e-6, 2)
        r['granted_windows_before_join'] = len({x['reference_stamp_ns'] for x in g if 0 < x['reference_stamp_ns'] <= sp})
    else:
        r['granted_first_window_minus_splice_ms'] = r['granted_windows_before_join'] = ''
    acc = os.path.join(run, 'accept.txt')
    if os.path.exists(acc):
        rows_a = list(csv.DictReader(open(acc)))
        r['refused_fresh_rounding'] = sum(1 for a in rows_a if a['what'] == 'match' and a['in_refs'] == '1'
                                          and a['result'] == 'none' and abs(float(a['src_age_ms'])) < 0.01)
        r['window_after_join'] = sum(1 for a in rows_a if a['what'] == 'window' and a['result'].startswith('False None'))
    else:
        r['refused_fresh_rounding'] = r['window_after_join'] = 'n/a'
    r['status_reasons'] = '|'.join(dict.fromkeys(s.get('reason', '') for s in st if t0 and s['_t'] >= t0 - 0.05
                                                 and s['_t'] <= (ev.get('return') or 1e18)))
    # replans
    rp = list(csv.DictReader(open(os.path.join(run, 'replans.csv'))))
    tg = ev.get('granted', 1e18)
    pre = [x for x in rp if float(x['request_ros_time']) < tg]
    post = [x for x in rp if float(x['request_ros_time']) >= tg and float(x['request_ros_time']) < ev.get('at_start', 1e18)]
    for tag, rows in (('pre', pre), ('post', post)):
        r[f'{tag}_attempted'] = sum(int(x['attempted']) for x in rows)
        r[f'{tag}_accepted'] = sum(int(x['accepted']) for x in rows)
        r[f'{tag}_installed'] = sum(int(x['authority_installed']) for x in rows)
        r[f'{tag}_late'] = sum(int(x['candidate_late']) for x in rows)
        r[f'{tag}_statuses'] = '|'.join(f'{k}:{v}' for k, v in
                                        sorted({s: sum(1 for x in rows if x['status'] == s)
                                                for s in {x['status'] for x in rows}}.items()))
        rej = [x['installation_rejection'] for x in rows if x['installation_rejection'] not in ('', 'install')]
        r[f'{tag}_install_rejections'] = '|'.join(f'{k}:{rej.count(k)}' for k in dict.fromkeys(rej))
        rt = [float(x['replan_runtime_ms']) for x in rows if int(x['attempted'])]
        r[f'{tag}_runtime_ms_max'] = round(max(rt), 1) if rt else ''
        mg = [float(x['installation_splice_margin_ms']) for x in rows if int(x['authority_installed'])]
        r[f'{tag}_splice_margin_ms_min'] = round(min(mg), 1) if mg else ''
    # reference joins
    tk = list(csv.DictReader(open(os.path.join(run, 'ticks.csv'))))
    T = np.array([float(x['t']) for x in tk])
    steps = np.array([[float(x['step_p']), float(x['step_v']), float(x['step_a'])] for x in tk])
    seq = np.array([int(x['seq']) for x in tk])
    phase = [x['phase'] for x in tk]
    special = set()

    def at(i, key):
        if i is None or i >= len(tk):
            r[key] = ''
            return
        special.add(i)
        r[key] = '/'.join(f'{v:.4f}' for v in steps[i])
    first = lambda cond: next((i for i in range(len(tk)) if cond(i)), None)
    at(first(lambda i: seq[i] == 1), 'join_latch')
    refresh = [i for i in range(1, len(tk)) if seq[i] > seq[i - 1] and seq[i] > 1]
    for i in refresh:
        special.add(i)
    r['n_refresh'] = len(refresh)
    r['join_refresh_max'] = ('/'.join(f'{v:.4f}' for v in np.max(steps[refresh], axis=0)) if refresh else '')
    at(first(lambda i: 'grant' in ev and T[i] >= ev['grant'] - 1e-6), 'join_grant')
    i_ret = first(lambda i: phase[i] == 'RETURN_TO_START')
    at(i_ret, 'join_switch')
    at(first(lambda i: phase[i] == 'AT_START'), 'join_at_start')
    at(first(lambda i: phase[i] == 'LANDING'), 'join_landing')
    ia = first(lambda i: phase[i] == 'AT_START')
    ats = [i for i in range(ia + 1, len(tk)) if phase[i] == 'AT_START'] if ia is not None else []
    r['join_at_start_ticks_max'] = '/'.join(f'{v:.4f}' for v in np.nanmax(steps[ats], axis=0)) if ats else ''
    special.update(ats)
    r['at_to_landing'] = round(ev['landing'] - ev['at_start'], 3) if 'landing' in ev and 'at_start' in ev else ''
    if i_ret is not None:
        cpp = [i for i in range(i_ret + 1, len(tk)) if phase[i] == 'RETURN_TO_START']
        r['join_cpp_ticks_max'] = '/'.join(f'{v:.4f}' for v in np.nanmax(steps[cpp], axis=0)) if cpp else ''
        special.update(cpp)
    else:
        r['join_cpp_ticks_max'] = ''
    for tag, cond in (('live', lambda i: seq[i] == 0), ('prefix', lambda i: seq[i] > 0)):
        other = [i for i in range(1, len(tk)) if i not in special and phase[i] == 'CLEAR_DROP_ZONE' and cond(i)
                 and np.all(np.isfinite(steps[i]))]
        r[f'tick_{tag}_p95'] = '/'.join(f'{v:.4f}' for v in np.percentile(steps[other], 95, axis=0)) if other else ''
        r[f'tick_{tag}_max'] = '/'.join(f'{v:.4f}' for v in np.max(steps[other], axis=0)) if other else ''
    r['tick_ms_max'] = round(max(float(x['tick_ms']) for x in tk), 1)
    # geometry along the executed exit: from the clearance end to AT_START (or the run's end)
    q = np.loadtxt(os.path.join(run, 'q4.csv'), delimiter=',', skiprows=1)
    tend = ev.get('at_start', ev.get('failed', q[-1, 0]))
    flight = cfg.split()[1]
    r['all_zmax'] = round(float(np.max(q[:, 6])), 3)
    r['all_czmax'] = round(float(np.max(q[:, 3])), 3)
    cr = carriers(flight, q[0, 0], q[-1, 0])
    cpos = np.stack([np.stack([np.interp(q[:, 0], t, P[:, j]) for j in range(3)], 1) for t, P in cr], 1)
    # harness health: the mission side's view of his status and reference (receive - stamp), and its tick period
    sl = np.array([float(x['status_lag_ms']) for x in tk if x.get('status_lag_ms') not in (None, '', 'nan')])
    rl = np.array([float(x['ref_lag_ms']) for x in tk if x.get('ref_lag_ms') not in (None, '', 'nan')])
    tp = 1e3 * np.diff(T)
    r['harness_status_lag_p95_ms'] = round(float(np.percentile(sl, 95)), 1) if sl.size else ''
    r['harness_ref_lag_p95_ms'] = round(float(np.percentile(rl, 95)), 1) if rl.size else ''
    r['harness_tick_p95_ms'] = round(float(np.percentile(tp, 95)), 1)
    # (ref_lag is not a harness measure: his pre-grant prepared windows keep their commit's start as stamp)
    r['harness_ok'] = int((not sl.size or np.percentile(sl, 95) < 60) and np.percentile(tp, 95) < 50)
    for tag, a, b in (('exit', t0 or q[0, 0], tend), ('cpp', ev.get('return', 1e18), tend)):
        m = (q[:, 0] >= a) & (q[:, 0] <= b)
        if not m.any():
            for k in ('zmax', 'czmax', 'cxy', 'box_gap', 'tether', 'basket'):
                r[f'{tag}_{k}'] = ''
            continue
        Q = q[m]
        C = cpos[m]
        r[f'{tag}_zmax'] = round(float(np.max(Q[:, 6])), 3)
        r[f'{tag}_czmax'] = round(float(np.max(Q[:, 3])), 3)
        r[f'{tag}_cxy'] = f'{np.max(np.abs(Q[:, 1])):.3f}/{np.max(np.abs(Q[:, 2])):.3f}'
        box, teth, bask = [], [], []
        for row, cc in zip(Q[::2], C[::2]):
            body = row[4:7]
            ring = row[10:13]
            top, bot = body - [0, 0, PIVOT], body - [0, 0, ROD + MAG]
            pts = top + np.linspace(0, 1, 15)[:, None] * (bot - top)
            bask.append(cyl_dist(pts, ring))
            for k in range(3):
                c = cc[k]
                box.append(float(np.max(np.abs(body - c) - BOX)))
                d = c[:2] - ring[:2]
                n = np.linalg.norm(d) or 1.0
                plate = np.array([ring[0] + 0.225 * d[0] / n, ring[1] + 0.225 * d[1] / n, ring[2] + 0.018])
                teth.append(seg_seg(top, bot, c - [0, 0, PIVOT], plate))
        r[f'{tag}_box_gap'] = round(min(box), 3)
        r[f'{tag}_tether'] = round(min(teth), 3)
        r[f'{tag}_basket'] = round(min(bask), 3)
    # where QUAD4 ended: distance to the start hover at AT_START
    return r


def main():
    runs = sys.argv[1:] or sorted(glob.glob(os.path.join(HERE, 'runs', '*')))
    rows = []
    for run in runs:
        try:
            rows.append(analyze(run))
        except Exception as exc:                     # noqa: BLE001
            print(f'{run}: {exc}', file=sys.stderr)
    keys = list(dict.fromkeys(k for r in rows for k in r))
    out = os.environ.get('SUMMARY_OUT', os.path.join(HERE, 'summary.csv'))
    with open(out, 'w', newline='\n') as fh:
        w = csv.DictWriter(fh, keys, lineterminator='\n')
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        print(json.dumps(r))


if __name__ == '__main__':
    main()
