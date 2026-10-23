#!/usr/bin/env python3
"""tools/slip_replay.py -- the two rig release detectors replayed over rig bags (read-only, no node is started).

    source ~/ros2_humble/install/setup.bash; source install/setup.bash
    python3 tools/slip_replay.py [--dates 2026-10-08] [--out DIR] [--cache DIR]
    python3 tools/slip_replay.py --sweep --dates 2026-10-07 2026-10-08 [--out DIR] [--cache DIR]

1. The slip detector (tools/drop_trigger_replay.py, the record's source for "a survivor let go at +0.155 s"): at
   the bag's full mocap rate, each rod's pivot-to-plate distance against its own length (the plate nearest the
   drone's azimuth in the ring frame, the length the median over the first 1-3 s airborne). The rod is rigid, so
   while the magnet holds the error stays within a few mm, and either sign means the magnet is off. A commanded
   release is timed from its OFF (drop_trigger_replay.release_marks: the onset 3 mm off the 1 s baseline on 6
   samples, and |rod error| > 3 cm on 6 samples, within 1.5 s); any other run of |rod error| > 3 cm on 6 samples
   while the drone is still on its plate is an uncommanded release, or a glitch if it comes back within 0.5 s
   under 10 cm. Every episode is kept, not only the first per drone.
2. The stretch detector that flies (dissipative_node._detect_tick; identical at the 8 Oct code b811d90 and on
   rig-prep): at each planner tick, a cable over max(typed, measured rod) by detach_detect_m on two ticks in a row,
   having jumped 3 cm since its last tick within the threshold; nothing when every cable is over at once (a ring
   pose jump); a ring or drone pose older than 0.1 s is not used; active in the planner phase until LAND (with
   drop_on_loss, until the load is down). Replayed at the planner log's own tick times on
     (bag) the bag's poses, the latest sample received at or before each tick (pivot 4 cm below the drone along
           its body z, attach points from the flight's azimuths at radius 0.225 in the ring frame);
     (log) the planner log's len columns, the distances the node itself computed on the flight (as flown). A
           tick on which the flight resized is logged without the leaver: there the leaver's cable is the one the
           T2 line printed, and on later ticks the bag's.
   The node stamps a log row after its solve but reads its poses at the tick's start, 73-87 ms earlier on the
   8 Oct detection ticks; the bag replay uses that start (tick_starts), which leaves about 20 ms of jitter: on a
   rod stretching at 1-2 m/s that is 2-4 cm, enough to move a fire by one tick. The log replay has no such error.
   A drone leaves the replay when the replay detects it, or after the planner's own magnet OFF for a drone it did
   not detect (an announced DETACH). The replay stops at the fleet stop (disarm, abort, envelope fault).
Ground truth: the magnet OFF commands in the bag (the operator's, or the planner's own, which the T2 console logs
as 'magnet OFF ->'), plus the slip detector's releases that no OFF explains (uncommanded). A detection is a fire for
a drone within 1.5 s after its release; a false alarm is a fire for a drone not released (or before its release);
a miss is a release that was not announced and drew no fire while the detector was active.

--sweep: the stretch detector's threshold from 0.02 to 0.12 m on every flight with a planner log and a bag (the
log replay): false alarms over the active time, and the delay from each release (OFF or slip onset) to the fire.
Writes slip_detector.csv, stretch_detector.csv, flights.csv (and sweep_*.csv) to --out and prints the tables.
"""
import argparse
import csv
import glob
import json
import os
import re
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tools'))
import drop_trigger_replay as dtr  # noqa: E402
from run_logs import planner_csvs  # noqa: E402

RIG_DEFAULT = '/home/wesley/multi_drone_control/results/rig'
TYPED_ROD = 0.55
DETECT_M = 0.06           # detach_detect_m on 8 Oct
JUMP_M = 0.03             # the node's jump guard
STALE_S = 0.1             # the node's DETECT_STALE_S
MATCH_S = 1.5             # a fire within this after a release is its detection
SETTLE_S = 0.5            # a separation episode that ends within this is a transient
SWEEP = (0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.10, 0.12)
STAMP = re.compile(r'\[(\d{9,}\.\d+)\]')


def t2_lines(path):
    """(rods by slot at the handover, [(t, drone, cable m)] UNANNOUNCED DETACH, [(t, drone)] planner magnet OFF,
    [(t, drone)] DETACH lines, LAND time) from the T2 console."""
    rods, unann, own, det, land = None, [], [], [], None
    if not os.path.exists(path):
        return rods, unann, own, det, land
    for line in open(path, errors='replace'):
        m = STAMP.search(line)
        if not m:
            continue
        t = float(m.group(1))
        if 'measure_rod_len: rods [' in line:
            rods = [float(v) for v in re.search(r'rods \[([0-9., ]+)\]', line).group(1).split(',')]
        elif 'UNANNOUNCED DETACH:' in line:
            g = re.search(r'/drone_(\d+)\) cable ([0-9.]+) m', line)
            unann.append((t, int(g.group(1)), float(g.group(2))))
        elif '[dissipative] magnet OFF -> /drone_' in line:
            own.append((t, int(re.search(r'/drone_(\d+)/magnet', line).group(1))))
        elif re.search(r'\[dissipative\] DETACH drone \d+ \(/drone_\d+\)', line):
            det.append((t, int(re.search(r'/drone_(\d+)\)', line).group(1))))
        elif 'LAND - descending' in line and land is None:
            land = t
    return rods, unann, own, det, land


def read_planner(logdir):
    """The flight's planner log (largest session): rows as dicts with floats where they parse."""
    fs = sorted(planner_csvs(logdir), key=os.path.getsize)
    if not fs:
        return None, None
    with open(fs[-1]) as f:
        rows = list(csv.DictReader(f))
    prm = json.load(open(os.path.join(os.path.dirname(fs[-1]), 'params.json')))
    return rows, prm


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return np.nan


# ── 1. the slip detector, every episode ───────────────────────────────────────────
def episodes(pd_, win, t_from, t_to):
    """[(onset, t_3cm, t_end of the run, peak |excess| m)] of a drone's sustained rod errors: |excess| over 3 cm on
    6 samples in a row (either sign: the rod is rigid, a drone that drifts inward is off its plate too); onset =
    the first sample of the run 3 mm off the 1.0-0.2 s baseline before it, on the same side."""
    t, e = pd_['t'], pd_['exc']
    ok = (t >= t_from) & (t < t_to) & win
    out, i = [], 0
    idx = np.where(ok & (np.abs(e) > dtr.SEP_M))[0]
    while i < len(idx):
        k = idx[i]
        if k + 5 < len(e) and np.all(np.abs(e[k:k + 6]) > dtr.SEP_M):
            sgn = np.sign(e[k])
            b = (t > t[k] - 1.0) & (t < t[k] - 0.2)
            base = float(np.median(e[b])) if b.any() else 0.0
            j = k
            while j > 0 and sgn * (e[j - 1] - base) > dtr.ONSET_M:
                j -= 1
            end = k
            while end + 1 < len(e) and np.abs(e[end + 1]) > dtr.SEP_M:
                end += 1
            out.append((float(t[j]), float(t[k]), float(t[end]), float(np.max(np.abs(e[k:end + 1])))))
            i = int(np.searchsorted(idx, end + 1))
        else:
            i += 1
    return out


# ── 2. the stretch detector ───────────────────────────────────────────────────────
class Stretch:
    """dissipative_node._detect_tick, state and rule."""

    def __init__(self, detect_m=DETECT_M, typed=TYPED_ROD):
        self.m, self.typed = detect_m, typed
        self.count, self.ok, self.gone = {}, {}, set()

    def tick(self, rods):
        """rods: [(drone, dist or None when stale, measured rod)]; None for a stale ring. Returns the drone fired."""
        if rods is None:
            self.count = {}
            return None
        over = {d: dist - max(self.typed, rod) for d, dist, rod in rods
                if d not in self.gone and dist is not None and np.isfinite(dist)}
        for d, dist, rod in rods:
            if d not in self.gone and (dist is None or not np.isfinite(dist)):
                self.count[d] = 0
        if len(over) >= 2 and all(v > self.m for v in over.values()):
            self.count = {}
            return None
        for d, ov in over.items():
            if ov <= self.m:
                self.count[d], self.ok[d] = 0, ov
                continue
            self.count[d] = self.count.get(d, 0) + 1
            if self.count[d] >= 2 and ov - self.ok.get(d, ov) >= JUMP_M:
                self.count = {}
                self.gone.add(d)
                return d
        return None


def rho_of(az_deg):
    return [np.array([0.225 * np.cos(np.radians(a)), 0.225 * np.sin(np.radians(a)), 0.0]) for a in az_deg]


def bag_dist(mocap, ring_topic, d, rho, tk):
    """|pivot - attach| from the latest ring and drone samples received at or before tk; None if either is stale."""
    R = mocap[ring_topic]
    D = mocap.get(f'/drone_{d}/motion_capture_state')
    if D is None:
        return None
    kr = np.searchsorted(R[:, 0], tk, side='right') - 1
    kd = np.searchsorted(D[:, 0], tk, side='right') - 1
    if kr < 0 or kd < 0 or tk - R[kr, 0] > STALE_S or tk - D[kd, 0] > STALE_S:
        return None
    ring, dr = R[kr], D[kd]
    attach = ring[2:5] + dtr.rotm(ring[5:9]) @ rho
    piv = dr[2:5] + dtr.rotm(dr[5:9]) @ dtr.PIVOT
    return float(np.linalg.norm(attach - piv))


def ring_stale(mocap, ring_topic, tk):
    R = mocap[ring_topic]
    kr = np.searchsorted(R[:, 0], tk, side='right') - 1
    return kr < 0 or tk - R[kr, 0] > STALE_S


TICK_WORK_S = 0.030       # a planner tick's work besides the solve, before its log row is stamped


def tick_starts(rows):
    """Planner tick start times. The node (single-threaded) reads its poses at the start of a tick, detects, solves,
    and stamps the log row after the solve, so a row's len columns are the poses of its start. Start = stamp -
    solve_ms - 30 ms; on 8 Oct the bag at these starts gives the leaver's logged len at the three ticks before the
    detections (73-87 ms before the stamps). A tick that resized also primed the new solver twice: its start is the
    previous start + 0.1 s."""
    out, prev_n, prev = [], None, None
    for r in rows:
        n = r.get('n')
        st = fnum(r['sim_time']) - fnum(r.get('solve_ms')) / 1000.0 - TICK_WORK_S
        if not np.isfinite(st):
            st = fnum(r['sim_time']) - 0.05 - TICK_WORK_S
        if prev is not None and n != prev_n:
            st = prev + 0.1
        out.append(st)
        prev, prev_n = st, n
    return out


def replay_stretch(rows, prm, rods_by_slot, mocap, detect_m, source, rest_z, unann=(), det_lines=(), t_stop=np.inf):
    """Fires [(tick start, drone, over m)], the first count-of-1 tick per drone and the active ticks of the stretch
    detector, on the bag at the tick starts ('bag') or on the logged len columns ('log'). A drone leaves when the
    replay fires for it or after the flight's DETACH line (announced). The tick on which the flight itself fired is
    logged without the leaver: the log replay takes the leaver's cable from the T2 line there, as the node printed
    it."""
    az = [float(v) for v in str(prm.get('attach_azimuths_deg') or '').split(',') if v.strip()] or [0, 90, 180, 270]
    pl = [r for r in rows if r.get('phase') == 'planner']
    if not pl:
        return [], {}, []
    starts = tick_starts(pl)
    s2d0 = [int(v) for v in pl[0]['slot2drone'].split()]
    typed = float(prm.get('cable_len') or TYPED_ROD)      # the flight's own typed rod (1 m from 22 Oct)
    rod_of = {d: (rods_by_slot[i] if rods_by_slot and i < len(rods_by_slot) else typed)
              for i, d in enumerate(s2d0)}
    rho = {d: rho_of(az)[i] for i, d in enumerate(s2d0)}
    announced = [(t, d) for t, d in det_lines if d not in {u[1] for u in unann}]     # the planner's own OFFs
    det = Stretch(detect_m, typed)
    fires, ones, active_t = [], {}, []
    drop = bool(prm.get('drop_on_loss', False))
    for r, tk in zip(pl, starts):
        if tk >= t_stop:                                 # the fleet was disarmed: nothing left to detect
            break
        for t_line, d in announced:
            if tk > t_line:
                det.gone.add(d)
        land = r.get('land') in ('1', '1.0', 'True')
        down = fnum(r['load_z']) <= rest_z + 0.05
        if land and not (drop and not down):
            continue
        active_t.append(tk)
        if source == 'bag':
            if ring_stale(mocap, '/payload/motion_capture_state', tk):
                det.tick(None)
                continue
            rods = [(d, bag_dist(mocap, '/payload/motion_capture_state', d, rho[d], tk), rod_of[d]) for d in s2d0]
        else:
            s2d = [int(v) for v in r['slot2drone'].split()]
            rods = [(d, fnum(r.get(f'len{i}')), rod_of[d]) for i, d in enumerate(s2d)]
            for d in s2d0:                              # a drone the flight already resized away
                if d not in s2d and d not in det.gone:
                    t2 = [c for t_u, dd, c in unann if dd == d and abs(t_u - tk) < 0.06]
                    rods.append((d, t2[0] if t2 else
                                 bag_dist(mocap, '/payload/motion_capture_state', d, rho[d], tk), rod_of[d]))
        before = dict(det.count)
        f = det.tick(rods)
        for d, c in det.count.items():
            if c == 1 and before.get(d, 0) == 0 and d not in ones:
                ones[d] = tk
        if f is not None:
            dist = next(x for dd, x, _ in rods if dd == f)
            fires.append((tk, f, dist - max(typed, rod_of[f])))
    return fires, ones, active_t


# ── one flight ────────────────────────────────────────────────────────────────────
def flight(date, run, rig, cache, detect_ms=(DETECT_M,)):
    dtr.RIG = rig
    bag = os.path.join(rig, date, run)
    logdir = os.path.join(rig, date, run + '_logs')
    fl = dict(date=date, run=run, data=dtr.load_bag(bag, cache),
              ticks=dtr.tracker_ticks(logdir) if os.path.isdir(logdir) else [])
    fl = dtr.analyse(fl)
    out = dict(date=date, run=run, flew=bool(fl.get('flew')), fl=fl)
    if not out['flew']:
        return out
    ring = fl['ring']
    rods, unann, own, det_lines, land = t2_lines(os.path.join(rig, date, run + '.log'))
    rows, prm = read_planner(logdir)
    out.update(t_air0=fl['t_air0'], t_stop=fl['stop_t'], stop=fl['stop_label'], land=land, rods=rods,
               unann=unann, own=own, det_lines=det_lines, prm=prm or {},
               air_s=float(np.sum(np.diff(ring.t)[fl['win'][1:]])))
    # releases: every magnet OFF in the flight, and the slip detector's episodes
    offs = []
    for t, w in fl['offs']:                              # `ros2 topic pub -t 3` sends one OFF three times
        d = int(w.split('_')[1])
        if fl['t_air0'] - 1 <= t <= fl['stop_t'] and not any(dd == d and t - to < 3.0 for to, dd in offs):
            offs.append((t, d))
    out['offs'] = [(t, d, 'planner' if any(abs(t - to) < 0.2 and dd == d for to, dd in own) else 'operator')
                   for t, d in offs]
    eps = []
    for name, p in fl['pdist'].items():
        d = int(p['name'].split('_')[1])
        off_d = min([t for t, dd, _ in out['offs'] if dd == d] + [np.inf])
        # each commanded release from its OFF (drop_trigger_replay.release_marks: 3 mm onset, |rod error| > 3 cm)
        if np.isfinite(off_d):
            on, s3 = dtr.release_marks(p, off_d, min(off_d + MATCH_S, fl['stop_t'] + 0.5))
            eps.append(dict(drone=d, plate_az=p['az'], L=p['L'], onset=on, t3=s3, tend=None, peak=np.nan,
                            kind='commanded', off=off_d))
        # any other sustained rod error while the drone is still on (before its OFF): uncommanded, or a glitch
        for on, t3, tend, peak in episodes(p, fl['win'][p['k']], fl['t_air0'] + 1.0, min(fl['stop_t'], off_d)):
            kind = 'transient (false alarm)' if tend - t3 < SETTLE_S and peak < 0.10 else 'UNCOMMANDED'
            eps.append(dict(drone=d, plate_az=p['az'], L=p['L'], onset=on, t3=t3, tend=tend, peak=peak, kind=kind,
                            off=None))
    out['episodes'] = sorted(eps, key=lambda e: e['onset'] if e['onset'] is not None else e['off'])
    # the largest |rod error| of each rod while it is on (airborne, before its own release)
    out['max_exc'] = {}
    for name, p in fl['pdist'].items():
        d = int(p['name'].split('_')[1])
        rel = min([e['onset'] for e in eps if e['drone'] == d and e['onset'] is not None] +
                  [e['off'] for e in eps if e['drone'] == d and e['off'] is not None] + [np.inf])
        m = fl['win'][p['k']] & (p['t'] >= fl['t_air0'] + 1.0) & (p['t'] < rel - 0.05)
        out['max_exc'][d] = float(np.max(np.abs(p['exc'][m]))) if m.any() else np.nan
    # the stretch detector, as flown and replayed
    if rows:
        rest = float(np.nanmedian([fnum(r['load_z']) for r in rows[:10]]))
        out['stretch'] = {}
        for m_ in detect_ms:
            for src in ('bag', 'log'):
                out['stretch'][(m_, src)] = replay_stretch(rows, prm, rods, fl['data']['mocap'], m_, src, rest,
                                                           unann, own, fl['stop_t'])
        out['detect_m_flown'] = float(prm.get('detach_detect_m', 0.0) or 0.0)
        out['log_len_vs_bag'] = len_check(rows, prm, rods, fl['data']['mocap'])
    return out


def len_check(rows, prm, rods, mocap):
    """Median and 95th-percentile |logged len - bag distance at the tick start| over the planner ticks (cm): the bag
    replay's geometry and tick timing against the node's own."""
    az = [float(v) for v in str(prm.get('attach_azimuths_deg') or '').split(',') if v.strip()] or [0, 90, 180, 270]
    pl = [r for r in rows if r.get('phase') == 'planner']
    if not pl:
        return None
    s2d0 = [int(v) for v in pl[0]['slot2drone'].split()]
    rho = {d: rho_of(az)[i] for i, d in enumerate(s2d0)}
    diffs = []
    for r, tk in zip(pl, tick_starts(pl)):
        for i, d in enumerate(int(v) for v in r['slot2drone'].split()):
            x = bag_dist(mocap, '/payload/motion_capture_state', d, rho[d], tk)
            if x is not None and np.isfinite(fnum(r.get(f'len{i}'))):
                diffs.append(abs(x - fnum(r[f'len{i}'])))
    return (100 * float(np.median(diffs)), 100 * float(np.percentile(diffs, 95)), len(diffs)) if diffs else None


def releases_of(f):
    """[(t, drone, kind)]: the magnet OFFs and the uncommanded slips of a flight."""
    rel = [(t, d, f'{src} OFF') for t, d, src in f['offs']]
    rel += [(e['onset'], e['drone'], 'uncommanded') for e in f['episodes'] if e['kind'] == 'UNCOMMANDED']
    return sorted(rel)


def score_stretch(f, m_, src):
    """(detections [(release, fire, delay)], false alarms [(t, drone, over)], misses [release])."""
    fires, ones, active = f['stretch'][(m_, src)]
    rel = releases_of(f)
    announced = {d for t, d in f['own']} - {d for t, d, _ in f['unann']}
    hits, fa = [], []
    for tf, d, ov in fires:
        r = [x for x in rel if x[1] == d and x[0] - 0.05 <= tf <= x[0] + MATCH_S]
        if r:
            hits.append((r[0], tf, tf - r[0][0], ov))
        else:
            fa.append((tf, d, ov))
    act = (min(active), max(active)) if active else (np.inf, -np.inf)
    miss = [x for x in rel if not any(h[0] == x for h in hits) and act[0] <= x[0] <= act[1]
            and not (x[2] == 'planner OFF' and x[1] in announced)]
    late = [x for x in miss if f['t_stop'] - x[0] < 0.2]          # two ticks cannot fit before the fleet stop
    late += [x for x in rel if not (act[0] <= x[0] <= act[1]) and not any(h[0] == x for h in hits)
             and x[2] == 'uncommanded']                           # after LAND: the node no longer detects
    return hits, fa, [x for x in miss if x not in late], late


# ── tables ────────────────────────────────────────────────────────────────────────
def rel_t(t, f):
    return f"{t - f['t0']:.3f}" if t is not None and np.isfinite(t) else '-'


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--dates', nargs='+', default=['2026-10-08'])
    ap.add_argument('--rig', default=RIG_DEFAULT, help='results/rig of the tree with the bags (read only)')
    ap.add_argument('--out', default=os.path.join(REPO, 'results', 'analysis', 'p6_2026-10-12', '2_slip'))
    ap.add_argument('--cache', default=None, help='directory for the extracted bag arrays')
    ap.add_argument('--sweep', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    ms = SWEEP if a.sweep else (DETECT_M,)
    fls = []
    for date in a.dates:
        for bag in sorted(glob.glob(os.path.join(a.rig, date, '*', '*.db3'))):
            run = os.path.basename(os.path.dirname(bag))
            f = flight(date, run, a.rig, a.cache, ms)
            tk = f['fl'].get('ticks') or []
            f['t0'] = max(x[0] for _, x in tk) if tk else f.get('t_air0', 0.0)   # rig_to_run's t0
            fls.append(f)
            print(f"{date}/{run}: {'flew' if f['flew'] else 'ring never airborne'}", file=sys.stderr)
    (sweep if a.sweep else report)(fls, a.out)


def report(fls, out):
    flown = [f for f in fls if f['flew']]
    fl_rows = [[f['date'], f['run'], 'yes' if f['flew'] else 'no (ring never airborne)',
                f"{f['air_s']:.1f}" if f['flew'] else '', f.get('stop') or ('none' if f['flew'] else ''),
                f"{f.get('detect_m_flown', '')}" if f['flew'] else '',
                ' '.join(f"/drone_{d}:{100 * v:.1f}" for d, v in sorted(f['max_exc'].items())) if f['flew'] else '',
                (f"{f['log_len_vs_bag'][0]:.2f} / {f['log_len_vs_bag'][1]:.2f}" if f.get('log_len_vs_bag') else '')]
               for f in fls]
    head = ['date', 'run', 'ring flew', 'airborne s', 'fleet stop (disarm, abort, fault)', 'detach_detect_m flown',
            'largest rod excess while on (cm)', '|log len - bag| median / p95 (cm)']
    write(out, 'flights.csv', head, fl_rows)
    print('## Flights\n')
    md(head, fl_rows)

    rows = []
    ms = lambda a, b: f'{1000 * (a - b):.0f}' if a is not None and b is not None else '-'  # noqa: E731
    for f in flown:
        for e in f['episodes']:
            if e['kind'] == 'commanded':
                src = next(s_ for t_, d_, s_ in f['offs'] if d_ == e['drone'] and t_ == e['off'])
                verdict = ('detected' if e['t3'] is not None else
                           'separated (3 mm), rod error < 3 cm by +1.5 s' if e['onset'] is not None else 'MISSED')
                rows.append([f['run'], f"/drone_{e['drone']}", f'magnet OFF ({src})', rel_t(e['off'], f),
                             rel_t(e['onset'], f), rel_t(e['t3'], f), ms(e['onset'], e['off']), ms(e['t3'], e['off']),
                             '-', verdict])
            else:
                rows.append([f['run'], f"/drone_{e['drone']}", e['kind'], '-', rel_t(e['onset'], f), rel_t(e['t3'], f),
                             '-', '-', f"{100 * e['peak']:.0f}", 'no OFF explains it'])
    head = ['run', 'drone', 'what', 'OFF (s)', 'onset (s)', '|rod error| > 3 cm (s)', 'onset - OFF (ms)',
            '3 cm - OFF (ms)', 'peak |rod error| (cm)', 'verdict']
    write(out, 'slip_detector.csv', head, rows)
    print('\n## 1. Slip detector (bag, full rate); times on the run clock of tools/rig_to_run.py\n')
    md(head, rows)

    rows = []
    for f in flown:
        if 'stretch' not in f:
            continue
        logged = [(t, d, c) for t, d, c in f['unann']]
        for src in ('log', 'bag'):
            hits, fa, miss, late = score_stretch(f, DETECT_M, src)
            fires = f['stretch'][(DETECT_M, src)][0]
            ones = f['stretch'][(DETECT_M, src)][1]
            for (r, tf, dly, ov) in hits:
                lg = [x for x in logged if x[1] == r[1]]
                rows.append([f['run'], src, f'/drone_{r[1]}', r[2], rel_t(r[0], f), rel_t(ones.get(r[1]), f),
                             rel_t(tf, f), f'{1000 * dly:.0f}', f'{100 * ov:.1f}',
                             rel_t(lg[0][0], f) if lg else 'none logged',
                             f'{1000 * (lg[0][0] - r[0]):.0f}' if lg else '-', 'detection'])
            for (tf, d, ov) in fa:
                rows.append([f['run'], src, f'/drone_{d}', '-', '-', rel_t(ones.get(d), f), rel_t(tf, f), '-',
                             f'{100 * ov:.1f}', '-', '-', 'FALSE ALARM'])
            for r in miss:
                rows.append([f['run'], src, f'/drone_{r[1]}', r[2], rel_t(r[0], f), rel_t(ones.get(r[1]), f),
                             '-', '-', '-', '-', '-', 'MISS'])
            for r in late:
                rows.append([f['run'], src, f'/drone_{r[1]}', r[2], rel_t(r[0], f), rel_t(ones.get(r[1]), f),
                             '-', '-', '-', '-', '-',
                             (f"no fire possible: the fleet stopped {1000 * (f['t_stop'] - r[0]):.0f} ms after it"
                              if f['t_stop'] - r[0] < 0.2 else 'detector not active (after LAND)')])
            if not fires and not miss:
                rows.append([f['run'], src, '-', '-', '-', '-', '-', '-', '-', '-', '-',
                             'no fire, no release to detect' if not releases_of(f) else
                             'no fire (announced release: the planner already knew)'])
    head = ['run', 'replayed on', 'drone', 'release', 'release (s)', 'first tick over (s)', 'fire (s)',
            'fire - release (ms)', 'over at fire (cm)', 'flight logged (s)', 'logged - release (ms)', 'verdict']
    write(out, 'stretch_detector.csv', head, rows)
    print(f'\n## 2. Stretch detector (detach_detect_m {DETECT_M}), replayed at the planner ticks\n')
    md(head, rows)


def sweep(fls, out):
    flown = [f for f in fls if f['flew'] and 'stretch' in f]
    rows_fa, rows_d = [], []
    for m_ in SWEEP:
        n_fa, air, delays = 0, 0.0, []
        for f in flown:
            hits, fa, miss, late = score_stretch(f, m_, 'log')
            act = f['stretch'][(m_, 'log')][2]
            air += 0.1 * len(act)
            n_fa += len(fa)
            for (r, tf, dly, ov) in hits:
                delays.append((f['run'], r[1], r[2], dly))
            for (tf, d, ov) in fa:
                rows_fa.append([f'{m_:.2f}', f['date'], f['run'], f'/drone_{d}', rel_t(tf, f), f'{100 * ov:.1f}'])
            for r in miss:
                delays.append((f['run'], r[1], r[2], np.nan))
        for run, d, kind, dly in delays:
            rows_d.append([f'{m_:.2f}', run, f'/drone_{d}', kind, '-' if not np.isfinite(dly) else f'{1000 * dly:.0f}'])
        rows_fa.append([f'{m_:.2f}', 'all', f'{len(flown)} flights', f'{air:.0f} s active', f'{n_fa} false alarms', ''])
    write(out, 'sweep_false_alarms.csv', ['threshold m', 'date', 'run', 'drone', 'fire (s)', 'over (cm)'], rows_fa)
    write(out, 'sweep_delays.csv', ['threshold m', 'run', 'drone', 'release', 'fire - release (ms)'], rows_d)
    print('## Stretch-detector sweep (log replay, as flown)\n')
    md(['threshold m', 'date', 'run', 'drone', 'fire (s)', 'over (cm)'], rows_fa)
    md(['threshold m', 'run', 'drone', 'release', 'fire - release (ms)'], rows_d)


def write(out, name, head, rows):
    with open(os.path.join(out, name), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(head)
        w.writerows(rows)


def md(head, rows):
    print('| ' + ' | '.join(head) + ' |')
    print('|' + '---|' * len(head))
    for r in rows:
        print('| ' + ' | '.join(str(x) for x in r) + ' |')
    print()


if __name__ == '__main__':
    main()
