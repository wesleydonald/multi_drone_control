#!/usr/bin/env python3
"""Replay candidate ring-tilt drop triggers over the rig flights of 30 Sep, 1 Oct and 7 Oct 2026.

Read-only. Ring and drone poses come from the rosbag2 bags (results/rig/<date>/<run>/*.db3, read
with rosbag2_py; no node is started, nothing is published). Tracker tick times come from the
trackers' log.csv. Flights with no ring pose in a bag fall back to the planner log (10 Hz).
Prints the tables of docs/experiments/2026-10-08_drop_trigger_replay.md.

    source ~/ros2_humble/install/setup.bash; source install/setup.bash
    python3 tools/drop_trigger_replay.py [--cache DIR]

--cache keeps the extracted bag arrays (one pickle per bag), so a second run takes about a minute.
"""
import argparse
import csv
import glob
import os
import pickle
import re
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tools'))
from run_logs import drone_of, node_csvs, planner_csvs  # noqa: E402
from utility_objects.safety import EnvelopeLimits  # noqa: E402

DATES = ('2026-09-30', '2026-10-01', '2026-10-07')
RIG = os.path.join(REPO, 'results', 'rig')
RING = '/payload/motion_capture_state'

AIR_DZ = 0.05            # airborne: ring z this far above its first-second median
TRIG = 30.0              # Wesley's ring-tilt bar (card 2026-10-08_drop_and_land)
RATE_FLOOR = 25.0        # rule (c): tilt floor of the rate rule
BACKSTOP = 45.0          # rule (d): slow-tilt backstop
LIM = EnvelopeLimits()   # the trackers' latch today: 60 deg on 3 consecutive ticks
TICK_S = 0.02            # tracker period (50 Hz)
PIVOT = np.array([0.0, 0.0, -0.04])   # rig pivot_offset (params.json)
PLATE_R = 0.225          # rig attach_radius
SEP_M = 0.03             # off the plate: pivot-to-plate distance past the rod length + 3 cm
ONSET_M = 0.003          # first visible separation: 3 mm over the pre-release baseline
EVENT_GAP_S = 1.0        # releases closer than this are one event
GLITCH_MARGIN = 3.0      # S = this x the largest physical attitude step in a real capsize


# ── bag reading ──────────────────────────────────────────────────────────────────
def read_bag(bagdir):
    """{'mocap': {topic: array[t_bag, t_stamp, x, y, z, qw, qx, qy, qz]}, 'events': [...]}.
    events: (t_bag, topic, rosout stamp or None, node or None, text). A bag with no ring pose
    returns no mocap (its drone topics are not read)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bagdir, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {'mocap': {}, 'events': [], 'topics': sorted(types)}
    if RING not in types:
        return out
    want = [t for t in types if t.endswith('motion_capture_state') or t.endswith('/magnet')
            or t in ('/fleet/abort', '/fleet/command', '/rosout')]
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}
    rows = {}
    while r.has_next():
        topic, data, tb = r.read_next()
        msg = deserialize_message(data, cls[topic])
        tb *= 1e-9
        if topic.endswith('motion_capture_state'):
            p, q = msg.pose.position, msg.pose.orientation
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            rows.setdefault(topic, []).append((tb, st, p.x, p.y, p.z, q.w, q.x, q.y, q.z))
        elif topic == '/rosout':
            out['events'].append((tb, topic, msg.stamp.sec + msg.stamp.nanosec * 1e-9,
                                  msg.name, msg.msg))
        else:
            out['events'].append((tb, topic, None, None, msg.data))
    out['mocap'] = {k: np.array(v) for k, v in rows.items()}
    return out


def load_bag(bagdir, cache):
    key = None
    if cache:
        os.makedirs(cache, exist_ok=True)
        key = os.path.join(cache, os.path.relpath(bagdir, RIG).replace(os.sep, '__') + '.pkl')
        if os.path.exists(key):
            with open(key, 'rb') as f:
                return pickle.load(f)
    data = read_bag(bagdir)
    if key:
        with open(key, 'wb') as f:
            pickle.dump(data, f)
    return data


# ── geometry ─────────────────────────────────────────────────────────────────────
def unit(q):
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def tilt_deg(q):
    """Angle of body z from world z, as safety.tilt_deg_from_quat (q = w, x, y, z)."""
    q = unit(q)
    return np.degrees(np.arccos(np.clip(1.0 - 2.0 * (q[..., 1] ** 2 + q[..., 2] ** 2), -1, 1)))


def att_step(qa, qb):
    """Rotation angle of qa^-1 qb, degrees; sign-invariant."""
    d = np.abs(np.sum(unit(qa) * unit(qb), axis=-1))
    return np.degrees(2.0 * np.arccos(np.clip(d, -1.0, 1.0)))


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def rotm(q):
    w, x, y, z = np.moveaxis(unit(q), -1, 0)
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                     np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                     np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)],
                    -2)


def plate_of_azimuth(az_deg):
    """As mpc_planner.geometry.plate_of_azimuth: plate 0 on the ring's +x, counting clockwise."""
    return int(round(-float(az_deg) / 30.0)) % 12


# ── ring, drones, events ─────────────────────────────────────────────────────────
class Ring:
    def __init__(self, P):
        ok = ~np.isnan(P).any(axis=1)
        self.n_nan = int((~ok).sum())
        P = P[ok]
        self.t, self.st, self.z = P[:, 0], P[:, 1], P[:, 4]
        self.pos = P[:, 2:5]
        self.q = P[:, 5:9]
        self.tilt = tilt_deg(self.q)
        self.dt = np.r_[np.nan, np.diff(self.t)]
        self.dst = np.r_[np.nan, np.diff(self.st)]
        self.step = np.r_[np.nan, att_step(self.q[:-1], self.q[1:])]
        qn = unit(self.q)
        self.flips = int(np.sum(np.sum(qn[1:] * qn[:-1], axis=1) < 0))
        self.rest_z = float(np.median(self.z[self.t < self.t[0] + 1.0]))
        self.air = self.z > self.rest_z + AIR_DZ

    def at(self, t):
        """Index of the latest sample received at or before t (-1 if none)."""
        return np.searchsorted(self.t, t, side='right') - 1


def tracker_ticks(logdir):
    """[(drone, tick times)] per tracker session (new and old log layouts)."""
    out = []
    for f in node_csvs(logdir, 'tracker'):
        with open(f) as fh:
            rd = csv.reader(fh)
            head = next(rd, None)
            if not head or 'sim_time' not in head:
                continue
            i = head.index('sim_time')
            t = np.unique([float(r[i]) for r in rd if len(r) > i and r[i]])
        if len(t) > 1:
            out.append((drone_of(f), t))
    return out


def armed_mask(t, ticks, pad=0.5):
    m = np.zeros(len(t), bool)
    for _, tk in ticks:
        m |= (t >= tk[0]) & (t <= tk[-1] + pad)
    return m


def stop_event(events):
    """First fleet disarm: operator DISARM, /fleet/abort or a tracker ENVELOPE FAULT."""
    c = []
    for tb, topic, st, node, text in events:
        if topic == '/fleet/command' and text == 'DISARM':
            c.append((tb, 'operator DISARM'))
        elif topic == '/fleet/abort':
            c.append((tb, '/fleet/abort'))
        elif topic == '/rosout' and 'ENVELOPE FAULT' in text:
            c.append((st, f'{node} ENVELOPE FAULT'))
    return min(c) if c else (np.inf, None)


def latch_of(fl):
    """First tracker ENVELOPE FAULT in the bag's /rosout: (its stamp, node, text)."""
    c = [(st, node, text) for tb, topic, st, node, text in fl['data']['events']
         if topic == '/rosout' and 'ENVELOPE FAULT' in text]
    return min(c) if c else (None, None, None)


def plate_distances(ring, drones, t0, t1):
    """Per drone topic: plate azimuth, rod length L and the pivot-to-plate excess over L.
    The plate is the 30 deg grid point nearest the drone's mean azimuth in the ring frame over
    [t0, t1] (airborne, rods taut); L is the median distance there. The rod is rigid, so while
    the magnet holds the excess stays within a few mm."""
    out = {}
    for topic, D in sorted(drones.items()):
        D = D[~np.isnan(D).any(axis=1)]
        k = ring.at(D[:, 0])
        D, k = D[k >= 0], k[k >= 0]
        Rr, Rd = rotm(ring.q[k]), rotm(D[:, 5:9])
        piv = D[:, 2:5] + Rd @ PIVOT
        rel = np.einsum('nji,nj->ni', Rr, piv - ring.pos[k])
        win = (D[:, 0] > t0) & (D[:, 0] < t1)
        if win.sum() < 20:
            continue
        az = np.arctan2(rel[win, 1], rel[win, 0])
        az = float(np.degrees(np.arctan2(np.median(np.sin(az)), np.median(np.cos(az)))))
        paz = np.radians(round(az / 30.0) * 30.0)
        rho = np.array([PLATE_R * np.cos(paz), PLATE_R * np.sin(paz), 0.0])
        d = np.linalg.norm(piv - (ring.pos[k] + Rr @ rho), axis=1)
        L = float(np.median(d[win]))
        out[topic] = dict(t=D[:, 0], k=k, exc=d - L, L=L, az=az, plate=plate_of_azimuth(az),
                          z=D[:, 4], name=topic.split('/')[1])
    return out


def separation(pd, t_from, t_to, valid=None, base=None):
    """First sustained (6 samples) excess > SEP_M in [t_from, t_to), and the onset: the first
    sample of the run above baseline + ONSET_M that ends there. Baseline = `base`, else the
    median excess over the 1.0-0.2 s before the crossing. Returns (onset, t3cm) or None."""
    t, e = pd['t'], pd['exc']
    m = (t >= t_from) & (t < t_to) & (e > SEP_M)
    if valid is not None:
        m &= valid
    for i in np.where(m)[0]:
        if i + 5 < len(e) and np.all(e[i:i + 6] > SEP_M):
            if base is None:
                b = (t > t[i] - 1.0) & (t < t[i] - 0.2)
                base = float(np.median(e[b])) if b.any() else 0.0
            j = i
            while j > 0 and e[j - 1] > base + ONSET_M:
                j -= 1
            return t[j], t[i]
    return None


def release_marks(pd, t_ref, t_end):
    """Against the median excess over the 1 s before t_ref: the separation (6 samples in a row
    > baseline + ONSET_M) and |rod error| > SEP_M (6 in a row; the rod is rigid, so either
    sign means the magnet is off), first in (t_ref, t_end). Returns (onset, t3cm), None if not."""
    t, e = pd['t'], pd['exc']
    pre = (t > t_ref - 1.0) & (t < t_ref)
    base = float(np.median(e[pre])) if pre.any() else 0.0
    after = np.where((t > t_ref) & (t < t_end))[0]
    on = next((t[i] for i in after if np.all(e[i:i + 6] > base + ONSET_M)), None)
    s3 = next((t[i] for i in after if np.all(np.abs(e[i:i + 6]) > SEP_M)), None)
    return on, s3


def crossing(ring, thr, t_from, t_to=np.inf):
    i = np.where((ring.t >= t_from) & (ring.t < t_to) & (ring.tilt > thr))[0]
    return (ring.t[i[0]], i[0]) if len(i) else (None, None)


def tilt_rate(ring):
    return np.r_[np.nan, np.diff(ring.tilt) / np.maximum(np.diff(ring.st), 1e-4)]


# ── trigger rules ────────────────────────────────────────────────────────────────
class Rule:
    """Fires when any clause holds on n consecutive accepted airborne samples. A clause is
    (tilt threshold, n, rate threshold or None); the rate is frame to frame on the mocap stamps.
    With glitch=True a sample whose attitude step from the previous sample exceeds S (per
    nominal frame, scaled up by the elapsed stamp time) resets every count and is ignored."""

    def __init__(self, name, clauses, glitch=False):
        self.name, self.clauses, self.glitch = name, clauses, glitch


def run_rule(rule, ring, seq_t, idx, s_rate, dt_nom, once=True):
    """Fire times of `rule` over the sample sequence idx (consecutive distinct samples), seen at
    times seq_t. After a fire the rule re-arms once no clause holds."""
    counts = [0] * len(rule.clauses)
    fires, prev, armed = [], None, True
    for ts, k in zip(seq_t, idx):
        p, prev = prev, k
        if not ring.air[k]:
            counts = [0] * len(counts)
            armed = True
            continue
        if p is None:
            continue
        dts = max(ring.st[k] - ring.st[p], 1e-4)
        if rule.glitch and att_step(ring.q[p], ring.q[k]) > s_rate * max(dts, dt_nom):
            counts = [0] * len(counts)
            continue
        rate = (ring.tilt[k] - ring.tilt[p]) / dts
        any_cond = False
        for j, (thr, n, rthr) in enumerate(rule.clauses):
            cond = ring.tilt[k] > thr and (rthr is None or rate > rthr)
            any_cond |= cond
            counts[j] = counts[j] + 1 if cond else 0
            if counts[j] >= n and armed:
                fires.append(ts)
                armed = False
                if once:
                    return fires
        if not any_cond:
            armed = True
    return fires


def mocap_sequence(ring, t0, t1):
    idx = np.where((ring.t >= t0) & (ring.t < t1))[0]
    return ring.t[idx], idx


def tick_times(ring, tk, t0, t1):
    """One tracker's ticks in [t0, t1); past the log's end (the faulting tick is never logged)
    they are extrapolated at 50 Hz."""
    t1 = min(t1, ring.t[-1])
    tt = np.r_[tk, np.arange(tk[-1] + TICK_S, t1 + TICK_S, TICK_S)]
    return tt[(tt >= t0) & (tt < t1)]


def tick_sequences(ring, ticks, t0, t1):
    """Per tracker: the distinct ring samples it sees at its ticks (a stale repeat is no new
    sample) and the tick time each was first seen."""
    out = []
    for drone, tk in ticks:
        tt = tick_times(ring, tk, t0, t1)
        k = ring.at(tt)
        keep = (k >= 0) & np.r_[True, np.diff(k) != 0]
        out.append((drone, tt[keep], k[keep]))
    return out


def latch_replay(ring, tk, t0, t1):
    """Today's check at one tracker's ticks: payload tilt > 60 on 3 consecutive ticks (a stale
    pose counts again, as in EnvelopeChecker). Returns (tick time, tilt seen) or (None, None)."""
    n = 0
    for t in tick_times(ring, tk, t0, t1):
        k = ring.at(t)
        if k < 0:
            continue
        n = n + 1 if ring.tilt[k] > LIM.max_payload_tilt_deg else 0
        if n >= LIM.fault_debounce:
            return t, ring.tilt[k]
    return None, None


# ── flights ──────────────────────────────────────────────────────────────────────
def flights(cache):
    out = []
    for date in DATES:
        for bag in sorted(glob.glob(os.path.join(RIG, date, '*', '*.db3'))):
            bagdir = os.path.dirname(bag)
            run = os.path.basename(bagdir)
            logdir = os.path.join(RIG, date, run + '_logs')
            out.append(dict(date=date, run=run, data=load_bag(bagdir, cache),
                            ticks=tracker_ticks(logdir) if os.path.isdir(logdir) else []))
    return out


def analyse(fl):
    """Ring series, flight window, releases (magnet OFF or a drone off its plate with no
    command), events (releases within EVENT_GAP_S) and whether each event capsized."""
    d = fl['data']
    fl['ring'] = None
    if RING not in d['mocap']:
        return fl
    ring = fl['ring'] = Ring(d['mocap'][RING])
    ev = d['events']
    fl['stop_t'], fl['stop_label'] = stop_event(ev)
    fl['takeoff'] = min([e[0] for e in ev if e[1] == '/fleet/command' and e[4] == 'TAKEOFF'],
                        default=None)
    fl['offs'] = sorted((e[0], e[1].split('/')[1]) for e in ev
                        if e[1].endswith('/magnet') and e[4] == 'OFF')
    fl['armed'] = armed_mask(ring.t, fl['ticks']) if fl['ticks'] else np.ones(len(ring.t), bool)
    fl['win'] = ring.air & fl['armed'] & (ring.t < fl['stop_t'])
    fl['flew'] = bool(fl['win'].any())
    fl['events'] = []
    if not fl['flew']:
        return fl
    fl['t_air0'] = t_air0 = ring.t[fl['win']][0]
    drones = {k: v for k, v in d['mocap'].items() if k != RING}
    fl['pdist'] = pdist = plate_distances(ring, drones, t_air0 + 1.0, t_air0 + 3.0)
    rel = [dict(t=t, kind='magnet OFF', who=w) for t, w in fl['offs'] if t < fl['stop_t']]
    for name, pd in pdist.items():
        s = separation(pd, t_air0 + 1.0, fl['stop_t'], valid=fl['win'][pd['k']])
        if s and not any(r['who'] == pd['name'] and r['t'] <= s[0] for r in rel):
            rel.append(dict(t=s[0], kind='off its plate, no command', who=pd['name']))
    rel.sort(key=lambda r: r['t'])
    for r in rel:
        if fl['events'] and r['t'] - fl['events'][-1]['rel'][-1]['t'] < EVENT_GAP_S:
            fl['events'][-1]['rel'].append(r)
        else:
            fl['events'].append(dict(t=r['t'], rel=[r]))
    for i, e in enumerate(fl['events']):
        e['end'] = fl['events'][i + 1]['t'] if i + 1 < len(fl['events']) else fl['stop_t'] + 0.5
        e['capsize'] = crossing(ring, LIM.max_payload_tilt_deg, e['t'], e['end'])[0] is not None
    return fl


# ── tables ───────────────────────────────────────────────────────────────────────
def fmt(t, base=None, nd=3):
    if t is None or not np.isfinite(t):
        return '-'
    return f'{t % 1000:.{nd}f}' if base is None else f'{t - base:+.{nd}f}'


def md(head, rows):
    print('| ' + ' | '.join(head) + ' |')
    print('|' + '---|' * len(head))
    for r in rows:
        print('| ' + ' | '.join(str(x) for x in r) + ' |')
    print()


def drone_label(fl, name):
    pd = next((v for v in fl['pdist'].values() if v['name'] == name), None)
    return f'/{name}' + (f" (az {pd['az']:+.0f}, plate {pd['plate']})" if pd else '')


def physical_steps(caps, dt_nom):
    """Largest attitude step per nominal frame inside a real capsize (first release to latch)."""
    out = []
    for fl, e in caps:
        ring = fl['ring']
        sel = (ring.t > e['t']) & (ring.t <= latch_of(fl)[0])
        s = ring.step[sel] * dt_nom / np.maximum(ring.dst[sel], dt_nom)
        out.append((fl['run'], float(np.nanmax(s)), float(np.nanmax(ring.step[sel]))))
    return out


def rate_bounds(flown):
    """Tilt rates above RATE_FLOOR (deg/s): the slowest of the first three samples of each
    capsize, and the fastest after a release the ring recovered from."""
    cap, rec = [], []
    for fl in flown:
        ring = fl['ring']
        rate = tilt_rate(ring)
        for e in fl['events']:
            sel = np.where((ring.t >= e['t']) & (ring.t < e['end']) & ring.air &
                           (ring.tilt > RATE_FLOOR))[0]
            if not len(sel):
                continue
            if e['capsize']:
                cap.append((fl['run'], float(np.min(rate[sel[:3]]))))
            else:
                rec.append((fl['run'], float(np.max(rate[sel]))))
    return cap, rec


def evaluate(rule, fl, rate_name, s_rate, dt_nom):
    """First fire per segment: before any release (false fire), per event (capsize or
    recovered). 50 Hz: per tracker; the fleet fires at the first tracker."""
    ring = fl['ring']
    segs = [(fl['t_air0'] - 1.0, fl['events'][0]['t'] if fl['events'] else fl['stop_t'], None)]
    segs += [(e['t'], e['end'], e) for e in fl['events']]
    out = []
    for t0, t1, e in segs:
        if rate_name == 'mocap':
            seqs = [(None, *mocap_sequence(ring, t0, t1))]
        else:
            tk = fl['ticks'] or [(None, np.arange(ring.t[0], ring.t[-1], TICK_S))]
            seqs = tick_sequences(ring, tk, t0, t1)
        per = []
        for drone, st, idx in seqs:
            m = fl['win'][idx] if e is None else np.ones(len(idx), bool)
            per.append((drone, run_rule(rule, ring, st[m], idx[m], s_rate, dt_nom,
                                        once=e is not None)))
        out.append((t0, t1, e, per))
    return out


def synthetic(ring, rules, s_rate, dt_nom, t_at):
    """Mis-fit ring poses injected into a quiet hover: (label, frames, rotation about body x)."""
    cases = [('mis-fit 90 deg', 1, 90), ('mis-fit 90 deg', 2, 90), ('mis-fit 90 deg', 3, 90),
             ('mis-fit 90 deg', 5, 90), ('mis-fit 90 deg, held', 50, 90),
             ('upside down', 1, 180), ('mis-fit 35 deg', 1, 35), ('mis-fit 35 deg, held', 50, 35)]
    k0 = int(ring.at(t_at))
    out = []
    for label, nfr, ang in cases:
        r = Ring.__new__(Ring)
        r.__dict__.update({k: (v.copy() if isinstance(v, np.ndarray) else v)
                           for k, v in ring.__dict__.items()})
        h = np.radians(ang) / 2.0
        for k in range(k0, k0 + nfr):
            r.q[k] = qmul(r.q[k], np.array([np.cos(h), np.sin(h), 0.0, 0.0]))
        r.tilt = tilt_deg(r.q)
        seq_t, idx = mocap_sequence(r, ring.t[k0] - 1.0, ring.t[k0] + 2.0)
        res = ['FIRES' if run_rule(rule, r, seq_t, idx, s_rate, dt_nom) else 'no'
               for rule in rules]
        out.append([f'{label}, {nfr} frame{"s" if nfr > 1 else ""}',
                    f'{float(np.max(r.tilt[k0:k0 + nfr])):.0f}'] + res)
    return out


def planner_log_flights(have_ring, R):
    """Flights with no ring pose in a bag: ring tilt from the planner log (10 Hz), counted while a
    tracker log is being written (armed) and the planner's load z is airborne. A session copied
    into two run folders is reported once, under both names."""
    sessions = {}
    for date in DATES:
        for logdir in sorted(glob.glob(os.path.join(RIG, date, '*_logs'))):
            run = os.path.basename(logdir)[:-5]
            if (date, run) in have_ring:
                continue
            ticks = tracker_ticks(logdir)
            for f in planner_csvs(logdir):
                key = (date, os.path.basename(os.path.dirname(f)))
                if key in sessions:
                    sessions[key]['runs'].append(run)
                else:
                    sessions[key] = dict(runs=[run], f=f, ticks=ticks)
    rows = []
    for (date, sess), v in sorted(sessions.items(), key=lambda kv: (kv[0][0], kv[1]['runs'][0])):
        with open(v['f']) as fh:
            rd = list(csv.DictReader(fh))
        if len(rd) < 20 or 'tilt_deg' not in rd[0]:
            continue
        t = np.array([float(r['sim_time']) for r in rd])
        z = np.array([float(r['load_z']) for r in rd])
        tilt = np.array([float(r['tilt_deg']) for r in rd])
        land = np.array([r.get('land', '0') in ('1', '1.0', 'True') for r in rd])
        rest = float(np.median(z[t < t[0] + 1.0]))
        win = (z > rest + AIR_DZ) & armed_mask(t, v['ticks'], pad=0.0)
        if not win.any():
            continue
        i = int(np.argmax(np.where(win, tilt, -1)))
        over = win & (tilt > TRIG)
        rate = np.r_[np.nan, np.diff(tilt) / np.diff(t)]
        c = win & (tilt > RATE_FLOOR) & (rate > R)
        c2 = c & np.r_[False, c[:-1]]
        r25 = win & (tilt > RATE_FLOOR)
        rows.append([date, ' / '.join(v['runs']), sess[-6:], f'{np.sum(win) * 0.1:.0f}',
                     f'{tilt[i]:.1f}', f'{TRIG - tilt[i]:+.1f}', f'{over.sum() * 0.1:.1f}',
                     f'{z[i]:.2f}', 'yes' if land[i] else 'no',
                     f'{np.nanmax(rate[r25]):.0f}' if r25.any() else '-',
                     'yes' if over.any() else 'no', 'yes' if c.any() else 'no',
                     'yes' if c2.any() else 'no'])
    return rows


# ── main ─────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--cache', help='directory for the extracted bag arrays (optional)')
    a = ap.parse_args()

    fls = [analyse(f) for f in flights(a.cache)]
    rings = [f for f in fls if f['ring'] is not None]
    flown = [f for f in rings if f['flew']]
    caps = [(f, e) for f in flown for e in f['events'] if e['capsize']]
    dt_nom = float(np.median(np.concatenate([f['ring'].dst[1:] for f in rings])))

    # the two data-driven constants
    steps = physical_steps(caps, dt_nom)
    s_frame = float(np.ceil(GLITCH_MARGIN * max(s for _, s, _ in steps) / 5.0) * 5.0)
    s_rate = s_frame / dt_nom
    cap_rates, rec_rates = rate_bounds(flown)
    r_lo = max([r for _, r in rec_rates] + [1.0])
    R = float(np.floor(np.sqrt(r_lo * min(r for _, r in cap_rates)) / 10.0) * 10.0)

    rules = [Rule(f'a{n}: tilt > {TRIG:.0f}, {n} sample{"s" * (n > 1)}', [(TRIG, n, None)])
             for n in (1, 2, 3)]
    rules += [Rule(f'b{n}: a{n} + glitch reset', [(TRIG, n, None)], glitch=True)
              for n in (1, 2, 3)]
    rules += [Rule(f'c{n}: tilt > {RATE_FLOOR:.0f} and rate > {R:.0f} deg/s, {n} sample'
                   f'{"s" * (n > 1)}, glitch reset', [(RATE_FLOOR, n, R)], glitch=True)
              for n in (1, 2)]
    rules += [Rule(f'd: c2, or tilt > {BACKSTOP:.0f} on 2 samples; glitch reset',
                   [(RATE_FLOOR, 2, R), (BACKSTOP, 2, None)], glitch=True)]

    print(f'<!-- tools/drop_trigger_replay.py: nominal mocap frame {dt_nom * 1e3:.2f} ms, '
          f'S {s_frame:.0f} deg/frame, R {R:.0f} deg/s -->\n')

    # 1. per-event timeline
    print('### Per-event timeline\n')
    print('Bag clock (recorder receive time; rosout stamps are the same host clock), shown as '
          'epoch seconds mod 1000. Offsets from the event\'s first release (the magnet OFF, '
          'or the onset of an uncommanded release).\n')
    rows = []
    for fl in flown:
        ring = fl['ring']
        for e in fl['events']:
            t0 = e['t']
            tag = f"{fl['run']} {'capsize' if e['capsize'] else 'recovered'}"
            ev_rows = []

            def add(t, what, k=None):
                ev_rows.append((t, [tag, what, fmt(t), fmt(t, t0),
                                    f'{ring.z[k]:.2f}' if k is not None else '',
                                    f'{ring.tilt[k]:.1f}' if k is not None else '']))
            for r in e['rel']:
                who = drone_label(fl, r['who'])
                pd = next((v for v in fl['pdist'].values() if v['name'] == r['who']), None)
                if r['kind'] == 'magnet OFF':
                    add(r['t'], f'magnet OFF -> /{r["who"]}/magnet (bag)', ring.at(r['t']))
                    w = [x for x in fl['data']['events'] if x[1] == '/rosout' and 'magnet OFF' in
                         x[4] and 'elrs' in (x[3] or '') and r['t'] - 0.01 <= x[2] < r['t'] + 0.5]
                    if w:
                        add(w[0][2], f'radio write ({w[0][3]}, rosout stamp)')
                if pd is None:
                    continue
                if r is e['rel'][0] and r['kind'] != 'magnet OFF':
                    on, s3 = separation(pd, r['t'], e['end']) or (None, None)
                else:
                    on, s3 = release_marks(pd, t0, e['end'])
                cmd = 'commanded' if r['kind'] == 'magnet OFF' else 'no command'
                if on is not None:
                    add(on, f'{who} off its plate ({cmd}): separation > {ONSET_M * 1e3:.0f} mm',
                        ring.at(on))
                if s3 is not None:
                    add(s3, f'{who}: rod error > {SEP_M * 100:.0f} cm (rod {pd["L"]:.3f} m)')
            for thr in (TRIG, BACKSTOP, LIM.max_payload_tilt_deg):
                tc, k = crossing(ring, thr, t0, e['end'])
                if tc is not None:
                    add(tc, f'ring tilt > {thr:.0f}', k)
            for x in fl['data']['events']:
                if x[1] == '/fleet/command' and x[4] == 'LAND' and t0 <= x[0] < e['end']:
                    add(x[0], 'operator LAND (/fleet/command, bag)')
                    break
            if e['capsize']:
                w = [x for x in fl['data']['events'] if x[1] == '/rosout' and
                     'envelope warning' in x[4] and x[2] >= t0]
                if w:
                    add(w[0][2], f'first 40 deg warning ({w[0][3]}, rosout stamp)')
                lt, node, text = latch_of(fl)
                add(lt, f'**LATCH** {node} "{text.split(": ", 1)[-1][:30]}" (rosout stamp)',
                    ring.at(lt))
                ab = [x[0] for x in fl['data']['events'] if x[1] == '/fleet/abort']
                if ab:
                    add(ab[0], '/fleet/abort (bag)')
            else:
                sel = np.where((ring.t >= t0) & (ring.t < e['end']) & ring.air)[0]
                k = sel[np.argmax(ring.tilt[sel])]
                add(ring.t[k], 'peak ring tilt (then recovered)', k)
            rows += [r for _, r in sorted(ev_rows, key=lambda x: x[0])]
    md(['event', 'what (source)', 'time', 'offset (s)', 'ring z (m)', 'tilt (deg)'], rows)

    # 2. rule results
    print(f'### Trigger rules (S = {s_frame:.0f} deg per {dt_nom * 1e3:.0f} ms frame = '
          f'{s_rate:.0f} deg/s; R = {R:.0f} deg/s)\n')
    print('Capsize columns: fire time minus the logged latch, ms (negative = before the latch; '
          'after it = FAIL). "mocap" = checked on every ring sample (bag time); "50 Hz" = '
          'checked at the trackers\' logged ticks, fleet = first tracker; "min" = the smallest '
          'lead of any tracker over its own replayed 60 deg latch.\n')
    rows = []
    for rate_name in ('mocap', '50 Hz'):
        for rule in rules:
            cap_cells, rec, false = {}, [], []
            for fl in flown:
                ring = fl['ring']
                for t0, t1, e, per in evaluate(rule, fl, rate_name, s_rate, dt_nom):
                    fired = sorted(x for _, f in per for x in f)
                    if e is None:
                        if fired:
                            false.append(f"{fl['run']} at {fmt(fired[0])}")
                    elif not e['capsize']:
                        if fired:
                            rec.append(f"{fl['run']} +{fired[0] - t0:.2f} s, peak "
                                       f"{ring.tilt[(ring.t >= t0) & (ring.t < t1)].max():.1f}")
                    else:
                        lt = latch_of(fl)[0]
                        cell = f'{(fired[0] - lt) * 1e3:+.0f}' if fired else 'never'
                        if fired and fired[0] > lt:
                            cell += ' FAIL'
                        if rate_name == '50 Hz' and fired and fl['ticks']:
                            leads = [latch_replay(ring, tk, t0, t1)[0] - f[0]
                                     for (_, f), (_, tk) in zip(per, fl['ticks'])
                                     if f and latch_replay(ring, tk, t0, t1)[0]]
                            if leads:
                                cell += f' (min {min(leads) * 1e3:+.0f})'
                        cap_cells[fl['run']] = cell
            rows.append([rate_name, rule.name] + [cap_cells.get(f['run'], '-') for f, _ in caps]
                        + ['; '.join(rec) or 'none', f'{len(false)}' +
                           (f" ({'; '.join(false)})" if false else '')])
    md(['rate', 'rule'] + [f"{f['run']} fire - latch (ms)" for f, _ in caps] +
       ['fires after a release the ring recovered from',
        f'false fires (no release; {len(flown)} flights)'], rows)

    # 2b. replay check
    print("### Replay check: today's 60 deg / 3-tick latch on each tracker's logged ticks\n")
    rows = []
    for fl, e in caps:
        logged = {}
        for tb, topic, st, node, text in fl['data']['events']:
            if topic == '/rosout' and 'ENVELOPE FAULT' in text and node not in logged:
                logged[node] = (st, re.search(r'tilt ([0-9.]+)', text).group(1))
        for drone, tk in fl['ticks']:
            lr, tl = latch_replay(fl['ring'], tk, e['t'], e['end'])
            lg = logged.get(f'tracker_{drone}')
            rows.append([fl['run'], f'tracker_{drone}', fmt(lr), f'{tl:.1f}' if tl else '-',
                         fmt(lg[0]) if lg else 'fleet abort came first', lg[1] if lg else '-',
                         f'{(lr - lg[0]) * 1e3:+.0f}' if lg and lr else '-'])
    md(['run', 'tracker', 'replayed latch', 'tilt seen (deg)', 'logged latch', 'logged tilt',
        'replay - logged (ms)'], rows)

    # 3. per-flight max airborne tilt
    print('### Largest airborne ring tilt per flight (bags)\n')
    print(f'Airborne = ring z > its first-second median + {AIR_DZ:.2f} m, while a tracker log '
          'runs (armed), before the first disarm.\n')
    rows = []
    for fl in rings:
        ring = fl['ring']
        if not fl['flew']:
            rows.append([fl['date'], fl['run'], '0', '-', '-', '-', '-', 'ring never airborne'])
            continue
        win = fl['win']
        t_ev = fl['events'][0]['t'] if fl['events'] else np.inf
        pre = win & (ring.t < t_ev)
        k = np.where(pre)[0][np.argmax(ring.tilt[pre])]
        post = np.zeros(len(win), bool)
        for e in fl['events']:
            if not e['capsize']:
                post |= win & (ring.t >= e['t']) & (ring.t < e['end'])
        floor = (~ring.air) & fl['armed'] & (ring.t < fl['stop_t'])
        if fl['takeoff']:
            floor &= ring.t > fl['takeoff']
        notes = [f"{'capsize' if e['capsize'] else 'recovered'}: " +
                 ', '.join(f"/{r['who']} ({'OFF' if r['kind'] == 'magnet OFF' else 'no command'})"
                           for r in e['rel']) for e in fl['events']]
        rows.append([fl['date'], fl['run'], f'{np.sum(np.diff(ring.t)[win[1:]]):.0f}',
                     f'{ring.tilt[k]:.1f}', f'{TRIG - ring.tilt[k]:+.1f}',
                     f'{ring.tilt[post].max():.1f}' if post.any() else '-',
                     f'{ring.tilt[floor].max():.1f}' if floor.any() else '-',
                     '; '.join(notes) or '-'])
    md(['date', 'run', 'airborne (s)', 'max tilt, no release (deg)', 'margin to 30',
        'max tilt after a recovered release', 'max tilt on the floor after TAKEOFF', 'releases'],
       rows)

    print('### Flights with no ring pose in a bag: planner log (10 Hz)\n')
    print('Armed = a tracker log is running; airborne = planner load z > its first-second median '
          f'+ {AIR_DZ:.2f} m. 30 Sep ran the earlier controller (controller_quad_load).\n')
    md(['date', 'run', 'planner session', 'airborne (s)', 'max tilt (deg)', 'margin to 30',
        'time > 30 (s)', 'ring z at max', 'in LAND', 'max 10 Hz rate above 25 (deg/s)',
        'a1-b3 fire (> 30 for >= 0.1 s)', f'c1 at 10 Hz (R {R:.0f})', 'c2 at 10 Hz'],
       planner_log_flights({(f['date'], f['run']) for f in rings}, R))

    # 4. magnet command-to-release delay
    print('### Magnet command to release (announced detaches)\n')
    print(f'The commanded drone, and any other drone with a rod error over {SEP_M * 100:.0f} cm within 1 s '
          f'of the OFF and before the fleet disarm. Baseline = median excess '
          f'over the 1 s before the OFF; separation = 6 samples in a row > baseline + '
          f'{ONSET_M * 1e3:.0f} mm; climb = peak drone vz in the 0.3 s after the separation. The '
          'ring-attitude column is confounded: the planner resizes the OCP at the same instant.\n')
    rows = []
    for fl in flown:
        ring = fl['ring']
        for t_off, who in fl['offs']:
            k0 = ring.at(t_off)
            dev = att_step(ring.q[k0], ring.q)
            rk = next((i for i in np.where((ring.t > t_off) & (dev > 1.0))[0]
                       if np.all(dev[i:i + 5] > 1.0)), None)
            w = [x for x in fl['data']['events'] if x[1] == '/rosout' and 'magnet OFF' in x[4]
                 and 'elrs' in (x[3] or '') and t_off - 0.01 <= x[2] < t_off + 0.5]
            for pd in sorted(fl['pdist'].values(), key=lambda v: v['name'] != who):
                cmd = pd['name'] == who
                pre = (pd['t'] > t_off - 1.0) & (pd['t'] < t_off)
                on, s3 = release_marks(pd, t_off, t_off + 3.0)
                if on is None or not (cmd or (s3 is not None and s3 < min(t_off + 1.0,
                                                                         fl['stop_t']))):
                    continue
                j = (pd['t'] >= on - 0.05) & (pd['t'] < on + 0.3)
                vz = np.gradient(pd['z'][j], pd['t'][j])
                rows.append([fl['run'], drone_label(fl, pd['name']), 'yes' if cmd else 'NO',
                             f"{pd['L'] + np.median(pd['exc'][pre]):.3f} "
                             f"({np.std(pd['exc'][pre]) * 1e3:.1f} mm sd)",
                             fmt(t_off) if cmd else '',
                             f'{(w[0][2] - t_off) * 1e3:+.0f}' if cmd and w else '',
                             f'{(on - t_off) * 1e3:+.0f}',
                             f'{(s3 - t_off) * 1e3:+.0f}' if s3 is not None else '> 3000',
                             f'{vz.max():+.2f}',
                             f'{(ring.t[rk] - t_off) * 1e3:+.0f}' if cmd and rk is not None else ''])
    md(['run', 'drone', 'commanded', 'rod before OFF (m)', 'OFF (bag)', 'radio write (ms)',
        f'separation > {ONSET_M * 1e3:.0f} mm (ms)', f'rod error > {SEP_M * 100:.0f} cm, either sign (ms)',
        'climb (m/s)', 'ring attitude 1 deg off its OFF pose (ms)'], rows)

    # 5. data quality
    print('### Ring pose data quality (bags)\n')
    rows = []
    for fl in fls:
        if fl['ring'] is None:
            rows.append([fl['date'], fl['run'], 'no ring pose in the bag'] + [''] * 6)
            continue
        ring = fl['ring']
        gaps = ', '.join(f'{ring.dt[i]:.2f} s at {fmt(ring.t[i - 1])} '
                         f'({"in flight" if fl["win"][i - 1] or fl["win"][i] else "not in flight"})'
                         for i in np.where(ring.dt > 0.1)[0]) or 'none'
        win = fl['win'] & np.r_[False, fl['win'][:-1]]
        for e in fl['events']:
            if e['capsize']:
                win &= ring.t < e['t']
        big = np.where(ring.step > s_frame)[0]
        big_in = [i for i in big if fl['win'][i] or fl['win'][i - 1]]
        rows.append([fl['date'], fl['run'], len(ring.t), f'{1.0 / np.median(ring.dt[1:]):.1f}',
                     gaps, ring.n_nan, ring.flips,
                     f'{np.nanmax(ring.step[win]):.2f}' if win.any() else '-',
                     f'{len(big)} ({len(big_in)} in flight)'])
    md(['date', 'run', 'ring samples', 'rate (Hz)', 'gaps > 0.1 s', 'NaN', 'quaternion sign flips',
        'largest step in flight, outside a capsize (deg)', f'steps > S ({s_frame:.0f} deg)'], rows)

    print('### How S and R were chosen\n')
    md(['quantity', 'flight', 'value'],
       [['largest attitude step in a capsize (release to latch), per nominal frame (raw)', r,
         f'{s:.2f} ({raw:.2f}) deg'] for r, s, raw in steps] +
       [[f'S = {GLITCH_MARGIN:.0f} x that, rounded up to 5 deg', '', f'{s_frame:.0f} deg per frame'
         f' ({s_rate:.0f} deg/s)']] +
       [['slowest of the first three capsize samples above 25 deg', r, f'{v:.0f} deg/s']
        for r, v in cap_rates] +
       [['fastest tilt rate above 25 deg after a release the ring recovered from', r,
         f'{v:.0f} deg/s'] for r, v in rec_rates] +
       [['R = geometric mean of the two, rounded down to 10', '', f'{R:.0f} deg/s']])

    # 6. synthetic glitches
    fl0, e0 = caps[0]
    t_at = e0['t'] - 5.0
    print(f"### Synthetic ring-pose mis-fits injected into {fl0['run']} hover at {fmt(t_at)} "
          '(mocap rate)\n')
    md(['injected', 'tilt (deg)'] + [r.name.split(':')[0] for r in rules],
       synthetic(fl0['ring'], rules, s_rate, dt_nom, t_at))


if __name__ == '__main__':
    main()
