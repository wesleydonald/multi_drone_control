#!/usr/bin/env python3
"""The real-world rows of the combined paper (main.tex, "Experiments and results"): one row per detach, per attach and
per application flight, from the launch log, the bag and the trackers' logs. paper_tables.py makes the tables.

    source ~/ros2_humble/install/setup.bash && source ~/multi_drone_control/install/setup.bash
    cd ~/multi_drone_control
    nice -n 10 python3 results/analysis/paper_rig_2026-11/paper_rig.py            # every relevant flight so far
    nice -n 10 python3 results/analysis/paper_rig_2026-11/paper_rig.py 2026-11-11 # only that day (CSVs then hold only it)
    nice -n 10 python3 results/analysis/paper_rig_2026-11/paper_rig.py 2026-11-05/sq8_2 2026-10-08/r303_4

With no argument: the 8 Oct unannounced releases (r303_4, r304_3, r305: 0.55 m links, the method's check), every
flown attach-mode flight of 4-5 Nov, and every flown flight of every day from 11 Nov on (found from launches.csv), so
the same command picks up each new session. Writes, beside this file: events.csv (one row per detach or attach),
phases.csv (one row per application or pickup-and-drop flight), flights.csv (one row per flight) and joins/ (joins.py
of rig_joins_2026-11-04_05 re-run on the attach flights: every descent of QUAD4's magnet, the misses). Bags are cached
(JOINS_CACHE, default ~/mdc_overnight/joins_cache; PAPER_CACHE, default ~/mdc_overnight/paper_rig_cache).

Definitions (state them; Wesley decides if they change). The ring's tilt and the windows are the twin's X1 metrics
(tools/x1_metrics.py, tools/release_bands.py: the T6 release metrics frozen 14 Oct), so the rig and the twin compare:
- event time t_e: detach (unannounced) = the operator's magnet OFF in the bag (/drone_N/magnet not sent by the planner,
  or /drone_3/magnet_operator) while QUAD4 is still a carrier; detach (planned) = our magnet OFF at the end of the
  3 s unload (the planner's or the magnet manager's OFF line); attach = the fold-in (FLEET RESIZED back to 4 after
  ATTACH: proven).
- window: from t_e to the first of the operator's LAND, a drop or fault, the next fleet resize, t_e + 15 s (the rel
  sheet's hold), the end of the record, and for a detach his next descent (DESCEND_TO_ATTACHMENT, LIFT_PROOF,
  DESCEND_TO_PICKUP: QUAD4 coming down onto the ring again). Dip and path error use t_e .. min(t_e + 10 s, end).
- "before" = the 5 s before t_e, cut to the part after the set-point last moved (release_bands' ref_still: a planned
  detach can come right at the lift's end); under 1 s of it, every metric read against "before" stays empty
  (resp_note says why) and the absolute ones (tilt peak, set-point and path deviation) remain.
- tilt: the ring normal's angle from vertical (mocap, 50 Hz, sample and hold). swing = the tilt vector's largest
  departure from its mean before (X1's "tilt rise", T6 delta-theta); tilt peak = the largest tilt in the window;
  tilt rise = tilt peak minus the mean tilt before.
- time to settle = X1's t_level: the first time after the swing's peak from which the tilt vector stays within
  max(2 sigma_pre, 1 deg) of its mean before for 1 s (sigma_pre: the RMS of the tilt vector about that mean);
  none if it never does before the window ends. settle_2s: the same held 2 s. settle_new (T6's time back in band,
  searched from t_e, with the same band): the tilt vector within the band of its new level (its mean over the
  window's last 5 s; windows of 10 s or more) for 1 s: an attach changes the ring's level on purpose (the square's T
  leaned 23-27 deg until the fold-in on 5 Nov), so the attach table reads this one. tilt shift = the new level's
  magnitude minus the mean level's magnitude before.
- dip = the ring's mean height before minus its lowest height in t_e .. t_e + 10 s (mm, + = down); horizontal
  excursion (hovers only) = the largest horizontal distance from its mean position before, same window.
- set-point deviation = RMS of the ring's 3-D distance to its published set-point (the planner's /payload/desired_position
  as the drone-0 tracker logs it), t_e .. t_e + 10 s, and the same over the 5 s before. path deviation (X1's path
  error): in a hover the same, on a path clock-free (across the path and height).
- detection delay = the planner's UNANNOUNCED DETACH line minus the operator's OFF; resize = its FLEET RESIZED line minus
  the OFF; separation = the first time QUAD4's magnet marker is 2 cm from where it sat on the plate (ring frame).
- success (detach) = the release was acted on (detected and resized, or our DETACH) and nothing was lost until the
  first of LAND and t_e + 15 s: no DROP AND LAND, no PAYLOAD DROPPED, no fault stop or position limit, the ring above
  0.3 m. success (attach) = ATTACH: proven, the fleet resized back to 4, then the same 15 s with nothing lost and
  QUAD4's magnet marker never 2 cm from where it sat on the plate (added 11 Nov: nrtfc_3 let go 4 s after its proof).
- attach descents and misses: joins.csv (feed F052's definitions: the miss is the magnet's horizontal distance from the
  plate's weld point when it first reached the plate's height).
- application phases (log lines): lift = TAKEOFF to the ring's set-point at the top; detach = UNLOAD to PARTNER RELEASE;
  pickup = PARTNER RELEASE (from the mat: his TAKEOFF) to "object lifted"; transit = to DROP_OBJECT; drop = DROP_OBJECT
  to CLEAR_DROP_ZONE (and where the object hit the net, joins.csv); re-attach = CLEAR_DROP_ZONE to the fold-in;
  land = LAND to "Landed - disarming". A phase that started and never ended is where the mission failed.
"""
import csv
import fnmatch
import glob
import math
import os
import pickle
import re
import sqlite3
import sys

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.expanduser('~/multi_drone_control')
RIG = os.path.join(REPO, 'results', 'rig')
os.environ.setdefault('JOINS_CACHE', os.path.expanduser('~/mdc_overnight/joins_cache'))
CACHE = os.environ.get('PAPER_CACHE', os.path.expanduser('~/mdc_overnight/paper_rig_cache'))
sys.path.insert(0, os.path.join(REPO, 'results', 'analysis', 'rig_joins_2026-11-04_05'))
sys.path.insert(1, os.path.join(REPO, 'tools'))
import joins  # noqa: E402
import release_bands as RB  # noqa: E402
import x1_metrics as X1  # noqa: E402

HZ = 50.0
PRE_S = RB.PRE_S                 # 5 s before the event
AFTER_S = X1.AFTER_S             # 10 s: dip and path deviation
HOLD_S = X1.HOLD_S               # 1 s: t_level held
BAND_MIN = X1.LEVEL_BAND_MIN     # 1 deg
DROP_Z = X1.DROP_Z               # 0.3 m
HORIZON_S = 15.0                 # the window's longest (the rel sheet holds 15 s on three, then LAND)
SAFE_S = 15.0                    # success: nothing lost until LAND or this long after the event
DESCENTS = ('DESCEND_TO_ATTACHMENT', 'LIFT_PROOF', 'DESCEND_TO_PICKUP')   # his phases that end a detach's window
SEP_M = 0.02                     # magnet marker this far from its plate seat: separated
CONTEXT = ('2026-10-08/r303_4', '2026-10-08/r304_3', '2026-10-08/r305')
ATTACH_DAYS = ('2026-11-04', '2026-11-05')
FIRST_SESSION = '2026-11-11'
EXTRA_TOPICS = {'tipstate': '/drone_3/magnet_tip_state', 'op': '/drone_3/magnet_operator'}
STAMP = re.compile(r'\[(\d{9,}\.\d+)\]')

RX = {
    'cmd': re.compile(r"\[fleet_manager\]: Fleet command received: '(\w+)'"),
    'landed': re.compile(r'\[fleet_manager\]: Landed - disarming'),
    'unload': re.compile(r'\[dissipative\] UNLOAD drone (\d+) \(/drone_(\d+)\)'),
    'unloaded': re.compile(r'\[dissipative\] UNLOADED drone (\d+): planned ([\d.]+) N, ring tilt ([\d.]+) deg'),
    'resize': re.compile(r'FLEET RESIZED to n=(\d+); slot->drone \[([\d, ]*)\]; attach (angles \(cw\)|azimuths) '
                         r'\[([-\d., ]*)\] deg'),
    'detach': re.compile(r'\[dissipative\] DETACH drone (\d+) \(/drone_(\d+)\)'),
    'unannounced': re.compile(r'\[dissipative\] UNANNOUNCED DETACH: drone (\d+) \(/drone_(\d+)\) (.*?) - resizing'),
    'drop_and_land': re.compile(r'\[dissipative\] DROP AND LAND: (.*?)(?: - every magnet|$)'),
    'payload_dropped': re.compile(r'\[fleet_manager\]: PAYLOAD DROPPED'),
    'poslimit': re.compile(r'\[fleet_manager\]: POSITION LIMIT: (.*)$'),
    'estop': re.compile(r'\[fleet_manager\]: EMERGENCY STOP broadcast to \d+ drones: (.*)$'),
    'planner_off': re.compile(r'\[dissipative\] magnet OFF -> /drone_(\d+)/magnet'),
    'manager_off': re.compile(r'\[magnet_attachment_manager\]: DETACH: magnet command OFF'),
    'partner_release': re.compile(r'\[dissipative\] PARTNER RELEASE'),
    'fold': re.compile(r'\[dissipative\] ATTACH drone (\d+) \(OCP resize\)'),
    'takeback': re.compile(r'\[dissipative\] TAKE-BACK of drone (\d+)'),
    'geometry': re.compile(r'geometry: n=(\d+) cable_len=([\d.]+) attach_radius=([\d.]+) .*?load_mass=([\d.]+).*?'
                           r'target_z=([\d.]+).*?load_traj=(\w+)(?:.*?traj_radius=([\d.]+))?'),
}


# ---------------------------------------------------------------------------------------------- flights and inputs
def launches(day):
    fn = os.path.join(RIG, day, 'launches.csv')
    if not os.path.exists(fn):
        return [{'day': day, 'name': os.path.basename(p)[:-4], 'flew': 'yes', 'args': {}}
                for p in sorted(glob.glob(os.path.join(RIG, day, '*.log')))]
    out = []
    with open(fn) as f:
        for r in csv.DictReader(f):
            args = {}
            for k, v in re.findall(r'(\S+?):=(\S+)', r['launch'].replace('\n', ' ')):
                args.setdefault(k, v)
            out.append({'day': day, 'name': r['name'], 'flew': r['flew'], 'args': args})
    return out


def discover(argv):
    if not argv:
        days = sorted(d for d in os.listdir(RIG) if re.fullmatch(r'\d{4}-\d\d-\d\d', d) and d >= FIRST_SESSION)
        names = list(CONTEXT)
        for day in ATTACH_DAYS:
            names += [f"{L['day']}/{L['name']}" for L in launches(day)
                      if L['flew'] == 'yes' and L['args'].get('mode') == 'attach']
        for day in days:
            names += [f"{L['day']}/{L['name']}" for L in launches(day) if L['flew'] == 'yes']
        return names
    out = []
    for a in argv:
        if re.fullmatch(r'\d{4}-\d\d-\d\d', a):
            out += [f"{L['day']}/{L['name']}" for L in launches(a) if L['flew'] == 'yes']
        else:
            out.append(a.strip('/'))
    return out


def registry():
    rows = {}
    with open(os.path.join(REPO, 'results', 'registry.csv'), newline='') as f:
        for r in csv.reader(f):
            if r and r[0].startswith('RIG-'):
                rows[r[0]] = r
    return rows


def conditions():
    """conditions.csv: per flight (day/name, fnmatch patterns allowed) the funnel and an exclusion; the last match wins."""
    fn = os.path.join(HERE, 'conditions.csv')
    rules = []
    if os.path.exists(fn):
        with open(fn, newline='') as f:
            for r in csv.DictReader(line for line in f if not line.lstrip().startswith('#')):
                rules.append(r)

    def lookup(fid):
        out = {'funnel': 'no' if fid[:10] < FIRST_SESSION else '?', 'exclude': '', 'note': ''}
        for r in rules:
            if fnmatch.fnmatch(fid, (r.get('flight') or '').strip()):
                for k in ('funnel', 'exclude', 'note'):
                    if (r.get(k) or '').strip():
                        out[k] = r[k].strip()
        return out
    return lookup


def parse_log(path):
    ev = {k: [] for k in RX}
    ev['geometry'] = None
    if not os.path.exists(path):
        return ev
    with open(path, errors='replace') as f:
        for ln in f:
            if ev['geometry'] is None:
                g = RX['geometry'].search(ln)
                if g:
                    ev['geometry'] = {'n': int(g.group(1)), 'cable_len': float(g.group(2)),
                                      'attach_radius': float(g.group(3)), 'load_mass': float(g.group(4)),
                                      'target_z': float(g.group(5)), 'load_traj': g.group(6),
                                      'traj_radius': float(g.group(7)) if g.group(7) else None}
            m = STAMP.search(ln)
            if not m:
                continue
            t = float(m.group(1))
            for k, rx in RX.items():
                if k == 'geometry':
                    continue
                g = rx.search(ln)
                if g:
                    ev[k].append((t,) + g.groups())
    return ev


def read_extra(folder):
    """The topics joins.py does not read: every /drone_N/magnet, the operator's topic, QUAD4's magnet marker as the
    carriers' release check reads it. Cached per bag."""
    fn = os.path.join(CACHE, folder.replace('/', '_') + '.pkl')
    if os.path.exists(fn):
        with open(fn, 'rb') as f:
            return pickle.load(f)
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    out = {'magnet': {}, 'op': [], 'tipstate': []}
    for db in sorted(glob.glob(os.path.join(folder, '*.db3'))):
        c = sqlite3.connect(db)
        tops = {n: (i, ty) for i, n, ty in c.execute('select id,name,type from topics')}
        for name, (i, ty) in tops.items():
            mm = re.fullmatch(r'/drone_(\d+)/magnet', name)
            if not (mm or name in EXTRA_TOPICS.values()):
                continue
            cls = get_message(ty)
            for ts, d in c.execute('select timestamp,data from messages where topic_id=? order by timestamp', (i,)):
                msg = deserialize_message(d, cls)
                t = ts * 1e-9
                if mm:
                    out['magnet'].setdefault(int(mm.group(1)), []).append((t, msg.data.strip().upper()))
                elif name == EXTRA_TOPICS['op']:
                    out['op'].append((t, msg.data.strip().upper()))
                else:
                    p, q = msg.pose.position, msg.pose.orientation
                    out['tipstate'].append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        c.close()
    out['tipstate'] = np.array(sorted(out['tipstate'])) if out['tipstate'] else np.zeros((0, 8))
    os.makedirs(CACHE, exist_ok=True)
    with open(fn + '.tmp%d' % os.getpid(), 'wb') as f:
        pickle.dump(out, f)
    os.replace(fn + '.tmp%d' % os.getpid(), fn)
    return out


def tracker_ref(day, name):
    """(t, x, y, z) of the ring's set-point from the drone-0 tracker log (/payload/desired_position)."""
    hits = sorted(glob.glob(os.path.join(RIG, day, name + '_logs', 'logs', 'tracker', 'drone0_*', 'log.csv')))
    if not hits:
        return np.zeros((0, 4))
    d = pd.read_csv(hits[-1], usecols=['sim_time', 'payload_ref_x', 'payload_ref_y', 'payload_ref_z'])
    d = d.dropna().sort_values('sim_time')
    return d[['sim_time', 'payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy(float)


def grid(B, ref):
    """The ring on a 50 Hz grid as tools/rig_to_run.py lays it out (sample and hold, a pose older than 0.1 s is NaN),
    with the set-point held (older than 0.5 s: NaN). Columns in the run.csv schema, t in epoch seconds."""
    ring = B['ring']
    if len(ring) < 20:
        return None
    t = np.arange(ring[0, 0], ring[-1, 0], 1.0 / HZ)
    k = np.searchsorted(ring[:, 0], t, side='right') - 1
    ok = (k >= 0) & (t - ring[np.clip(k, 0, None), 0] <= 0.1)
    R = ring[np.clip(k, 0, None)]
    col = {'t': t}
    for j, c in enumerate(('payload_x', 'payload_y', 'payload_z', 'payload_qx', 'payload_qy', 'payload_qz', 'payload_qw')):
        col[c] = np.where(ok, R[:, 1 + j], np.nan)
    if len(ref):
        kr = np.searchsorted(ref[:, 0], t, side='right') - 1
        okr = (kr >= 0) & (t - ref[np.clip(kr, 0, None), 0] <= 0.5)
        Q = ref[np.clip(kr, 0, None)]
        for j, c in enumerate(('payload_ref_x', 'payload_ref_y', 'payload_ref_z')):
            col[c] = np.where(okr, Q[:, 1 + j], np.nan)
    else:
        for c in ('payload_ref_x', 'payload_ref_y', 'payload_ref_z'):
            col[c] = np.full(len(t), np.nan)
    G = pd.DataFrame(col)
    q = G[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy(float)
    gap = np.isnan(q[:, 0])
    q[gap] = (1.0, 0.0, 0.0, 0.0)
    th = RB.tilt_vector(q)
    th[gap] = np.nan
    G.attrs['th'] = th
    return G


# ------------------------------------------------------------------------------------------------- the ring's response
def ref_still_from(G, te):
    """When the ring's set-point last changed before te (release_bands' ref_still: its height more than 5 mm from its
    value at te, or its speed, a 2 s chord, more than 0.02 m/s from its median over the last 2 s): the lift's end."""
    t = G.t.to_numpy(float)
    ref = G[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy(float)
    ok = np.all(np.isfinite(ref), 1) & (t <= te)
    if ok.sum() < 3 * HZ:
        return -math.inf
    t, ref = t[ok], ref[ok]
    back = np.stack([np.interp(t - 2.0, t, ref[:, i]) for i in range(2)], 1)
    v = np.linalg.norm(ref[:, :2] - back, axis=1) / 2.0
    v_e = float(np.median(v[t >= te - 2.0]))
    moved = (np.abs(ref[:, 2] - ref[-1, 2]) > 0.005) | (np.abs(v - v_e) > 0.02)
    moved &= t >= t[0] + 2.0
    return float(t[np.nonzero(moved)[0][-1]]) if moved.any() else float(t[0])


def response(G, te, tend, moving=False):
    """X1's metrics of the ring around one event (te; the window ends at tend), on the grid G. The 5 s before is cut
    to the part after the set-point last moved (the lift's end); under 1 s of it, the metrics read against it stay
    empty and resp_note says why."""
    out = {'window_s': round(tend - te, 2)}
    if G is None:
        out['resp_note'] = 'no ring pose'
        return out
    t = G.t.to_numpy(float)
    th = G.attrs['th']
    okt = np.all(np.isfinite(th), 1)
    t_still = ref_still_from(G, te)
    lo = max(te - PRE_S, t_still)
    out['pre_s'] = round(te - lo, 2)
    pre = (t >= lo) & (t < te) & okt
    post = (t >= te) & (t < tend) & okt
    if post.sum() < 1.0 * HZ:
        out['resp_note'] = f'too little ring pose after the event ({post.sum() / HZ:.1f} s)'
        return out
    pre_ok = pre.sum() >= 1.0 * HZ
    if not pre_ok:
        out['resp_note'] = (f'not steady before: the set-point moved until {te - t_still:.1f} s before the event'
                            if te - t_still < 1.0 else f'too little ring pose before ({pre.sum() / HZ:.1f} s)')
    idx = np.nonzero(post)[0]
    mag = np.linalg.norm(th, axis=1)
    out['tilt_peak_deg'] = float(np.nanmax(mag[idx]))
    aft = (t >= te) & (t <= min(te + AFTER_S, tend))
    z = G.payload_z.to_numpy(float)
    xy = G[['payload_x', 'payload_y']].to_numpy(float)
    p = G[['payload_x', 'payload_y', 'payload_z']].to_numpy(float)
    r = G[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy(float)
    e = np.linalg.norm(p - r, axis=1)

    def rms(v):
        v = v[np.isfinite(v)]
        return 100.0 * float(np.sqrt(np.mean(v ** 2))) if v.size else None
    out['sp_rms_cm'] = rms(e[aft])
    ea = e[aft]
    out['sp_max_cm'] = 100.0 * float(np.nanmax(ea)) if np.isfinite(ea).any() else None
    err = None
    if moving:      # a hover's set-point is a point; on a path X1's clock-free distance (across the path and height)
        try:
            err = X1._clock_free_err(G[['payload_x', 'payload_y', 'payload_z', 'payload_ref_x', 'payload_ref_y',
                                        'payload_ref_z']].assign(t=t - t[0]), t - t[0], aft)
        except Exception:  # noqa: BLE001  (a reference too short to make a path)
            err = None
    if err is None:
        out['path_kind'] = 'point'
        out['path_rms_cm'] = out['sp_rms_cm']
    else:
        out['path_kind'] = 'clock-free'
        out['path_rms_cm'] = rms(np.asarray(err))
    w1 = (t >= te - 1.0) & (t < te) & np.all(np.isfinite(xy), 1)
    if w1.sum() > 10:
        a, b = np.nonzero(w1)[0][[0, -1]]
        out['ring_speed_mps'] = float(np.linalg.norm(xy[b] - xy[a]) / max(t[b] - t[a], 1e-3))
    out['ring_z_min_m'] = float(np.nanmin(z[(t >= te) & (t < tend)]))
    long_enough = tend - te >= 10.0
    if long_enough:
        last = (t >= tend - RB.POST_S) & (t < tend) & okt
        level = th[last].mean(0)
        out['tilt_after_deg'] = float(np.linalg.norm(level))
    if not pre_ok:
        return out
    mean = th[pre].mean(0)
    sig = float(np.sqrt(np.mean(np.sum((th[pre] - mean) ** 2, 1))))
    dev = np.linalg.norm(th - mean, axis=1)
    k = idx[int(np.argmax(dev[idx]))]
    out.update(tilt_pre_deg=float(mag[pre].mean()), sigma_pre_deg=sig, swing_deg=float(dev[k]),
               t_swing_peak_s=float(t[k] - te))
    out['tilt_rise_deg'] = out['tilt_peak_deg'] - out['tilt_pre_deg']
    band = max(2.0 * sig, BAND_MIN)
    out['settle_band_deg'] = band

    def first_held(inb, t_from, hold):
        for i in np.nonzero((t >= t_from) & (t + hold <= tend))[0]:
            if inb[(t >= t[i]) & (t < t[i] + hold)].all():
                return float(t[i] - te)
        return None
    inb = (dev <= band) & okt
    out['settle_s'] = first_held(inb, t[k], HOLD_S)
    out['settle_2s_s'] = first_held(inb, t[k], 2.0)
    if long_enough:
        out['tilt_shift_deg'] = out['tilt_after_deg'] - float(np.linalg.norm(mean))
        inb_new = (np.linalg.norm(th - level, axis=1) <= band) & okt
        out['settle_new_s'] = first_held(inb_new, te, HOLD_S)
    out['dip_mm'] = 1000.0 * float(np.nanmean(z[pre]) - np.nanmin(z[aft]))
    if not moving:  # on a path the ring is meant to move: the path deviation says it there
        out['xy_excursion_cm'] = 100.0 * float(np.nanmax(np.linalg.norm(xy[aft] - np.nanmean(xy[pre], 0), axis=1)))
    out['sp_pre_rms_cm'] = rms(e[pre])
    return out


def separation(B, X, te, tip_drone=3, horizon=5.0):
    """First time after te the magnet marker is SEP_M from where it sat on its plate (ring frame, the 1 s before)."""
    tip = X['tipstate'] if len(X['tipstate']) > 20 else B['tip']
    ring = B['ring']
    if len(tip) < 20 or len(ring) < 20:
        return None
    w0 = (tip[:, 0] >= te - 1.0) & (tip[:, 0] < te)
    w1 = (tip[:, 0] >= te) & (tip[:, 0] < te + horizon)
    if w0.sum() < 10 or w1.sum() < 10:
        return None
    seat = np.median(joins.ring_frame(ring, tip[w0, 0], tip[w0, 1:4])[0], axis=0)
    rel, _ = joins.ring_frame(ring, tip[w1, 0], tip[w1, 1:4])
    far = np.nonzero(np.linalg.norm(rel - seat, axis=1) > SEP_M)[0]
    return float(tip[w1][far[0], 0] - te) if far.size else None


# ------------------------------------------------------------------------------------------------- one flight
class Flight:
    def __init__(self, fid, launch, regs, cond):
        self.fid = fid
        self.day, self.name = fid.split('/')
        self.args = launch.get('args', {})
        self.rid = f'RIG-{self.day[5:7]}{self.day[8:10]}-{self.name}'
        reg = regs.get(self.rid, [])
        self.void = any(len(reg) > i and reg[i].strip().upper().startswith('VOID') for i in (7, 9))
        self.cond = cond(fid)
        self.L = parse_log(os.path.join(RIG, self.day, self.name + '.log'))
        self.J = joins.parse_log(os.path.join(RIG, self.day, self.name + '.log'))
        geo = self.L['geometry'] or {}
        self.geo = geo
        self.motion_kind = (self.args.get('load_traj') or geo.get('load_traj') or 'hover')
        self.motion = 'static' if self.motion_kind == 'hover' else 'moving'
        folder = os.path.join(RIG, self.day, self.name)
        self.B = joins.read_bag(folder) if os.path.isdir(folder) else None
        self.X = read_extra(folder) if os.path.isdir(folder) else {'magnet': {}, 'op': [], 'tipstate': np.zeros((0, 8))}
        self.G = grid(self.B, tracker_ref(self.day, self.name)) if self.B is not None else None
        cmds = self.L['cmd']
        self.t_takeoff = next((c[0] for c in cmds if c[1] == 'TAKEOFF'), None)
        self.t_land = next((c[0] for c in cmds if c[1] == 'LAND' and (self.t_takeoff is None or c[0] > self.t_takeoff)),
                           None)
        self.t_landed = self.L['landed'][0][0] if self.L['landed'] else None
        self.t_end = (self.B['t1'] if self.B is not None else (self.t_landed or math.inf))
        self.losses = self._losses()

    # the fleet size over time (the start weld's resize included)
    def n_at(self, t):
        n = self.geo.get('n')
        for r in self.L['resize']:
            if r[0] <= t:
                n = int(r[1])
        return n

    def carriers_after(self, t):
        """The carriers' plates (clockwise numbering) from the first FLEET RESIZED line at or after t."""
        r = next((r for r in self.L['resize'] if r[0] >= t - 0.5), None)
        if r is None:
            return None
        ang = [float(a) for a in r[4].split(',') if a.strip()]
        if not r[3].startswith('angles'):
            ang = [-a for a in ang]          # 8 Oct: counter-clockwise azimuths
        return sorted(int(round(a / 30.0)) % 12 for a in ang)

    def _losses(self):
        """Every way the ring or the fleet was lost, with its time: [(t, text)]."""
        out = [(t, 'DROP AND LAND: ' + why) for t, why in self.L['drop_and_land']]
        out += [(t[0], 'PAYLOAD DROPPED') for t in self.L['payload_dropped']]
        out += [(t, 'POSITION LIMIT: ' + why) for t, why in self.L['poslimit'] if 'already landing' not in why]
        out += [(t, 'fault stop: ' + why) for t, why in self.L['estop'] if 'operator DISARM' not in why]
        if self.G is not None and self.t_takeoff is not None:
            t = self.G.t.to_numpy(float)
            z = self.G.payload_z.to_numpy(float)
            top = self.t_top()
            if top is not None:
                low = np.nonzero((t > top) & (t < (self.t_land or math.inf)) & (z < DROP_Z))[0]
                if low.size:
                    out.append((float(t[low[0]]), f'the ring under {DROP_Z} m before LAND'))
        return sorted(out)

    def loss_between(self, a, b):
        return next((x for x in self.losses if a <= x[0] < b), None)

    def t_top(self):
        """The ring's set-point first at the top of the lift (within 5 mm of its highest value before LAND)."""
        if self.G is None or self.t_takeoff is None:
            return None
        t = self.G.t.to_numpy(float)
        rz = self.G.payload_ref_z.to_numpy(float)
        w = (t > self.t_takeoff) & (t < (self.t_land or math.inf)) & np.isfinite(rz)
        if not w.any():
            return None
        top = np.nanmax(rz[w])
        if top < 0.3:
            return None
        k = np.nonzero(w & (rz >= top - 0.005))[0]
        return float(t[k[0]]) if k.size else None

    def ring_up(self, margin=0.10):
        """The lift's end: the set-point at the top (t_top) and the ring itself within margin of that height."""
        t0 = self.t_top()
        if t0 is None:
            return None
        t = self.G.t.to_numpy(float)
        z = self.G.payload_z.to_numpy(float)
        rz = self.G.payload_ref_z.to_numpy(float)
        top = float(rz[np.searchsorted(t, t0)])
        k = np.nonzero((t >= t0) & (t < (self.t_land or math.inf)) & (z >= top - margin))[0]
        lo = self.loss_between(self.t_takeoff, math.inf)
        if not k.size or (lo and t[k[0]] >= lo[0]):
            return None
        return float(t[k[0]])

    def window_end(self, te, kind):
        ends = [te + HORIZON_S, self.t_end]
        if self.t_land is not None and self.t_land > te:
            ends.append(self.t_land)
        lo = self.loss_between(te, math.inf)
        if lo:
            ends.append(lo[0])
        nxt = next((r[0] for r in self.L['resize'] if r[0] > te + 2.0), None)
        if nxt:
            ends.append(nxt)
        if kind.startswith('detach'):
            # QUAD4 coming down onto the ring (or the object) again ends a detach's window
            c = next((p[0] for p in self.J['phases'] if p[0] > te and p[2] in DESCENTS), None)
            if c:
                ends.append(c)
        return min(ends)

    def rel(self, t):
        return round(t - self.t_takeoff, 2) if (t is not None and self.t_takeoff is not None) else None

    # ---------------------------------------------------------------------------- the kind of flight
    def kind(self):
        a = self.args
        mode = a.get('mode') or ('dissipative' if self.L['resize'] or self.geo else '?')
        # an operator OFF in an attach-mode flight is one more unannounced detach event; the flight keeps its kind
        if mode != 'attach' and self.operator_offs():
            return 'release'
        if mode == 'attach':
            welded = a.get('partner_attached', 'false') == 'true'
            mm = a.get('mission_mode') or 'floor_join'
            if mm.startswith('pickup'):
                return 'application' if welded else 'pickup-drop'
            if mm.startswith('delivery'):
                return 'delivery-rejoin'      # fm9h: the object on QUAD4's magnet from its mat: drop, then rejoin
            return 're-attach' if welded else 'attach from the mat'
        return 'other'

    def operator_offs(self):
        """[(t, drone)]: an OFF of a magnet while that quad was still a carrier (n = 4) that the planner did not send:
        the first ON -> OFF of /drone_N/magnet not within 0.2 s of the planner's own logged OFF, or an OFF on
        /drone_3/magnet_operator; before any drop, before LAND."""
        if not hasattr(self, '_ops'):
            stop = min([x[0] for x in self.losses if not x[1].startswith('the ring')] + [self.t_land or math.inf])
            own = [(t, int(d)) for t, d in self.L['planner_off']]
            out = []
            for d, msgs in sorted(self.X['magnet'].items()):
                state = None
                for t, v in msgs:
                    if v == 'OFF' and state == 'ON' and t < stop and not any(dd == d and abs(t - to) < 0.2
                                                                             for to, dd in own):
                        out.append((t, d))
                        break
                    state = v
            for t, v in self.X['op']:
                if v == 'OFF' and t < stop and (self.n_at(t) or 0) >= 4:
                    out.append((t, 3))
                    break
            # an OFF is a release only while the quad was still on the ring (n = 4 then)
            self._ops = [(t, d) for t, d in sorted(out) if (self.n_at(t) or 0) >= 4]
        return self._ops

    def formation(self, plates):
        if plates is None:
            return '?'
        s = set(plates)
        if s == {3, 7, 11}:
            return 'triangle'
        if s == {2, 5, 11}:
            return 'square'
        return 'plates ' + '/'.join(map(str, plates))

    def base_row(self):
        g = self.geo
        return {'day': self.day, 'flight': self.name, 'registry_id': self.rid, 'flight_kind': self.kind(),
                'motion': self.motion, 'traj': self.motion_kind, 'funnel': self.cond['funnel'],
                'cable_m': g.get('cable_len'), 'load_kg_config': g.get('load_mass'),
                'void': 'yes' if self.void else '', 'exclude': self.cond['exclude'],
                'cond_note': self.cond['note']}

    # ---------------------------------------------------------------------------- detach events
    def detaches(self):
        rows = []
        for t_off, d in self.operator_offs():
            det = next((u for u in self.L['unannounced'] if int(u[2]) == d and t_off - 0.5 <= u[0] <= t_off + 5.0), None)
            rs = next((r for r in self.L['resize'] if t_off - 0.5 <= r[0] <= t_off + 5.0), None)
            row = self.base_row()
            row.update(event='detach (unannounced)', drone=f'/drone_{d}', t_event_s=self.rel(t_off),
                       formation=self.formation(self.carriers_after(t_off - 0.5)))
            row['detected'] = 'yes' if det else 'no'
            row['detect_line'] = det[3] if det else ''
            row['detect_delay_s'] = round(det[0] - t_off, 3) if det else None
            row['resize_s'] = round(rs[0] - t_off, 3) if rs else None
            row['separation_s'] = separation(self.B, self.X, t_off) if self.B is not None else None
            acted = bool(det and rs)
            rows.append(self._finish(row, t_off, 'detach', acted, 'not detected or not resized'))
        for u in self.L['unload']:
            t_u = u[0]
            dn = next((x for x in self.L['unloaded'] if t_u <= x[0] <= t_u + 10.0), None)
            if dn is None:
                continue
            offs = [x[0] for x in self.L['planner_off'] if x[0] >= dn[0] - 0.05] + \
                   [x[0] for x in self.L['manager_off'] if x[0] >= dn[0] - 0.05]
            t_off = min([x for x in offs if x <= dn[0] + 1.0], default=dn[0])
            dt = next((x for x in self.L['detach'] if dn[0] - 0.05 <= x[0] <= dn[0] + 2.0), None)
            rs = next((r for r in self.L['resize'] if dn[0] - 0.05 <= r[0] <= dn[0] + 2.0), None)
            row = self.base_row()
            row.update(event='detach (planned)', drone=f'/drone_{u[2]}', t_event_s=self.rel(t_off),
                       formation=self.formation(self.carriers_after(dn[0] - 0.05)))
            row['unload_s'] = round(t_off - t_u, 2)
            row['unloaded_tilt_deg'] = float(dn[3])
            row['resize_s'] = round(rs[0] - t_off, 3) if rs else None
            row['separation_s'] = separation(self.B, self.X, t_off) if self.B is not None else None
            rows.append(self._finish(row, t_off, 'detach', dt is not None, 'no DETACH line'))
        return rows

    def _finish(self, row, te, kind, acted, why_not):
        tend = self.window_end(te, kind)
        row['window_end'] = ('LAND' if self.t_land and abs(tend - self.t_land) < 1e-6 else
                             'loss' if self.loss_between(te, math.inf) and abs(tend - self.loss_between(te, math.inf)[0]) < 1e-6
                             else 'end of record' if abs(tend - self.t_end) < 1e-6 else
                             f'{HORIZON_S:.0f} s' if abs(tend - te - HORIZON_S) < 1e-6 else 'next change')
        row.update(response(self.G, te, tend, self.motion == 'moving'))
        lim = min(te + SAFE_S, self.t_land or math.inf)
        lost = self.loss_between(te - 0.05, lim)
        row['loss'] = f'{lost[1]} at {lost[0] - te:+.1f} s' if lost else ''
        row['carried_on_s'] = round(min(lim, lost[0] if lost else math.inf, self.t_end) - te, 1)
        row['success'] = 'yes' if (acted and not lost) else 'no'
        if not acted:
            row['fail_reason'] = why_not
        elif lost:
            row['fail_reason'] = lost[1]
        return row

    # ---------------------------------------------------------------------------- the attach
    def attaches(self, descents):
        if self.kind() not in ('re-attach', 'attach from the mat', 'application', 'delivery-rejoin'):
            return []
        mine = [d for d in descents if d['flight'] == self.name and d['day'] == self.day
                and d['kind'] in ('re-attach', 'attach from the mat')]
        mine.sort(key=lambda d: float(d['descent_start_s']))
        start = self.J['release'][0] if self.J['release'] else None
        his_to = self.J['his_takeoff'][0] if self.J['his_takeoff'] else None
        if self.kind() == 'attach from the mat':
            start = his_to
        if self.kind() in ('application', 'delivery-rejoin'):
            # the re-attach after the drop: from his CLEAR_DROP_ZONE onward
            cz = next((p[0] for p in self.J['phases'] if p[1] == 'CLEAR_DROP_ZONE'), None)
            start = cz
            if cz is None:
                return []
            t_bag = self.B['t0'] if self.B is not None else 0.0
            mine = [d for d in mine if float(d['descent_start_s']) + t_bag >= cz - 0.1]
            if not mine and not any(p[2] in ('APPROACH_ABOVE_TARGET', 'MATCH_VELOCITY') for p in self.J['phases']
                                    if p[0] >= cz):
                return []
        pv = [p for p in self.J['proven'] if start is None or p[0] >= start]
        t_pv = pv[0][0] if pv else None
        fold = None
        if t_pv is not None:
            fold = next((r[0] for r in self.L['resize'] if t_pv - 0.5 <= r[0] <= t_pv + 5.0 and int(r[1]) >= 4), None)
        row = self.base_row()
        row.update(event='attach', drone='/drone_3', t_event_s=self.rel(fold) if fold else None,
                   formation=self.formation(self.typed_carriers()),
                   plate=self.args.get('plate') or self.args.get('reattach_plate') or '9',
                   t_start_s=self.rel(start), start_from=('his TAKEOFF' if self.kind() == 'attach from the mat' else
                                                          'CLEAR_DROP_ZONE' if self.kind() in ('application',
                                                                                               'delivery-rejoin')
                                                          else 'PARTNER RELEASE'))
        before = [d for d in mine if t_pv is None or float(d['descent_start_s']) + self.B['t0'] <= t_pv]
        row['descents'] = len(before)
        row['descents_at_plate_height'] = sum(1 for d in before if d['eval'] == 'plate height')
        row['contacts'] = sum(1 for d in before if d.get('contact_log_xy_mm', '') != '')
        misses = [float(d['tip_xy_cm']) for d in before if d['eval'] == 'plate height' and d.get('tip_xy_cm', '') != '']
        row['miss_median_cm'] = float(np.median(misses)) if misses else None
        row['miss_min_cm'] = min(misses) if misses else None
        row['miss_max_cm'] = max(misses) if misses else None
        row['miss_list_cm'] = ' '.join(f'{m:.1f}' for m in misses)
        row['proven'] = 'yes' if t_pv else 'no'
        row['folded_in'] = 'yes' if fold else 'no'
        if t_pv and start:
            row['start_to_proven_s'] = round(t_pv - start, 1)
        if fold and start:
            row['start_to_foldin_s'] = round(fold - start, 1)
        if t_pv and his_to:
            row['his_takeoff_to_proven_s'] = round(t_pv - his_to, 1)
        if fold:
            con = [c for c in self.J['contacts'] if c[0] <= t_pv]
            if con:
                row['contact_to_foldin_s'] = round(fold - con[-1][0], 2)
                row['contact_xy_mm'] = con[-1][2]
            row = self._finish(row, fold, 'attach', True, '')
            # the catch must stay on its plate until LAND or 15 s (RIG-1111-nrtfc_3: proven and folded in, then the
            # magnet let go 4 s later with no command and no loss line)
            lim = min(fold + SAFE_S, self.t_land or math.inf)
            off = separation(self.B, self.X, fold, horizon=max(0.5, lim - fold)) if self.B is not None else None
            if off is not None and row['success'] == 'yes':
                row['success'] = 'no'
                row['fail_reason'] = f'the magnet left the plate {off:.1f} s after the fold-in (no command)'
                row['loss'] = f'magnet off its plate at +{off:.1f} s'
                row['carried_on_s'] = round(off, 1)
            return [row]
        row['success'] = 'no'
        tail = self._why_no_attach(start)
        row['fail_reason'] = tail
        return [row]

    def typed_carriers(self):
        """The carriers' plates as typed (attach mode types the three carriers; QUAD4 joins on its own plate)."""
        az = [a for a in self.args.get('attach_azimuths_deg', '').split(',') if a.strip()]
        if self.args.get('mode') == 'attach' and len(az) == 3:
            return sorted(int(round(float(a) / 30.0)) % 12 for a in az)
        return None

    def _why_no_attach(self, start):
        if not self.J['contacts'] or (start and all(c[0] < start for c in self.J['contacts'])):
            why = 'no contact'
        elif self.J['proof_failed']:
            why = f"contact, proof failed ({len(self.J['proof_failed'])}x)"
        else:
            why = 'contact, no proof'
        lo = self.loss_between(start or -math.inf, math.inf)
        if lo:
            why += f'; {lo[1]}'
        elif self.t_land:
            why += '; operator LAND'
        if any(p[2] == 'TAKEN_BACK' for p in self.J['phases']):
            why += ' (taken back)'
        return why

    # ---------------------------------------------------------------------------- the application
    def phases(self, descents):
        k = self.kind()
        if k not in ('application', 'pickup-drop', 'delivery-rejoin'):
            return []
        P = self.J['phases']

        def first(to=None, frm=None, after=-math.inf):
            return next((p[0] for p in P if p[0] > after and (to is None or p[2] == to) and (frm is None or p[1] == frm)),
                        None)
        r = self.base_row()
        top = self.ring_up()
        unload = self.L['unload'][0][0] if self.L['unload'] else None
        prel = self.J['release'][0] if self.J['release'] else None
        his_to = self.J['his_takeoff'][0] if self.J['his_takeoff'] else None
        p_start = prel if k == 'application' else his_to
        lifted = (first(to='TRANSIT_TO_DROP_POINT', frm='LIFT_OBJECT', after=p_start or -math.inf)
                  if k != 'delivery-rejoin' else first(to='TRANSIT_TO_DROP_POINT'))
        t_drop = first(to='DROP_OBJECT', after=lifted or math.inf)
        t_clear = first(frm='DROP_OBJECT', after=t_drop or math.inf)
        att = self.attaches(descents) if k in ('application', 'delivery-rejoin') else []
        fold = None
        if att and att[0].get('folded_in') == 'yes':
            fold = self.t_takeoff + att[0]['t_event_s']
        spans = [('lift', self.t_takeoff, top)]
        if k == 'application':
            spans.append(('detach', unload, prel))
        if k != 'delivery-rejoin':
            spans.append(('pickup', p_start, lifted))
        spans += [('transit', lifted, t_drop), ('drop', t_drop, t_clear)]
        if k in ('application', 'delivery-rejoin'):
            spans.append(('re-attach', t_clear, fold))
        spans.append(('land', self.t_land, self.t_landed))
        r['event'] = {'application': 'application', 'pickup-drop': 'pickup and drop',
                      'delivery-rejoin': 'delivery and rejoin'}[k]
        r['formation'] = 'triangle' if self.args.get('attach_azimuths_deg', '') == '90,330,210' else '?'
        failed = None
        for name, a, b in spans:
            r[f'{name}_start_s'] = self.rel(a)
            r[f'{name}_s'] = round(b - a, 1) if (a is not None and b is not None) else None
            if failed is None and name != 'land' and (a is None or b is None):
                failed = name
        lost = self.loss_between(self.t_takeoff or -math.inf, self.t_land or math.inf)
        # where it stopped: the last phase that had started when the first loss or the operator's LAND came
        t_stop = min([x for x in (lost[0] if lost else None, self.t_land) if x is not None], default=None)
        if t_stop is not None and (failed is not None or lost):
            started = [name for name, a, b in spans if name != 'land' and a is not None and a <= t_stop]
            failed = started[-1] if started else failed
        r['pickup_attempts'] = sum(1 for p in P if p[2] == 'DESCEND_TO_PICKUP' and (p_start is None or p[0] >= p_start))
        drops = [d for d in descents if d['flight'] == self.name and d['day'] == self.day
                 and d['kind'] == 'drop into the net']
        if drops:
            r['drop_outcome'] = drops[0]['outcome']
            r['drop_impact_xy_cm'] = drops[0].get('impact_xy_cm', '')
        if att:
            r['reattach_descents'] = att[0].get('descents')
            r['reattach_miss_median_cm'] = att[0].get('miss_median_cm')
        r['total_s'] = round(self.t_landed - self.t_takeoff, 1) if (self.t_landed and self.t_takeoff) else None
        r['completed'] = 'yes' if failed is None and not lost else 'no'
        if failed is not None or lost:
            where = failed
            why = []
            if failed == 're-attach' and att:
                why.append(att[0].get('fail_reason', ''))
            else:
                if lost:
                    why.append(f'{lost[1]} at +{lost[0] - self.t_takeoff:.1f} s')
                if self.t_land and failed:
                    why.append(f'operator LAND at +{self.t_land - self.t_takeoff:.1f} s')
                if any(p[2] == 'TAKEN_BACK' for p in P):
                    why.append('QUAD4 taken back')
            r['failed_in'] = where
            r['fail_reason'] = '; '.join(w for w in why if w)
        r['landed'] = 'yes' if self.t_landed else 'no'
        return [r]

    def summary_row(self, n_ev):
        r = self.base_row()
        r.update(events=n_ev, t_takeoff_epoch=f'{self.t_takeoff:.3f}' if self.t_takeoff else '',
                 bag='yes' if self.B is not None else 'no',
                 losses='; '.join(f'{x[1]} at +{x[0] - (self.t_takeoff or x[0]):.1f} s' for x in self.losses))
        acc = [u for u in self.L['unannounced']
               if not any(abs(u[0] - t) < 5.0 and int(u[2]) == d for t, d in self.operator_offs())]
        r['other_unannounced'] = '; '.join(f'/drone_{u[2]} at +{u[0] - (self.t_takeoff or u[0]):.1f} s ({u[3]})'
                                           for u in acc)
        return r


# ------------------------------------------------------------------------------------------------------- main
def run_joins(fids):
    """joins.py's descents for these flights, written to joins/ (its own definitions, unchanged)."""
    if not fids:
        return []
    out = os.path.join(HERE, 'joins')
    os.makedirs(out, exist_ok=True)
    joins.OUT = out
    joins.DAYS = tuple(sorted({f.split('/')[0] for f in fids}))
    argv = sys.argv
    sys.argv = ['joins.py'] + list(fids)
    try:
        joins.main()
    finally:
        sys.argv = argv
    with open(os.path.join(out, 'joins.csv'), newline='') as f:
        return list(csv.DictReader(f))


def write(name, rows, first):
    cols = list(first)
    for r in rows:
        for k in r:
            if k not in cols:
                cols.append(k)

    def fmt(v):
        if isinstance(v, float):
            return '' if not math.isfinite(v) else f'{v:.3f}' if abs(v) < 100 else f'{v:.1f}'
        return '' if v is None else v
    with open(os.path.join(HERE, name), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=cols, lineterminator='\n')
        w.writeheader()
        for r in rows:
            w.writerow({k: fmt(r.get(k)) for k in cols})


def main(argv):
    fids = discover(argv)
    regs = registry()
    cond = conditions()
    launch = {}
    for day in sorted({f.split('/')[0] for f in fids}):
        for L in launches(day):
            launch[f"{L['day']}/{L['name']}"] = L
    flights = []
    for fid in fids:
        print(f'{fid}: reading ...', flush=True)
        try:
            flights.append(Flight(fid, launch.get(fid, {}), regs, cond))
        except Exception as ex:  # noqa: BLE001
            print(f'  {fid}: skipped ({type(ex).__name__}: {ex})')
    att = [f.fid for f in flights if f.kind() in ('re-attach', 'attach from the mat', 'application', 'pickup-drop',
                                                  'delivery-rejoin')
           and os.path.exists(os.path.join(RIG, f.day, 'launches.csv'))]
    descents = run_joins(att)
    events, phases, summ = [], [], []
    for f in flights:
        ev = f.detaches() + f.attaches(descents)
        ph = f.phases(descents)
        events += ev
        phases += ph
        summ.append(f.summary_row(len(ev) + len(ph)))
        print(f'{f.fid}: {f.kind()}, {len(ev)} events' + (', application phases' if ph else ''))
    head = ['day', 'flight', 'registry_id', 'event', 'success', 'fail_reason', 'flight_kind', 'motion', 'traj',
            'formation', 'funnel', 'cable_m', 'load_kg_config', 'void', 'exclude', 't_event_s']
    write('events.csv', events, head)
    write('phases.csv', phases, ['day', 'flight', 'registry_id', 'event', 'completed', 'failed_in', 'fail_reason'])
    write('flights.csv', summ, ['day', 'flight', 'registry_id', 'flight_kind'])
    print(f'wrote {len(events)} events, {len(phases)} application rows, {len(summ)} flights to {HERE}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
