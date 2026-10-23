#!/usr/bin/env python3
"""tools/rig_night.py -- one command per rig flight: the bag and logs of 14 Oct into the thesis's rows and figure inputs.
    source ~/ros2_humble/install/setup.bash; source install/setup.bash       # only for the bag (conversion, attach traces)
    python3 tools/rig_night.py --date 2026-10-14 R2:r2 R3:r3 R4:r4 R5:r5 R7:r7 R8:r8 R9:r9
    python3 tools/rig_night.py --date 2026-10-08 R4:r303_4 R5:r305          # the 8 Oct flights as a test
Each LABEL:NAME is a sheet line (docs/rig/wed_2026-10-14.md) and the flight's name under results/rig/<date>/
(tools/rig_flight.sh: <name> the bag, <name>.log the T2 console, <name>_logs the node logs). The flight is converted
with tools/rig_to_run.py when <name>_run is missing (the bag is needed for the ring's pose; ROS sourced), then by label:
  R1       the 3-min hover (T7 row K7): ring xy error, height sd, tilt, pack voltage per drone.
  R2, R3   the circle (T7 rows K3, K4): per lap the xy error mean and max, the along-track lag, the height sd, the
           ring tilt; the lap-to-lap sd; R3's frozen falsifier against 2.91 cm (the sheet).
  R4, R5   the unannounced detach (T8 rows L8, L9; T13; the F16 ledger): the T6 metrics of docs/rig/freeze_2026-10-14.md
           (tools/release_bands.py) to LAND and cut at 15 s, the detection delay, the dip, the landing lean, the
           survivors' planned pull before and after, Delta-theta against the rig's no-event floor, and the score against
           the frozen W4/W5 band with the two naive predictors (freeze 1.4 and 2.4).
  R6-R8    the attach (F15 input): the console timeline (contact, proof, ATTACH, mux, take-back) and, with the bag, the
           tip, plate, ring and QUAD4 traces at 50 Hz.
  R9       the full mission: the announced detach as R4 (announced), then the attach as R7.
Outputs under results/analysis/rig_<date>/<LABEL>_<name>/: metrics.json, summary.md (with the LaTeX row text, run ids
in comments), laps.csv / ledger.csv / timeline.csv / trace.csv as the label needs; and rows.md over every flight done.
Reads results/ only; writes nothing into results/rig/ except the missing <name>_run conversions."""
import argparse
import glob
import json
import os
import re
import sys

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tools'))
import release_bands as rb  # noqa: E402

RIG_ROOT = os.path.join(os.environ.get('MDC_RESULTS_ROOT', os.path.join(REPO, 'results')), 'rig')
STAMP = re.compile(r'\[(\d{9,}\.\d+)\]')
PLATE_R, PIVOT_DZ = 0.225, -0.04
M_DRONE = 0.55

# docs/rig/freeze_2026-10-14.md section 2.4 (computed 11 Oct 17:35 after the signature) and 1.4: the frozen numbers.
FROZEN = {
    'R4': dict(cell='W4', name='hover detach, unannounced', band=(1.75, 14.51),
               twin={'R1459': 6.12, 'R1460': 5.74, 'R1626': 6.67}, prev=('r303_4', 5.29), traj='hover'),
    'R5': dict(cell='W5', name='circle detach, unannounced', band=(1.00, 13.20),
               twin={'R1457': 4.87, 'R1627': 5.91}, prev=('r305', 8.02), traj='circle'),
}
SIGMA_RIG = 1.568
# results/analysis/p6_2026-10-12/5a_null/summary.txt (12 Oct): dtheta at no-event pseudo-events on the rig
NULL = {('hover', 7): dict(mean=4.79, sd=0.77, p95=6.00, n=23), ('path', 7): dict(mean=5.58, sd=1.21, p95=7.22, n=107),
        ('path', 15): dict(mean=6.27, sd=1.52, p95=8.32, n=43)}
NULL_RATIO = {'hover': (3.25, 4.06), 'path': (3.07, 3.87)}     # dtheta / sigma_pre: median, p95
K2_CM = 2.91                                                   # the frozen R3 falsifier's reference (sheet, plan 4.1)
TIMELINE_KEYS = ('fleet command received', 'emergency stop', 'landed - disarming', 'handover settle complete',
                 'hover hold done', 'unannounced detach', 'fleet resized', 'magnet off', 'detach drone', 'drop and land',
                 'disarmed alone', 'down at its rest', 'mux:', 'take-back', 'partner', 'his takeoff', 'his stream',
                 'release guard', 'attach:', 'attach commanded', 'attach drone', 'attach of', 'attach refused', 'proof',
                 'contact', 'weld of', 'welded', 'fold', 'elrs magnet', 'magnet command',
                 'tilt_fault', 'fault_land', 'refused', 'kinematic', 'give up', 'gives up')


# ---------------------------------------------------------------- files
def flight_paths(date, name):
    base = os.path.join(RIG_ROOT, date, name)
    p = dict(name=name, date=date, bag=base, log=base + '.log', logs=base + '_logs', run=base + '_run')
    for k in ('log', 'logs'):
        if not os.path.exists(p[k]):
            sys.exit(f'{name}: {p[k]} not found')
    p['has_bag'] = os.path.isdir(p['bag'])
    return p


def ensure_run(p):
    if os.path.isfile(os.path.join(p['run'], 'logs', 'run.csv')):
        return
    trackers = sorted(glob.glob(os.path.join(p['logs'], 'logs', 'tracker', 'drone*')))
    if not trackers:
        sys.exit(f"{p['name']}: no tracker logs under {p['logs']}")
    try:
        import rig_to_run
    except ImportError as e:
        sys.exit(f'rig_to_run: {e}')
    try:
        rig_to_run.convert(trackers, p['run'], bag=p['bag'] if p['has_bag'] else None, t2_log=p['log'])
    except ModuleNotFoundError as e:
        sys.exit(f"{p['name']}: converting needs ROS sourced for the bag ({e}); or convert first with tools/rig_to_run.py")
    print(f"  converted -> {p['run']}")


def load_run(p):
    run = pd.read_csv(os.path.join(p['run'], 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    ev = pd.read_csv(os.path.join(p['run'], 'logs', 'events.csv'))
    evd = {}
    for _, r in ev.iterrows():
        evd.setdefault(str(r['event']), float(r['sim_time']))
    man = json.load(open(os.path.join(p['run'], 'manifest.json')))
    return run, evd, man


def planner_log(p):
    hits = (glob.glob(os.path.join(p['logs'], 'logs', 'dissipative_planner', '*', 'log.csv'))
            + glob.glob(os.path.join(p['logs'], 'logs', 'mpc_planner', '*', 'log.csv')))
    if not hits:
        return None, {}
    P = pd.read_csv(hits[0])
    prm = json.load(open(os.path.join(os.path.dirname(hits[0]), 'params.json'))) if os.path.isfile(
        os.path.join(os.path.dirname(hits[0]), 'params.json')) else {}
    return P, prm


def tracker_params(p):
    out = {}
    for f in glob.glob(os.path.join(p['logs'], 'logs', 'tracker', 'drone*', 'params.json')):
        d = int(re.search(r'drone(\d+)_', f).group(1))
        out[d] = json.load(open(f))
    return out


def timeline(p, t0):
    rows = []
    for line in open(p['log'], errors='replace'):
        m = STAMP.search(line)
        if not m:
            continue
        low = line.lower()
        if any(k in low for k in TIMELINE_KEYS):
            text = line[m.end():].strip()
            text = re.sub(r'^\[\w+\]:\s*', '', text)
            rows.append(dict(t=round(float(m.group(1)) - t0, 3), text=text[:220]))
    return pd.DataFrame(rows)


def code_line(p):
    f = os.path.join(os.path.dirname(p['logs']), p['name'] + '_code.txt')
    return open(f, errors='replace').readline().strip()[:80] if os.path.isfile(f) else ''


def conditions(p, prm, tprm):
    az = str(prm.get('attach_azimuths_deg', '')).replace(' ', '')
    plates = [str(int(round((-float(a)) % 360 / 30)) % 12) for a in az.split(',') if a] if az else []
    srcs = sorted({v.get('cable_source', '?') for v in tprm.values()})
    return dict(mode=prm.get('mode', ''), cable_source='/'.join(srcs),
                cable_elev_deg=prm.get('cable_elev_deg', prm.get('handover_elev_deg', '')),
                int_mode=prm.get('int_mode', ''), attach_azimuths_deg=az, plates_clockwise=plates,
                departed_land=prm.get('departed_land', ''), departed_ground_idle=prm.get('departed_ground_idle', ''),
                drop_on_loss=prm.get('drop_on_loss', ''), detach_detect_m=prm.get('detach_detect_m', ''),
                code=code_line(p))


# ---------------------------------------------------------------- carry (R1-R3)
def circle_fit(pts):
    A = np.c_[2 * pts, np.ones(len(pts))]
    b = (pts ** 2).sum(1)
    cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]
    return np.array([cx, cy]), float(np.sqrt(c + cx * cx + cy * cy))


def carry_metrics(run, evd):
    """Per lap (the reference's unwrapped angle advancing 2 pi) and over all laps: xy error, along-track lag, height."""
    t = run.t.to_numpy(float)
    ref = run[['payload_ref_x', 'payload_ref_y']].to_numpy(float)
    pos = run[['payload_x', 'payload_y']].to_numpy(float)
    t_h = evd.get('HANDOVER', evd.get('TAKEOFF', t[0]) + 15.0)
    t_end = evd.get('LAND', t[-1])
    hov = np.nanmedian(ref[(t > t_h) & (t < t_h + 2.0)], 0)
    moved = (np.linalg.norm(ref - hov, axis=1) > 0.01) & (t > t_h)
    if not moved.any():
        return None
    t0 = float(t[moved][0])
    w = (t >= t0) & (t < t_end) & np.all(np.isfinite(pos), 1)
    c, R = circle_fit(ref[w])
    ang = lambda q: np.arctan2(q[:, 1] - c[1], q[:, 0] - c[0])          # noqa: E731
    a_ref = np.unwrap(ang(ref[w]))
    adv = np.abs(a_ref - a_ref[0])
    n_laps = int(adv[-1] // (2 * np.pi))
    bounds = [t[w][0]] + [float(t[w][np.flatnonzero(adv >= 2 * np.pi * k)[0]]) for k in range(1, n_laps + 1)]
    ez = (run.payload_z - run.payload_ref_z).to_numpy(float)
    tilt = run.payload_tilt_deg.to_numpy(float)
    tw, rw = t[w], ref[w]
    ahead = np.stack([np.interp(tw + 1.0, tw, rw[:, i]) for i in range(2)], 1)
    v_ref = np.linalg.norm(ahead - rw, axis=1)
    v = float(np.median(v_ref[(tw > t0 + 5.0) & (tw < tw[-1] - 1.5)])) if (tw[-1] - t0) > 8 else float(np.median(v_ref))

    def stats(a, b):
        m = (t >= a) & (t < b) & np.all(np.isfinite(pos), 1)
        dang = np.angle(np.exp(1j * (ang(pos[m]) - ang(ref[m]))))
        along = R * dang
        exy = np.linalg.norm(pos[m] - ref[m], axis=1)
        return dict(t_from=round(a, 2), t_to=round(b, 2), dur_s=round(b - a, 1), exy_mean_cm=round(100 * exy.mean(), 2),
                    exy_max_cm=round(100 * exy.max(), 1), along_mean_cm=round(100 * along.mean(), 2),
                    lag_s=round(-float(along.mean()) / v, 3) if v > 0.01 else None, z_sd_cm=round(100 * np.nanstd(ez[m]), 2),
                    z_mean_cm=round(100 * np.nanmean(ez[m]), 2), tilt_mean_deg=round(float(np.nanmean(tilt[m])), 2),
                    tilt_max_deg=round(float(np.nanmax(tilt[m])), 2), n=int(m.sum()))
    def best_shift(a, b):
        m = (t >= a) & (t < b) & np.all(np.isfinite(pos), 1)
        cands = [(float(np.nanmean(np.linalg.norm(pos[m] - np.stack([np.interp(t[m] - L, t, ref[:, i]) for i in range(2)], 1), axis=1))), L)
                 for L in np.arange(0.0, 1.0, 0.01)]
        return min(cands)[1]
    laps = [dict(lap=k + 1, **stats(bounds[k], bounds[k + 1])) for k in range(n_laps)]
    plus4 = stats(t0 + 4.0, t_end)
    plus4['lag_shift_s'] = round(best_shift(t0 + 4.0, t_end), 2)
    out = dict(orbit_start_s=round(t0, 2), radius_m=round(R, 3), speed_mps=round(v, 3), n_laps=n_laps,
               whole=stats(t0, t_end), plus4_to_land=plus4, laps=laps)
    if n_laps >= 2:
        out['lap_sd'] = {k: round(float(np.std([L[k] for L in laps], ddof=1)), 2)
                         for k in ('exy_mean_cm', 'exy_max_cm', 'lag_s', 'z_sd_cm', 'tilt_mean_deg') if laps[0][k] is not None}
        out['full_laps'] = stats(bounds[0], bounds[-1])
    return out


def hover_metrics(run, evd, tprm_dir):
    t = run.t.to_numpy(float)
    a = evd.get('HANDOVER', evd.get('TAKEOFF', t[0]) + 15.0) + 2.0
    b = evd.get('LAND', t[-1])
    m = (t >= a) & (t < b)
    exy = np.linalg.norm(run[['payload_x', 'payload_y']].to_numpy(float)[m] - run[['payload_ref_x', 'payload_ref_y']].to_numpy(float)[m], axis=1)
    ez = (run.payload_z - run.payload_ref_z).to_numpy(float)[m]
    out = dict(t_from=round(a, 2), t_to=round(b, 2), hold_s=round(b - a, 1), exy_mean_cm=round(100 * np.nanmean(exy), 2),
               exy_max_cm=round(100 * np.nanmax(exy), 1), z_sd_cm=round(100 * np.nanstd(ez), 2), z_mean_cm=round(100 * np.nanmean(ez), 2),
               tilt_mean_deg=round(float(np.nanmean(run.payload_tilt_deg[m])), 2), tilt_max_deg=round(float(np.nanmax(run.payload_tilt_deg[m])), 2))
    volts = {}
    for f in glob.glob(os.path.join(tprm_dir, 'drone*', 'log.csv')):
        d = int(re.search(r'drone(\d+)_', f).group(1))
        T = pd.read_csv(f, usecols=['sim_time', 'battery_v'])
        if T.battery_v.notna().any():
            volts[d] = dict(v_first=round(float(T.battery_v.dropna().iloc[0]), 2), v_last=round(float(T.battery_v.dropna().iloc[-1]), 2),
                            v_min=round(float(T.battery_v.min()), 2))
    out['pack_v'] = volts
    return out


# ---------------------------------------------------------------- release (R4, R5, R9)
def resized_survivors(P, te, t0_epoch):
    if P is None or 'n' not in P or 'slot2drone' not in P:
        return None
    rt = P.sim_time - t0_epoch
    res = P[(P.n < int(P.n.max())) & (rt > te - 1.0)]
    return [int(v) for v in str(res.slot2drone.iloc[0]).split()] if len(res) else None


def release_metrics(p, run, evd, P, prm, t0_epoch):
    r = rb.Run(p['run'])
    out = dict(kind=r.kind, t_event=round(r.te, 3), t_land=round(r.tland, 3), hold_s=round(r.tland - r.te, 2))
    for key, hz in (('to_land', None), ('cut_7s', 7.0), ('cut_15s', 15.0)):
        m = r.metrics(hz)
        out[key] = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in m.items()
                    if k in ('window_s', 'dtheta', 't_peak', 'abs_peak', 'rise', 'sigma_pre', 'ref_still', 'steady', 't_back', 'post', 'band_deg')}
    if 'DETECTED' in evd and 'RELEASE' in evd:
        out['detection_s'] = round(evd['DETECTED'] - evd['RELEASE'], 3)
    t = run.t.to_numpy(float)
    z = run.payload_z.to_numpy(float)
    pre = (t >= r.te - 5.0) & (t < r.te)
    win = (t >= r.te) & (t <= r.tland)
    out['dip_cm'] = round(100 * (np.nanmean(z[pre]) - np.nanmin(z[win])), 1)
    out['dip_at_s'] = round(float(t[win][np.nanargmin(z[win])] - r.te), 2)
    # landing lean (tools/release_transient.py's scorer): each survivor's tilt 1.2 s after the ring touches down
    # (ring z within 5 cm of its rest); the leaver is the drone whose attached flag drops at the event
    rest = float(np.nanmedian(z[:50]))
    survivors = resized_survivors(P, r.te, t0_epoch)
    if survivors is None:
        k1 = int(np.argmin(np.abs(t - (r.te + 1.0))))
        survivors = [d for d in range(4) if f'd{d}_attached' in run and float(run[f'd{d}_attached'].iloc[k1]) > 0.5] or list(range(4))
    out['survivors'] = survivors
    down = np.flatnonzero((t >= r.tland) & (z <= rest + 0.05))
    if len(down):
        td = float(t[down[0]])
        k = int(np.argmin(np.abs(t - (td + 1.2))))
        leans = {d: round(float(run[f'd{d}_tilt_deg'].iloc[k]), 1) for d in survivors
                 if f'd{d}_tilt_deg' in run and np.isfinite(run[f'd{d}_tilt_deg'].iloc[k])}
        out['landing_lean_deg'] = leans
        out['ring_down_s'] = round(td, 2)
    # the planned pull: per survivor before the release and on the first resized plan
    if P is not None and 'n' in P and 'slot2drone' in P:
        P = P.copy()
        P['rt'] = P.sim_time - t0_epoch
        n0 = int(P.n.max())
        res = P[(P.n < n0) & (P.rt > r.te - 1.0)]
        if len(res):
            res = res.iloc[0]
            before = P[(P.rt >= r.te - 2.0) & (P.rt < r.te - 0.2) & (P.n == n0)]
            s2d_b = [int(v) for v in str(before.slot2drone.iloc[-1]).split()]
            s2d_a = [int(v) for v in str(res.slot2drone).split()]
            pulls = {}
            for d in s2d_a:
                kb, ka = s2d_b.index(d), s2d_a.index(d)
                pulls[d] = dict(before_N=round(float(before[f't{kb}'].mean()), 2), after_N=round(float(res[f't{ka}']), 2))
            leaver = [d for d in s2d_b if d not in s2d_a]
            out['planned_pull'] = dict(survivors=pulls, leaver=leaver, leaver_before_N=round(float(before[f't{s2d_b.index(leaver[0])}'].mean()), 2) if leaver else None,
                                       resize_arrival_s=round(float(res.rt) - r.te, 3), survivors_max_after_N=max(v['after_N'] for v in pulls.values()))
    return out


def floor_reading(rm, traj):
    """Delta-theta against the rig's no-event floor (P6 5a): which window, the null's p95, dtheta / sigma_pre."""
    key = ('hover' if traj == 'hover' else 'path')
    out = {}
    for label, win in (('cut_7s', 7), ('cut_15s', 15)):
        nl = NULL.get((key, win))
        d, s = rm[label]['dtheta'], rm[label]['sigma_pre']
        out[label] = dict(dtheta=d, null=nl, above_null_p95=(d > nl['p95']) if nl else None,
                          ratio=round(d / s, 2) if s > 0 else None, null_ratio_median_p95=NULL_RATIO[key],
                          above_ratio_p95=(d / s > NULL_RATIO[key][1]) if s > 0 else None)
    return out


def ledger(label, rm, date):
    fz = FROZEN.get(label)
    if fz is None:
        return None
    d = rm['cut_15s']['dtheta']
    tw = list(fz['twin'].values())
    lo, hi = fz['band']
    rows = [dict(predictor='frozen band (freeze 2.4)', lo=lo, hi=hi)]
    rows.append(dict(predictor='rig = twin', lo=round(max(0.0, min(tw) - 2 * SIGMA_RIG), 2), hi=round(max(tw) + 2 * SIGMA_RIG, 2)))
    rows.append(dict(predictor=f'rig = previous rig flight ({fz["prev"][0]})', lo=round(max(0.0, fz['prev'][1] - 2 * SIGMA_RIG), 2),
                     hi=round(fz['prev'][1] + 2 * SIGMA_RIG, 2)))
    for r in rows:
        r.update(cell=fz['cell'], flight_date=date, rig_dtheta_15s=d, hit=bool(r['lo'] <= d <= r['hi']), width=round(r['hi'] - r['lo'], 2))
    mid = (min(tw) + max(tw)) / 2
    return dict(cell=fz['cell'], rig_dtheta_15s=d, twin=fz['twin'], twin_midpoint=round(mid, 2),
                log_ratio=round(float(np.log(d / mid)), 3), factor_rig_over_twin_mean=round(d / float(np.mean(tw)), 2),
                scored=(date >= '2026-10-14'), predictors=rows)


# ---------------------------------------------------------------- attach (R6-R9)
def bag_traces(p, t0_epoch, plate=9):
    """Tip, ring, plate and QUAD4 at 50 Hz from the bag, plus every message on the attach's event topics."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as e:
        return None, None, f'bag not read ({e}: source ROS and the install)'
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=p['bag'], storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    want = {'/magnet_tip_pose': 'tip', '/payload/motion_capture_state': 'ring', '/drone_3/motion_capture_state': 'q4',
            '/pickup/motion_capture_state': 'puck', '/tejen/magnet_tip_pose': 'tip_t'}
    events_topics = [t for t in types if any(k in t for k in ('/magnet/', '/join_planner/', 'mux_state', 'take_back', '/partner/',
                                                                '/tejen/magnet', '/tejen/object', 'departed', 'down_alone', '/fleet/'))]
    series, evs = {}, []
    while rd.has_next():
        topic, data, ts = rd.read_next()
        if topic in want and topic in types:
            m = deserialize_message(data, get_message(types[topic]))
            pose = m.pose.pose if hasattr(m.pose, 'pose') else m.pose
            o = pose.orientation
            series.setdefault(want[topic], []).append((ts * 1e-9 - t0_epoch, pose.position.x, pose.position.y, pose.position.z, o.w, o.x, o.y, o.z))
        elif topic in events_topics:
            m = deserialize_message(data, get_message(types[topic]))
            txt = str(m).replace('\n', ' ')
            evs.append(dict(t=round(ts * 1e-9 - t0_epoch, 3), topic=topic, msg=txt[:160]))
    if 'ring' not in series:
        return None, pd.DataFrame(evs), 'no ring pose in the bag'
    cols = ['t', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz']
    S = {k: pd.DataFrame(v, columns=cols).sort_values('t') for k, v in series.items()}
    t_lo = max(df.t.iloc[0] for df in S.values())
    t_hi = min(df.t.iloc[-1] for df in S.values())
    grid = np.arange(np.ceil(t_lo * 50) / 50, t_hi, 0.02)
    out = pd.DataFrame({'t': grid})
    for k, df in S.items():
        for c in cols[1:]:
            out[f'{k}_{c}'] = np.interp(grid, df.t, df[c])
    # plate <plate> clockwise from +x in the ring frame, PLATE_R out, through the ring's quaternion
    az = np.deg2rad(-30.0 * plate)
    local = np.array([PLATE_R * np.cos(az), PLATE_R * np.sin(az), 0.0])
    q = out[['ring_qw', 'ring_qx', 'ring_qy', 'ring_qz']].to_numpy(float)
    w, x, y, z = q.T
    Rm = np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
                   np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
                   np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], 1)
    pl = out[['ring_x', 'ring_y', 'ring_z']].to_numpy(float) + Rm @ local
    out['plate_x'], out['plate_y'], out['plate_z'] = pl.T
    out['ring_tilt_deg'] = np.degrees(np.arccos(np.clip(Rm[:, 2, 2], -1, 1)))
    if 'tip_x' in out:
        out['tip_to_plate_m'] = np.linalg.norm(out[['tip_x', 'tip_y', 'tip_z']].to_numpy(float) - pl, axis=1)
        out['tip_over_plate_m'] = out.tip_z - out.plate_z
    return out, pd.DataFrame(evs), None


def attach_summary(tl, trace, bag_ev):
    out = {}
    def first(*keys):
        for _, r in tl.iterrows():
            low = r.text.lower()
            if all(k in low for k in keys):
                return float(r.t), r.text
        return None, None
    for name, keys in (('his_takeoff', ('his takeoff',)), ('attach_cmd', ('attach commanded',)), ('attach', ('attach:',)),
                       ('kinematic_attach', ('kinematic attach',)), ('mux_ours', ('mux:', 'ours')), ('take_back', ('take-back',)),
                       ('release_guard', ('release guard',)), ('disarmed_alone', ('disarmed alone',)), ('land', ("received: 'land'",))):
        t, text = first(*keys)
        if t is not None:
            out[name] = dict(t=round(t, 3), text=text[:160])
    if bag_ev is not None and len(bag_ev):
        firsts = bag_ev.groupby('topic').first().reset_index()
        out['bag_first_messages'] = {r.topic: dict(t=r.t, msg=r.msg) for _, r in firsts.iterrows()}
        for topic in ('/magnet/contact', '/magnet/attach_proof', '/magnet/object_attached'):
            sub = bag_ev[(bag_ev.topic == topic) & bag_ev.msg.str.contains('True', case=False)]
            if len(sub):
                out[topic.strip('/').replace('/', '_') + '_true_s'] = float(sub.t.iloc[0])
    if trace is not None and 'tip_to_plate_m' in trace:
        d = trace.tip_to_plate_m.to_numpy(float)
        out['tip_to_plate_min_m'] = round(float(np.nanmin(d)), 4)
        out['tip_to_plate_min_at_s'] = round(float(trace.t[int(np.nanargmin(d))]), 2)
        close = trace[d < 0.02]
        if len(close):
            out['tip_within_2cm_s'] = dict(first=round(float(close.t.iloc[0]), 2), last=round(float(close.t.iloc[-1]), 2))
    return out


# ---------------------------------------------------------------- rows (LaTeX text; run ids only in comments)
def tex_rows(label, name, date, mx):
    L = []
    if label in ('R2', 'R3') and mx.get('carry'):
        c = mx['carry']
        st = c['plus4_to_land']
        k = 'K3' if label == 'R2' else 'K4'
        pull = 'measured' if label == 'R2' else 'planned'
        sd = c.get('lap_sd', {}).get('exy_mean_cm')
        L.append(f"    {k} & circle 0.125\\,m/s & {pull} & 60$^\\circ$ & {st['exy_mean_cm']:.1f} / {st['exy_max_cm']:.1f} & "
                 f"{st['lag_shift_s']:.2f} & {st['z_sd_cm']:.1f} & 1 \\\\ % {date} {name}: orbit start + 4 s to LAND, lag = best time shift; "
                 f"{c['n_laps']} full laps{'' if sd is None else f', lap-to-lap sd of the xy mean {sd:.2f} cm'}")
    if label == 'R1' and mx.get('hover'):
        h = mx['hover']
        L.append(f"    K7 & hover {h['hold_s'] / 60:.1f} min & measured & 60$^\\circ$ & {h['exy_mean_cm']:.2f} / {h['exy_max_cm']:.1f} & -- & "
                 f"{h['z_sd_cm']:.1f} & 1 \\\\ % {date} {name}")
    if label in ('R4', 'R5', 'R9') and mx.get('release'):
        r = mx['release']
        ev = 'L8' if label == 'R4' else ('L9' if label == 'R5' else 'L?')
        det = f"{r['detection_s']:.2f}" if 'detection_s' in r else '--'
        tb = r['to_land'].get('t_back')
        tb = '--' if tb is None else f'{tb:.1f}'
        lean = max(r.get('landing_lean_deg', {}).values()) if r.get('landing_lean_deg') else None
        pull = r.get('planned_pull', {})
        pulls = ' / '.join(f"{v['before_N']:.1f}$\\to${v['after_N']:.1f}" for v in pull.get('survivors', {}).values())
        band = FROZEN.get(label, {}).get('band')
        tw = f"{band[0]:.2f}--{band[1]:.2f} band" if band else '--'
        L.append(f"    {ev} & {det} & {r['cut_15s']['dtheta']:.1f} & {r['to_land']['abs_peak']:.1f} & {r['dip_cm']:.1f} & {tb} & "
                 f"{'--' if lean is None else f'{lean:.1f}'} & {pulls} & {tw} \\\\ % {date} {name}: dtheta cut at 15 s "
                 f"(to LAND {r['to_land']['dtheta']:.1f}, hold {r['hold_s']:.1f} s)")
        if mx.get('ledger'):
            lg = mx['ledger']
            fb = lg['predictors'][0]
            L.append(f"    {FROZEN[label]['name'].capitalize()} & {', '.join(f'{v:.1f}' for v in lg['twin'].values())}, {len(lg['twin'])} & "
                     f"{lg['rig_dtheta_15s']:.1f}, 1 & {lg['factor_rig_over_twin_mean']:.2f} & after & {fb['lo']:.2f}--{fb['hi']:.2f} & "
                     f"{'hit' if fb['hit'] else 'miss'} \\\\ % T13, {date} {name}; log ratio {lg['log_ratio']:+.2f}")
    return L


def summary_md(label, name, date, mx, p):
    s = [f'# {label}: {name} ({date})', '']
    cond = mx.get('conditions', {})
    s.append('Conditions from the logs: ' + ', '.join(f'{k} {v}' for k, v in cond.items() if v not in ('', None, [])))
    s.append('')
    if mx.get('hover'):
        h = mx['hover']
        s += ['## Hover (T7 K7)', '', '| hold (s) | xy error mean / max (cm) | height sd (cm) | height mean (cm) | tilt mean / max (deg) |', '|---|---|---|---|---|',
              f"| {h['hold_s']} | {h['exy_mean_cm']} / {h['exy_max_cm']} | {h['z_sd_cm']} | {h['z_mean_cm']} | {h['tilt_mean_deg']} / {h['tilt_max_deg']} |", '']
        if h.get('pack_v'):
            s += ['| drone | V first | V last | V min |', '|---|---|---|---|'] + [f"| {d} | {v['v_first']} | {v['v_last']} | {v['v_min']} |" for d, v in sorted(h['pack_v'].items())] + ['']
    if mx.get('carry'):
        c = mx['carry']
        s += [f"## Circle (T7 {'K3' if label == 'R2' else 'K4'}): radius {c['radius_m']} m, {c['speed_mps']} m/s, {c['n_laps']} full laps "
              f"(orbit from {c['orbit_start_s']} s; lap rows: along-track lag = mean along-track error / speed)", '',
              '| lap | duration (s) | xy error mean / max (cm) | along-track mean (cm) | lag (s) | height sd (cm) | tilt mean (deg) |', '|---|---|---|---|---|---|---|']
        for L in c['laps']:
            s.append(f"| {L['lap']} | {L['dur_s']} | {L['exy_mean_cm']} / {L['exy_max_cm']} | {L['along_mean_cm']} | {L['lag_s']} | {L['z_sd_cm']} | {L['tilt_mean_deg']} |")
        w = c['plus4_to_land']
        s.append(f"| +4 s to LAND (T7's window; lag by best shift {w['lag_shift_s']} s) | {w['dur_s']} | {w['exy_mean_cm']} / {w['exy_max_cm']} | {w['along_mean_cm']} | {w['lag_s']} | {w['z_sd_cm']} | {w['tilt_mean_deg']} |")
        if c.get('lap_sd'):
            s.append(f"| lap-to-lap sd | | {c['lap_sd'].get('exy_mean_cm')} / {c['lap_sd'].get('exy_max_cm')} | | {c['lap_sd'].get('lag_s')} | {c['lap_sd'].get('z_sd_cm')} | {c['lap_sd'].get('tilt_mean_deg')} |")
        s.append('')
        if mx.get('falsifier'):
            f = mx['falsifier']
            s += [f"**R3 falsifier (frozen, sheet):** path error {f['exy_mean_cm']} cm against {K2_CM} cm; tolerance max(1 cm, lap-to-lap sd "
                  f"{f['lap_sd_cm']}) = {f['tol_cm']} cm; |difference| {f['diff_cm']} cm -> **{f['verdict']}**. ({f['note']})", '']
    if mx.get('release'):
        r = mx['release']
        s += [f"## Release ({r['kind']}), T6 metrics (freeze 1.1, tools/release_bands.py)", '',
              '| window | Delta-theta (deg) | abs peak (deg) | scalar rise (deg) | t_peak (s) | sigma_pre (deg) | ref still (s) | back in band (s) | post tilt (deg) |', '|---|---|---|---|---|---|---|---|---|']
        for k in ('to_land', 'cut_7s', 'cut_15s'):
            m = r[k]
            s.append(f"| {k} ({m['window_s']} s) | {m['dtheta']} | {m['abs_peak']} | {m['rise']} | {m['t_peak']} | {m['sigma_pre']} | {m['ref_still']} | "
                     f"{m.get('t_back') if m.get('t_back') is not None else 'not back' if r['hold_s'] >= 10 else 'not scored (LAND < 10 s)'} | {m.get('post', '')} |")
        s += ['', f"detection {r.get('detection_s', '-- (announced)')} s; dip {r['dip_cm']} cm at +{r['dip_at_s']} s; landing lean {r.get('landing_lean_deg', {})} deg; "
              f"planned pull {r.get('planned_pull', {})}", '']
        if r['cut_15s']['ref_still'] < 5.0:
            s.append(f"**Flag (Q12):** the ring reference was still only {r['cut_15s']['ref_still']} s before the event (< 5 s).")
            s.append('')
        if mx.get('floor'):
            s += ['Against the rig\'s no-event floor (P6 5a):', '', '| window | Delta-theta | null mean / sd / p95 (n) | above p95 | dtheta / sigma_pre | null ratio median / p95 | above |', '|---|---|---|---|---|---|---|']
            for k, f in mx['floor'].items():
                nl = f['null']
                s.append(f"| {k} | {f['dtheta']} | {nl['mean']} / {nl['sd']} / {nl['p95']} ({nl['n']}) | {f['above_null_p95']} | {f['ratio']} | "
                         f"{f['null_ratio_median_p95'][0]} / {f['null_ratio_median_p95'][1]} | {f['above_ratio_p95']} |" if nl else f"| {k} | {f['dtheta']} | no null for this window | | {f['ratio']} | | |")
            s.append('')
        if mx.get('ledger'):
            lg = mx['ledger']
            s += [f"## Ledger ({lg['cell']}), Delta-theta cut at 15 s = {lg['rig_dtheta_15s']} deg; twin {lg['twin']} (midpoint {lg['twin_midpoint']}); "
                  f"log ratio {lg['log_ratio']}; {'scored' if lg['scored'] else 'NOT scored: the band was not frozen before this flight'}", '',
                  '| predictor | lo | hi | width | hit |', '|---|---|---|---|---|']
            for pr in lg['predictors']:
                s.append(f"| {pr['predictor']} | {pr['lo']} | {pr['hi']} | {pr['width']} | {'hit' if pr['hit'] else 'miss'} |")
            s.append('')
    if mx.get('attach'):
        s += ['## Attach timeline (F15 input)', '', '```'] + [f"{k}: {v}" for k, v in mx['attach'].items() if k != 'bag_first_messages'] + ['```', '']
        if mx['attach'].get('bag_first_messages'):
            s += ['First message per event topic in the bag:', '', '| t (s) | topic | message |', '|---|---|---|']
            s += [f"| {v['t']} | {k} | {v['msg'][:100]} |" for k, v in sorted(mx['attach']['bag_first_messages'].items(), key=lambda kv: kv[1]['t'])]
            s.append('')
    if mx.get('notes'):
        s += ['Notes: ' + '; '.join(mx['notes']), '']
    rows = tex_rows(label, name, date, mx)
    if rows:
        s += ['## Thesis row text (run ids in the comments only)', '', '```latex'] + rows + ['```', '']
    s.append(f"Files: {p['run']} (converted), timeline.csv, " + ', '.join(mx.get('files', [])))
    return '\n'.join(s) + '\n'


def process(label, name, date, out_root, no_bag=False):
    p = flight_paths(date, name)
    ensure_run(p)
    run, evd, man = load_run(p)
    t0 = man['t0_epoch']
    P, prm = planner_log(p)
    tprm = tracker_params(p)
    out = os.path.join(out_root, f'{label}_{name}')
    os.makedirs(out, exist_ok=True)
    mx = dict(label=label, name=name, date=date, conditions=conditions(p, prm, tprm), files=[], notes=[])
    tl = timeline(p, t0)
    tl.to_csv(os.path.join(out, 'timeline.csv'), index=False)
    mx['events'] = {k: round(v, 3) for k, v in evd.items()}
    if label == 'R1':
        mx['hover'] = hover_metrics(run, evd, os.path.join(p['logs'], 'logs', 'tracker'))
    if label in ('R2', 'R3'):
        c = carry_metrics(run, evd)
        if c is None:
            mx['notes'].append('no orbit found in the reference')
        else:
            mx['carry'] = c
            pd.DataFrame(c['laps']).to_csv(os.path.join(out, 'laps.csv'), index=False)
            mx['files'].append('laps.csv')
            if c['n_laps'] < 2:
                mx['notes'].append('one full lap only: no lap-to-lap sd (the sheet asks for two laps)')
            if label == 'R3':
                st = c['plus4_to_land']
                sd = c.get('lap_sd', {}).get('exy_mean_cm')
                r2 = glob.glob(os.path.join(out_root, 'R2_*', 'metrics.json'))
                sd_r2 = None
                if r2:
                    sd_r2 = json.load(open(r2[0])).get('carry', {}).get('lap_sd', {}).get('exy_mean_cm')
                use = sd_r2 if sd_r2 is not None else sd
                tol = max(1.0, use if use is not None else 0.0)
                diff = abs(st['exy_mean_cm'] - K2_CM)
                mx['falsifier'] = dict(exy_mean_cm=st['exy_mean_cm'], lap_sd_cm=use, tol_cm=round(tol, 2), diff_cm=round(diff, 2),
                                       verdict=('within tolerance: the gain came from the rod angle, not the measured pull'
                                                if diff <= tol else 'outside tolerance: the measured pull carries the gain'),
                                       note=('lap sd from R2 (the measured-pull repeat)' if sd_r2 is not None else 'lap sd from this flight (no R2 metrics found)'))
    if label in ('R4', 'R5', 'R9'):
        try:
            rm = release_metrics(p, run, evd, P, prm, t0)
            mx['release'] = rm
            traj = 'hover' if label in ('R4', 'R9') else 'circle'
            mx['floor'] = floor_reading(rm, traj)
            lg = ledger(label, rm, date)
            if lg:
                mx['ledger'] = lg
                pd.DataFrame(lg['predictors']).to_csv(os.path.join(out, 'ledger.csv'), index=False)
                mx['files'].append('ledger.csv')
        except SystemExit as e:
            mx['notes'].append(f'release metrics: {e}')
    if label in ('R6', 'R7', 'R8', 'R9'):
        trace, bag_ev, err = (None, None, 'bag skipped (--no-bag)') if no_bag or not p['has_bag'] else bag_traces(p, t0)
        if err:
            mx['notes'].append(err)
        if trace is not None:
            trace.to_csv(os.path.join(out, 'trace.csv'), index=False)
            mx['files'].append('trace.csv (tip, ring, plate 9, QUAD4 at 50 Hz)')
        if bag_ev is not None and len(bag_ev):
            bag_ev.to_csv(os.path.join(out, 'bag_events.csv'), index=False)
            mx['files'].append('bag_events.csv')
        mx['attach'] = attach_summary(tl, trace, bag_ev)
    json.dump(mx, open(os.path.join(out, 'metrics.json'), 'w'), indent=1, default=str)
    open(os.path.join(out, 'summary.md'), 'w').write(summary_md(label, name, date, mx, p))
    print(f'{label} {name}: -> {out}/summary.md')
    return mx


def rows_md(out_root):
    secs = ['# Thesis rows and figure inputs from the rig flights (tools/rig_night.py)', '']
    for f in sorted(glob.glob(os.path.join(out_root, '*', 'metrics.json'))):
        mx = json.load(open(f))
        rows = tex_rows(mx['label'], mx['name'], mx['date'], mx)
        secs += [f"## {mx['label']} {mx['name']}", '']
        if rows:
            secs += ['```latex'] + rows + ['```']
        if mx.get('notes'):
            secs.append('Notes: ' + '; '.join(mx['notes']))
        secs.append(f"Details: {os.path.relpath(os.path.dirname(f), REPO)}/summary.md")
        secs.append('')
    open(os.path.join(out_root, 'rows.md'), 'w').write('\n'.join(secs) + '\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('flights', nargs='+', help='LABEL:NAME, e.g. R4:r4')
    ap.add_argument('--date', required=True)
    ap.add_argument('--out', default=None, help='default results/analysis/rig_<date>')
    ap.add_argument('--no-bag', action='store_true', help='skip the attach traces from the bag')
    a = ap.parse_args()
    out_root = a.out or os.path.join(REPO, 'results', 'analysis', f'rig_{a.date}')
    os.makedirs(out_root, exist_ok=True)
    for fl in a.flights:
        label, name = fl.split(':', 1)
        if label not in ('R1', 'R2', 'R3', 'R4', 'R5', 'R6', 'R7', 'R8', 'R9'):
            sys.exit(f'{fl}: the label must be R1-R9 (the sheet)')
        process(label, name, a.date, out_root, a.no_bag)
    rows_md(out_root)
    print('->', os.path.join(out_root, 'rows.md'))


if __name__ == '__main__':
    main()
