#!/usr/bin/env python3
"""tools/release_transient.py -- every release in the record: the ring's transient against the planned tension step.

    python3 tools/release_transient.py [--out results/analysis/release_transient] [--results /path/to/results]
                                       [--rig-runs DIR] [--convert]

For every twin and rig release in RUNS (the 8 Oct twin pairs; the rig's commanded releases of 7 and 8 Oct and the
magnets that let go with no command: r0013's two survivors and r200006's two), from the logs on disk:
  * the planner's planned tensions per drone before the resize and in its first second after it (log.csv t0..t3,
    per slot, mapped to drones through slot2drone), so each survivor's planned step in newtons;
  * the static prediction of that step from the layout alone (tools/admissibility_map: the leaver's level-hover
    share handed to the survivors), to show that the resize is the static redistribution;
  * the detection delay: on the rig the bag's DETECTED event against RELEASE; in the twin the planner's first
    smaller problem against RELEASE at the log's 10 Hz (an announced DETACH has none); and the time from the
    event to the resize in both;
  * the ring's transient in the T6 form signed on 11 Oct (docs/freeze_2026-10-14.md section 1.1), computed by
    tools/release_bands.py's Run class so that this table and the ledger bands use one implementation: the tilt
    vector, a 5 s pre-window, the tilt rise dtheta to LAND and, for the twin rows, also cut at the paired rig
    flight's event-to-LAND (Q1), the absolute peak, the scalar rise, sigma_pre, the reference-still flag (Q12) and
    the time back in band (scored only when LAND comes 10 s or more after the event; t_back_note says why not);
  * beside the freeze, with their own scorers: the dip (the ring's lowest height against its pre-window mean,
    event to the window end; drop_below_ref_cm, its largest drop below its reference over the same window, is the
    other reading in use); the steady offset (the ring against its reference, xy and z, over the pre-window and
    over the last 2 s of the window); the landing lean (the largest survivor tilt 1.2 s after the ring touches
    down, ring z within 5 cm of its rest: the 7 Oct cable_mocap card's reading); the survivors' support margin and
    largest gap after the leaver (metrics.cog_margin);
  * t_back_2s_s, NOT the frozen metric: the same band rule with the post-event tilt taken over the last 2 s
    before the window end, scored when that end is 4 s or more after the event, because only one rig release
    (r305) held 10 s before LAND;
  * for the record, the scalar reading used before the freeze (the scalar tilt's peak over 5 s above its mean
    over the 3 s before), so that the change of metric is visible per run.

Rig rows are read from tools/rig_to_run.py run directories made with the bag (a 50 Hz grid): <results>/rig/<date>/
<run>_run when it exists, else --rig-runs, where --convert makes them (rosbag2_py: source the ROS workspace) and
adds a SLIP row to events.csv for every rod whose pivot-to-plate distance left its length with no magnet OFF
(tools/drop_trigger_replay.py's separation onset at the bag's full rate, the slip detector).

Two cases the freeze does not time are labelled in event_rule and window_end, not silently substituted:
  * an uncommanded release (r200006) has no magnet OFF and no DETACH: t_e is its SLIP onset;
  * a flight that capsized has no LAND after its event (r0013; r200006's second slip came after LAND, in the
    descent): the window ends at the fleet abort.
Clocks: rig rows use the run directory's t0_epoch (every rig node logs epoch time); twin rows align the planner
log on the ring's lift-off, as tools/detach_tensions.py does. Rig layouts and leavers are typed here from the
registry rows and the flights' params.json; plates are given as angles anticlockwise from the ring's +x (the 8 Oct
sheets count plates clockwise, so rig plate 9 is 90 deg).
"""
import argparse
import csv
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from metrics import cog_margin                       # noqa: E402
import admissibility_map as am                       # noqa: E402

G = 9.81
RING_KG = 0.86
RADIUS = 0.225
STEP_S = 1.0
PRE_OLD_S, POST_OLD_S = 3.0, 5.0                     # the scalar reading used before the freeze
POST2_S, POST2_MIN_S = 2.0, 4.0                      # t_back_2s: post tilt over the last 2 s, end >= event + 4 s
LEAN_AFTER_S = 1.2                                   # landing lean: survivor tilt this long after touchdown
TOUCH_DZ = 0.05                                      # touchdown: ring within this of its rest height

# run id -> where it is, its layout (deg, anticlockwise) and how the release came. gone_deg: already off the ring.
RUNS = [
    # twin, 8 Oct pairs (registry R1140/R1145, R1206/R1211, R1207/R1212, R1208/R1213; layouts from manifest.json)
    dict(run='R1207', source='twin', kind='announced', path='hover', pair='rig:2026-10-08/r302_3'),
    dict(run='R1212', source='twin', kind='announced', path='hover', pair='rig:2026-10-08/r302_3'),
    dict(run='R1140', source='twin', kind='unannounced', path='hover', pair='rig:2026-10-08/r303_4'),
    dict(run='R1145', source='twin', kind='unannounced', path='hover', pair='rig:2026-10-08/r303_4'),
    dict(run='R1206', source='twin', kind='unannounced', path='hover', pair='rig:2026-10-08/r304_3'),
    dict(run='R1211', source='twin', kind='unannounced', path='hover', pair='rig:2026-10-08/r304_3'),
    dict(run='R1208', source='twin', kind='unannounced', path='circle', pair='rig:2026-10-08/r305'),
    dict(run='R1213', source='twin', kind='unannounced', path='circle', pair='rig:2026-10-08/r305'),
    # rig, 7 Oct round 1 (registry RIG-1007-r0012, r0013: plates 1/3/5/9 and 1/3/6/9 anticlockwise, drone_3 leaves
    # from 90 deg; planned pull, rods planned at 45 deg, reference integral)
    dict(run='RIG-1007-r0012', source='rig', kind='announced', path='hover', azimuths=(30, 90, 150, 270),
         leaver_deg=90, date='2026-10-07', name='r0012', pull='planned', plan_deg=45, integral='reference'),
    dict(run='RIG-1007-r0013', source='rig', kind='announced', path='hover', azimuths=(30, 90, 180, 270),
         leaver_deg=90, date='2026-10-07', name='r0013', pull='planned', plan_deg=45, integral='reference',
         note='capsized: survivors /drone_1 and /drone_2 let go with no command (slips_after_event_s)'),
    # rig, 7 Oct round 2 (registry RIG-1007-r200006: even ring 0/90/180/270, mode mpc, no resize; measured pull,
    # rods planned at 45 deg, model integral). Two magnets let go with no command: /drone_1 (270 deg) in the
    # circle, /drone_2 (180 deg) 2.8 s later in the LAND descent
    dict(run='RIG-1007-r200006 slip 1', source='rig', kind='uncommanded', path='circle',
         azimuths=(0, 90, 180, 270), leaver_deg=270, date='2026-10-07', name='r200006', slip=1, pull='measured',
         plan_deg=45, integral='model', note='recovered; operator LAND 0.59 s after the slip'),
    dict(run='RIG-1007-r200006 slip 2', source='rig', kind='uncommanded', path='LAND descent',
         azimuths=(0, 90, 180, 270), gone_deg=(270,), leaver_deg=180, date='2026-10-07', name='r200006', slip=2,
         pull='measured', plan_deg=45, integral='model', note='capsized; the pre-window holds slip 1'),
    # rig, 8 Oct (registry RIG-1008-r302_3, r303_4, r304_3, r305: plates 3/11/7/9 and 3/11/6/9 clockwise, drone 4
    # leaves from plate 9 = 90 deg; measured pull, rods planned at 60 deg, reference integral)
    dict(run='RIG-1008-r302_3', source='rig', kind='announced', path='hover', azimuths=(30, 90, 150, 270),
         leaver_deg=90, date='2026-10-08', name='r302_3', pull='measured', plan_deg=60, integral='reference'),
    dict(run='RIG-1008-r303_4', source='rig', kind='unannounced', path='hover', azimuths=(30, 90, 150, 270),
         leaver_deg=90, date='2026-10-08', name='r303_4', pull='measured', plan_deg=60, integral='reference'),
    dict(run='RIG-1008-r304_3', source='rig', kind='unannounced', path='hover', azimuths=(30, 90, 180, 270),
         leaver_deg=90, date='2026-10-08', name='r304_3', pull='measured', plan_deg=60, integral='reference'),
    dict(run='RIG-1008-r305', source='rig', kind='unannounced', path='circle', azimuths=(30, 90, 150, 270),
         leaver_deg=90, date='2026-10-08', name='r305', pull='measured', plan_deg=60, integral='reference'),
]


def liftoff_time(t, z, rise=0.03):
    t, z = np.asarray(t, float), np.asarray(z, float)
    rest = np.nanmedian(z[:20])
    idx = np.where(z > rest + rise)[0]
    return float(t[idx[0]]) if len(idx) else np.nan


# ── where a run lives ─────────────────────────────────────────────────────────────
def rig_run_dir(spec, results, rig_runs):
    """The converted run directory of a rig flight: the results root's own, else --rig-runs."""
    for root in (os.path.join(results, 'rig'), rig_runs):
        d = os.path.join(root, spec['date'], spec['name'] + '_run')
        if os.path.isfile(os.path.join(d, 'logs', 'run.csv')):
            return d
    return None


def convert(spec, results, rig_runs):
    """tools/rig_to_run.py with the bag into --rig-runs, then a SLIP row per uncommanded rod separation."""
    import rig_to_run
    base = os.path.join(results, 'rig', spec['date'], spec['name'])
    out = os.path.join(rig_runs, spec['date'], spec['name'] + '_run')
    trackers = sorted(glob.glob(base + '_logs/logs/tracker/drone*_*'))
    rig_to_run.convert(trackers, out, bag=base, t2_log=base + '.log')
    t0 = json.load(open(os.path.join(out, 'manifest.json')))['t0_epoch']
    f = os.path.join(out, 'logs', 'events.csv')
    ev = pd.read_csv(f)
    add = pd.DataFrame([(t - t0, 'SLIP', f'{who} az {az:+.1f}') for t, who, az in slips(base, results)],
                       columns=['sim_time', 'event', 'arg'])
    pd.concat([ev, add]).sort_values('sim_time').to_csv(f, index=False)
    return out


def slips(bagdir, results):
    """[(epoch onset, 'drone N (/drone_i)', ring-frame azimuth)] of every rod that left its plate with no magnet OFF:
    tools/drop_trigger_replay.py's analyse() (sustained 3 cm over the rod, onset 3 mm over its baseline)."""
    import drop_trigger_replay as dtr
    dtr.RIG = os.path.join(results, 'rig')
    fl = dict(date=os.path.basename(os.path.dirname(bagdir)), run=os.path.basename(bagdir),
              data=dtr.load_bag(bagdir, None), ticks=dtr.tracker_ticks(bagdir + '_logs'))
    fl = dtr.analyse(fl)
    out = []
    for e in fl.get('events', []):
        for r in e['rel']:
            if r['kind'] != 'magnet OFF':
                i = int(r['who'].split('_')[1])
                az = next(v['az'] for v in fl['pdist'].values() if v['name'] == r['who'])
                out.append((float(r['t']), f'drone {i + 1} (/drone_{i})', float(az)))
    return out


def shown_path(d, results):
    for root in (os.path.dirname(results), REPO):
        if os.path.abspath(d).startswith(os.path.abspath(root) + os.sep):
            return os.path.relpath(d, root)
    return d


# ── planner log: the resize and the planned step ─────────────────────────────────
def planner_log(spec, results):
    if spec['source'] == 'twin':
        d = glob.glob(os.path.join(results, '*', f"{spec['run']}_*"))[0]
        man = json.load(open(os.path.join(d, 'manifest.json')))
        la = man.get('launch_args', {})
        if not isinstance(la, dict):
            la = dict(a.split(':=', 1) for a in la if ':=' in a)
        az = tuple(float(v) for v in str(la['attach_azimuths_deg']).split(','))
        P = pd.read_csv(glob.glob(os.path.join(d, 'logs', '*planner', '*', 'log.csv'))[0])
        return P, az
    base = os.path.join(results, 'rig', spec['date'], spec['name'] + '_logs', 'logs')
    f = sorted(glob.glob(os.path.join(base, 'dissipative_planner', '*', 'log.csv')) +
               glob.glob(os.path.join(base, 'mpc_planner', '*', 'log.csv')), key=os.path.getsize)
    return (pd.read_csv(f[-1]), tuple(spec['azimuths'])) if f else (None, tuple(spec['azimuths']))


def resize_and_steps(P, az):
    """The planner's resize: its time on the planner clock, the leaver, the planned tensions before and after."""
    P = P.drop_duplicates('sim_time').reset_index(drop=True)
    n0 = int(P.n.max())
    after = P[P.n < n0]
    if after.empty:
        return None
    i_res = int(after.index[0])
    t_res = float(P.sim_time.iloc[i_res])
    pre = P[(P.sim_time >= t_res - 2.0) & (P.sim_time < t_res - 0.2)]
    post = P[(P.sim_time >= t_res) & (P.sim_time < t_res + STEP_S)]
    slots_before = [int(v) for v in str(pre.slot2drone.iloc[-1]).split()]
    slots_after = [int(v) for v in str(post.slot2drone.iloc[0]).split()]
    leaver = [dr for dr in slots_before if dr not in slots_after][0]
    t_before = {dr: float(np.nanmean(pre[f't{k}'].astype(float))) for k, dr in enumerate(slots_before)}
    t_after = {dr: float(np.nanmean(post[f't{k}'].astype(float))) for k, dr in enumerate(slots_after)}
    elev = float(np.nanmean(pre[[f'elev{k}' for k in range(n0)]].astype(float).to_numpy()))
    az_of = {dr: az[k] for k, dr in enumerate(slots_before)}
    survivors = [dr for dr in slots_before if dr != leaver]
    return dict(P=P, t_res=t_res, leaver=leaver, az_of=az_of, survivors=survivors, t_before=t_before,
                t_after=t_after, elev=elev if np.isfinite(elev) else 60.0)


def pre_event_pull(P, az, t_ev, leaver_deg):
    """No resize (an uncommanded release): the leaver's planned tension and the mean rod elevation over the 2 s
    before the event, on the planner clock (slots in layout order)."""
    P = P.drop_duplicates('sim_time')
    pre = P[(P.sim_time >= t_ev - 2.0) & (P.sim_time < t_ev - 0.2) & (P.n == P.n.max())]
    if pre.empty:
        return None, None
    n = int(pre.n.iloc[-1])
    k = [i for i in range(n) if abs(((az[i] - leaver_deg + 180) % 360) - 180) < 1]
    elev = float(np.nanmean(pre[[f'elev{i}' for i in range(n)]].astype(float).to_numpy()))
    return (float(np.nanmean(pre[f't{k[0]}'].astype(float))) if k else None), elev


def static_steps(az, leaver_deg, elev):
    stat = {r['leaver_deg']: r for r in am.removal_rows('x', az, RADIUS, elev)}
    out = {}
    for item in stat[leaver_deg]['survivor_steps'].split():
        a, v = item.split(':')
        out[float(a)] = float(v) * RING_KG * G / np.sin(np.deg2rad(elev))
    return out


# ── metrics beside the freeze ─────────────────────────────────────────────────────
def old_scalar_reading(t, tilt, z, t_ev):
    """The reading used before the freeze: the scalar tilt's peak over 5 s above its mean over the 3 s before."""
    pre = (t >= t_ev - PRE_OLD_S) & (t < t_ev)
    post = (t >= t_ev) & (t <= t_ev + POST_OLD_S)
    return (round(float(np.nanmax(tilt[post]) - np.nanmean(tilt[pre])), 2),
            round(float(np.nanmax(tilt[post])), 2),
            round(100 * float(np.nanmean(z[pre]) - np.nanmin(z[post])), 1))


def t_back_2s(run, sig, t_peak):
    """NOT the frozen rule: the band about the mean tilt over the last 2 s before the window end."""
    if run.tland - run.te < POST2_MIN_S:
        return None
    post = run.th[run.win(run.tland - POST2_S, run.tland)].mean(0)
    inb = np.linalg.norm(run.th - post, axis=1) <= max(2 * sig, 0.5)
    for i in np.nonzero((run.t >= run.te + t_peak) & (run.t + 1.0 <= run.tland))[0]:
        if inb[run.win(run.t[i], run.t[i] + 1.0)].all():
            return float(run.t[i] - run.te)
    return 'not back'


def offsets(r, a, b):
    """The ring against its reference over [a, b): (xy cm, z cm), means."""
    w = ((r.t >= a) & (r.t < b)).to_numpy()
    if w.sum() < 5:
        return '', ''
    dx = r.payload_x[w] - r.payload_ref_x[w]
    dy = r.payload_y[w] - r.payload_ref_y[w]
    dz = r.payload_z[w] - r.payload_ref_z[w]
    return round(100 * float(np.hypot(np.nanmean(dx), np.nanmean(dy))), 1), round(100 * float(np.nanmean(dz)), 1)


def landing_lean(r, ev, t_land, survivors):
    """Largest survivor tilt 1.2 s after the ring touches down (ring z within 5 cm of its rest), per drone."""
    z = r.payload_z.to_numpy(float)
    t = r.t.to_numpy(float)
    rest = float(np.nanmedian(z[:50]))
    down = np.nonzero((t >= t_land) & (z <= rest + TOUCH_DZ))[0]
    t_down = float(t[down[0]]) if len(down) else np.inf
    if ((ev.event.isin(['FLEET_ABORT', 'ESTOP'])) & (ev.sim_time >= t_land) & (ev.sim_time <= t_down)).any():
        return '', 'fleet abort before the touchdown'
    if not len(down):
        return '', 'the ring never touched down'
    k = int(np.argmin(np.abs(t - (t_down + LEAN_AFTER_S))))
    tilts = {d: float(r[f'd{d}_tilt_deg'].iloc[k]) for d in survivors if f'd{d}_tilt_deg' in r}
    tilts = {d: v for d, v in tilts.items() if np.isfinite(v)}
    if not tilts:
        return '', 'no survivor pose'
    return round(max(tilts.values()), 1), ' '.join(f'd{d}:{v:.1f}' for d, v in sorted(tilts.items()))


# ── one row ───────────────────────────────────────────────────────────────────────
def rig_event(spec, ev):
    """(t_e, window end, event rule, window end label) of a rig row, from its events.csv."""
    def first(name):
        hit = ev[ev.event == name].sim_time
        return float(hit.iloc[0]) if len(hit) else None
    if spec['kind'] == 'uncommanded':
        s = ev[ev.event == 'SLIP'].sim_time.to_numpy(float)
        te, rule = float(s[spec['slip'] - 1]), 'SLIP onset in the bag (not a frozen event time)'
    elif spec['kind'] == 'unannounced':
        te, rule = first('RELEASE'), 'RELEASE: first magnet OFF in the bag'
    else:
        te, rule = first('DETACH'), 'DETACH: the planner detach line'
    land = ev[(ev.event == 'LAND') & (ev.sim_time > te)].sim_time
    if len(land):
        return te, float(land.iloc[0]), rule, 'LAND'
    abort = ev[(ev.event == 'FLEET_ABORT') & (ev.sim_time > te)].sim_time
    return te, float(abort.iloc[0]), rule, 'FLEET_ABORT (no LAND after the event)'


def analyse(spec, results, rig_runs, rb):
    out = dict(run=spec['run'], source=spec['source'], kind=spec['kind'], path=spec['path'],
               pull=spec.get('pull', 'measured'), plan_deg=spec.get('plan_deg', 60),
               integral=spec.get('integral', 'reference'))
    if spec['source'] == 'twin':
        d = os.path.dirname(rb.locate(spec['run']))
        run = rb.Run(spec['run'])
        rule, end_label, off = f'runner {"RELEASE" if run.kind == "unannounced" else "DETACH"}', 'LAND', None
    else:
        d = rig_run_dir(spec, results, rig_runs)
        if d is None:
            raise FileNotFoundError(f"no run directory for {spec['date']}/{spec['name']}: --convert (ROS sourced)")
        te, tend, rule, end_label = rig_event(spec, pd.read_csv(os.path.join(d, 'logs', 'events.csv')))
        run = rb.Run(d, te=te, kind=spec['kind'], tland=tend)
        off = json.load(open(os.path.join(d, 'manifest.json')))['t0_epoch']
    r = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    ev = pd.read_csv(os.path.join(d, 'logs', 'events.csv'))
    t, z = r.t.to_numpy(float), r.payload_z.to_numpy(float)
    t_ev, t_end = run.te, run.tland
    out.update(event_rule=rule, window_end=end_label, run_dir=shown_path(d, results))

    # layout, leaver and the planned step
    P, az = planner_log(spec, results)
    rs = resize_and_steps(P, az) if P is not None and spec['kind'] != 'uncommanded' else None
    if off is None:
        off = liftoff_time(P.sim_time, P.load_z) - liftoff_time(r.t, r.payload_z)
    gone = tuple(spec.get('gone_deg', ()))
    if rs is not None:
        leaver_deg = rs['az_of'][rs['leaver']]
        surv_deg = [rs['az_of'][dr] for dr in rs['survivors']]
        planned = {dr: rs['t_after'][dr] - rs['t_before'][dr] for dr in rs['survivors']}
        elev = rs['elev']
        out.update(leaver_t_before_N=round(rs['t_before'][rs['leaver']], 2),
                   planned_step_max_N=round(max(planned.values()), 2),
                   planned_step_sum_N=round(sum(planned.values()), 2),
                   planned_steps=' '.join(f'{int(rs["az_of"][dr])}:{planned[dr]:+.2f}' for dr in rs['survivors']),
                   resize_after_event_s=round(rs['t_res'] - (t_ev + off), 2))
        survivors = list(rs['survivors'])
    else:
        leaver_deg = spec['leaver_deg']
        surv_deg = [a for a in az if a != leaver_deg and a not in gone]
        lt, elev = pre_event_pull(P, az, t_ev + off, leaver_deg) if P is not None else (None, None)
        out.update(leaver_t_before_N=round(lt, 2) if lt is not None else '', planned_step_max_N='',
                   planned_step_sum_N='', planned_steps='no resize (mode mpc)', resize_after_event_s='')
        off_ring = [int(str(a).split('/drone_')[1].split(')')[0]) for a in ev[ev.event == 'SLIP'].arg]
        survivors = [k for k in range(4) if f'd{k}_tilt_deg' in r and k not in off_ring[:spec.get('slip', 0)]]
    margin, gap, ok = cog_margin(am.rho_of(surv_deg, RADIUS))
    out.update(layout_deg=' '.join(str(int(a)) for a in az if a not in gone), leaver_deg=int(leaver_deg),
               elev_log_deg=round(elev, 1) if elev else '', gap_after_deg=round(gap, 1),
               margin_after_mm=round(1000 * margin, 1))
    if len(surv_deg) >= 3 and elev:
        static = static_steps([a for a in az if a not in gone], leaver_deg, elev)
        out.update(static_step_max_N=round(max(static.values()), 2), static_step_sum_N=round(sum(static.values()), 2),
                   static_steps=' '.join(f'{int(a)}:{v:+.2f}' for a, v in sorted(static.items())))

    # T6, one implementation (tools/release_bands.py)
    m = run.metrics(None)
    out.update(t_event_s=round(t_ev, 3), window_s=round(m['window_s'], 2), dtheta_deg=round(m['dtheta'], 2),
               t_peak_s=round(m['t_peak'], 2), abs_peak_deg=round(m['abs_peak'], 2),
               scalar_rise_deg=round(m['rise'], 2),
               sigma_pre_deg=round(m['sigma_pre'], 2), n_pre=m['n_pre'], ref_still_s=round(m['ref_still'], 1),
               pre_flag='' if m['steady'] else 'FLAG')
    end_word = end_label.split()[0]
    if m['post'] is not None:
        out['t_back_s'] = round(m['t_back'], 2) if m['t_back'] is not None else 'not back'
        out['t_back_note'] = f"band {m['band_deg']:.2f} deg about the mean over the last 5 s before {end_word}"
    else:
        out['t_back_s'] = 'not scored'
        out['t_back_note'] = (f'LAND {t_end - t_ev:.1f} s after the event (< 10 s)' if end_label == 'LAND'
                              else f'no LAND after the event ({end_word} {t_end - t_ev:.2f} s after it)')
    tb2 = t_back_2s(run, m['sigma_pre'], m['t_peak'])
    out['t_back_2s_s'] = '' if tb2 is None else (round(tb2, 2) if isinstance(tb2, float) else tb2)
    if spec['source'] == 'twin':                      # Q1: the pair is compared at the rig flight's hold
        rig = rb.Run(spec['pair'])
        hz = rig.tland - rig.te
        out['dtheta_pairwin_deg'] = round(run.metrics(hz)['dtheta'], 2)
        out['pair_window_s'] = round(hz, 1)
    else:
        out['dtheta_pairwin_deg'], out['pair_window_s'] = out['dtheta_deg'], out['window_s']

    # beside the freeze: dip, offsets, landing lean, detection, the old scalar reading
    pre5 = (t >= t_ev - 5.0) & (t < t_ev)
    win = (t >= t_ev) & (t <= t_end)
    out['dip_cm'] = round(100 * float(np.nanmean(z[pre5]) - np.nanmin(z[win])), 1)
    out['drop_below_ref_cm'] = round(-100 * float(np.nanmin(z[win] - r.payload_ref_z.to_numpy(float)[win])), 1)
    out['offset_pre_xy_cm'], out['offset_pre_z_cm'] = offsets(r, t_ev - 5.0, t_ev)
    out['offset_post_xy_cm'], out['offset_post_z_cm'] = (offsets(r, t_end - POST2_S, t_end)
                                                         if t_end - t_ev >= POST2_MIN_S else ('', ''))
    if end_label == 'LAND':
        out['landing_lean_deg'], out['landing_lean_per_drone'] = landing_lean(r, ev, t_end, survivors)
    else:
        out['landing_lean_deg'], out['landing_lean_per_drone'] = '', 'no landing (fleet abort)'
    tilt = r.payload_tilt_deg.to_numpy(float)
    if t_end - t_ev >= POST_OLD_S:                    # the old reading needs its 5 s inside the flight
        out['old_scalar_rise_deg'], out['old_abs_peak_deg'], out['old_dip_cm'] = old_scalar_reading(t, tilt, z, t_ev)
    else:
        out['old_scalar_rise_deg'] = out['old_abs_peak_deg'] = out['old_dip_cm'] = ''
    rel, detd = ev[ev.event == 'RELEASE'], ev[ev.event == 'DETECTED']
    out['detection_s'], out['detection_source'] = '', ''
    if spec['kind'] == 'unannounced' and not rel.empty:
        if not detd.empty:
            out['detection_s'] = round(float(detd.sim_time.iloc[0]) - float(rel.sim_time.iloc[0]), 2)
            out['detection_source'] = 'DETECTED event'
        elif rs is not None:
            out['detection_s'] = round(rs['t_res'] - (float(rel.sim_time.iloc[0]) + off), 2)
            out['detection_source'] = 'planner resize, 10 Hz'
    elif spec['kind'] == 'uncommanded':
        out['detection_source'] = 'no detector in the flight (mode mpc)'
    slip_rows = ev[ev.event == 'SLIP']
    out['slips_after_event_s'] = ' '.join(f'{str(a).split()[2].strip("()")} {s - t_ev:+.3f}' for s, a in
                                          zip(slip_rows.sim_time, slip_rows.arg))
    out['note'] = spec.get('note', '')
    return out


def plot(rows, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows = [r for r in rows if r.get('planned_step_max_N') not in ('', None) and r.get('window_end') == 'LAND']
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.6))
    mk = {'announced': 'o', 'unannounced': 's'}
    col = {'twin': '0.55', 'rig': 'C1'}
    for r in rows:
        kw = dict(marker=mk[r['kind']], color=col[r['source']], ms=6,
                  mfc=col[r['source']] if r['source'] == 'rig' else 'none', ls='none')
        axes[0].plot(r['planned_step_max_N'], r['dtheta_pairwin_deg'], **kw)
        axes[1].plot(r['margin_after_mm'], r['dtheta_pairwin_deg'], **kw)
        axes[2].plot(r['static_step_max_N'], r['planned_step_max_N'], **kw)
    axes[0].set_xlabel('largest planned tension step at the resize (N)')
    axes[0].set_ylabel('tilt rise $\\Delta\\theta$, T6, at the pair\'s rig hold (deg)')
    axes[1].set_xlabel('survivors\' support margin (mm)')
    axes[1].set_ylabel('tilt rise $\\Delta\\theta$ (deg)')
    lim = max(max(r['static_step_max_N'], r['planned_step_max_N']) for r in rows) * 1.1
    axes[2].plot([0, lim], [0, lim], '-', color='0.7', lw=0.8)
    axes[2].set_xlabel('static prediction of the largest step (N)')
    axes[2].set_ylabel('planner\'s largest step (N)')
    axes[0].set_title('filled: rig; open: twin; square: unannounced (rows that reached LAND)', fontsize=8)
    for ax in axes:
        ax.grid(True, lw=0.3, alpha=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'release_transient.png'), dpi=150)


SHOW = ['run', 'kind', 'path', 'pull', 'plan_deg', 'layout_deg', 'leaver_deg', 'gap_after_deg', 'margin_after_mm',
        'leaver_t_before_N', 'planned_step_max_N', 'static_step_max_N', 'detection_s', 'window_end', 'window_s',
        'dtheta_deg', 'dtheta_pairwin_deg', 'pair_window_s', 't_peak_s', 'abs_peak_deg', 'scalar_rise_deg',
        'sigma_pre_deg', 'ref_still_s', 'pre_flag', 't_back_s', 't_back_note', 't_back_2s_s', 'dip_cm',
        'drop_below_ref_cm',
        'offset_pre_xy_cm', 'offset_pre_z_cm', 'offset_post_xy_cm', 'offset_post_z_cm', 'landing_lean_deg',
        'slips_after_event_s', 'old_scalar_rise_deg', 'note']


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(REPO, 'results', 'analysis', 'release_transient'))
    ap.add_argument('--results', default='/home/wesley/multi_drone_control/results',
                    help='the results root with the run directories (read only)')
    ap.add_argument('--rig-runs', default=os.path.join(REPO, 'results', 'rig_runs'),
                    help='converted rig flights that the results root has no run directory for')
    ap.add_argument('--convert', action='store_true', help='make the missing rig run directories (ROS sourced)')
    ap.add_argument('--no-plot', action='store_true')
    a = ap.parse_args(argv)
    os.environ['MDC_RESULTS_ROOT'] = a.results            # release_bands.locate reads it at import
    import release_bands as rb
    os.makedirs(a.out, exist_ok=True)
    if a.convert:
        done = set()
        for spec in RUNS:
            key = (spec.get('date'), spec.get('name'))
            if spec['source'] == 'rig' and key not in done and rig_run_dir(spec, a.results, a.rig_runs) is None:
                print('converted', convert(spec, a.results, a.rig_runs))
                done.add(key)
    rows = []
    for spec in RUNS:
        try:
            rows.append(analyse(spec, a.results, a.rig_runs, rb))
        except Exception as e:                       # one bad run must not hide the rest
            rows.append(dict(run=spec['run'], note=f'failed: {type(e).__name__}: {e}'))
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(os.path.join(a.out, 'table.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(a.out, 'table.md'), 'w') as f:
        f.write('T6 release metrics (docs/freeze_2026-10-14.md 1.1, via tools/release_bands.py) beside the planned '
                'tension step, by tools/release_transient.py. dtheta_deg: event to the window end (LAND, or the '
                'fleet abort where the flight had no LAND after the event); dtheta_pairwin_deg: twin rows cut at the '
                'paired rig flight\'s event-to-LAND (Q1). t_back_2s_s is not the frozen metric (see the script). '
                'old_scalar_rise_deg: the pre-freeze reading, for the record.\n\n')
        f.write('| ' + ' | '.join(SHOW) + ' |\n|' + '---|' * len(SHOW) + '\n')
        for r in rows:
            f.write('| ' + ' | '.join(str(r.get(k, '')) for k in SHOW) + ' |\n')
    print(open(os.path.join(a.out, 'table.md')).read())
    if not a.no_plot:
        plot(rows, a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
