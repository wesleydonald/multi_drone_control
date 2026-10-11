#!/usr/bin/env python3
"""tools/release_transient.py -- the ring's transient at a release against the planned tension step (desk analysis).

    python3 tools/release_transient.py [--out results/analysis/release_transient] [--results /path/to/results]

For every twin and rig release in RUNS (the 8 Oct pairs; the 7 Oct rig detaches, which are not tabulated in the
thesis since 11 Oct but stay here as the record), from the logs already on disk and nothing else:
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
    the time back in band; the dip (the ring's lowest height against its pre-window mean, event to LAND) keeps its
    own scorer; the survivors' support margin and largest gap after the leaver (metrics.cog_margin);
  * for the record, the scalar reading used before the freeze (the scalar tilt's peak over 5 s above its mean
    over the 3 s before), so that the change of metric is visible per run.
A rig flight without a converted run directory (r0013) is read from its planner log alone, at 10 Hz, with the
same tilt-vector formula. Clocks: the planner log is aligned to the run clock on the ring's lift-off, as
tools/detach_tensions.py does. Rig layouts and leavers are typed here from the registry rows they cite; plates are
given as angles anticlockwise from the ring's +x (the 8 Oct sheets count plates clockwise, so rig plate 9 is 90 deg).
"""
import argparse
import csv
import glob
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
STEP_S = 1.0
PRE_OLD_S, POST_OLD_S = 3.0, 5.0                     # the scalar reading used before the freeze

# run id -> where it is, its layout (deg, anticlockwise) and how the release was commanded.
# rig: 'rig_dir'/'rig_name' locate the T2 planner log; the converted run directory (tools/rig_to_run.py) is read
# through release_bands.Run with the tag 'rig:<date>/<run>'.
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
    # rig, 7 Oct (registry RIG-1007-r0012, r0013: plates 1/3/5/9 and 1/3/6/9 anticlockwise, the leaver on plate 3
    # = 90 deg; round 1: planned pull, rods planned at 45 deg, reference integral). Not tabulated in the thesis.
    dict(run='RIG-1007-r0012', source='rig', kind='announced', path='hover', azimuths=(30, 90, 150, 270),
         leaver_deg=90, rig_dir='results/rig/2026-10-07', rig_name='r0012', tag='rig:2026-10-07/r0012',
         note='7 Oct, not tabulated in the thesis'),
    dict(run='RIG-1007-r0013', source='rig', kind='announced', path='hover', azimuths=(30, 90, 180, 270),
         leaver_deg=90, rig_dir='results/rig/2026-10-07', rig_name='r0013', tag=None,
         note='7 Oct, not tabulated; capsized: a survivor magnet let go'),
    # rig, 8 Oct (registry RIG-1008-r302_3, r303_4, r304_3, r305: plates 3/11/7/9 and 3/11/6/9 clockwise, drone 4
    # leaves from plate 9 = 90 deg; measured pull, rods planned at 60 deg, reference integral)
    dict(run='RIG-1008-r302_3', source='rig', kind='announced', path='hover', azimuths=(30, 90, 150, 270),
         leaver_deg=90, rig_dir='results/rig/2026-10-08', rig_name='r302_3', tag='rig:2026-10-08/r302_3'),
    dict(run='RIG-1008-r303_4', source='rig', kind='unannounced', path='hover', azimuths=(30, 90, 150, 270),
         leaver_deg=90, rig_dir='results/rig/2026-10-08', rig_name='r303_4', tag='rig:2026-10-08/r303_4'),
    dict(run='RIG-1008-r304_3', source='rig', kind='unannounced', path='hover', azimuths=(30, 90, 180, 270),
         leaver_deg=90, rig_dir='results/rig/2026-10-08', rig_name='r304_3', tag='rig:2026-10-08/r304_3'),
    dict(run='RIG-1008-r305', source='rig', kind='unannounced', path='circle', azimuths=(30, 90, 150, 270),
         leaver_deg=90, rig_dir='results/rig/2026-10-08', rig_name='r305', tag='rig:2026-10-08/r305'),
]


def liftoff_time(t, z, rise=0.03):
    t, z = np.asarray(t, float), np.asarray(z, float)
    rest = np.nanmedian(z[:20])
    idx = np.where(z > rest + rise)[0]
    return float(t[idx[0]]) if len(idx) else np.nan


def planner_log(spec, results):
    if spec['source'] == 'twin':
        d = glob.glob(os.path.join(results, '*', f"{spec['run']}_*"))[0]
        import json
        man = json.load(open(os.path.join(d, 'manifest.json')))
        la = man.get('launch_args', {})
        if not isinstance(la, dict):
            la = dict(a.split(':=', 1) for a in la if ':=' in a)
        az = tuple(float(v) for v in str(la['attach_azimuths_deg']).split(','))
        P = pd.read_csv(glob.glob(os.path.join(d, 'logs', '*planner', '*', 'log.csv'))[0])
        return P, az
    base = os.path.join(os.path.dirname(results), spec['rig_dir'])
    P = pd.read_csv(glob.glob(os.path.join(base, f"{spec['rig_name']}_logs", 'logs', 'dissipative_planner', '*',
                                           'log.csv'))[0])
    return P, tuple(spec['azimuths'])


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


def static_steps(az, leaver_deg, elev):
    stat = {r['leaver_deg']: r for r in am.removal_rows('x', az, 0.225, elev)}
    out = {}
    for item in stat[leaver_deg]['survivor_steps'].split():
        a, v = item.split(':')
        out[float(a)] = float(v) * RING_KG * G / np.sin(np.deg2rad(elev))
    return out


def old_scalar_reading(t, tilt, z, t_ev):
    """The reading used before the freeze: the scalar tilt's peak over 5 s above its mean over the 3 s before."""
    pre = (t >= t_ev - PRE_OLD_S) & (t < t_ev)
    post = (t >= t_ev) & (t <= t_ev + POST_OLD_S)
    return (round(float(np.nanmax(tilt[post]) - np.nanmean(tilt[pre])), 2),
            round(float(np.nanmax(tilt[post])), 2),
            round(100 * float(np.nanmean(z[pre]) - np.nanmin(z[post])), 1))


def analyse(spec, results, rb):
    P, az = planner_log(spec, results)
    rs = resize_and_steps(P, az)
    if rs is None:
        return dict(run=spec['run'], note='no resize in the planner log')
    leaver_deg = rs['az_of'][rs['leaver']]
    planned = {dr: rs['t_after'][dr] - rs['t_before'][dr] for dr in rs['survivors']}
    static = static_steps(az, leaver_deg, rs['elev'])
    margin, gap, ok = cog_margin(am.rho_of([rs['az_of'][dr] for dr in rs['survivors']], 0.225))
    out = dict(run=spec['run'], source=spec['source'], kind=spec['kind'], path=spec['path'],
               layout_deg=' '.join(str(int(a)) for a in az), leaver_deg=int(leaver_deg),
               elev_log_deg=round(rs['elev'], 1), gap_after_deg=round(gap, 1), margin_after_mm=round(1000 * margin, 1),
               leaver_t_before_N=round(rs['t_before'][rs['leaver']], 2),
               planned_step_max_N=round(max(planned.values()), 2), planned_step_sum_N=round(sum(planned.values()), 2),
               static_step_max_N=round(max(static.values()), 2), static_step_sum_N=round(sum(static.values()), 2),
               planned_steps=' '.join(f'{int(rs["az_of"][dr])}:{planned[dr]:+.2f}' for dr in rs['survivors']),
               static_steps=' '.join(f'{int(a)}:{v:+.2f}' for a, v in sorted(static.items())),
               note=spec.get('note', ''))
    tag = spec['run'] if spec['source'] == 'twin' else spec.get('tag')
    if tag:
        run = rb.Run(tag)                                 # the signed T6 metrics, one implementation
        m = run.metrics(None)
        out.update(dtheta_deg=round(m['dtheta'], 2), t_peak_s=round(m['t_peak'], 2), abs_peak_deg=round(m['abs_peak'], 2),
                   scalar_rise_deg=round(m['rise'], 2), sigma_pre_deg=round(m['sigma_pre'], 2),
                   ref_still_s=round(m['ref_still'], 1), pre_flag='' if m['steady'] else 'FLAG',
                   t_back_s=(round(m['t_back'], 2) if m['t_back'] is not None else 'not back' if m['post'] is not None else 'n/a'),
                   window_s=round(m['window_s'], 1))
        if spec['source'] == 'twin':                      # Q1: the pair is compared at the rig flight's hold
            rig = rb.Run(spec['pair'])
            hz = rig.tland - rig.te
            out['dtheta_pairwin_deg'] = round(run.metrics(hz)['dtheta'], 2)
            out['pair_window_s'] = round(hz, 1)
        else:
            out['dtheta_pairwin_deg'] = out['dtheta_deg']
            out['pair_window_s'] = out['window_s']
        # the dip and the old scalar reading, on the run clock; detection from the events
        d = rb.locate(tag)
        r = pd.read_csv(os.path.join(d, 'run.csv')).drop_duplicates('t')
        ev = pd.read_csv(os.path.join(d, 'events.csv'))
        t, tilt, z = r.t.to_numpy(float), r.payload_tilt_deg.to_numpy(float), r.payload_z.to_numpy(float)
        t_ev = run.te
        pre5 = (t >= t_ev - 5.0) & (t < t_ev)
        win = (t >= t_ev) & (t <= run.tland)
        out['dip_cm'] = round(100 * float(np.nanmean(z[pre5]) - np.nanmin(z[win])), 1)
        out['old_scalar_rise_deg'], out['old_abs_peak_deg'], out['old_dip_cm'] = old_scalar_reading(t, tilt, z, t_ev)
        rel, detd = ev[ev.event == 'RELEASE'], ev[ev.event == 'DETECTED']
        off = liftoff_time(rs['P'].sim_time, rs['P'].load_z) - liftoff_time(r.t, r.payload_z)
        if spec['kind'] == 'unannounced' and not rel.empty:
            if not detd.empty:
                out['detection_s'] = round(float(detd.sim_time.iloc[0]) - float(rel.sim_time.iloc[0]), 2)
                out['detection_source'] = 'DETECTED event'
            else:
                out['detection_s'] = round(rs['t_res'] - (float(rel.sim_time.iloc[0]) + off), 2)
                out['detection_source'] = 'planner resize, 10 Hz'
        else:
            out['detection_s'], out['detection_source'] = np.nan, ''
        out['resize_after_event_s'] = round(rs['t_res'] - (t_ev + off), 2)
    else:                                                 # r0013: the planner log only, 10 Hz
        P = rs['P']
        q = P[['qw', 'qx', 'qy', 'qz']].to_numpy(float)
        th = rb.tilt_vector(q)
        t = P.sim_time.to_numpy(float)
        t_ev = rs['t_res']
        pre = (t >= t_ev - 5.0) & (t < t_ev)
        mean = th[pre].mean(0)
        sig = float(np.sqrt(np.mean(np.sum((th[pre] - mean) ** 2, 1))))
        win = t >= t_ev
        dev = np.linalg.norm(th[win] - mean, axis=1)
        mag = np.linalg.norm(th, axis=1)
        z = P.load_z.to_numpy(float)
        out.update(dtheta_deg=round(float(dev.max()), 2), t_peak_s=round(float(t[win][np.argmax(dev)] - t_ev), 2),
                   abs_peak_deg=round(float(mag[win].max()), 2),
                   scalar_rise_deg=round(float(mag[win].max() - mag[pre].mean()), 2), sigma_pre_deg=round(sig, 2),
                   ref_still_s=np.nan, pre_flag='planner log', t_back_s='n/a', window_s=round(float(t[-1] - t_ev), 1),
                   dtheta_pairwin_deg=round(float(dev.max()), 2), pair_window_s=round(float(t[-1] - t_ev), 1),
                   dip_cm=round(100 * float(np.nanmean(z[pre]) - np.nanmin(z[win])), 1))
        out['old_scalar_rise_deg'], out['old_abs_peak_deg'], out['old_dip_cm'] = old_scalar_reading(
            t, P.tilt_deg.to_numpy(float), z, t_ev)
        out['detection_s'], out['detection_source'], out['resize_after_event_s'] = np.nan, '', 0.0
    return out


def plot(rows, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows = [r for r in rows if 'dtheta_deg' in r and 'not tabulated' not in r.get('note', '')]
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
    axes[0].set_title('filled: rig; open: twin; square: unannounced', fontsize=8)
    for ax in axes:
        ax.grid(True, lw=0.3, alpha=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'release_transient.png'), dpi=150)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(REPO, 'results', 'analysis', 'release_transient'))
    ap.add_argument('--results', default='/home/wesley/multi_drone_control/results',
                    help='the results root with the run directories (read only)')
    ap.add_argument('--no-plot', action='store_true')
    a = ap.parse_args(argv)
    os.environ['MDC_RESULTS_ROOT'] = a.results            # release_bands.locate reads it at import
    import release_bands as rb
    os.makedirs(a.out, exist_ok=True)
    rows = []
    for spec in RUNS:
        try:
            rows.append(analyse(spec, a.results, rb))
        except Exception as e:                       # one bad run must not hide the rest
            rows.append(dict(run=spec['run'], note=f'failed: {type(e).__name__}: {e}'))
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(os.path.join(a.out, 'table.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    show = ['run', 'kind', 'path', 'layout_deg', 'gap_after_deg', 'margin_after_mm', 'leaver_t_before_N',
            'planned_step_max_N', 'static_step_max_N', 'detection_s', 'dtheta_deg', 'dtheta_pairwin_deg',
            'pair_window_s', 'abs_peak_deg', 'scalar_rise_deg', 'sigma_pre_deg', 'ref_still_s', 'pre_flag',
            't_back_s', 'dip_cm', 'old_scalar_rise_deg', 'note']
    with open(os.path.join(a.out, 'table.md'), 'w') as f:
        f.write('T6 release metrics (docs/freeze_2026-10-14.md 1.1, via tools/release_bands.py) beside the planned '
                'tension step. dtheta_deg: to LAND; dtheta_pairwin_deg: twin rows cut at the paired rig flight\'s '
                'event-to-LAND (Q1). old_scalar_rise_deg: the pre-freeze reading (scalar peak over 5 s above the 3 s '
                'pre-event mean), for the record.\n\n')
        f.write('| ' + ' | '.join(show) + ' |\n|' + '---|' * len(show) + '\n')
        for r in rows:
            f.write('| ' + ' | '.join(str(r.get(k, '')) for k in show) + ' |\n')
    print(open(os.path.join(a.out, 'table.md')).read())
    if not a.no_plot:
        plot(rows, a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
