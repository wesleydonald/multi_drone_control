#!/usr/bin/env python3
"""The 1.15 kg ring's 5.3 Hz bending and the filters on the trackers' measured cable pull (real world, 29 Oct).

Message: the later ring bends at about 5.3 Hz and the trackers' measured pull feeds it: filtering only what the trackers
feed back shrinks the bending in the raw measured pull (one 2 Hz low-pass stage, then two). With three carriers the two
stages' lag let the ring slide sideways at 1.1 Hz; the 5.3 Hz band-stop flew without the bending and without the slide.
Hero: (a) the amplitude spectrum of the measured pull (vertical, raw: logged before any filter), mean over the
carriers; (b) the amplitude spectrum of the ring's sideways motion (motion capture). One colour per filter, solid four
carriers, dashed three, one line weight; the band-stop's centre dotted grey (dashed means three carriers).
Left out: the analysis bands (4-6.5 and 0.8-1.5 Hz: in index.md, critic), the pull below 2 Hz and the sideways motion below 0.5 Hz (the lift's own motion and drift, not the two
oscillations), x and y apart, time series, the other 29 Oct flights.
Window, one rule for every flight: from the start of the lift ramp ('pretension done') to its first event (release,
unload, drop or LAND), at most WINDOW_S. The registry's own windows differ per flight; their numbers are reproduced
in the printout ('registry windows').
Data: results/rig/2026-10-29/<flight>: the launch log (events), the tracker logs (acm_z; params.json for the filter),
the bag's /payload/motion_capture_state (decoded with rclpy: source ~/ros2_humble/install/setup.bash and
install/setup.bash first), the planner log (the registry's sideways numbers were read from its load_x/load_y).
"""
import glob
import json
import os
import re
import sys

import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt, welch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from review30_style import S, TEXT  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.ticker import FuncFormatter  # noqa: E402

DAY = os.path.join(S.REPO, 'results', 'rig', '2026-10-29')
FLIGHTS = ['h4', 'h4f_3', 'ch1b', 'sq2_2', 'sq2_5']
TRACKER_HZ = 50.0          # the tracker's tick (tracker_node.FREQUENCY_HZ); rows taken as its samples, as the registry
WINDOW_S = 16.0            # the longest lift-to-event span among the five (sq2_2, 15.7 s), applied to the open one
SEG_S = 4.0                # Welch segment
BEND_BAND, SLIDE_BAND = (4.0, 6.5), (0.8, 1.5)
END_EVENTS = (r'\[dissipative\] UNANNOUNCED DETACH', r'\[dissipative\] UNLOAD drone', r'\[dissipative\] DROP AND LAND',
              r"Fleet command received: 'LAND'")
# the registry rows' windows, s after TAKEOFF (RIG-1029-h4/-h4f/-ch1b/-sq2_2); ch1b's is the 4 s before its loss
REGISTRY_BEND = {'h4': (11.4, 19.8), 'h4f_3': (11.4, 19.8), 'ch1b': ('loss', -4.0, -0.3), 'sq2_2': (19.0, 27.2)}
REGISTRY_SLIDE = {'h4': (11.4, 19.8), 'h4f_3': (11.4, 24.0), 'ch1b': (12.0, 24.5), 'sq2_2': (19.0, 27.2)}

FILTERS = {'none': ('No filter', '#ef8a62'), 'lp1': (r'One 2\,Hz low-pass stage', '#c43c39'),
           'lp2': (r'Two 2\,Hz low-pass stages', '#7f0d1f'), 'notch': (r'5.3\,Hz band-stop', S.BLUE)}
DASH = (0, (4.5, 1.6))
LW = 1.0                   # one weight for every flight in both panels (the legend shows one)
MARK = dict(color='0.45', lw=0.7, ls=(0, (1, 1.5)), zorder=1)   # dotted: dashed means three carriers here


def log_times(name, pattern):
    text = open(os.path.join(DAY, f'{name}.log'), errors='ignore').read()
    return [float(m.group(1)) for m in re.finditer(r'\[(\d{10}\.\d+)\] \[[a-z_0-9]+\]: ' + pattern, text)]


def filter_of(params):
    if params.get('cable_notch_hz'):
        return 'notch'
    if params.get('cable_lp_hz'):
        return 'lp2' if params.get('cable_lp_order') == 2 else 'lp1'
    return 'none'


def read_bag(name):
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message
    r = SequentialReader()
    r.open(StorageOptions(uri=os.path.join(DAY, name), storage_id='sqlite3'), ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    msg_type = get_message(types['/payload/motion_capture_state'])
    rows = []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == '/payload/motion_capture_state':
            m = deserialize_message(raw, msg_type)
            rows.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, m.pose.position.x, m.pose.position.y))
    return np.array(rows)


def load(name):
    t_takeoff = log_times(name, r"Fleet command received: 'TAKEOFF'")[0]
    t_lift = log_times(name, r'\[planner\] pretension done')[0]
    ends = [t for p in END_EVENTS for t in log_times(name, p) if t > t_lift]
    t_end = min(ends + [t_lift + WINDOW_S])
    (planner,) = glob.glob(os.path.join(DAY, f'{name}_logs', 'logs', 'dissipative_planner', '*', 'log.csv'))
    planner = pd.read_csv(planner, low_memory=False)
    n = int(planner.n[planner.sim_time > t_lift].iloc[0])
    dirs = sorted(glob.glob(os.path.join(DAY, f'{name}_logs', 'logs', 'tracker', 'drone*_*')))[:n]   # carriers: drone0..
    params = [json.load(open(os.path.join(d, 'params.json'))) for d in dirs]
    kinds = {filter_of(p) for p in params}
    assert len(kinds) == 1, (name, kinds)
    loss = log_times(name, r'\[dissipative\] UNANNOUNCED DETACH')
    return dict(name=name, n=n, kind=kinds.pop(), notch_hz=params[0].get('cable_notch_hz'),
                t_takeoff=t_takeoff, t_lift=t_lift, t_end=t_end, t_loss=loss[0] if loss else None,
                trackers=[pd.read_csv(os.path.join(d, 'log.csv')) for d in dirs], planner=planner, ring=read_bag(name))


def pull_rows(tr, a, b):
    w = tr[(tr.sim_time > a) & (tr.sim_time < b) & tr.acm_z.notna()]
    return w.acm_z.to_numpy(float)


def band_sd(x, band, fs):
    bb, aa = butter(2, band, btype='band', fs=fs)
    return float(filtfilt(bb, aa, x - x.mean()).std())


def amp_spectrum(x, fs):
    """Welch, Hann, SEG_S segments at 75 % overlap, 'spectrum' scaling: sqrt(2 P) is a sinusoid's amplitude."""
    nper = min(int(SEG_S * fs), x.size)
    f, p = welch(x, fs=fs, window='hann', nperseg=nper, noverlap=3 * nper // 4, nfft=8 * nper, detrend='linear',
                 scaling='spectrum')
    return f, p


def ring_xy(fl, a, b, fs=100.0):
    r = fl['ring']
    m = (r[:, 0] > a) & (r[:, 0] < b)
    t = np.arange(r[m, 0][0], r[m, 0][-1], 1.0 / fs)
    return np.interp(t, r[m, 0], r[m, 1]), np.interp(t, r[m, 0], r[m, 2]), fs


def analyse(fl):
    a, b = fl['t_lift'], fl['t_end']
    ps, sds = [], []
    for tr in fl['trackers']:
        x = pull_rows(tr, a, b)
        f, p = amp_spectrum(x, TRACKER_HZ)
        ps.append(p)
        sds.append(band_sd(x, BEND_BAND, TRACKER_HZ))
    fl['f_pull'], fl['a_pull'] = f, np.sqrt(2 * np.mean(ps, axis=0))
    fl['bend_sd'] = sds
    x, y, fs = ring_xy(fl, a, b)
    f, px = amp_spectrum(x, fs)
    _, py = amp_spectrum(y, fs)
    fl['f_ring'], fl['a_ring'] = f, 100 * np.sqrt(2 * (px + py))
    fl['slide_sd'] = (100 * band_sd(x, SLIDE_BAND, fs), 100 * band_sd(y, SLIDE_BAND, fs))
    k = (f > SLIDE_BAND[0]) & (f < SLIDE_BAND[1])
    fl['slide_peak'] = (float(f[k][np.argmax(fl['a_ring'][k])]), float(fl['a_ring'][k].max()))
    k = (fl['f_pull'] > 2) & (fl['f_pull'] < 8)
    fl['bend_peak'] = (float(fl['f_pull'][k][np.argmax(fl['a_pull'][k])]), float(fl['a_pull'][k].max()))


def registry_check(fl):
    """The registry rows' numbers with their own windows: rows as 50 Hz samples, the planner log's load_x/load_y at 10 Hz."""
    out = []
    w = REGISTRY_BEND.get(fl['name'])
    if w:
        a, b = (fl['t_loss'] + w[1], fl['t_loss'] + w[2]) if w[0] == 'loss' else (fl['t_takeoff'] + w[0],
                                                                                   fl['t_takeoff'] + w[1])
        sd = [band_sd(pull_rows(tr, a, b), BEND_BAND, TRACKER_HZ) for tr in fl['trackers']]
        out.append(f'4-6.5 Hz pull sd [{a - fl["t_takeoff"]:.1f}, {b - fl["t_takeoff"]:.1f}] s: '
                   f'{min(sd):.2f}-{max(sd):.2f} m/s^2 ({", ".join(f"{v:.2f}" for v in sd)})')
    w = REGISTRY_SLIDE.get(fl['name'])
    if w:
        a, b = fl['t_takeoff'] + w[0], fl['t_takeoff'] + w[1]
        p = fl['planner'][(fl['planner'].sim_time > a) & (fl['planner'].sim_time < b)]
        sx, sy = (100 * band_sd(p[c].to_numpy(float), SLIDE_BAND, 10.0) for c in ('load_x', 'load_y'))
        x, y, fs = ring_xy(fl, a, b)
        bx, by = 100 * band_sd(x, SLIDE_BAND, fs), 100 * band_sd(y, SLIDE_BAND, fs)
        out.append(f'0.8-1.5 Hz ring [{w[0]:.1f}, {w[1]:.1f}] s: planner x {sx:.1f} y {sy:.1f} cm; bag x {bx:.1f} '
                   f'y {by:.1f} cm')
    return out


def main():
    flights = [load(f) for f in FLIGHTS]
    for fl in flights:
        analyse(fl)
        sd = fl['bend_sd']
        print(f"{fl['name']:6s} {fl['kind']:5s} n={fl['n']}  window lift + [0, {fl['t_end'] - fl['t_lift']:.1f}] s "
              f"(TAKEOFF + {fl['t_lift'] - fl['t_takeoff']:.1f} .. {fl['t_end'] - fl['t_takeoff']:.1f})")
        print(f"   figure window: 4-6.5 Hz pull sd {min(sd):.2f}-{max(sd):.2f} m/s^2, pull peak "
              f"{fl['bend_peak'][0]:.2f} Hz {fl['bend_peak'][1]:.2f} m/s^2; 0.8-1.5 Hz ring x {fl['slide_sd'][0]:.1f} "
              f"y {fl['slide_sd'][1]:.1f} cm, sideways peak {fl['slide_peak'][0]:.2f} Hz {fl['slide_peak'][1]:.1f} cm")
        for line in registry_check(fl):
            print('   registry window: ' + line)

    S.setup()
    fig = S.figure('full', 2.55)
    gs = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.0])
    ax_p, ax_r = fig.add_subplot(gs[0]), fig.add_subplot(gs[1])

    notch = next(fl['notch_hz'] for fl in flights if fl['kind'] == 'notch')
    ax_p.axvline(notch, **MARK)
    ax_p.annotate(rf'{notch:g}\,Hz', xy=(notch, 1.0), xycoords=('data', 'axes fraction'), xytext=(-2, -2),
                  textcoords='offset points', ha='right', va='top', fontsize=8, color=TEXT)

    for fl in flights:
        label, colour = FILTERS[fl['kind']]
        ls = '-' if fl['n'] == 4 else DASH
        ax_p.plot(fl['f_pull'], fl['a_pull'], color=colour, ls=ls, lw=LW, zorder=3)
        ax_r.plot(fl['f_ring'], fl['a_ring'], color=colour, ls=ls, lw=LW, zorder=3)
    slide = max(flights, key=lambda fl: fl['slide_peak'][1])
    fp, ap = slide['slide_peak']
    ax_r.annotate(rf'{fp:.1f}\,Hz', xy=(fp, ap), xytext=(6, 1), textcoords='offset points', ha='left', va='bottom',
                  fontsize=8, color=FILTERS[slide['kind']][1])

    ax_p.set_xlim(2, 8)
    ax_p.set_xticks([2, 4, 6, 8])
    ax_p.set_ylim(0, 0.85)
    ax_p.set_yticks([0, 0.2, 0.4, 0.6, 0.8])
    ax_p.set_ylabel(r'Amplitude (\si{\metre\per\second\squared})')
    ax_r.set_xlim(0.5, 2.5)
    ax_r.set_xticks([0.5, 1, 1.5, 2, 2.5])
    ax_r.set_ylim(0, 3)
    ax_r.set_yticks([0, 1, 2, 3])
    ax_r.set_ylabel(r'Amplitude (\si{\centi\metre})')
    for ax in (ax_p, ax_r):
        ax.set_xlabel(r'Frequency $f$ (\si{\hertz})')
        ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f'${v:g}$'))
        ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f'${v:g}$'))
    fig.align_ylabels()
    S.panel(ax_p, 'a', 'Measured cable pull')
    S.panel(ax_r, 'b', 'Ring centre, horizontal motion')

    handles = [Line2D([], [], color=c, lw=LW) for _, c in FILTERS.values()]
    labels = [lab for lab, _ in FILTERS.values()]
    handles += [Line2D([], [], color=S.INK, lw=LW), Line2D([], [], color=S.INK, lw=LW, ls=DASH)]
    labels += ['Four carriers', 'Three carriers']
    S.ps.legend_below(fig, handles, labels, ncol=3)
    S.save(fig, 'fig_res_bend')


if __name__ == '__main__':
    main()
