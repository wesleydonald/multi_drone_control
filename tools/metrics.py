"""
tools/metrics.py — THESIS_PLAN §9.2

**Every number in the thesis comes from here and nowhere else.** One function per
metric, each unit-tested against a synthetic signal with a known answer
(tools/test/test_metrics.py). If a number appears in a figure, a table or a claim, it
was produced by a function in this file.

Pure numpy. No ROS, no file formats, no plotting -- the metric functions take arrays so
that a test can hand them a signal whose answer is known analytically. Reading a run
directory is a separate concern and lives in `load_run` at the bottom.

CONVENTIONS, fixed here so they cannot drift between chapters (§9.2):

  * STEADY WINDOW = event time + 5 s, to the end of the sweep. Set once, in
    `steady_window`, and used by every settled-value metric.
  * EVENT TIME comes from the run's events.csv, never from eyeballing a plot.
  * REPEATS: 5 in sim, >= 3 on hardware.
  * REPORT median and full range, never a mean alone -- `summarise`. A mean over 5 runs
    hides the one that diverged, and the one that diverged is usually the finding.
"""
import csv
import json
import math
import os

import numpy as np

SETTLE_S = 5.0            # steady window starts this long after the event
THROTTLE_SAT = 0.59       # controller_mpc's own saturation annotation threshold


# ── helpers ──────────────────────────────────────────────────────────────────

def summarise(values):
    """Median and full range of a set of repeats -- the reporting convention (§9.2).

    Returns a dict, not a formatted string, so the same numbers feed a table, a plot
    annotation and a threshold check without being re-derived."""
    v = np.asarray([x for x in np.ravel(values) if np.isfinite(x)], dtype=float)
    if v.size == 0:
        return {'n': 0, 'median': math.nan, 'min': math.nan, 'max': math.nan}
    return {'n': int(v.size), 'median': float(np.median(v)),
            'min': float(np.min(v)), 'max': float(np.max(v))}


def steady_window(t, t_event=None, settle_s=SETTLE_S):
    """Boolean mask for the steady window: event + settle_s to the end (§9.2).

    With no event, the window is the last (duration - settle_s) of the run, so a hover
    run and an event run are windowed by the same rule."""
    t = np.asarray(t, float)
    start = (float(t[0]) if t_event is None else float(t_event)) + settle_s
    return t >= start


# ── tracking accuracy ────────────────────────────────────────────────────────

def payload_rmse(actual, desired, mask=None):
    """RMS of the 3-D position error norm, in metres.

    RMS of the NORM, not per-axis RMS: the quantity of interest is how far the payload
    was from where it should have been, and a per-axis figure understates that by
    splitting one distance across three numbers."""
    a = np.asarray(actual, float)
    d = np.asarray(desired, float)
    e = np.linalg.norm(a - d, axis=1)
    if mask is not None:
        e = e[mask]
    e = e[np.isfinite(e)]
    return float(np.sqrt(np.mean(e ** 2))) if e.size else math.nan


def radius_ratio(actual_xy, desired_xy, mask=None):
    """Flown radius / commanded radius about the commanded path's own centre.

    The headline number for "the circle came out too small": DISSIPATIVE_TRACKING_ISSUE
    records a flown radius of 0.374 m against a commanded 0.500 m, i.e. 0.75. Both radii
    are measured about the mean of the DESIRED path, so a constant position offset does
    not masquerade as a radius error."""
    a = np.asarray(actual_xy, float)
    d = np.asarray(desired_xy, float)
    if mask is not None:
        a, d = a[mask], d[mask]
    good = np.all(np.isfinite(a), axis=1) & np.all(np.isfinite(d), axis=1)
    a, d = a[good], d[good]
    if a.shape[0] < 2:
        return math.nan
    c = d.mean(axis=0)
    r_des = np.mean(np.linalg.norm(d - c, axis=1))
    r_act = np.mean(np.linalg.norm(a - c, axis=1))
    return float(r_act / r_des) if r_des > 1e-9 else math.nan


def dominant_period_s(t, x):
    """Period of the strongest non-DC spectral line in `x`, or nan if there isn't one.

    Used to bound the phase-lag search (see `phase_lag_s`). Uniform sampling assumed."""
    t = np.asarray(t, float)
    x = np.asarray(x, float)
    good = np.isfinite(t) & np.isfinite(x)
    t, x = t[good], x[good]
    if x.size < 8:
        return math.nan
    dt = float(np.median(np.diff(t)))
    if not np.isfinite(dt) or dt <= 0:
        return math.nan
    x = x - x.mean()
    if np.std(x) < 1e-12:
        return math.nan
    mag = np.abs(np.fft.rfft(x * np.hanning(x.size)))
    mag[0] = 0.0                                    # DC is not a period
    k = int(np.argmax(mag))
    if k == 0:
        return math.nan
    freqs = np.fft.rfftfreq(x.size, dt)
    f = float(freqs[k])
    # Interpolate the peak across neighbouring bins. Bin spacing is 1/record_length, so
    # a 60 s log of a 0.19 Hz circle resolves only to 0.0167 Hz and the nearest bin reads
    # 0.183 Hz -- a 3.6% period error, inherited straight into the phase-lag search bound.
    if 0 < k < mag.size - 1:
        y0, y1, y2 = (math.log(max(v, 1e-300)) for v in mag[k - 1:k + 2])
        den = y0 - 2 * y1 + y2
        if den < 0:
            delta = 0.5 * (y0 - y2) / den
            if abs(delta) <= 0.5:
                f += delta * (freqs[1] - freqs[0])
    return float(1.0 / f) if f > 0 else math.nan


def phase_lag_s(t, reference, signal, max_lag_s=None):
    """Seconds by which `signal` LAGS `reference`. Positive = signal is behind.

    Cross-correlation of the mean-removed signals, with a parabolic fit around the peak
    so the answer is not quantised to the sample period (at 10 Hz logging that
    quantisation is 0.1 s, and the lag being chased is 0.589 s -- a 17% error from
    rounding alone).

    PERIOD ALIASING is why `max_lag_s` defaults the way it does. Every trajectory this is
    used on is periodic (circles, figure-eights), so the cross-correlation has EQUAL peaks
    at lag, lag+T, lag-T, ... and `argmax` picks between them on floating-point noise: a
    synthetic 0.6 s delay on a 5 s circle read back as 10.637 s, and the 0 s stage of the
    stage-split read back as -21.04 s (= -4T). Phase lag is only defined modulo a period,
    so the search is capped at HALF the reference's dominant period, which selects the
    smallest-magnitude representative -- the only one that means "tracking lag".

    A tracker lagging by more than half a period is not tracking, so nothing real is
    excluded. Pass `max_lag_s` explicitly to override.

    Requires uniform sampling; `load_run` resamples for exactly this reason."""
    t = np.asarray(t, float)
    r = np.asarray(reference, float)
    s = np.asarray(signal, float)
    good = np.isfinite(r) & np.isfinite(s)
    r, s = r[good], s[good]
    if r.size < 8:
        return math.nan
    dt = float(np.median(np.diff(t[good]))) if t[good].size > 1 else math.nan
    if max_lag_s is None:
        period = dominant_period_s(t[good], r)
        if np.isfinite(period):
            max_lag_s = 0.5 * period
    r = r - r.mean()
    s = s - s.mean()
    if np.std(r) < 1e-12 or np.std(s) < 1e-12:
        return math.nan          # a constant signal has no phase
    n = r.size
    c = np.correlate(s, r, mode='full')
    lags = np.arange(-(n - 1), n)
    # NORMALISED cross-correlation: divide each lag by the energy actually inside ITS
    # overlap window, sqrt(E_r(L) * E_s(L)), so every lag is a correlation COEFFICIENT in
    # [-1, 1] and identical signals peak at exactly 1.0.
    #
    # The two cheaper normalisations both move the peak. Dividing by nothing leaves the
    # triangular envelope of a finite record, which drags the peak toward zero lag (a
    # synthetic 0.500 s delay read back as 0.461 -- 8% low). Dividing by the overlap
    # COUNT overcorrects: it inflates the variance of the edge lags, and a true 0.0 s lag
    # then peaked 3 samples out at -0.032 s. Energy normalisation has neither bias
    # because it removes the window, not just its length.
    er = np.concatenate(([0.0], np.cumsum(r * r)))
    es = np.concatenate(([0.0], np.cumsum(s * s)))
    pos = lags >= 0
    e_r = np.where(pos, er[n - np.abs(lags)] - er[0], er[n] - er[np.abs(lags)])
    e_s = np.where(pos, es[n] - es[np.abs(lags)], es[n - np.abs(lags)] - es[0])
    overlap = n - np.abs(lags)
    # Lags with less than a quarter of the record overlapping are dropped: too little
    # data behind them to trust, however they are normalised.
    keep = overlap >= max(8, n // 4)
    if max_lag_s is not None and np.isfinite(dt):
        keep &= np.abs(lags) <= max_lag_s / dt
    denom = np.sqrt(np.maximum(e_r * e_s, 1e-300))
    c = c[keep] / denom[keep]
    lags = lags[keep]
    if lags.size == 0:
        return math.nan
    k = int(np.argmax(c))
    lag = float(lags[k])
    if 0 < k < len(c) - 1:       # parabolic interpolation about the peak
        y0, y1, y2 = c[k - 1], c[k], c[k + 1]
        den = (y0 - 2 * y1 + y2)
        # den < 0 is the curvature test: a parabola through three samples straddling a
        # MAXIMUM opens downward. The |delta| <= 0.5 clamp is not a safety margin, it is
        # the definition -- the continuous peak always lies within half a sample of the
        # discrete argmax. Without it, an oversampled peak (500 samples/period at dt=0.01)
        # is so flat that y0, y1, y2 agree to ~1e-9, the fit divides by that near-zero
        # curvature, and a true 0.0 s lag reads back as -0.032 s.
        if den < 0 and abs(den) > 1e-12:
            delta = 0.5 * (y0 - y2) / den
            if abs(delta) <= 0.5:
                lag += delta
    return float(lag * dt)


def stage_split(t, desired_xy, ref_centroid_xy, actual_centroid_xy, payload_xy):
    """The four-stage diagnostic from DISSIPATIVE_TRACKING_ISSUE.md §2.

        desired -> reference centroid -> actual drone centroid -> payload

    Each stage gets a radius ratio and a phase lag MEASURED AGAINST THE PREVIOUS STAGE,
    plus the cumulative figure against `desired`. Which stage moves is the whole answer:
    the lag was root-caused to the tracker stage (reference centroid -> actual centroid)
    exactly this way."""
    stages = [('desired', np.asarray(desired_xy, float)),
              ('ref_centroid', np.asarray(ref_centroid_xy, float)),
              ('actual_centroid', np.asarray(actual_centroid_xy, float)),
              ('payload', np.asarray(payload_xy, float))]
    out = []
    for (n0, a), (n1, b) in zip(stages[:-1], stages[1:]):
        out.append({
            'stage': f'{n0} -> {n1}',
            'radius_ratio': radius_ratio(b, a),
            'phase_lag_s': phase_lag_s(t, a[:, 0], b[:, 0]),
        })
    d = stages[0][1]
    out.append({
        'stage': 'desired -> payload (cumulative)',
        'radius_ratio': radius_ratio(stages[-1][1], d),
        'phase_lag_s': phase_lag_s(t, d[:, 0], stages[-1][1][:, 0]),
    })
    return out


# ── payload attitude ─────────────────────────────────────────────────────────

def load_tilt_deg(quat_wxyz):
    """Tilt of the payload's body z-axis from vertical, per sample, in degrees.

    This is the attach-failure signature: the 2026-07-28 ring-attach runaway went
    10 -> 33 -> 72 deg, and the stable-but-tilted 65 deg configuration parks at ~25."""
    q = np.asarray(quat_wxyz, float)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    r22 = 1 - 2 * (x * x + y * y)
    return np.degrees(np.arccos(np.clip(r22, -1.0, 1.0)))


def tilt_peak_settled(t, quat_wxyz, t_event=None, settle_s=SETTLE_S):
    """(peak tilt over the whole run, settled tilt in the steady window), degrees.

    Both, always: a configuration can settle level after nearly capsizing, and a peak
    alone or a settled value alone would each call that a success."""
    tilt = load_tilt_deg(quat_wxyz)
    m = steady_window(t, t_event, settle_s)
    settled = float(np.mean(tilt[m])) if np.any(m) else math.nan
    return float(np.nanmax(tilt)), settled


# ── load sharing ─────────────────────────────────────────────────────────────

def tension_share(tensions, mask=None):
    """Per-drone share of total cable tension, and the spread across the fleet.

    `tensions` is (T, n). Returns per-drone mean FRACTION (sums to 1) and the spread
    max-min. The A3 success criterion "newcomer carrying >= 15% of total tension" is
    read straight off `fraction`."""
    x = np.asarray(tensions, float)
    if mask is not None:
        x = x[mask]
    x = x[np.all(np.isfinite(x), axis=1)]
    if x.size == 0:
        return {'fraction': [], 'spread': math.nan, 'mean_total': math.nan}
    total = np.sum(x, axis=1)
    good = total > 1e-9
    frac = np.mean(x[good] / total[good, None], axis=0) if np.any(good) else \
        np.full(x.shape[1], math.nan)
    return {'fraction': [float(f) for f in frac],
            'spread': float(np.max(frac) - np.min(frac)),
            'mean_total': float(np.mean(total))}


# ── event response ───────────────────────────────────────────────────────────

def event_peak_error(t, err, t_event, window_s=10.0):
    """Worst tracking error in `window_s` after the event. The transient size."""
    t = np.asarray(t, float)
    e = np.asarray(err, float)
    m = (t >= t_event) & (t <= t_event + window_s) & np.isfinite(e)
    return float(np.max(e[m])) if np.any(m) else math.nan


def event_settling_time(t, err, t_event, band, window_s=30.0):
    """Seconds after the event until the error enters `band` AND STAYS there.

    'And stays' is the whole point: a signal that dips through the band on its way to
    diverging is not settled, and a first-crossing definition would score the 45 deg
    ring attach -- which is briefly level before it capsizes -- as settling fastest.
    Returns inf if it never settles inside the window."""
    t = np.asarray(t, float)
    e = np.asarray(err, float)
    m = (t >= t_event) & (t <= t_event + window_s)
    tt, ee = t[m], e[m]
    if tt.size == 0:
        return math.nan
    inside = np.isfinite(ee) & (np.abs(ee) <= band)
    if not np.any(inside):
        return math.inf
    # last index where it is OUTSIDE the band; settling is the sample after that
    outside = np.where(~inside)[0]
    if outside.size == 0:
        return 0.0
    k = int(outside[-1]) + 1
    if k >= tt.size:
        return math.inf
    return float(tt[k] - t_event)


# ── control effort and health ────────────────────────────────────────────────

def control_effort(throttle, mask=None, sat=THROTTLE_SAT):
    """Mean throttle and the fraction of samples at saturation, per drone.

    `throttle` is (T, n) in [0, 1]. Saturation matters more than the mean: a drone that
    is commanding maximum thrust has no authority left, which is what an under-thrusting
    newcomer looks like just before it is dragged down."""
    x = np.asarray(throttle, float)
    if mask is not None:
        x = x[mask]
    good = np.all(np.isfinite(x), axis=1)
    x = x[good]
    if x.size == 0:
        return {'mean': [], 'saturated_fraction': []}
    return {'mean': [float(v) for v in np.mean(x, axis=0)],
            'saturated_fraction': [float(v) for v in np.mean(x >= sat, axis=0)]}


def solver_health(status, consec_fail_limit=15):
    """acados solver health: failure rate and the worst consecutive-failure run.

    The worst RUN, not just the rate: the tracker holds its last command through
    isolated failures and only disarms after MAX_CONSEC_SOLVE_FAILS in a row
    (controller_mpc.py), so a 2% failure rate is harmless if scattered and fatal if
    consecutive."""
    s = np.asarray(status, float)
    s = s[np.isfinite(s)]
    if s.size == 0:
        return {'fail_fraction': math.nan, 'max_consecutive': 0, 'would_disarm': False}
    bad = s != 0
    worst = run = 0
    for b in bad:
        run = run + 1 if b else 0
        worst = max(worst, run)
    return {'fail_fraction': float(np.mean(bad)), 'max_consecutive': int(worst),
            'would_disarm': bool(worst > consec_fail_limit)}


def cable_accel_gap(measured, modelled, mask=None):
    """|measured - modelled| cable acceleration: THE attach-runaway signature.

    The tracker feeds its MPC the planner's modelled `a_cable` (and caps a measured one
    at CABLE_ACCEL_CAP = 6.0). When the real pull exceeds that, the drone is being
    physically dragged and no amount of tracking fixes it -- 2026-07-28 logged modelled
    ~1.66 against measured 17. Returns peak and mean of the gap, and the peak measured
    magnitude, which is the |aCm| quoted in the diagnostics."""
    m = np.asarray(measured, float)
    d = np.asarray(modelled, float)
    if mask is not None:
        m, d = m[mask], d[mask]
    good = np.all(np.isfinite(m), axis=1) & np.all(np.isfinite(d), axis=1)
    m, d = m[good], d[good]
    if m.size == 0:
        return {'peak_gap': math.nan, 'mean_gap': math.nan, 'peak_measured': math.nan}
    gap = np.linalg.norm(m - d, axis=1)
    return {'peak_gap': float(np.max(gap)), 'mean_gap': float(np.mean(gap)),
            'peak_measured': float(np.max(np.linalg.norm(m, axis=1)))}


# ── reading a run directory ──────────────────────────────────────────────────

def read_csv(path):
    """CSV -> dict of numpy arrays. Non-numeric columns come back as object arrays."""
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        return {}
    out = {}
    for k in rows[0].keys():
        vals = [r[k] for r in rows]
        try:
            out[k] = np.array([float(v) if v not in ('', None) else math.nan
                               for v in vals])
        except (TypeError, ValueError):
            out[k] = np.array(vals, dtype=object)
    return out


def read_events(run_path):
    """events.csv -> {EVENT: first sim time}. Event-relative metrics depend on this
    file existing; §4.1 makes it mandatory for exactly that reason."""
    path = os.path.join(run_path, 'logs', 'events.csv')
    out = {}
    if not os.path.exists(path):
        return out
    with open(path) as fh:
        for row in csv.DictReader(fh):
            ev = row.get('event')
            if ev and ev not in out:
                try:
                    out[ev] = float(row['sim_time'])
                except (TypeError, ValueError):
                    pass
    return out


def load_run(run_path):
    """Load a run directory into {'manifest', 'events', 'data', 'source'}.

    Understands the SIL bench layout (logs/sil.csv, one wide table) and falls back to
    the per-drone controller CSVs. Deliberately does NOT normalise the two into one
    schema: they record different things, and pretending otherwise would invent
    columns. Callers ask `source` which they got."""
    out = {'manifest': {}, 'events': read_events(run_path), 'data': {},
           'source': None, 'path': run_path}
    mf = os.path.join(run_path, 'manifest.json')
    if os.path.exists(mf):
        with open(mf) as fh:
            out['manifest'] = json.load(fh)
    sil = os.path.join(run_path, 'logs', 'sil.csv')
    if os.path.exists(sil):
        out['data'] = read_csv(sil)
        out['source'] = 'sil'
        return out
    logs = os.path.join(run_path, 'logs')
    if os.path.isdir(logs):
        per = {}
        for name in sorted(os.listdir(logs)):
            if name.endswith('.csv') and name != 'events.csv':
                per[os.path.splitext(name)[0]] = read_csv(os.path.join(logs, name))
        if per:
            out['data'] = per
            out['source'] = 'controller'
    return out


def n_drones(data):
    """How many drones a SIL table describes, from its column names."""
    i = 0
    while f'd{i}_x' in data:
        i += 1
    return i


def summarise_sil_run(run_path):
    """Every metric this file defines, applied to one SIL run. The thing a runner calls
    to produce metrics.json (§4.1) -- so a finished run is already readable."""
    run = load_run(run_path)
    d = run['data']
    if run['source'] != 'sil' or not d:
        return {'error': f'not a SIL run: {run_path}'}
    n = n_drones(d)
    t = d['t'] - d['t'][0]
    ev = run['events']
    t_ev = ev.get('WELD', ev.get('DETACH'))
    mask = steady_window(t, t_ev)

    payload = np.column_stack([d['payload_x'], d['payload_y'], d['payload_z']])
    quat = np.column_stack([d['payload_qw'], d['payload_qx'],
                            d['payload_qy'], d['payload_qz']])
    peak_tilt, settled_tilt = tilt_peak_settled(t, quat, t_ev)
    tens = np.column_stack([d[f'd{i}_tension'] for i in range(n)])
    thr = np.column_stack([d[f'd{i}_thr'] for i in range(n)])

    out = {
        'run': os.path.basename(run_path),
        'run_id': run['manifest'].get('run_id'),
        'events': ev,
        'n_drones': n,
        'payload_tilt_peak_deg': peak_tilt,
        'payload_tilt_settled_deg': settled_tilt,
        'payload_z_settled_m': (float(np.mean(payload[mask, 2])) if np.any(mask)
                                else math.nan),
        'tension_share': tension_share(tens, mask),
        'control_effort': control_effort(thr, mask),
    }
    if 'payload_ref_x' in d:
        des = np.column_stack([d['payload_ref_x'], d['payload_ref_y'],
                               d['payload_ref_z']])
        if np.any(np.isfinite(des)):
            out['payload_rmse_m'] = payload_rmse(payload, des, mask)
            out['payload_radius_ratio'] = radius_ratio(payload[:, :2], des[:, :2], mask)
            out['payload_phase_lag_s'] = phase_lag_s(t, des[:, 0], payload[:, 0])
    per_drone = []
    for i in range(n):
        err = d[f'd{i}_track_err']
        rec = {
            'drone': i,
            'peak_track_err_m': float(np.nanmax(err)) if np.any(np.isfinite(err))
            else math.nan,
            'settled_track_err_m': (float(np.nanmean(err[mask])) if np.any(mask)
                                    else math.nan),
            'peak_cable_accel': float(np.nanmax(d[f'd{i}_acm'])),
            'settled_elev_deg': (float(np.nanmean(d[f'd{i}_elev_deg'][mask]))
                                 if np.any(mask) else math.nan),
        }
        if t_ev is not None:
            rec['event_peak_err_m'] = event_peak_error(t, err, t_ev)
            rec['event_settling_s'] = event_settling_time(t, err, t_ev, band=0.10)
        per_drone.append(rec)
    out['per_drone'] = per_drone
    return out
