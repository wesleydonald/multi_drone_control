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
import datetime
import glob
import json
import math
import os
import re

import numpy as np

import run_logs

SETTLE_S = 5.0            # steady window starts this long after the event
THROTTLE_SAT = 0.59       # tracker_node's own saturation annotation threshold


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


def sweep_window(t, desired_xy, frac=0.01):
    """Boolean mask for the part of the run where the COMMANDED path is moving.

    The companion convention to `steady_window`, and mandatory for every trajectory
    metric. A `load_traj: circle` run is one eased sweep inside a much longer flight --
    lift, hover, sweep, hover, land -- so on a 75 s run the circle occupies about 8 s.
    Measured over the whole record, the mean of the desired path is the hover point
    rather than the circle's centre, and `radius_ratio` reads 0.96 on a run whose sweep
    ratio is 0.80 (R0054). It looks like a good number and means nothing.

    The mask is CONTIGUOUS, from the first moving sample to the last, because the
    reference is published at 10 Hz and logged at 50 Hz: sample-wise, four of every five
    steps are exactly zero, and a per-sample test would keep a fifth of the sweep and
    inflate the apparent commanded speed fivefold."""
    d = np.asarray(desired_xy, float)
    step = np.zeros(d.shape[0])
    good = np.all(np.isfinite(d), axis=1)
    step[1:] = np.where(good[1:] & good[:-1],
                        np.linalg.norm(np.diff(d, axis=0), axis=1), 0.0)
    peak = float(np.max(step)) if step.size else 0.0
    moving = np.where(step > max(frac * peak, 1e-9))[0]
    mask = np.zeros(d.shape[0], bool)
    if moving.size:
        mask[moving[0] - 1 if moving[0] else 0:moving[-1] + 1] = True
    return mask


def steady_window(t, t_event=None, settle_s=SETTLE_S, t_end=None):
    """Boolean mask for the steady window: event + settle_s to the end (§9.2), or to
    `t_end` when given -- the LAND event on a hover-then-land run. Without that cutoff
    a "settled" height averaged the descent to the floor into the hover (SIL R0275 read
    0.597 m for a hover at 0.634 m; reviewer, 2026-09-23).

    With no event, the window is the last (duration - settle_s) of the run, so a hover
    run and an event run are windowed by the same rule."""
    t = np.asarray(t, float)
    start = (float(t[0]) if t_event is None else float(t_event)) + settle_s
    m = t >= start
    if t_end is not None and np.isfinite(t_end) and float(t_end) > start:
        m &= t < float(t_end)
    return m


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


def tilt_peak_settled(t, quat_wxyz, t_event=None, settle_s=SETTLE_S, t_end=None):
    """(peak tilt over the whole run, settled tilt in the steady window), degrees.

    Both, always: a configuration can settle level after nearly capsizing, and a peak
    alone or a settled value alone would each call that a success."""
    tilt = load_tilt_deg(quat_wxyz)
    m = steady_window(t, t_event, settle_s, t_end)
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


# ── per-event windows (step 2a, 2026-09-26) ──────────────────────────────────
#
# One window per disturbing event: from the event to the next disturbing event or
# WINDOW_S later. RELEASED, WAIT_* and LIFTED are bookkeeping, not disturbances. Events
# of one group (the weld, the runner's REWELD edge and the dissipative FOLD_IN ~0.1 s
# later) are one disturbance and do not cut each other's window.

WINDOW_S = 15.0
SETTLE_BAND_DEG = 2.0
SETTLE_HOLD_S = 1.0
DISTURBING = ('DETACH', 'WELD', 'REWELD', 'FOLD_IN', 'DROP', 'PHASE:DROP_OBJECT', 'LAND')
_SAME_EVENT = ({'WELD', 'REWELD', 'FOLD_IN'}, {'DROP', 'PHASE:DROP_OBJECT'})
_SAME_EVENT_S = 2.0


def window_end(events, t_event, name=None, horizon_s=WINDOW_S):
    """End of the window that `name` opens at `t_event`, from [(t, name, ...)]."""
    end = float(t_event) + horizon_s
    group = next((g for g in _SAME_EVENT if name in g), set())
    for ev in events:
        t, ev_name = float(ev[0]), ev[1]
        if not np.isfinite(t) or t <= t_event or ev_name not in DISTURBING:
            continue
        if ev_name in group and t - t_event <= _SAME_EVENT_S:
            continue
        end = min(end, t)
    return end


def settle_time(t, x, t_event, t_end, band=SETTLE_BAND_DEG, hold_s=SETTLE_HOLD_S):
    """(seconds from the event until `x` is under `band` and stays there for `hold_s`,
    censored). Searched from the window's peak on, so a transient that builds after the
    event cannot score as settled before it. Censored (nan, True) when the window ends
    first -- a lower bound of t_end - t_event, not a number."""
    t = np.asarray(t, float)
    x = np.asarray(x, float)
    m = (t >= t_event) & (t <= t_end)
    tt, xx = t[m], x[m]
    if tt.size == 0 or not np.any(np.isfinite(xx)):
        return math.nan, True
    under = np.isfinite(xx) & (xx < band)
    k = int(np.nanargmax(xx))
    if under[k]:
        return 0.0, False                   # never left the band
    n = tt.size
    while k < n:
        if not under[k]:
            k += 1
            continue
        j = k
        while j < n and under[j]:
            j += 1
        if tt[j - 1] - tt[k] >= hold_s:
            return float(tt[k] - t_event), False
        k = j
    return math.nan, True


def _drone_ids(data):
    i = 0
    while f'd{i}_z' in data:
        i += 1
    return range(i)


def _ring_tilt(data):
    if has(data, 'payload_tilt_deg'):
        return np.asarray(data['payload_tilt_deg'], float)
    return load_tilt_deg(np.column_stack([data[f'payload_q{a}'] for a in 'wxyz']))


def event_window_metrics(t, data, t_event, t_end=None, exclude=(), horizon_s=WINDOW_S,
                         band_deg=SETTLE_BAND_DEG, hold_s=SETTLE_HOLD_S):
    """The transient an event causes, over [t_event, min(t_end, t_event + horizon_s)].

    * peak ring tilt, and peak drone tilt over every drone (mask a drone's columns to
      NaN to leave it out -- see mask_released);
    * carrier dip, actual and reference separately: the largest fall of a carrier below
      its own height at the event. The event drone(s) in `exclude` are not carriers;
    * settle: time to be under `band_deg` for `hold_s` (see settle_time), censored if
      the window ends first;
    * ring z error (actual - reference) averaged over the window's last second."""
    t = np.asarray(t, float)
    t0 = float(t_event)
    end = t0 + horizon_s if t_end is None else min(float(t_end), t0 + horizon_s)
    m = (t >= t0) & (t <= end)
    out = {'t_event': t0, 't_end': end, 'window_s': end - t0}
    if not np.any(m):
        return out
    tw = t[m]
    tilt = _ring_tilt(data)[m]
    k = int(np.nanargmax(tilt)) if np.any(np.isfinite(tilt)) else None
    out['ring_tilt_peak_deg'] = float(tilt[k]) if k is not None else math.nan
    out['ring_tilt_peak_after_s'] = float(tw[k] - t0) if k is not None else math.nan

    best = (math.nan, None)
    for i in _drone_ids(data):
        v = _nanmax(np.asarray(data.get(f'd{i}_tilt_deg', [math.nan] * t.size), float)[m])
        if np.isfinite(v) and not (v <= best[0]):
            best = (v, i)
    out['drone_tilt_peak_deg'], out['drone_tilt_peak_drone'] = best

    for key, col in (('carrier_dip_m', 'z'), ('carrier_dip_ref_m', 'ref_z')):
        dip = (math.nan, None)
        for i in _drone_ids(data):
            if i in exclude or f'd{i}_{col}' not in data:
                continue
            z = np.asarray(data[f'd{i}_{col}'], float)[m]
            good = np.isfinite(z)
            if not np.any(good):
                continue
            d = float(z[good][0] - np.min(z[good]))
            if not (d <= dip[0]):
                dip = (d, i)
        out[key], out[key.replace('_m', '_drone')] = dip

    out['settle_s'], out['settle_censored'] = settle_time(t, _ring_tilt(data), t0, end,
                                                          band_deg, hold_s)
    if has(data, 'payload_ref_z'):
        last = m & (t >= end - 1.0)
        err = np.asarray(data['payload_z'], float) - np.asarray(data['payload_ref_z'], float)
        out['ring_z_err_last1s_m'] = _nanmean(err[last])
    return out


def _event_drone(name, arg, n):
    """The drone an event is about: DETACH's arg, else the reserved newcomer (last id),
    which is the one drone the runner's weld edge can mean."""
    if name in ('DETACH', 'FOLD_IN'):
        try:
            return int(float(arg))
        except (TypeError, ValueError):
            pass
    return n - 1


def mask_released(data, events, drone):
    """Copy of `data` with `drone`'s columns NaN from RELEASED to REWELD: in between it
    is flying someone else's mission and is not part of the fleet's transient."""
    t_rel = next((e[0] for e in events if e[1] == 'RELEASED' and np.isfinite(e[0])), None)
    if t_rel is None:
        return data
    t_rew = next((e[0] for e in events if e[1] == 'REWELD' and np.isfinite(e[0])
                  and e[0] > t_rel), math.inf)
    t = np.asarray(data['t'], float)
    gone = (t >= t_rel) & (t < t_rew)
    out = dict(data)
    pre = f'd{drone}_'
    for k, v in data.items():
        if k.startswith(pre) and v.dtype.kind == 'f':
            v = v.copy()
            v[gone] = math.nan
            out[k] = v
    return out


def event_windows(t, data, events, names=('DETACH', 'WELD', 'REWELD', 'DROP')):
    """event_window_metrics for every finite occurrence of `names` in [(t, name, arg)]."""
    n = len(_drone_ids(data))
    out = []
    for ev in events:
        te, name = float(ev[0]), ev[1]
        if name not in names or not np.isfinite(te):
            continue
        arg = ev[2] if len(ev) > 2 else ''
        rec = {'event': name, 'arg': arg}
        rec.update(event_window_metrics(
            t, data, te, window_end(events, te, name),
            exclude=(_event_drone(name, arg, n),)))
        out.append(rec)
    return out


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
    (tracker_node.py), so a 2% failure rate is harmless if scattered and fatal if
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


# ── tethered hold (plan 2026-10, W6): the rig and its sim twin, from the planner log ──

HOLD_WINDOW_S = 20.0        # judged over the last this-many s of the hold
REF_AGE_STALE_S = 0.3       # a reference older than this is stale
HEAVE_MIN_PERIODS = 3       # a window shorter than this many periods cannot claim a period
AIRBORNE_DZ_M = 0.05        # ring above its creep rest by this = airborne (thrust_fit.py)


def _sample_dt(t):
    """Per-sample duration, so a fraction of TIME survives uneven logging (skipped ticks)."""
    t = np.asarray(t, float)
    if t.size < 2:
        return np.ones_like(t)
    dt = np.diff(t)
    return np.r_[dt, np.median(dt)]


def heave_metrics(t, z, min_periods=HEAVE_MIN_PERIODS):
    """Ring height oscillation over a hold window: peak-to-peak, sd and dominant period.

    `short` is True when the window holds fewer than `min_periods` of its own dominant
    period (or none can be found): model-f1 was judged "14 cm at 0.17 Hz" off 3.8 s of
    ramp data, which cannot resolve a 6 s period. The period comes from the linearly
    detrended signal, so a slow drift does not pose as the heave; p-p and sd are raw."""
    t = np.asarray(t, float)
    z = np.asarray(z, float)
    good = np.isfinite(t) & np.isfinite(z)
    t, z = t[good], z[good]
    if z.size < 2:
        return {'pp_m': math.nan, 'sd_m': math.nan, 'period_s': math.nan,
                'window_s': 0.0, 'n_periods': 0.0, 'short': True}
    span = float(t[-1] - t[0])
    period = dominant_period_s(t, z - np.polyval(np.polyfit(t, z, 1), t)) if z.size >= 8 else math.nan
    n_per = span / period if np.isfinite(period) and period > 0 else 0.0
    return {'pp_m': float(np.ptp(z)), 'sd_m': float(np.std(z)), 'period_s': period,
            'window_s': span, 'n_periods': float(n_per), 'short': bool(n_per < min_periods)}


def hold_window(t, z_tgt, target_z, land=None, window_s=HOLD_WINDOW_S, tol=1e-3):
    """(t0, t1, hold_s) of the judged hold: the last `window_s` between the lift reaching
    `target_z` and LAND (or the end of the log). None when the target is never reached.
    hold_s is the whole hold, so a caller can see a window cut short (hold_s < window_s)."""
    t = np.asarray(t, float)
    zt = np.asarray(z_tgt, float)
    at = np.where(np.isfinite(zt) & (zt >= float(target_z) - tol))[0]
    if at.size == 0:
        return None
    start = float(t[at[0]])
    end = float(t[-1])
    if land is not None:
        ld = np.where((np.asarray(land, float) > 0.5) & (t >= start))[0]
        if ld.size:
            end = float(t[ld[0]])
    return max(start, end - float(window_s)), end, end - start


def hold_z_error(z, target_z, mask=None):
    """Mean ring height minus the target over the hold, signed (m)."""
    z = np.asarray(z, float)
    if mask is not None:
        z = z[mask]
    return _nanmean(z) - float(target_z)


def airborne_mask(z, phase=None, dz=AIRBORNE_DZ_M):
    """(mask, rest z): ring above its creep rest + dz. The rest is the median ring z over the
    creep phase, else over the first 10 samples (the thrust_fit.py convention)."""
    z = np.asarray(z, float)
    rest = math.nan
    if phase is not None:
        creep = np.asarray(phase).astype(str) == 'creep'
        if np.any(creep & np.isfinite(z)):
            rest = float(np.nanmedian(z[creep]))
    if not np.isfinite(rest):
        rest = float(np.nanmedian(z[:10])) if z.size else math.nan
    with np.errstate(invalid='ignore'):
        return z > rest + dz, rest


def ref_age_frac(t, ref_age, mask=None, stale_s=REF_AGE_STALE_S):
    """Fraction of (masked) TIME with the planner reference older than `stale_s`.

    Takes the planner log's ref_age, which is sim time since the last good solve: the
    trackers' own ref_age column is WALL time, and parallel lab runs at a low real-time
    factor read stale on it with nothing wrong in the planner. Samples with no age yet
    (before the first solve) are left out."""
    t = np.asarray(t, float)
    a = np.asarray(ref_age, float)
    w = _sample_dt(t)
    keep = np.isfinite(a) & np.isfinite(w)
    if mask is not None:
        keep &= np.asarray(mask, bool)
    total = float(np.sum(w[keep]))
    if total <= 0:
        return math.nan
    return float(np.sum(w[keep & (a > stale_s)]) / total)


def max_payload_vz(vz, mask=None):
    """Peak |ring vz| (m/s): the run-up that released a magnet in f6 (0.38 m/s)."""
    v = np.abs(np.asarray(vz, float))
    if mask is not None:
        v = v[mask]
    return _nanmax(v)


SOLVE_STATUS_RE = re.compile(r'solve status (-?\d+)')


def planner_solve_failures(status=None, published=None, phase=None, log_text=None):
    """Failed planner solves, from the tick log and/or the node's 'solve status N' lines.

    Tick log: OCP ticks only -- `published` non-blank, else phase != creep -- because a creep
    tick's solve_status is the priming solve's (card A critic). `not_solved` counts OCP ticks
    that published a shifted horizon or nothing (a held-back unconverged solve reads status 0).
    `n` is the tick count when the log has the column, else the line count (model-f1: 25)."""
    out = {'ticks': None, 'not_solved': None, 'lines': None, 'n': None}
    if status is not None:
        s = np.asarray(status, float)
        ocp = np.ones(s.size, bool)
        pub = None
        if published is not None:
            pub = np.array(['' if (isinstance(v, float) and math.isnan(v)) else str(v).strip()
                            for v in published])
            if np.any(pub != ''):
                ocp = pub != ''
            else:
                pub = None
        if pub is None and phase is not None:
            ocp = np.asarray(phase).astype(str) != 'creep'
        out['ticks'] = int(np.sum(ocp & np.isfinite(s) & (s != 0)))
        if pub is not None:
            out['not_solved'] = int(np.sum(np.isin(pub, ('shifted', 'none'))))
        out['n'] = out['ticks']
    if log_text is not None:
        out['lines'] = sum(1 for m in SOLVE_STATUS_RE.finditer(log_text) if int(m.group(1)) != 0)
        if out['n'] is None:
            out['n'] = out['lines']
    return out


def plant_law(run_path=None, n=4, thrust_map=None):
    """The thrust law u -> supported kg that the run's PLANT obeys, in thrust_fit.py's form
    u = a_i + b M + c (V - 23.5). Rig flights and the sim 'rig' map: the RIG-0930-ladder4 law
    (per-drone a_i from the bridges' read-back when they carry one). The sim 'linear' map:
    thrust = 4 mc w_max^2 u, so a = c = 0. `thrust_map` None reads it from params/bf_comm_*."""
    import thrust_fit as tf
    a = list(tf.LAW_0930['a']) + [tf.LAW_0930['a'][-1]] * max(0, n - 4)
    law = {'a': a[:max(n, 4)], 'b': tf.LAW_0930['b'], 'c': tf.LAW_0930['c'], 'map': 'rig'}
    params = {}
    if run_path is not None:
        import yaml
        for i in range(n):
            try:
                with open(os.path.join(run_path, 'params', f'bf_comm_{i}.yaml')) as fh:
                    params[i] = (next(iter(yaml.safe_load(fh).values())) or {}).get('ros__parameters', {})
            except (OSError, AttributeError, StopIteration, TypeError):
                pass
    if thrust_map is None and run_path is not None:
        # a sim run: the bridges' map, and 'linear' (every world before W4) when not read back
        thrust_map = str(next(iter(params.values())).get('thrust_map', 'linear')) if params else 'linear'
    if thrust_map == 'linear':
        try:
            from simulation_communication.rig_thrust import linear_thrust
        except ImportError:
            import sys
            sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))), 'src', 'simulation_communication'))
            from simulation_communication.rig_thrust import linear_thrust
        return {'a': [0.0] * max(n, 4), 'b': 9.81 / linear_thrust(1.0), 'c': 0.0, 'map': 'linear'}
    for i, p in params.items():
        for key in ('thrust_a', 'rig_a', 'thrust_offset', 'rig_offset'):
            if isinstance(p.get(key), (int, float)) and p[key] > 0:
                law['a'][i] = float(p[key])
                break
    return law


def carried_fraction(planner_path, trackers, ring_kg=0.86, law=None, drone_kg=0.55, window=None):
    """Rod-force sum over the ring's weight (thrust_fit.py --tethered), airborne and over
    `window` = (t0, t1) in the planner's sim time. The pass metric that replaced the
    breakaway % (plan, critic must-fix 4). `ring_kg` is the TRUE ring (the world's in sim),
    not the planner's belief, so the mass-mismatch arm reads its real balance."""
    import thrust_fit as tf
    law = law or tf.LAW_0930
    with np.errstate(divide='ignore', invalid='ignore'):     # repeated tracker stamps
        df, rest = tf.rod_forces(planner_path, trackers, law, drone_kg)
    if df.empty:
        return None
    scale = tf.RING_KG / float(ring_kg)
    # rod_forces rebases t to its first common sample on the lowest-id tracker's clock
    pl = tf.read_log(planner_path)
    if not np.isfinite(rest):                           # no creep phase (taut air start)
        rest = airborne_mask(pl.load_z.to_numpy(float))[1]
    logs = {i: tf.read_log(p) for i, p in trackers.items()}
    logs = {i: d for i, d in logs.items() if len(d) > 10}
    t_lo = max(pl.sim_time.iloc[0], *(d.sim_time.iloc[0] for d in logs.values()))
    ref_t = logs[min(logs)].sim_time.to_numpy(float)
    t_abs = df.t.to_numpy(float) + ref_t[ref_t >= t_lo][0]
    frac = df.frac.to_numpy(float) * scale
    air = df.ring_z.to_numpy(float) > rest + AIRBORNE_DZ_M
    out = {'ring_kg': float(ring_kg), 'rest_z': rest, 'airborne_s': float(np.sum(_sample_dt(t_abs)[air]))}

    def med(mask):
        return float(np.median(frac[mask])) if np.any(mask) else math.nan
    out['airborne_median'] = med(air)
    if np.any(air):
        out['airborne_iqr'] = [float(v) for v in np.percentile(frac[air], [25, 75])]
    if 'frac_dyn' in df:
        dyn = df.frac_dyn.to_numpy(float) * scale
        out['airborne_accel_corrected_median'] = float(np.median(dyn[air])) if np.any(air) else math.nan
    if window is not None:
        w = air & (t_abs >= window[0]) & (t_abs <= window[1])
        out['hold_median'] = med(w)
    return out


def tethered_metrics(planner, target_z=None, hold_window_s=HOLD_WINDOW_S, log_text=None,
                     stale_s=REF_AGE_STALE_S):
    """The hold scored from the planner's tick log (a read_csv dict), the same way for a rig
    flight and a sim run: sim time throughout, so a slow lab slot does not read stale.

    The hold is the last `hold_window_s` from the lift reaching `target_z` to LAND. vz is
    judged from the hand-over (first 'planner' tick: pretension, breakaway, lift) to the end
    of the hold. Columns a log does not have (model-f1 predates W1) are simply absent."""
    t = np.asarray(planner['sim_time'], float)
    z = np.asarray(planner['load_z'], float)
    phase = planner.get('phase')
    z_tgt = planner.get('z_tgt')
    if target_z is None and z_tgt is not None:
        target_z = _nanmax(z_tgt)
    air, rest = airborne_mask(z, phase)
    out = {'target_z': target_z, 'rest_z': rest,
           'airborne_s': float(np.sum(_sample_dt(t)[air]))}
    hw = hold_window(t, z_tgt, target_z, planner.get('land'), hold_window_s) \
        if z_tgt is not None and target_z is not None else None
    end = t[-1] if t.size else math.nan
    out['hold'] = None
    if hw is not None:
        t0, t1, hold_s = hw
        end = t1
        m = (t >= t0) & (t <= t1)
        out['hold'] = {'t0': t0, 't1': t1, 'hold_s': hold_s, 'window_s': t1 - t0,
                       'full': bool(hold_s >= hold_window_s)}
        out['hold_z_mean_m'] = _nanmean(z[m])
        out['hold_z_err_m'] = hold_z_error(z, target_z, m)
        out['heave'] = heave_metrics(t[m], z[m])
        if 'tilt_deg' in planner:
            out['hold_tilt_mean_deg'] = _nanmean(planner['tilt_deg'][m])
            out['hold_tilt_max_deg'] = _nanmax(planner['tilt_deg'][m])
        if has(planner, 'z_bias'):
            out['hold_z_bias_max_abs'] = _nanmax(np.abs(planner['z_bias'][m]))
    if 'tilt_deg' in planner:
        out['tilt_max_airborne_deg'] = _nanmax(planner['tilt_deg'][air])
    if phase is not None and 'load_vz' in planner:
        ph = np.asarray(phase).astype(str)
        hand = np.where(ph != 'creep')[0]
        if hand.size:
            out['max_payload_vz_mps'] = max_payload_vz(
                planner['load_vz'], (t >= t[hand[0]]) & (t <= end))
    if has(planner, 'ref_age'):
        out['ref_age_frac'] = ref_age_frac(t, planner['ref_age'], air, stale_s)
    out['solve_failures'] = planner_solve_failures(
        planner.get('solve_status') if has(planner, 'solve_status') else None,
        planner.get('published'), phase, log_text)
    return out


def planner_logs(run_path):
    """The load planner's log.csv files under a run (or rig log) folder, longest first."""
    paths = run_logs.node_csvs(run_path, 'mpc_planner')
    return sorted(paths, key=lambda p: -os.path.getsize(p))


def tethered_run_metrics(run_path, hold_window_s=HOLD_WINDOW_S, log_text=None, ring_kg=None,
                         thrust_map=None):
    """tethered_metrics + carried_fraction for one run directory or rig log folder, or None
    when there is no planner log. Ring mass: the argument, else the manifest world's, else
    the planner's own load_mass. Solve-status lines come from logs/launch.log unless given."""
    import thrust_fit as tf
    logs = planner_logs(run_path)
    if not logs:
        return None
    ppath = logs[0]
    planner = read_csv(ppath)
    if not planner or 'sim_time' not in planner:
        return None
    try:
        with open(os.path.join(os.path.dirname(ppath), 'params.json')) as fh:
            pparams = json.load(fh)
    except (OSError, ValueError):
        pparams = {}
    manifest = {}
    try:
        with open(os.path.join(run_path, 'manifest.json')) as fh:
            manifest = json.load(fh)
    except (OSError, ValueError):
        pass
    if log_text is None:
        log_text = _read_text(os.path.join(run_path, 'logs', 'launch.log')) or None
    target = pparams.get('target_z')
    out = tethered_metrics(planner, target, hold_window_s, log_text)
    out['planner_log'] = os.path.relpath(ppath, run_path)
    if ring_kg is None and manifest.get('world'):
        try:
            from check_geometry import world_geometry
            w = manifest['world']
            w = w if os.path.isabs(w) else os.path.join(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))), 'simulation_assets', w)
            ring_kg = world_geometry(w).get('load_mass')
        except Exception:                                   # noqa: BLE001
            ring_kg = None
    ring_kg = ring_kg or pparams.get('load_mass') or tf.RING_KG
    if thrust_map is None and manifest.get('kind') == 'sim':
        thrust_map = str((manifest.get('launch_args') or {}).get('sim_thrust_map', '')) or None
    n = int(pparams.get('num_drones') or 4)
    law = plant_law(run_path if manifest.get('kind') == 'sim' else None, n, thrust_map)
    groups = tf.launches(run_path)
    if manifest.get('kind') == 'sim' and len(groups) == 1 and not groups[0][1]:
        # one launch per sim run: a planner building its solver cold starts > 30 s after the trackers
        groups = [(groups[0][0], run_logs.trackers(run_logs.run_root(ppath)))]
    for p, trackers in groups:
        if p == ppath and trackers:
            window = (out['hold']['t0'], out['hold']['t1']) if out.get('hold') else None
            try:
                cf = carried_fraction(p, trackers, ring_kg, law,
                                      float(pparams.get('drone_mass') or tf.DRONE_KG), window)
            except Exception as e:                          # noqa: BLE001
                cf = {'error': f'{type(e).__name__}: {e}'}
            if cf is not None:
                cf['law'] = law.get('map', 'rig')
                out['carried_fraction'] = cf
    return out


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


def read_event_list(run_path):
    """events.csv -> [(sim time, EVENT, arg)] in file order, every occurrence (a run
    can HANDOFF twice or change phase twenty times; `read_events` keeps only the first)."""
    path = os.path.join(run_path, 'logs', 'events.csv')
    out = []
    if not os.path.exists(path):
        return out
    with open(path) as fh:
        for row in csv.DictReader(fh):
            try:
                t = float(row['sim_time'])
            except (TypeError, ValueError):
                t = math.nan
            out.append((t, row.get('event') or '', row.get('arg') or ''))
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

    ONE LOADER FOR BOTH HARNESSES. The SIL bench writes `logs/sil.csv` and the Gazebo
    runner writes `logs/run.csv`, and they share a column schema on purpose -- same
    `t`, `payload_*` and `dN_*` names, with the columns a Gazebo run cannot observe
    from outside the tracker (`dN_acm`, `dN_tension`, `dN_elev_deg`) present but NaN.
    So both come back as one wide table and every downstream consumer is source-blind,
    which is what lets compare_runs.py put a bench run and a Gazebo run on one axis.
    `source` is still reported, because a caller writing a caption needs to say which.

    Both files share their time origin with events.csv (t=0 at t_ready), so `t` and
    event times are directly comparable with no re-basing.

    The legacy per-drone controller CSVs are the third, unschema'd case: those come
    back as {name: table} and `source` is 'controller'."""
    out = {'manifest': {}, 'events': read_events(run_path), 'data': {},
           'source': None, 'path': run_path}
    mf = os.path.join(run_path, 'manifest.json')
    if os.path.exists(mf):
        with open(mf) as fh:
            out['manifest'] = json.load(fh)
    for name, source in (('sil.csv', 'sil'), ('run.csv', 'gazebo')):
        path = os.path.join(run_path, 'logs', name)
        if os.path.exists(path):
            out['data'] = read_csv(path)
            out['source'] = source
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
    """How many drones a run table describes, from its column names."""
    i = 0
    while f'd{i}_x' in data:
        i += 1
    return i


def has(data, name):
    """True if a column exists AND holds at least one finite sample.

    The schema keeps Gazebo-unobservable columns as all-NaN rather than omitting them,
    so `name in data` is not the question a caller means to ask."""
    v = data.get(name)
    return v is not None and v.dtype.kind == 'f' and bool(np.any(np.isfinite(v)))


def stack(data, template, n):
    """(T, n) array of one per-drone column, e.g. stack(d, 'd{i}_thr', 3)."""
    return np.column_stack([data[template.format(i=i)] for i in range(n)])


def centroid(data, n, prefix='', drones=None):
    """(T, 3) centroid of the fleet's positions (prefix='') or references ('ref_').

    Averaged over the drones with finite data at each sample, so one drone dropping out
    of the log shifts the centroid's noise rather than turning it into NaN.

    Over ALL drones by default, deliberately. The `dN_attached` column would be the
    natural filter, but it does not mean the same thing in both harnesses -- the SIL
    plant marks every cable-linked drone attached, while the Gazebo runner only ever
    marks the welded newcomer -- so filtering on it would silently compare a 3-drone
    centroid against a 1-drone one. Pass `drones` explicitly when a subset is wanted."""
    idx = range(n) if drones is None else drones
    xs = np.dstack([np.column_stack([data[f'd{i}_{prefix}{ax}'] for ax in 'xyz'])
                    for i in idx])                        # (T, 3, n)
    with np.errstate(invalid='ignore'):
        return np.nanmean(xs, axis=2)


def cog_margin(rho):
    """Can the load hang LEVEL on this set of attach points? (margin_m, gap_deg, ok)

    The load's weight acts at its centre of mass and each cable can only PULL, upward,
    at its own attach point. For the vertical components to balance with every tension
    non-negative AND produce no net moment, the centre of mass must lie strictly inside
    the polygon of attach points -- the classic support-polygon condition. Equivalently,
    for points on a ring: the largest angular gap between consecutive attach points must
    be < 180 deg.

    This is GEOMETRY, not control, and the cables are bolted to the payload, so no
    controller can move the polygon. Removing one cable from a symmetric n-ring leaves a
    largest gap of 4*pi/n, so a ring survives one detach only for n >= 5:

        3 -> 2   360 deg   outside   (cannot hang level at all)
        4 -> 3   180 deg   ON THE BOUNDARY   (marginal)
        5 -> 4   144 deg   inside
        6 -> 5   120 deg   inside

    which is why a 4->3 detach parks the load at a persistent tilt however well it is
    flown. `margin_m` is the distance from the centre of mass to the nearest polygon
    edge, negative when outside."""
    p = np.asarray([[float(r[0]), float(r[1])] for r in rho], float)
    if p.shape[0] < 3:
        return -float(np.max(np.linalg.norm(p, axis=1))), 360.0, False
    ang = np.sort(np.mod(np.arctan2(p[:, 1], p[:, 0]), 2 * np.pi))
    gap = float(np.degrees(np.max(np.diff(np.r_[ang, ang[0] + 2 * np.pi]))))
    order = np.argsort(np.mod(np.arctan2(p[:, 1], p[:, 0]), 2 * np.pi))
    q = p[order]
    d = []
    for i in range(q.shape[0]):
        a, b = q[i], q[(i + 1) % q.shape[0]]
        e = b - a
        n = np.array([-e[1], e[0]])
        ln = float(np.linalg.norm(n))
        if ln < 1e-12:
            continue
        d.append(float(np.dot(n / ln, -a)))
    ok = gap < 180.0 - 1e-9
    margin = float(np.min(np.abs(d))) if d else 0.0
    return (margin if ok else -margin), gap, ok


def azimuth_deg(drone_xy, payload_xy):
    """Bearing of each drone from the payload, degrees in [0, 360).

    The formation-reconfiguration view: even spacing is three drones 120 deg apart, and
    a newcomer bunching against an incumbent shows up here before it shows up anywhere
    else."""
    d = np.asarray(drone_xy, float) - np.asarray(payload_xy, float)
    return np.degrees(np.arctan2(d[:, 1], d[:, 0])) % 360.0


def stage_split_run(t, data, n, mask=None):
    """`stage_split` applied to a loaded run table: desired -> reference centroid ->
    actual centroid -> payload. Returns [] if the run has no payload reference.

    `mask` should be the sweep window -- the diagnostic is about a trajectory, and the
    hover either side of it has no radius and no phase."""
    if not has(data, 'payload_ref_x'):
        return []
    keep = slice(None) if mask is None else mask
    desired = np.column_stack([data['payload_ref_x'], data['payload_ref_y']])
    return stage_split(np.asarray(t, float)[keep], desired[keep],
                       centroid(data, n, 'ref_')[keep, :2],
                       centroid(data, n)[keep, :2],
                       np.column_stack([data['payload_x'],
                                        data['payload_y']])[keep])


EVENT_PRIORITY = ('WELD', 'REWELD', 'DETACH', 'MAGNET', 'TAKEOFF')


def primary_event(events):
    """The event the steady window and all event-relative metrics key off.

    A run has several events; only one of them is the thing under test. The weld is the
    subject of an attach run, the detach of a detach run, and a plain carry run has
    neither -- in which case `steady_window` falls back to the whole record."""
    for name in EVENT_PRIORITY:
        if name in events and np.isfinite(events[name]):
            return name, float(events[name])
    return None, None


def _nanmax(x):
    x = np.asarray(x, float)
    return float(np.nanmax(x)) if np.any(np.isfinite(x)) else math.nan


def _nanmean(x):
    x = np.asarray(x, float)
    return float(np.nanmean(x)) if np.any(np.isfinite(x)) else math.nan


def summarise_run(run_path):
    """Every metric this file defines, applied to one run -- SIL bench or Gazebo.

    The thing a harness calls to produce metrics.json (§4.1), and the thing
    compare_runs.py calls to build its table, so a number in a comparison and the same
    number in the run's own directory cannot disagree.

    Metrics whose input columns are all-NaN for this source are OMITTED, not reported
    as nan: a missing key says "this harness does not observe that", where a nan reads
    as "it was measured and came out undefined"."""
    run = load_run(run_path)
    d = run['data']
    if run['source'] not in ('sil', 'gazebo') or not d:
        return {'error': f'no wide-schema log in {run_path} (source={run["source"]})'}
    n = n_drones(d)
    t = np.asarray(d['t'], float)
    ev = run['events']
    ev_name, t_ev = primary_event(ev)
    t_land = ev.get('LAND') if isinstance(ev, dict) else None
    t_land = float(t_land) if t_land is not None and np.isfinite(t_land) else None
    mask = steady_window(t, t_ev, t_end=t_land)

    payload = np.column_stack([d['payload_x'], d['payload_y'], d['payload_z']])
    quat = np.column_stack([d['payload_qw'], d['payload_qx'],
                            d['payload_qy'], d['payload_qz']])
    peak_tilt, settled_tilt = tilt_peak_settled(t, quat, t_ev, t_end=t_land)

    out = {
        'run': os.path.basename(run_path),
        'run_id': run['manifest'].get('run_id'),
        'source': run['source'],
        'kind': run['manifest'].get('kind'),
        'events': ev,
        'primary_event': ev_name,
        'primary_event_s': t_ev,
        'steady_window_s': [float(t[mask][0]), float(t[mask][-1])] if np.any(mask) else None,
        'n_drones': n,
        'n_rows': int(t.size),
        'duration_s': float(t[-1] - t[0]) if t.size else 0.0,
        'payload_tilt_peak_deg': peak_tilt,
        'payload_tilt_settled_deg': settled_tilt,
        'payload_z_settled_m': _nanmean(payload[mask, 2]) if np.any(mask) else math.nan,
        'control_effort': control_effort(stack(d, 'd{i}_thr', n), mask),
    }
    if all(has(d, f'd{i}_tension') for i in range(n)):
        out['tension_share'] = tension_share(stack(d, 'd{i}_tension', n), mask)
    if has(d, 'payload_ref_x'):
        des = np.column_stack([d['payload_ref_x'], d['payload_ref_y'],
                               d['payload_ref_z']])
        out['payload_rmse_m'] = payload_rmse(payload, des, mask)
        # Trajectory metrics come from the SWEEP window, never the whole record --
        # see sweep_window. Reported with the window so a number can be checked
        # against the manoeuvre it claims to describe.
        sweep = sweep_window(t, des[:, :2])
        if np.any(sweep):
            out['sweep_window_s'] = [float(t[sweep][0]), float(t[sweep][-1])]
            out['payload_rmse_sweep_m'] = payload_rmse(payload, des, sweep)
            out['payload_radius_ratio'] = radius_ratio(payload[:, :2], des[:, :2],
                                                       sweep)
            out['payload_phase_lag_s'] = phase_lag_s(t[sweep], des[sweep, 0],
                                                     payload[sweep, 0])
            out['stage_split'] = stage_split_run(t, d, n, sweep)
    per_drone = []
    for i in range(n):
        err = d[f'd{i}_track_err']
        rec = {'drone': i,
               'peak_track_err_m': _nanmax(err),
               'settled_track_err_m': _nanmean(err[mask]) if np.any(mask) else math.nan}
        if has(d, f'd{i}_acm'):
            rec['peak_cable_accel'] = _nanmax(d[f'd{i}_acm'])
        if has(d, f'd{i}_elev_deg'):
            rec['settled_elev_deg'] = (_nanmean(d[f'd{i}_elev_deg'][mask])
                                       if np.any(mask) else math.nan)
        if t_ev is not None:
            rec['event_peak_err_m'] = event_peak_error(t, err, t_ev)
            rec['event_settling_s'] = event_settling_time(t, err, t_ev, band=0.10)
        per_drone.append(rec)
    out['per_drone'] = per_drone
    ev_list = read_event_list(run_path)
    if any(e[1].startswith('PHASE:') or e[1] == 'PARTNER_RELEASE'
           or (e[1] == 'LAUNCH' and 'partner_mission' in e[2]) for e in ev_list):
        text = ''.join(_read_text(f) for f in sorted(glob.glob(
            os.path.join(run_path, 'logs', 'extra_launch_*.log'))))
        out['m1'] = m1_stage_metrics(t, d, ev_list,
                                     phase_log=text if 'Planner phase:' in text else None)
        # validity: a start-attached partner must be folded in at its plate before TAKEOFF;
        # a late mocap made R0676 fold drone 3 in after TAKEOFF at the clamped radius (VOID)
        try:
            man = json.load(open(os.path.join(run_path, 'manifest.json')))
            az = [float(v) for v in str((man.get('launch_args') or {}).get('attach_azimuths_deg', '')).split(',') if v]
            if az:
                out['m1']['clearance'] = partner_clearance(t, d, ev_list, az)
        except Exception:
            pass
        launch = _read_text(os.path.join(run_path, 'logs', 'launch.log'))
        if 'welded at start' in launch or 'no mocap yet' in launch:
            out['m1']['start_weld_ok'] = 'welded at start: plate at' in launch
    elif ev_list:
        out['event_windows'] = event_windows(t, mask_released(d, ev_list, n - 1), ev_list)
    # the manager grounded the fleet before it flew (pre-TAKEOFF gate, failed ARM): such a run
    # reads as "never lifted" in the numbers, which is how R0749 was misread; mark it VOID
    for name in ('launch.log', 'console.log'):
        text = _read_text(os.path.join(run_path, 'logs', name)) or _read_text(
            os.path.join(run_path, name))
        if text:
            out['fleet_grounded'] = [g for g in FLEET_GROUNDED if g in text]
            break
    try:
        teth = tethered_run_metrics(run_path)
    except Exception as e:                                  # noqa: BLE001
        teth = {'error': f'{type(e).__name__}: {e}'}
    if teth is not None:
        out['tethered'] = teth
    return out


# fleet-manager lines that end a run before it flies (fleet_manager_node.py)
FLEET_GROUNDED = ('disarmed before TAKEOFF', 'TAKEOFF REFUSED', 'ARM FAILED', 'ARM REFUSED')


def summarise_sil_run(run_path):
    """Back-compatible alias; `summarise_run` handles both harnesses."""
    return summarise_run(run_path)


# ── M1: the partner's pickup / drop / rejoin, stage by stage ────────────────

M1_STAGES = ('TAKEOFF', 'LIFTED', 'DETACH', 'RELEASED', 'PARTNER_RELEASE', 'PICKUP',
             'DROP', 'HANDOFF', 'PARTNER_HANDOFF', 'REWELD', 'FOLD_IN', 'LAND', 'LANDED')
NET_RHO_M = 0.20                # ball inside the net
NET_Z_REL_M = (-0.05, 0.12)     # ball height in the ring frame while in the net
FOOTPRINT_RHO_M = 0.28          # ball anywhere over the ring


def ball_in_ring_frame(data):
    """(rho, z_rel) of the ball in the ring frame, R^T (p_ball - p_ring), per sample."""
    q = np.column_stack([data[f'payload_q{a}'] for a in 'wxyz']).astype(float)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    R = np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1),
    ], 1)                                                   # (T, 3, 3), body -> world
    dp = np.column_stack([np.asarray(data[f'ball_{a}'], float)
                          - np.asarray(data[f'payload_{a}'], float) for a in 'xyz'])
    rel = np.einsum('tji,tj->ti', R, dp)                    # R^T dp
    return np.hypot(rel[:, 0], rel[:, 1]), rel[:, 2]


def ball_state(rho, z_rel, rho_only=False):
    in_net = rho <= NET_RHO_M
    if not rho_only:
        in_net = in_net and NET_Z_REL_M[0] <= z_rel <= NET_Z_REL_M[1]
    return {'rho_m': float(rho), 'z_rel_m': float(z_rel), 'in_net': bool(in_net),
            'on_footprint': bool(rho <= FOOTPRINT_RHO_M)}


def parse_phase_log(text):
    """[(from, to)] from his planner's 'Planner phase: A -> B.' lines."""
    return re.findall(r'Planner phase: ([A-Z0-9_]+) -> ([A-Z0-9_]+)', text)


CLEAR_BODY_M = 0.30     # partner drone body to a carrier body (props ~0.12 m radius each + margin)
CLEAR_ROD_M = 0.15      # partner drone body to a carrier's rod segment (drone -> ring plate)


def _seg_dist(p, a, b):
    ab = b - a
    s = np.clip(np.einsum('ij,ij->i', p - a, ab) / np.maximum(np.einsum('ij,ij->i', ab, ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(p - (a + ab * s[:, None]), axis=1)


def partner_clearance(t, data, events, azimuths_deg, radius=0.25, attach_z=0.025, partner=None):
    """Minimum distance of the partner drone (default: the last) to each carrier body and to each
    carrier's rod (carrier drone -> its ring plate) over the step-out (DETACH -> PARTNER_RELEASE),
    the partner mission (PARTNER_RELEASE -> HANDOFF) and our rejoin (HANDOFF -> REWELD). Pass/fail
    against CLEAR_BODY_M / CLEAR_ROD_M (Tejen's review: a justified safe operating region)."""
    n = n_drones(data)
    k = n - 1 if partner is None else partner
    ev = lambda name: next((float(e[0]) for e in events if e[1] == name and np.isfinite(e[0])), np.nan)
    wins = {'step_out': (ev('DETACH'), ev('PARTNER_RELEASE')), 'mission': (ev('PARTNER_RELEASE'), ev('HANDOFF')),
            'rejoin': (ev('HANDOFF'), ev('REWELD'))}
    P = np.stack([data[f'd{k}_x'], data[f'd{k}_y'], data[f'd{k}_z']], 1)
    L = np.stack([data['payload_x'], data['payload_y'], data['payload_z']], 1)
    q = np.stack([data['payload_qw'], data['payload_qx'], data['payload_qy'], data['payload_qz']], 1)
    w, x, y, z = q.T
    R = np.stack([np.stack([1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], -1),
                  np.stack([2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)], -1),
                  np.stack([2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)], -1)], 1)
    out = {}
    for name, (a, b) in wins.items():
        m = (t >= a) & (t <= b) if np.isfinite(a) and np.isfinite(b) else np.zeros_like(t, bool)
        if not m.any():
            continue
        body = rod = np.inf
        for i, az in zip(range(n - 1), azimuths_deg):
            C = np.stack([data[f'd{i}_x'], data[f'd{i}_y'], data[f'd{i}_z']], 1)
            rho = np.array([radius * np.cos(np.radians(az)), radius * np.sin(np.radians(az)), attach_z])
            plate = L + np.einsum('nij,j->ni', R, rho)
            ok = m & np.all(np.isfinite(P), 1) & np.all(np.isfinite(C), 1)
            if ok.any():
                body = min(body, float(np.min(np.linalg.norm(P[ok] - C[ok], axis=1))))
                rod = min(rod, float(np.min(_seg_dist(P[ok], C[ok], plate[ok]))))
        out[name] = {'min_body_m': round(body, 3), 'min_rod_m': round(rod, 3),
                     'ok': bool(body >= CLEAR_BODY_M and rod >= CLEAR_ROD_M)}
    return out


def m1_stage_metrics(t, data, events, phase_log=None, partner=None):
    """The M1 run scored stage by stage, from run.csv and the full event list.

    The partner (default: the last drone) is masked from RELEASED to REWELD. Reattach
    retries are re-entries of APPROACH_ABOVE_TARGET before the first HANDOFF (after it our
    tracker flies the descent and his phase no longer matters). `phase_log` is his
    planner's launch log text; its transitions are cross-checked against the PHASE events,
    which the runner only sees if the phase lasted long enough to be published."""
    t = np.asarray(t, float)
    n = len(_drone_ids(data))
    partner = n - 1 if partner is None else partner

    def first(name, after=-math.inf):
        return next((float(e[0]) for e in events if e[1] == name and np.isfinite(e[0])
                     and e[0] > after), math.nan)

    stage_t = {s: first(s) for s in M1_STAGES}
    out = {'stage_t': stage_t,
           'missing': [s for s in M1_STAGES if not np.isfinite(stage_t[s])],
           'takeoff_to_landed_s': stage_t['LANDED'] - stage_t['TAKEOFF']}

    masked = mask_released(data, events, partner)
    out['windows'] = event_windows(t, masked, events, names=('DETACH', 'REWELD', 'DROP'))

    phases = [(float(e[0]), e[1][len('PHASE:'):]) for e in events if e[1].startswith('PHASE:')]
    t_hand = stage_t['HANDOFF']
    approaches = sum(1 for tp, p in phases if p == 'APPROACH_ABOVE_TARGET'
                     and not (tp >= t_hand))
    out['reattach_retries'] = max(0, approaches - 1)
    out['n_phase_events'] = len(phases)
    if phase_log is not None:
        logged = parse_phase_log(phase_log)
        seen = {p for _, p in phases}
        out['n_phase_log_transitions'] = len(logged)
        out['phases_missed'] = sorted({b for _, b in logged} - seen)

    if has(data, 'ball_x'):
        rho, z_rel = ball_in_ring_frame(data)
        at = {}
        for name in ('REWELD', 'LAND', 'LANDED'):
            te = stage_t[name]
            if np.isfinite(te) and np.any(t >= te):
                k = int(np.argmax(t >= te))
                at[name] = ball_state(rho[k], z_rel[k], rho_only=name == 'LANDED')
        out['ball_at'] = at
        t_drop, t_land = stage_t['DROP'], stage_t['LAND']
        if np.isfinite(t_drop):
            hold = (t >= t_drop + 2.0) & (t < (t_land if np.isfinite(t_land) else math.inf))
            good = hold & np.isfinite(rho)
            if np.any(good):
                in_net = (rho <= NET_RHO_M) & (z_rel >= NET_Z_REL_M[0]) & (z_rel <= NET_Z_REL_M[1])
                out['ball_in_net_fraction'] = float(np.mean(in_net[good]))
    return out


# ── M2: the four-drone hand-over run by tools/sim_test/drive_m2_handover.py ──

M2_TOUCHDOWN_Z = 0.105        # ring back on its hangers/floor (it rests at 0.100)
M2_DRONE_TILT_MAX_DEG = 25.0


def _read_text(path):
    try:
        with open(path, errors='replace') as fh:
            return fh.read()
    except OSError:
        return ''


def _stamp(path):
    s = run_logs.stamp_of(path)
    if not s:
        return None
    return datetime.datetime.strptime(s, '%Y%m%d_%H%M%S').timestamp()


def m2_logs(run_dir):
    """(dissipative log.csv, {drone: tracker log.csv}, runtime.log) of an M2 driver run.

    With MDC_RUN_DIR set by the driver our nodes log under <run>/logs (run_logs.py).
    Older runs (T0015) logged to the results root: the dissipative node prints its log
    dir ('params.json -> ...') into ours*.log, and the trackers are its siblings started
    within 5 min of it. runtime.log is in his evidence dir, named in runner.log."""
    ours = ''.join(_read_text(os.path.join(run_dir, f))
                   for f in sorted(os.listdir(run_dir)) if f.startswith('ours'))
    diss = run_logs.node_csvs(run_dir, 'dissipative_planner')
    diss = diss[-1] if diss else None
    if diss is None:
        m = re.findall(r'params\.json -> (\S+)', ours)
        if m and os.path.exists(os.path.join(m[-1], 'log.csv')):
            diss = os.path.join(m[-1], 'log.csv')
    trackers = {}
    if diss is not None:
        t0 = _stamp(diss)
        for f in run_logs.node_csvs(run_logs.run_root(diss), 'tracker'):
            i = run_logs.drone_of(f)
            ts = _stamp(f)
            if t0 is None or ts is None or abs(ts - t0) > 300.0:
                continue
            if i not in trackers or abs(ts - t0) < abs(_stamp(trackers[i]) - t0):
                trackers[i] = f
    m = re.search(r'Evidence:\s+(\S+)', _read_text(os.path.join(run_dir, 'runner.log')))
    runtime = os.path.join(run_dir, m.group(1), 'runtime.log') if m else None
    return diss, dict(sorted(trackers.items())), runtime, ours


def summarise_m2(run_dir, target_z=None):
    """M2 hand-over metrics for one driver run directory (-> metrics.json via the CLI).

    load_z is judged from 5 s after the lift first reaches the target to LAND; ring tilt
    per segment (lift: takeover -> target, hover: -> LAND, descent: -> touchdown at
    load_z <= M2_TOUCHDOWN_Z); drone tilt over takeover -> touchdown."""
    diss, trackers, runtime, ours = m2_logs(run_dir)
    out = {'run': os.path.basename(os.path.normpath(run_dir)), 'dissipative_log': diss,
           'tracker_logs': trackers, 'runtime_log': runtime}
    driver = _read_text(os.path.join(run_dir, 'driver.log'))
    m = re.findall(r'landed=(True|False)', driver)
    out['landed'] = (m[-1] == 'True') if m else None

    rt = _read_text(runtime) if runtime else ''
    states = re.findall(r'M2D state=\S+ .*?attached=\[([^\]]*)\]', rt)
    out['join_attached'] = (max(len(re.findall(r"'(drone_\d+)'", s)) for s in states)
                            if states else None)
    # one ERROR line per fallback install; 'fallback=true' is a periodic status line
    installs = re.findall(r'C1F\.4 (?:execution certificate invalid \((\S*)'
                          r'|failed to install execution fallback)', rt)
    out['fallback_count'] = len(installs)
    out['tube_violation_count'] = sum(r == 'TRACKING_TUBE_VIOLATION' for r in installs)
    out['envelope_faults'] = ours.count('ENVELOPE FAULT')
    ages = [float(a) for a in re.findall(r'reference ([0-9.]+) s old', ours)]
    out['max_ref_age_s'] = max(ages) if ages else None

    if diss is None:
        out['error'] = 'no dissipative log found'
        return out
    d = read_csv(diss)
    t = np.asarray(d['sim_time'], float)
    if target_z is None:
        try:
            with open(os.path.join(os.path.dirname(diss), 'params.json')) as fh:
                target_z = float(_find_key(json.load(fh), 'target_z'))
        except (OSError, TypeError, ValueError):
            target_z = _nanmax(d['z_tgt'])
    out['target_z'] = target_z
    z, z_tgt, tilt = (np.asarray(d[k], float) for k in ('load_z', 'z_tgt', 'tilt_deg'))
    land = np.asarray(d['land'], float) > 0.5
    phase = np.asarray(d['phase']).astype(str)

    def first_t(mask, after=-math.inf):
        k = np.where(mask & (t >= after))[0]
        return float(t[k[0]]) if k.size else math.nan

    t_take = first_t(phase != 'creep')
    t_tgt = first_t(np.isfinite(z_tgt) & (z_tgt >= target_z - 1e-4), t_take)
    t_land = first_t(land)
    t_down = first_t(z <= M2_TOUCHDOWN_Z, t_land)
    out['t'] = {'takeover': t_take, 'target': t_tgt, 'land': t_land, 'touchdown': t_down}
    w = (t >= t_tgt + SETTLE_S) & (t < t_land)
    out['load_z_window_s'] = [t_tgt + SETTLE_S, t_land]
    out['load_z_mean_m'] = _nanmean(z[w])
    out['load_z_min_m'] = float(np.nanmin(z[w])) if np.any(w) else math.nan
    out['load_z_max_m'] = _nanmax(z[w])
    end = t_down if np.isfinite(t_down) else math.inf
    seg = {}
    for name, a, b in (('lift', t_take, t_tgt), ('hover', t_tgt, t_land),
                       ('descent', t_land, end)):
        s = (t >= a) & (t < b)
        seg[name] = {'peak_deg': _nanmax(tilt[s]), 'mean_deg': _nanmean(tilt[s])}
    out['ring_tilt'] = seg
    out['ring_tilt_peak_deg'] = _nanmax(tilt[(t >= t_take) & (t < end)])

    peaks = {}
    for i, f in trackers.items():
        c = read_csv(f)
        if 'sim_time' not in c:          # header only: the tracker never ran (abort before the hand-over)
            continue
        tc = np.asarray(c['sim_time'], float)
        q = np.column_stack([c[f'pose_q{a}'] for a in 'wxyz'])
        s = (tc >= t_take) & (tc <= end)
        peaks[i] = _nanmax(load_tilt_deg(q)[s])
    out['drone_tilt_peak_deg'] = peaks
    worst = _nanmax(list(peaks.values())) if peaks else math.nan
    out['drone_tilt_ok'] = bool(worst <= M2_DRONE_TILT_MAX_DEG) if np.isfinite(worst) else None
    return out


def _find_key(obj, key):
    """First value of `key` anywhere in a nested params dump."""
    if isinstance(obj, dict):
        if key in obj:
            return obj[key]
        for v in obj.values():
            r = _find_key(v, key)
            if r is not None:
                return r
    elif isinstance(obj, list):
        for v in obj:
            r = _find_key(v, key)
            if r is not None:
                return r
    return None


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description='metrics.json for a run directory')
    ap.add_argument('kind', choices=('run', 'm2', 'tethered'),
                    help='run: a run_experiment/SIL run dir; m2: a drive_m2_handover.py dir; '
                         'tethered: a rig log folder (printed, not written)')
    ap.add_argument('run_dir')
    ap.add_argument('--log', help='tethered: console log with the planner\'s solve-status lines')
    ap.add_argument('--hold-window', type=float, default=HOLD_WINDOW_S)
    a = ap.parse_args(argv)
    if a.kind == 'tethered':
        text = _read_text(a.log) if a.log else None
        print(json.dumps(tethered_run_metrics(a.run_dir, a.hold_window, text), indent=2, default=str))
        return
    out = summarise_m2(a.run_dir) if a.kind == 'm2' else summarise_run(a.run_dir)
    path = os.path.join(a.run_dir, 'metrics.json')
    with open(path, 'w') as fh:
        json.dump(out, fh, indent=2, default=str)
    print(path)


if __name__ == '__main__':
    main()
