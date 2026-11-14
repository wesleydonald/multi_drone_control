#!/usr/bin/env python3
"""Every rig pickup try: how the magnet sat on the object before the lift, and whether the object rose with it.

Reads extract/<day>_<flight>/ (extract.py) and the flight's log. One row per DESCEND_TO_PICKUP -> tries.csv.
usage: analyze.py
Conventions
  gap      tip marker z - object marker z - contact height (the height of the tip marker over the object's marker
           with the magnet seated flat, measured props off); 0 = seated as calibrated, + = above
  seat     the 0.5 s before LIFT_OBJECT (a lift try); for a try without a lift, the 0.5 s around its lowest gap
  tilt     angle between the magnet face normal and the object's up axis (flat seat = ~0). The face normal in body
           24's frame is calibrated from the samples where the magnet carries the object (it then hangs flat on the
           steel), per body-24 definition; the object's up axis from the object at rest before the try
  slack    free-hang body-to-tip height (before the descent) - body-to-tip height in the seat window; > 0: the drone
           is lower than the hanging magnet would put it (cable slack, the magnet resting on the object)
  ref depth  the reference's tip (reference body z - free-hang body-to-tip height) relative to the seated tip height
  held     the object rose >= 3 cm with the tip in the lift"""
import os, re, gzip, json
import numpy as np, pandas as pd
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = '/home/wesley/multi_drone_control/results/rig'
PICK = ('DESCEND_TO_PICKUP', 'MAGNET_ATTACH_WAIT', 'LIFT_OBJECT')
# contact height (m) the try is judged against, the object, and body 24's definition (A: to 29 Oct, B: 30 Oct-4 Nov
# morning, C: redefined 4 Nov ~17:25). ch_src says where the height comes from.
FLIGHTS = {
    '2026-10-14/t5_4': (0.0379, 'puck (his commissioning puck)', 'A', 'props-off 14 Oct (RIG-1014-puck 3.79 cm; planner used 3.45)'),
    '2026-10-14/t5_5': (0.0379, 'puck (his commissioning puck)', 'A', 'props-off 14 Oct (RIG-1014-puck 3.79 cm; planner used 3.45)'),
    '2026-10-14/t6': (0.0379, 'puck (his commissioning puck)', 'A', 'props-off 14 Oct (RIG-1014-puck 3.79 cm; planner used 3.45)'),
    '2026-10-29/k1_8': (0.0345, 'object 29 Oct (not recorded)', 'A?', 'his default 3.45 cm (not measured for this object)'),
    '2026-11-04/k1m': (0.018, 'big object 100x100x10 mm (0.15 kg)', 'B', 'typed 0.018 (measured 21 Oct with QUAD1 marker 21)'),
    '2026-11-04/k1m_2': (0.018, 'big object 100x100x10 mm (0.15 kg)', 'B', 'typed 0.018 (measured 21 Oct with QUAD1 marker 21)'),
    '2026-11-04/k1m_3': (0.018, 'big object 100x100x10 mm (0.15 kg)', 'B', 'typed 0.018 (measured 21 Oct with QUAD1 marker 21)'),
    '2026-11-04/k1m_4': (0.018, 'big object 100x100x10 mm (0.15 kg)', 'B', 'typed 0.018 (measured 21 Oct with QUAD1 marker 21)'),
    '2026-11-05/k1d': (0.038, 'small object, bare (54 mm disc, 60 g)', 'C', 'typed 0.038 (11 Nov re-check 37.6 mm)'),
    '2026-11-05/fm9y_2': (0.038, 'small object, bare (54 mm disc, 60 g)', 'C', 'typed 0.038 (11 Nov re-check 37.6 mm)'),
    '2026-11-05/k1o_2': (0.038, 'small object, bare (54 mm disc, 60 g)', 'C', 'typed 0.038; body 6 redefined off-centre in Motive (RIG-1105-k1o_2)'),
    '2026-11-05/k1ob_2': (0.018, 'big object 100x100x10 mm (0.15 kg)', 'C', 'typed 0.018 (measured 21 Oct with QUAD1 marker 21)'),
    '2026-11-06/q4p': (0.038, 'small object, bare (54 mm disc, 60 g)', 'C', 'overlay 0.0155 + 0.0225 = 0.038'),
    '2026-11-11/fm9c': (0.038, 'small object, bare (54 mm disc, 60 g)', 'C', 'typed 0.038 (11 Nov 08:30 re-check 37.6 mm)'),
    '2026-11-11/fm9c_3': (0.038, 'small object, bare (54 mm disc, 60 g)', 'C', 'typed 0.038 (11 Nov 08:30 re-check 37.6 mm)'),
    '2026-11-11/fm9c_4': (0.018, 'big object 100x100x10 mm (0.15 kg)', 'C', 'typed 0.018 (measured 21 Oct with QUAD1 marker 21)'),
    '2026-11-11/fm9c_6': (0.018, 'big object 100x100x10 mm (0.15 kg)', 'C', 'typed 0.018 (measured 21 Oct with QUAD1 marker 21)'),
    '2026-11-11/k1zo': (0.038, 'small object, bare (54 mm disc, 60 g)', 'C', 'typed 0.038 (11 Nov 08:30 re-check 37.6 mm)'),
    '2026-11-11/k1zo_3': (0.038, 'small object, bare (54 mm disc, 60 g)', 'C', 'typed 0.038 (11 Nov 08:30 re-check 37.6 mm)'),
    '2026-11-12/fm9_2': (0.011, 'small object + funnel', 'C', 'typed 0.011 (11 Nov 11.4 / 11.0 mm, 12 Nov 10.7 mm)'),
    '2026-11-12/fm9_5': (0.011, 'small object + funnel', 'C', 'typed 0.011; body 6 redefined 25 mm lower during attempt 1'),
    '2026-11-12/fm9c_3': (0.011, 'small object + funnel', 'C', 'typed 0.011 (11 Nov 11.4 / 11.0 mm, 12 Nov 10.7 mm)'),
    '2026-11-12/fm9c_5': (0.011, 'small object + funnel', 'C', 'typed 0.011 (11 Nov 11.4 / 11.0 mm, 12 Nov 10.7 mm)'),
    '2026-11-12/q4d_3': (0.011, 'small object + funnel', 'C', 'overlay -0.0115 + 0.0225 = 0.011'),
    '2026-11-12/q4d_4': (0.011, 'small object + funnel', 'C', 'overlay -0.0115 + 0.0225 = 0.011'),
    '2026-11-12/q4d_7': (0.011, 'small object + funnel', 'C', 'overlay -0.0115 + 0.0225 = 0.011'),
    '2026-11-13/fm9c_2': (0.011, 'small object + funnel', 'C', 'typed 0.011 (11 Nov 11.4 / 11.0 mm, 12 Nov 10.7 mm)'),
}
SEAT_S = 0.5


def failures(log):
    """his 'M1 pickup attempt N failed: <why>' and 'ASSUMED ATTACH...' lines, with their times"""
    out = []
    for line in open(log, errors='replace'):
        m = re.search(r'\[(\d+\.\d+)\] \[[^\]]+\]: (M1 pickup attempt \d+ failed: (.*?); magnet OFF|ASSUMED ATTACH[^:.]*)', line)
        if m:
            out.append((float(m[1]), (m[3] or m[2]).strip()))
    return out


def phases(log):
    ph, pts = [], {}
    for line in open(log, errors='replace'):
        m = re.search(r'\[(\d+\.\d+)\] \[online_join_planner\]: Planner phase: (\S+) -> (\S+)\. Reason: (.*)', line)
        if m:
            ph.append((float(m[1]), m[2], m[3], m[4].strip()))
        m = re.search(r'contact point ([+-]\d\.\d+) m over its marker.*?, ([+-]\d\.\d+) / ([+-]\d\.\d+) m sideways', line)
        if m:
            pts = {'px': float(m[2]), 'py': float(m[3])}
    return pd.DataFrame(ph, columns=['t', 'a', 'b', 'why']), pts


def load(d, k):
    return pd.read_csv(os.path.join(d, k + '.csv.gz'))


def interp(df, cols, t):
    return np.c_[[np.interp(t, df.t.values, df[c].values) for c in cols]].T


def quat_interp(df, t):
    # nearest sample (the poses come at ~90-240 Hz; no slerp needed for medians)
    i = np.clip(np.searchsorted(df.t.values, t), 0, len(df) - 1)
    q = df[['qx', 'qy', 'qz', 'qw']].values[i]
    return R.from_quat(q)


def angle(a, b):
    c = np.sum(a * b, axis=1) / np.linalg.norm(a, axis=1) / np.linalg.norm(b, axis=1)
    return np.degrees(np.arccos(np.clip(c, -1, 1)))


def coil_series(d):
    c = load(d, 'coil')
    if not len(c):
        c = load(d, 'his_cmd')
        src = 'his /magnet/command'
    else:
        src = '/drone_3/magnet_coil'
    c = c[c.v.isin(['ON', 'OFF'])]
    ch = c[c.v.ne(c.v.shift())]
    return ch, src


def tries_of(spec):
    day, fl = spec.split('/')
    ch_m, objname, era, ch_src = FLIGHTS[spec]
    d = os.path.join(HERE, 'extract', f'{day}_{fl}')
    ph, pts = phases(f'{RIG}/{day}/{fl}.log')
    px, py = pts.get('px', 0.0), pts.get('py', 0.0)
    tip, obj, q4, ref = load(d, 'tip'), load(d, 'obj'), load(d, 'q4'), load(d, 'ref')
    coil, coil_src = coil_series(d)
    fails = failures(f'{RIG}/{day}/{fl}.log')
    rows, n = [], 0
    for i, r0 in ph[ph.b == 'DESCEND_TO_PICKUP'].iterrows():
        n += 1
        nxt = ph[ph.t > r0.t]
        out = nxt[~nxt.b.isin(PICK[1:]) | (nxt.a == 'LIFT_OBJECT')]
        # the try ends at the first phase that leaves the pickup chain
        t_x = out.t.iloc[0] if len(out) else tip.t.iloc[-1]
        end_row = out.iloc[0] if len(out) else None
        chain = nxt[nxt.t <= t_x]
        t_w = chain[chain.b == 'MAGNET_ATTACH_WAIT'].t.min() if (chain.b == 'MAGNET_ATTACH_WAIT').any() else np.nan
        t_l = chain[chain.b == 'LIFT_OBJECT'].t.min() if (chain.b == 'LIFT_OBJECT').any() else np.nan
        why_w = chain[chain.b == 'MAGNET_ATTACH_WAIT'].why.iloc[0] if np.isfinite(t_w) else ''
        lifted = np.isfinite(t_l)
        t_d = r0.t
        # free hang and the object at rest: the 2 s before the descent
        pre = (tip.t >= t_d - 2.0) & (tip.t <= t_d)
        Tp = tip.t[pre].values
        L_free = np.median(interp(q4, ['z'], Tp)[:, 0] - tip.z[pre].values)
        o_rest = obj[(obj.t >= t_d - 2.0) & (obj.t <= t_d)]
        R_rest = R.from_quat(o_rest[['qx', 'qy', 'qz', 'qw']].values)
        up_obj = R_rest.inv().apply([0, 0, 1.0]).mean(0)          # the object's up axis in its own frame
        up_obj /= np.linalg.norm(up_obj)
        obj_rest_sd = o_rest.z.std() * 1000
        # the descent to the lift (or to the try's end)
        t_bot_end = t_l if lifted else t_x
        w = (tip.t >= t_d) & (tip.t <= t_bot_end)
        T = tip.t[w].values
        P = tip.loc[w, ['x', 'y', 'z']].values
        O = interp(obj, ['x', 'y', 'z'], T)
        Ro = quat_interp(obj, T)
        Rt = R.from_quat(tip.loc[w, ['qx', 'qy', 'qz', 'qw']].values)
        cp = O + Ro.apply(np.r_[px, py, 0.0] * np.r_[1, 1, 0])  # contact point (typed sideways offset, yaw+tilt)
        gap = P[:, 2] - O[:, 2] - ch_m
        gap_s = pd.Series(gap).rolling(9, center=True, min_periods=1).median().values
        dxy = np.hypot(*(P[:, :2] - cp[:, :2]).T)
        zq = interp(q4, ['z'], T)[:, 0]
        slack = L_free - (zq - P[:, 2])
        rz = interp(ref, ['z'], T)[:, 0] if len(ref) else np.full(len(T), np.nan)
        refdepth = rz - L_free - O[:, 2] - ch_m
        # tip velocity (5-sample central difference)
        v = np.gradient(P, T, axis=0)
        v = pd.DataFrame(v).rolling(9, center=True, min_periods=1).mean().values
        # seat window
        if lifted:
            sw = T >= t_l - SEAT_S
        else:
            k = int(np.nanargmin(gap_s)) if len(gap_s) else 0
            sw = np.abs(T - T[k]) <= SEAT_S / 2
        first5 = T[np.argmax(gap_s < 0.005)] if (gap_s < 0.005).any() else np.nan
        # dwell: the last unbroken stretch within 5 mm of the seat, ending at the lift / try end
        inside = gap_s < 0.005
        dwell = 0.0
        if inside.size and inside[-1]:
            j = len(inside) - 1
            while j > 0 and inside[j - 1]:
                j -= 1
            dwell = T[-1] - T[j]
        # coil: the last ON before the lift (or the try's end); any OFF between it and the lift
        cb = coil[coil.t <= t_bot_end]
        on_rows = cb[cb.v == 'ON']
        t_on = on_rows.t.iloc[-1] if len(on_rows) and cb.v.iloc[-1] == 'ON' else np.nan
        row = dict(
            flight=f'{day[5:]}/{fl}', try_n=n, object=objname, body24=era, contact_h_mm=ch_m * 1000, ch_src=ch_src,
            t_descent=round(t_d, 2), descent_s=round(t_bot_end - t_d, 2),
            wait_reason=why_w[:70], lifted=lifted,
            end=(end_row.why[:80] if end_row is not None else 'end of log'),
            his_messages=' | '.join(w for t, w in fails if t_d <= t <= t_x + 0.5)[:160],
            seat_gap_mm=np.median(gap[sw]) * 1000 if sw.any() else np.nan,
            seat_gap_sd_mm=np.std(gap[sw]) * 1000 if sw.any() else np.nan,
            min_gap_mm=np.nanmin(gap_s) * 1000 if len(gap_s) else np.nan,
            seat_xy_mm=np.median(dxy[sw]) * 1000 if sw.any() else np.nan,
            # the same offset in the object's own axes (x, y), from the typed contact point
            seat_ox_mm=np.median(Ro[sw].inv().apply(P[sw] - O[sw])[:, 0] - px) * 1000 if sw.any() else np.nan,
            seat_oy_mm=np.median(Ro[sw].inv().apply(P[sw] - O[sw])[:, 1] - py) * 1000 if sw.any() else np.nan,
            obj_rest_z_mm=np.median(o_rest.z) * 1000,
            seat_tilt_raw_deg=np.median(angle(Rt[sw].apply([0, 0, 1.0]), Ro[sw].apply(up_obj))) if sw.any() else np.nan,
            seat_slack_mm=np.median(slack[sw]) * 1000 if sw.any() else np.nan,
            seat_speed_mms=np.median(np.hypot(v[sw, 0], v[sw, 1])) * 1000 if sw.any() else np.nan,
            seat_vz_mms=np.median(v[sw, 2]) * 1000 if sw.any() else np.nan,
            ref_depth_seat_mm=np.median(refdepth[sw]) * 1000 if sw.any() else np.nan,
            ref_depth_min_mm=np.nanmin(refdepth) * 1000 if len(refdepth) else np.nan,
            body_over_ref_mm=np.median(zq[sw] - rz[sw]) * 1000 if sw.any() else np.nan,
            dwell_5mm_s=round(dwell, 2),
            bottom_s=round(t_bot_end - first5, 2) if np.isfinite(first5) else np.nan,
            coil_on_s=round(t_bot_end - t_on, 2) if np.isfinite(t_on) else np.nan,
            coil_on_before_touch=bool(np.isfinite(t_on) and np.isfinite(first5) and t_on <= first5),
            coil_src=coil_src, L_free_m=L_free, obj_rest_sd_mm=obj_rest_sd,
        )
        # the face normal of body 24 seen from the object, for the tilt calibration (seat window samples)
        row['_seat_tipq'] = tip.loc[w, ['qx', 'qy', 'qz', 'qw']].values[sw].tolist() if sw.any() else []
        row['_seat_objup'] = Ro[sw].apply(up_obj).tolist() if sw.any() else []
        # the magnet hanging free before the descent (its own tilt on the cable)
        row['_pre_tipq'] = tip.loc[pre, ['qx', 'qy', 'qz', 'qw']].values[::3].tolist()
        row['_pre_objup'] = R.from_quat(o_rest[['qx', 'qy', 'qz', 'qw']].values[:1]).apply(up_obj).repeat(
            len(row['_pre_tipq']), axis=0).tolist()
        # the lift
        if lifted:
            lw = (tip.t >= t_l) & (tip.t <= min(t_x + 2.0, t_l + 12.0))
            TL = tip.t[lw].values
            PL = tip.loc[lw, ['x', 'y', 'z']].values
            OL = interp(obj, ['x', 'y', 'z'], TL)
            o0 = np.median(O[sw, 2]); p0 = np.median(P[sw, 2])
            orise = OL[:, 2] - o0; prise = PL[:, 2] - p0
            row['obj_rise_cm'] = orise.max() * 100
            row['tip_rise_cm'] = prise.max() * 100
            held = orise.max() >= 0.03
            row['held'] = held
            # carried: samples with the object 5 cm up; hang = tip - object height there (the in-flight seat)
            car = orise > 0.05
            if car.any():
                RtL = R.from_quat(tip.loc[lw, ['qx', 'qy', 'qz', 'qw']].values[car])
                RoL = quat_interp(obj, TL[car])
                # the tip in the object's frame, turned back to the object's rest attitude: the hanging object tilts,
                # and its marker sits off the contact point, so the plain height difference would move with the tilt
                rel = R_rest.mean().apply(RoL.inv().apply(PL[car] - OL[car]))
                row['hang_gap_mm'] = (np.median(rel[:, 2]) - ch_m) * 1000
                row['hang_gap_sd_mm'] = np.std(rel[:, 2]) * 1000
                row['hang_gap_world_mm'] = (np.median(PL[car, 2] - OL[car, 2]) - ch_m) * 1000
                cpo = R_rest.mean().apply(np.r_[px, py, 0.0])
                row['hang_xy_mm'] = np.median(np.hypot(rel[:, 0] - cpo[0], rel[:, 1] - cpo[1])) * 1000
                row['_hang_tipq'] = RtL.as_quat().tolist()
                row['_hang_objup'] = RoL.apply(up_obj).tolist()
                # fell later: the object 5 cm further below the tip than while carried (a separation, not a dip of
                # the whole lift: RIG-1104-k1m_3's lift bounced the object back to 1 cm off the floor, still held)
                kc = np.argmax(car)
                sep = PL[:, 2] - OL[:, 2]
                s_car = np.median(sep[kc:kc + 30])
                # only inside LIFT_OBJECT: after it the object may be dropped or set down on purpose (q4p's LAND)
                gone = np.where((sep[kc:] - s_car > 0.05) & (TL[kc:] <= t_x))[0]
                row['fell_after_s'] = round(TL[kc + gone[0]] - t_l, 2) if len(gone) else np.nan
            else:
                row['_hang_tipq'] = []; row['_hang_objup'] = []
            row['outcome'] = ('held' if held and not np.isfinite(row.get('fell_after_s', np.nan))
                              else 'held, fell later' if held else 'not held')
        else:
            row['held'] = np.nan
            row['outcome'] = 'no lift'
            row['_hang_tipq'] = []; row['_hang_objup'] = []
        rows.append(row)
    return rows


def main():
    rows = []
    for spec in FLIGHTS:
        rows += tries_of(spec)
    df = pd.DataFrame(rows)
    # body 24's face normal in its own frame, per definition: from the carried samples (the object hangs flat on the
    # magnet), else (A) none: the raw z axis
    face = {}
    for era, g in df.groupby('body24'):
        tq = sum(g._hang_tipq.tolist(), []); up = sum(g._hang_objup.tolist(), [])
        if len(tq) > 20:
            f = R.from_quat(np.array(tq)).inv().apply(np.array(up))
            fm = np.median(f, axis=0); fm /= np.linalg.norm(fm)
            face[era] = (fm, float(np.degrees(np.median(np.arccos(np.clip(f @ fm, -1, 1))))), len(tq))
    tilt, htilt, ftilt = [], [], []
    for _, r in df.iterrows():
        f = face.get(r.body24, (np.r_[0, 0, 1.0],))[0]
        for src, dst in (('_seat', tilt), ('_hang', htilt), ('_pre', ftilt)):
            tq, up = r[src + '_tipq'], r[src + '_objup']
            dst.append(np.median(angle(R.from_quat(np.array(tq)).apply(f), np.array(up))) if len(tq) else np.nan)
    df['seat_tilt_deg'] = tilt
    df['hang_tilt_deg'] = htilt
    df['free_tilt_deg'] = ftilt
    df['tilt_calibrated'] = df.body24.map(lambda e: e in face)
    df = df.drop(columns=[c for c in df.columns if c.startswith('_')])
    num = df.select_dtypes('float').columns
    df[num] = df[num].round(2)
    df.to_csv(os.path.join(HERE, 'tries.csv'), index=False, lineterminator='\n')
    with open(os.path.join(HERE, 'face_normal.json'), 'w') as f:
        json.dump({k: {'normal_in_body24': v[0].round(4).tolist(), 'spread_deg': round(v[1], 2), 'samples': v[2]}
                   for k, v in face.items()}, f, indent=1)
    print(json.dumps({k: (v[0].round(3).tolist(), round(v[1], 2), v[2]) for k, v in face.items()}))
    pd.set_option('display.width', 250); pd.set_option('display.max_columns', 40); pd.set_option('display.max_rows', 200)
    print(df[['flight', 'try_n', 'outcome', 'contact_h_mm', 'seat_gap_mm', 'seat_gap_sd_mm', 'min_gap_mm', 'seat_xy_mm',
              'free_tilt_deg', 'seat_tilt_deg', 'seat_slack_mm', 'body_over_ref_mm', 'seat_speed_mms',
              'ref_depth_min_mm', 'dwell_5mm_s', 'coil_on_s', 'obj_rise_cm', 'tip_rise_cm',
              'hang_gap_mm', 'hang_gap_world_mm', 'hang_tilt_deg', 'fell_after_s']].to_string())


if __name__ == '__main__':
    main()
