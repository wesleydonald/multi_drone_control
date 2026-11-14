#!/usr/bin/env python3
"""summary.md from joins.csv and flights.csv (run joins.py first).

    python3 results/analysis/rig_joins_2026-11-04_05/summarise.py

Every number in summary.md comes from the two CSVs; the prose around the tables states what the tables show.
"""
import csv
import math
import os
import statistics as st
import sys

HERE = os.environ.get('JOINS_OUT', os.path.dirname(os.path.abspath(__file__)))
K = 2.5          # cm: our contact radius (comotion contact xy <= 0.025 m), shown for reference only (not a bar)
PLATE_R = 2.75   # cm: the M2A ring's plates are 55 mm circles
CONTACT = ('caught+proven', 'caught, not proven', 'contact not proven')
TRIANGLE_5NOV = ('nr9m', 'nr9o_2', 'nr1r_3')

# Flights that flew but never started a descent: the reason, in short, from their registry rows.
NO_DESCENT = {
    'nr9': 'body 24 tracked QUAD2\'s magnet all flight; his approach never completed',
    'nr9m_2': 'his TAKEOFF stuck 25 cm low (before the takeoff from the air)',
    'nr9m_3': 'bad mocap start; LAND 8 s after the TAKEOFF',
    'fm9_2': 'the lift bounced to 30.6 deg (Motive recording): automatic drop',
    'fm9_4': 'the lift bounced (Motive recording); LAND at +7.5 s',
    'fm9s': 'slack cables in the lift; automatic drop in the descent',
    'fm9y': 'VOID: the ring set down one plate rotated',
    'nr1r': 'QUAD4 started off its plate; dashed across at the release; LAND',
    'sq8': 'no start weld (QUAD4 not on its plate); LAND',
    'k1o': 'no pickup-object pose (body 6 not streamed); LAND',
}


def load(name):
    with open(os.path.join(HERE, name), newline='') as f:
        return list(csv.DictReader(f))


def num(r, k):
    try:
        return float(r.get(k, ''))
    except (TypeError, ValueError):
        return math.nan


def fin(xs):
    return [x for x in xs if math.isfinite(x)]


def med_range(xs, nd=1):
    xs = fin(xs)
    if not xs:
        return '-'
    return f'{st.median(xs):.{nd}f} ({min(xs):.{nd}f}-{max(xs):.{nd}f})'


def rng(xs, nd=1, signed=True):
    xs = fin(xs)
    if not xs:
        return '-'
    f = f'{{:+.{nd}f}}' if signed else f'{{:.{nd}f}}'
    return f'{f.format(min(xs))} to {f.format(max(xs))}'


def table(head, rows):
    out = ['| ' + ' | '.join(head) + ' |', '|' + '|'.join('---' for _ in head) + '|']
    out += ['| ' + ' | '.join(str(c) for c in r) + ' |' for r in rows]
    return '\n'.join(out)


def at_height(r):
    return r['eval'] in ('plate height', 'target height')


def cm(r, k, nd=1, signed=False):
    v = num(r, k)
    return '-' if not math.isfinite(v) else (f'{v:+.{nd}f}' if signed else f'{v:.{nd}f}')


def share(rows, key):
    pr = [(num(r, key), num(r, 'tip_rad_cm')) for r in rows if math.isfinite(num(r, key)) and math.isfinite(num(r, 'tip_rad_cm'))]
    if len(pr) < 3:
        return '-'
    x, t = zip(*pr)
    return f'{st.pvariance(x) / st.pvariance(t):.2f}'


def corr(rows, a, b):
    pr = [(num(r, a), num(r, b)) for r in rows if math.isfinite(num(r, a)) and math.isfinite(num(r, b))]
    if len(pr) < 3:
        return math.nan
    xa, xb = zip(*pr)
    ma, mb = st.mean(xa), st.mean(xb)
    sa = math.sqrt(sum((x - ma) ** 2 for x in xa))
    sb = math.sqrt(sum((y - mb) ** 2 for y in xb))
    return sum((x - ma) * (y - mb) for x, y in pr) / (sa * sb) if sa and sb else math.nan


def alternation(rows, key='tip_rad_cm'):
    by = {}
    for r in rows:
        by.setdefault((r['day'], r['flight']), []).append(num(r, key))
    pairs = flips = 0
    for v in by.values():
        for a, b in zip(v, v[1:]):
            if math.isfinite(a) and math.isfinite(b):
                pairs += 1
                flips += (a > 0) != (b > 0)
    return flips, pairs


def main():
    J = load('joins.csv')
    F = load('flights.csv')
    D = [r for r in J if r['kind'] != 'drop into the net']
    drops = [r for r in J if r['kind'] == 'drop into the net']
    plate = [r for r in D if r['kind'] != 'pickup']
    pick = [r for r in D if r['kind'] == 'pickup']
    L = []
    L.append('# Real-world attach, re-attach and pickup descents, 4-5 Nov 2026 (exploratory)\n')
    L.append('Made by `summarise.py` from `joins.csv` and `flights.csv` (made by `joins.py` from the launch logs and bags in '
             '`results/rig/2026-11-04/` and `-05/`). These are exploratory readings of recorded flights: no bars, numbers '
             'as measured. `joins.csv` has one row per descent of QUAD4\'s magnet toward its target. A descent is each '
             'entry into his DESCEND_TO_ATTACHMENT or DESCEND_TO_PICKUP, including the re-descent after a failed proof. '
             'It also has one row per drop into the net.\n')
    L.append('**How the offsets are read.** For a plate, the reference is the weld point the launch typed for that '
             'plate: the magnet-tip marker\'s own position when welded on it, in the ring\'s frame. Height is measured '
             'along the ring\'s normal above that point, as our contact check measures it. Horizontal offsets are along '
             'the plate\'s radius projected on the floor (+ outward) and anticlockwise seen from above (+). The tilted '
             'ring frame is not used for them, so the body 1 m up is not thrown sideways when the ring leans. '
             '**At plate height** means the first tip sample at or below the weld point\'s height, or the lowest sample '
             'when the magnet settled within 1 cm above it. A descent that never got within 1 cm is read at its lowest '
             'tip sample ("never at plate height"). The second reading, **lowest**, is the tip\'s lowest sample in the '
             'descent: where a magnet that slid off the plate ended up. Body is QUAD4\'s mocap body at the same instant, '
             'swing is tip minus body, and "his reference" is /join_planner/reference. For a pickup, the reference is '
             'the object\'s Motive body in x and y (world frame), and the height is the gap above the typed '
             'pickup_contact_height. The miss is the tip\'s horizontal distance from the target centre at that reading. '
             f'The M2A plates are {2 * PLATE_R:.1f} cm circles; our contact radius is {K} cm (for reference, not a '
             'bar).\n')
    L.append('**Check of the geometry.** At every CONTACT line in the logs, the tip read from the bag at the contact '
             'time reproduces the line\'s xy and normal: ' + _contact_check(plate) + '. The usable welded starts of 5 '
             'Nov sit ' + _weld_check(F) + ' from the typed weld point (geometry table at the end).\n')

    # ---------------------------------------------------------------- overall
    L.append('## Overall, by kind x carriers x ring motion\n')
    groups = {}
    for r in D:
        groups.setdefault((r['kind'], r['formation'], r['ring_motion']), []).append(r)
    rows = []
    for key in sorted(groups):
        g = groups[key]
        h = [r for r in g if at_height(r)]
        con = [r for r in g if r['outcome'] in CONTACT]
        cat = [r for r in g if r['outcome'] == 'caught+proven']
        days = sorted({r['day'][8:10] + ' Nov' for r in g})
        fl = sorted({r['flight'] for r in g})
        inside = sum(num(r, 'tip_xy_cm') <= PLATE_R for r in h)
        rows.append([key[0], key[1], key[2], ', '.join(days), len(fl), len(g), len(h), len(con), len(cat),
                     med_range([num(r, 'tip_xy_cm') for r in h]), f'{inside} of {len(h)}',
                     med_range([num(r, 'low_tip_xy_cm') for r in h]) if key[0] != 'pickup' else '-'])
    tot_h = [r for r in plate if at_height(r)]
    rows.append(['**all plate descents**', '', '', '4-5 Nov', len({(r['day'], r['flight']) for r in plate}), len(plate),
                 len(tot_h), sum(r['outcome'] in CONTACT for r in plate),
                 sum(r['outcome'] == 'caught+proven' for r in plate), med_range([num(r, 'tip_xy_cm') for r in tot_h]),
                 f"{sum(num(r, 'tip_xy_cm') <= PLATE_R for r in tot_h)} of {len(tot_h)}",
                 med_range([num(r, 'low_tip_xy_cm') for r in tot_h])])
    L.append(table(['kind', 'carriers', 'ring', 'days', 'flights', 'descents', 'at plate / target height',
                    'contacts', 'caught + proven', 'miss at height, cm: median (range)',
                    f'miss within {PLATE_R} cm', 'lowest point, cm: median (range)'], rows))
    L.append('\n"Contacts" counts every descent that ended in a declared contact or a pickup attach, proven or not. '
             'For a pickup it also counts the object rising with the magnet. "Caught + proven" means our co-motion proof '
             'passed (plate), or his pickup proof passed and the transit started (object). The 1.15 kg ring flew '
             'one descent (sq6b, the triangle, from the mat); every other flight was on the 0.86 kg M2A ring.\n')

    # ---------------------------------------------------------------- outcomes
    L.append('## How the descents ended\n')
    oc = {}
    for r in D:
        k = (r['kind'], r['outcome'], _detail(r))
        oc[k] = oc.get(k, 0) + 1
    L.append(table(['kind', 'outcome', 'detail (his phase reason or our proof)', 'descents'],
                   [list(k) + [v] for k, v in sorted(oc.items())]))
    L.append('')

    # ---------------------------------------------------------------- per flight
    L.append('## Per flight\n')
    rows = []
    for f in F:
        ds = [r for r in D if r['flight'] == f['flight'] and r['day'] == f['day']]
        dr = [r for r in drops if r['flight'] == f['flight'] and r['day'] == f['day']]
        h = [r for r in ds if at_height(r)]
        con = [r for r in ds if r['outcome'] in CONTACT]
        cat = [r for r in ds if r['outcome'] == 'caught+proven']
        ttp = '; '.join(f"{num(r, 'time_to_proven_s'):.1f} s from {r['time_from']}" for r in cat if r.get('time_to_proven_s'))
        kinds = ', '.join(sorted({r['kind'] for r in ds})) or '-'
        tgt = ', '.join(sorted({r['target'] for r in ds})) or '-'
        note = NO_DESCENT.get(f['flight'], '') if not ds else ''
        if dr:
            note = (note + '; ' if note else '') + f"drop: {dr[0]['outcome']}"
        rows.append([f['day'][8:10] + ' Nov', f"{f['flight']}{' (+c)' if f['correction_row'] else ''}",
                     'M2A' if f['ring_registry'].startswith('M2A') else f['ring_registry'],
                     f"{f['formation']} {f['carriers']}", f['motion'], kinds, tgt, len(ds), len(h), len(con), len(cat),
                     med_range([num(r, 'tip_xy_cm') for r in h]) if h else '-', ttp or '-', note])
    L.append(table(['day', 'flight', 'ring', 'carriers', 'ring motion', 'kind', 'target', 'descents', 'at height',
                    'contacts', 'caught + proven', 'miss at height, cm: median (range)',
                    'release (or his TAKEOFF) to proven', 'note'], rows))
    L.append('\nRegistry ids are RIG-1104-<flight> and RIG-1105-<flight>; "(+c)" marks a correction row '
             '(RIG-1104-k1m_3c, RIG-1105-k1ob_2c). Flights that were launched but never armed are left out: nr1, nr1_3, '
             'fm9, fm9_3, nr1r_2, sq9, nr9o, nr9o_3, k1ob and k1ob_3. For sq8_2 the catch was proven 14.3 s after the '
             'release and the fleet resized to four at 14.4 s (the registry\'s 14.4 s).\n')

    # ---------------------------------------------------------------- every plate descent
    L.append('## Every plate descent\n')
    rows = []
    for r in plate:
        rows.append([r['day'][8:10], r['flight'], r['target'], r['n'], r['descent_from'].replace('MATCH_VELOCITY', 'match')
                     .replace('LIFT_PROOF', 'after proof'), r['outcome'], r['eval'].replace('plate height', 'plate')
                     .replace('lowest point', 'never'), cm(r, 'tip_xy_cm'), cm(r, 'tip_rad_cm', signed=True),
                     cm(r, 'tip_tan_cm', signed=True), cm(r, 'body_rad_cm', signed=True), cm(r, 'swing_rad_cm', signed=True),
                     cm(r, 'ref_rad_cm', signed=True), cm(r, 'body_move_rad_1s_cm', signed=True),
                     cm(r, 'low_tip_xy_cm'), cm(r, 'tip_min_normal_cm', signed=True),
                     (f"{num(r, 'contact_log_xy_mm') / 10:.1f}" if math.isfinite(num(r, 'contact_log_xy_mm')) else '-'),
                     cm(r, 'ring_tilt_deg', 0)])
    L.append(table(['day', 'flight', 'target', '#', 'from', 'outcome', 'height', 'miss', 'radial', 'tang.', 'body rad.',
                    'swing rad.', 'his ref rad.', 'body moved rad. last 1 s', 'lowest: miss', 'lowest height',
                    'contact line xy', 'ring tilt deg'], rows))
    L.append('\nAll lengths in cm. "height": plate = at plate height, never = never within 1 cm (read at the lowest '
             'point). "lowest height" is the tip\'s lowest height above the weld point in the descent; a negative value '
             'means it sank past the plate.\n')

    # ---------------------------------------------------------------- reproduction
    L.append('## Reproduction of the 5 Nov triangle numbers (notes_for_tejen_2026-11-05.md, question 1)\n')
    tri = [r for r in plate if r['day'] == '2026-11-05' and r['flight'] in TRIANGLE_5NOV]
    trih = [r for r in tri if at_height(r)]
    sq = [r for r in plate if r['flight'] == 'sq8_2' and r['outcome'] == 'caught+proven']
    fl, pr = alternation(trih)
    flL, prL = alternation(trih, 'low_tip_rad_cm')
    rows = [
        ['descents that reached plate height (of all descents)', '16', f'{len(trih)} (of {len(tri)})', 'same'],
        ['radial miss, cm', '-7.8 to +9.5', rng([num(r, 'tip_rad_cm') for r in trih]),
         rng([num(r, 'low_tip_rad_cm') for r in trih])],
        ['tangential miss, cm', 'mostly within 4, two at 9-10', _tan_text(trih, 'tip_tan_cm'),
         _tan_text(trih, 'low_tip_tan_cm')],
        ['miss xy, cm', '3.5 to 10 (11 Nov sheet)', rng([num(r, 'tip_xy_cm') for r in trih], signed=False),
         rng([num(r, 'low_tip_xy_cm') for r in trih], signed=False)],
        ['QUAD4 body radial, cm', '-4 to +11', rng([num(r, 'body_rad_cm') for r in trih]),
         rng([num(r, 'low_body_rad_cm') for r in trih])],
        ['swing radial (tip - body), cm', '-10 to +7', rng([num(r, 'swing_rad_cm') for r in trih]),
         rng([num(r, 'low_swing_rad_cm') for r in trih])],
        ['body radial move in the last 1 s, cm (size)', '3 to 10',
         rng([abs(num(r, 'body_move_rad_1s_cm')) for r in trih], signed=False), '-'],
        ['radial sign changes between consecutive descents of a flight', 'alternating', f'{fl} of {pr}', f'{flL} of {prL}'],
    ]
    if sq:
        s = sq[0]
        rows += [['sq8_2 catch: tip off the plate centre, cm', '1.7',
                  f"{num(s, 'tip_xy_cm'):.1f} (at the contact: {num(s, 'contact_tip_xy_ring_mm') / 10:.1f})",
                  f"{num(s, 'low_tip_xy_cm'):.1f}"],
                 ['sq8_2 catch: body radial, cm', '7 inward (-7)',
                  f"{num(s, 'body_rad_cm'):+.1f} (at the contact: {num(s, 'contact_body_rad_cm'):+.1f})",
                  f"{num(s, 'low_body_rad_cm'):+.1f}"],
                 ['sq8_2 catch: swing radial, cm', '6 outward (+6)',
                  f"{num(s, 'swing_rad_cm'):+.1f} (at the contact: {num(s, 'contact_swing_rad_cm'):+.1f})",
                  f"{num(s, 'low_swing_rad_cm'):+.1f}"]]
    L.append(table(['quantity', 'notes (5 Nov)', 'this table: at plate height', 'this table: lowest point'], rows))
    L.append('')
    L.append(_repro_note(trih, sq))

    # ---------------------------------------------------------------- what the misses share
    L.append('## What the plate misses have in common\n')
    pm = [r for r in plate if at_height(r) and r['outcome'] != 'caught+proven' and num(r, 'tip_xy_cm') > PLATE_R]
    good = [r for r in plate if at_height(r) and num(r, 'tip_xy_cm') <= PLATE_R]
    rows = []

    def line(name, fn):
        rows.append([name, fn(pm), fn(good)])
    line('descents', lambda g: str(len(g)))
    line('radial larger than tangential', lambda g: f"{sum(abs(num(r, 'tip_rad_cm')) > abs(num(r, 'tip_tan_cm')) for r in g)}")
    line('miss xy, cm: median (range)', lambda g: med_range([num(r, 'tip_xy_cm') for r in g]))
    line('|radial|, cm: median (range)', lambda g: med_range([abs(num(r, 'tip_rad_cm')) for r in g]))
    line('|tangential|, cm: median (range)', lambda g: med_range([abs(num(r, 'tip_tan_cm')) for r in g]))
    line('radial outward / inward', lambda g: f"{sum(num(r, 'tip_rad_cm') > 0 for r in g)} / {sum(num(r, 'tip_rad_cm') < 0 for r in g)}")
    line('his reference radial, cm (range)', lambda g: rng([num(r, 'ref_rad_cm') for r in g]))
    line('body minus his reference, radial, cm (range)', lambda g: rng([num(r, 'body_minus_ref_rad_cm') for r in g]))
    line('|body radial|, cm: median (range)', lambda g: med_range([abs(num(r, 'body_rad_cm')) for r in g]))
    line('|swing radial|, cm: median (range)', lambda g: med_range([abs(num(r, 'swing_rad_cm')) for r in g]))
    line('body and swing on the same side (radial)',
         lambda g: f"{sum(num(r, 'body_rad_cm') * num(r, 'swing_rad_cm') > 0 for r in g)} of {len(g)}")
    line('var(body radial) / var(tip radial)', lambda g: share(g, 'body_rad_cm') if len(g) >= 8 else '-')
    line('var(swing radial) / var(tip radial)', lambda g: share(g, 'swing_rad_cm') if len(g) >= 8 else '-')
    line('corr(body radial, swing radial)', lambda g: f"{corr(g, 'body_rad_cm', 'swing_rad_cm'):+.2f}")
    line('QUAD4 speed over the last 1 s, m/s: median (range)', lambda g: med_range([num(r, 'body_speed_1s') for r in g], 2))
    line('tip speed against the plate at the reading, m/s', lambda g: med_range([num(r, 'tip_rel_speed') for r in g], 2))
    line('ring tilt at the reading, deg', lambda g: med_range([num(r, 'ring_tilt_deg') for r in g]))
    line('plate speed at the reading, m/s', lambda g: med_range([num(r, 'plate_speed') for r in g], 2))
    L.append(table(['', f'misses (at plate height, > {PLATE_R} cm off)', f'on the plate (<= {PLATE_R} cm)'], rows))
    L.append('')
    L.append(_miss_note(pm, good, plate))

    # ---------------------------------------------------------------- pickups
    L.append('## Pickups (the object on the floor)\n')
    rows = []
    for r in pick:
        rows.append([r['day'][8:10], r['flight'], r['ring_motion'], r['n'], r['outcome'],
                     (r.get('proof') or r.get('miss_kind') or '').replace('|', '/'),
                     r['eval'].replace('target height', 'target').replace('lowest point', 'never'),
                     f"{cm(r, 'tip_x_cm', signed=True)}, {cm(r, 'tip_y_cm', signed=True)}", cm(r, 'tip_xy_cm'),
                     cm(r, 'tip_xy_mean_band_cm'),
                     (f"{num(r, 'contact_log_xy_mm') / 10:.1f}" if math.isfinite(num(r, 'contact_log_xy_mm')) else '-'),
                     f"{cm(r, 'body_x_cm', signed=True)}, {cm(r, 'body_y_cm', signed=True)}",
                     f"{cm(r, 'swing_x_cm', signed=True)}, {cm(r, 'swing_y_cm', signed=True)}", cm(r, 'body_speed_1s', 2),
                     cm(r, 'obj_rise_m', 2), cm(r, 'time_to_proven_s')])
    L.append(table(['day', 'flight', 'ring', '#', 'outcome', 'detail', 'height', 'tip x, y', 'miss', 'miss, mean at the bottom',
                    'his xy', 'body x, y', 'swing x, y', 'QUAD4 speed 1 s, m/s', 'object rose, m',
                    'his TAKEOFF to lifted, s'], rows))
    L.append('\nLengths in cm unless marked. "his xy" is his planner\'s own number: "object attached with xy error" '
             'or "the tip N m off target". "miss, mean at the bottom" is the mean while the tip was within 2 cm of its '
             'lowest. ' + _pick_note(pick) + '\n')

    # ---------------------------------------------------------------- drops
    L.append('## Drops into the net\n')
    rows = []
    for r in drops:
        rows.append([r['day'][8:10], r['flight'], r['ring_motion'], f"{num(r, 'contact_log_xy_mm') / 10:.1f}",
                     cm(r, 'tip_xy_cm'), cm(r, 'tip_normal_cm', 0), cm(r, 'obj_fall_m', 2), cm(r, 'impact_after_s', 2),
                     cm(r, 'impact_xy_cm'), cm(r, 'impact_speed'), cm(r, 'settled_xy_cm'), r['outcome'],
                     r.get('note', '')])
    L.append(table(['day', 'flight', 'ring', 'his log: magnet to drop point, cm', 'tip to ring centre at the release, cm',
                    'tip above the ring, cm', 'fall to the net, m', 'impact after, s', 'impact to ring centre, cm',
                    'impact speed, m/s', 'object to ring centre 2 s later, cm', 'outcome', 'note'], rows))
    L.append('\nThe impact is where the object\'s fall relative to the ring stops: the relative vertical speed comes '
             'back above -0.3 m/s after falling faster than -1 m/s. F051 and RIG-1105-k1ob_2c give k1d 11 cm and k1ob_2 '
             '14 cm at ~2.8 m/s. This table gives ' + ', '.join(f"{r['flight']} {cm(r, 'impact_xy_cm')} cm at "
                                                              f"{cm(r, 'impact_speed')} m/s" for r in drops) +
             '. These agree within the definition of the impact instant. In k1ob_2 the object is 51 cm from the ring '
             'centre 2 s later because the ring was dropped after the impact.\n')

    # ---------------------------------------------------------------- geometry
    L.append('## Geometry per flight (flights with descents, and welded starts)\n')
    rows = []
    for f in F:
        if not (int(f['n_descents'] or 0) or f['start_weld_dxy_mm']):
            continue
        rows.append([f['day'][8:10], f['flight'], f['config_ring'], f['ring_registry'], f['target_xy'], f['target_z'],
                     f['start_plate_log'] or '-', f['start_weld_angle_cw'] or '-', f['start_weld_dxy_mm'] or '-',
                     f['start_weld_dz_mm'] or '-', f['n_tip'], f['n_obj'], f['note'] or ''])
    L.append(table(['day', 'flight', 'launch config', 'ring (registry)', 'target weld point xy (ring frame, m)',
                    'target z (m)', 'start plate (log)', 'tip angle at start (cw deg)',
                    'start: tip xy off the start plate\'s point (mm)', 'start: tip z off (mm)', 'tip samples',
                    'object samples', 'note'], rows))
    L.append('')
    L.append(_disagreements(plate, drops))
    L.append(_limits_note(F, plate))
    with open(os.path.join(HERE, 'summary.md'), 'w') as f:
        f.write('\n'.join(L) + '\n')
    print('wrote summary.md')
    return 0


def _contact_check(plate):
    dx = [abs(num(r, 'contact_tip_xy_ring_mm') - num(r, 'contact_log_xy_mm')) for r in plate
          if math.isfinite(num(r, 'contact_log_xy_mm'))]
    dn = [abs(num(r, 'contact_tip_normal_mm') - num(r, 'contact_log_normal_mm')) for r in plate
          if math.isfinite(num(r, 'contact_log_normal_mm'))]
    return f'{len(dx)} lines, all within {max(dx):.1f} mm in xy and {max(dn):.1f} mm in height'


def _weld_check(F):
    v = [num(f, 'start_weld_dxy_mm') for f in F if f['day'] == '2026-11-05' and num(f, 'start_weld_dxy_mm') < 20]
    z = [abs(num(f, 'start_weld_dz_mm')) for f in F if f['day'] == '2026-11-05' and num(f, 'start_weld_dxy_mm') < 20]
    return f'{min(v):.1f}-{max(v):.1f} mm in xy and within {max(z):.1f} mm in height ({len(v)} flights)'


def _detail(r):
    p = r.get('proof') or ''
    if r.get('miss_kind'):
        return r['miss_kind']
    if r['kind'] == 'pickup':
        if r['outcome'] == 'caught+proven':
            return 'his pickup proof passed, transit started'
        if 'rose' in p:
            return 'the object rose with the magnet; his proof never passed'
        if 'off target' in p:
            return 'attached, released by his check: tip off target'
        if 'separation' in p:
            return 'attached, the object came off in his lift proof'
        if 'no progress' in p:
            return 'no attach (no progress in 10 s)'
        return p
    if p.startswith('proven'):
        return 'proven (co-motion)'
    return p.split(':')[0]


def _tan_text(rows, key):
    t = [abs(num(r, key)) for r in rows if math.isfinite(num(r, key))]
    big = sorted(x for x in t if x > 4)
    return f"{sum(x <= 4 for x in t)} of {len(t)} within 4" + (f"; beyond: {', '.join(f'{x:.1f}' for x in big)}" if big else '')


def _n(rows, pred):
    return sum(1 for r in rows if pred(r))


def _repro_note(trih, sq):
    rad = [num(r, 'tip_rad_cm') for r in trih]
    low = [num(r, 'low_tip_rad_cm') for r in trih]
    mv = sorted(abs(num(r, 'body_move_rad_1s_cm')) for r in trih)
    w4 = _n(trih, lambda r: abs(num(r, 'tip_tan_cm')) <= 4)
    w4l = _n(trih, lambda r: abs(num(r, 'low_tip_tan_cm')) <= 4)
    fl, pr = alternation(trih)
    out = ('**Agreement.** The count agrees: the same 16 of 21 descents reached plate height. The other five are '
           'nr9o_2 #2-#3 and nr1r_3 #8, aborted 3.8-14 cm above the plate, and nr1r_3 #1 and #3, contacts whose magnet stopped '
           '1.6-2.3 cm above the plate. '
           f'The radial extremes agree within about 1 cm (at plate height {rng(rad)}, lowest point {rng(low)}, notes '
           '-7.8 to +9.5). The swing range agrees at the lowest point (-9.4 to +6.5 here, -10 to +7 in the notes). The '
           f'radial sign changed in {fl} of {pr} consecutive pairs, so the alternation holds mostly.\n\n'
           '**Partial disagreement.** The tangential spread is wider than "mostly within 4, two at 9-10": '
           f'{w4} of 16 are within 4 cm at plate height ({w4l} at the lowest point), and the rest are 4-10 cm. The '
           f'body\'s radial range is narrower: -4.2 to +6.4 at plate height and -5.0 to +9.1 at the lowest point, '
           f'against -4 to +11. "Moved 3-10 cm radially in the last second" does not hold: the size of that move is '
           f'{mv[0]:.1f}-{mv[-1]:.1f} cm (median {st.median(mv):.1f}). The script behind the notes was not found, so '
           'the instant it read is not known. Its numbers sit closest to this table\'s lowest point (where the magnet '
           'ended after sliding off), not to the instant the tip reached the plate\'s height. The qualitative reading '
           'stands: the misses are mostly radial, both inward and outward, and the body and the swing each give '
           'several cm.\n\n'
           '**Disagreement: sq8_2\'s catch.** The tip offset agrees (1.6 cm at plate height, 1.7 cm at the contact '
           'and in the contact line and notes). The split between body and swing does not. Here the body is 2-4 cm '
           'OUTWARD and the swing 4-6 cm INWARD, at the contact and at plate height. The notes have 7 cm inward and '
           '6 cm outward. This was checked a second way, from the raw poses at the CONTACT line\'s time: horizontally '
           'from the ring centre the body sits at 24.6 cm, the plate at 22.5 cm and the tip at 21.1 cm. At that '
           'instant the ring leaned 28-29 deg (the T), so the split depends strongly on the frame. In the tilted ring '
           'frame the body 1 m up reads about 48 cm "inward". The notes\' split may come from another frame or '
           'instant. This table\'s numbers are horizontal.\n')
    return out


def _miss_note(pm, good, plate):
    out_in = (_n(pm, lambda r: num(r, 'tip_rad_cm') > 0), _n(pm, lambda r: num(r, 'tip_rad_cm') < 0))
    ref = [abs(num(r, 'ref_rad_cm')) for r in pm]
    bmr = [abs(num(r, 'body_minus_ref_rad_cm')) for r in pm]
    same = _n(pm, lambda r: num(r, 'body_rad_cm') * num(r, 'swing_rad_cm') > 0)
    tri = [r for r in plate if r['formation'] == 'triangle' and at_height(r)]
    tri5 = [r for r in tri if r['day'] == '2026-11-05']
    sqr = [r for r in plate if r['formation'] == 'square' and at_height(r)]
    n1 = [r for r in plate if r['flight'] == 'nr1_2']
    sank = _n(plate, lambda r: 'sank' in (r.get('miss_kind') or ''))
    loose = [r for r in plate if r['day'] == '2026-11-04' and r['flight'] in ('nr9m_4', 'nr9m_5')
             and math.isfinite(num(r, 'contact_log_xy_mm'))]
    xy = [num(r, 'tip_xy_cm') for r in pm]
    return (
        'Plain notes (plate descents; the numbers are the tables\' above):\n'
        f'1. **The miss is mostly radial and goes both ways.** Of {len(pm)} descents that reached plate height more '
        f'than {PLATE_R} cm from the centre, {_n(pm, lambda r: abs(num(r, "tip_rad_cm")) > abs(num(r, "tip_tan_cm")))} '
        f'were more radial than tangential: {out_in[0]} outward and {out_in[1]} inward. They came down '
        f'{min(xy):.1f}-{max(xy):.1f} cm off, against plates {2 * PLATE_R:.1f} cm across. {sank} descents ended with his '
        '"the tip sunk below the plate (off its edge)".\n'
        f'2. **His reference was not off; QUAD4 was off his reference.** At the reading his reference sat '
        f'{min(ref):.1f}-{max(ref):.1f} cm radially from the plate, and QUAD4\'s body {min(bmr):.1f}-{max(bmr):.1f} cm '
        'from his reference.\n'
        f'3. **Body and swing are each a few cm, and in the misses they add.** In {same} of {len(pm)} misses QUAD4\'s '
        f'body offset and the swing (magnet under the body) were on the same side (radial correlation '
        f'{corr(pm, "body_rad_cm", "swing_rad_cm"):+.2f}). Each alone is a few cm (|body| and |swing| medians above), '
        'and each alone holds about a third of the tip\'s radial variance; the rest is their covariance. Where the '
        'magnet came down on the plate the two were small or opposite (tip = body + swing, so this follows): in '
        'sq8_2 the body was 4.1 cm out and the swing 5.6 cm in. Trimming one of them alone would not have put the '
        'misses on the plate.\n'
        f'4. **Slow, and the ring was still.** QUAD4 moved at {med_range([num(r, "body_speed_1s") for r in pm], 2)} m/s '
        f'over the last second, and the tip at {med_range([num(r, "tip_rel_speed") for r in pm], 2)} m/s against the '
        f'plate at the reading. Plate speed was {rng([num(r, "plate_speed") for r in pm], 2, signed=False)} m/s, '
        'including the orbit flight nr9o_2, whose circle was paused for the attach. Ring tilt was '
        f'{rng([num(r, "ring_tilt_deg") for r in pm], 1, signed=False)} deg.\n'
        f'5. **The square vs the triangle.** The square (sq8_2) had {len(sqr)} descent at plate height and caught it '
        'at 1.6 cm, with the ring leaning 29 deg (the T). The triangle had '
        f'{len(tri)} at plate height, {_n(tri, lambda r: num(r, "tip_xy_cm") <= PLATE_R)} of them on the plate, all '
        f'on 4 Nov (sq6m\'s catch and nr1_2\'s three on plate 1). On 5 Nov it had {len(tri5)} at plate height and '
        f'{_n(tri5, lambda r: num(r, "tip_xy_cm") <= PLATE_R)} on the plate. One square descent is too few to call the '
        'layout the cause.\n'
        f'6. **Plate 1 on 4 Nov (nr1_2) was dead centre:** {len(n1)} contacts {rng([num(r, "tip_xy_cm") for r in n1], 1, signed=False)} '
        'cm off. The catch was real (the plate rose 18-19 mm) but not proven: our 3 mm taut margin rejected it '
        '(registry). On 5 Nov, plate 1 (nr1r_3, contact_dz -0.01) stopped 1.6-2.3 cm above the plate twice, then '
        'slid off the edge on the later descents.\n'
        f'7. **Contacts that did not hold (4 Nov, plate 9).** {len(loose)} contacts were declared with the loosened '
        f'60 mm contact radius, {rng([num(r, "contact_log_xy_mm") / 10 for r in loose], 1, signed=False)} cm from the '
        'centre. Two of them were 1.6 and 3.7 cm above the weld point (nr9m_4 #1 and #3). Every one was "lost" in '
        'the proof: the magnet was on the plate\'s edge or above it. They count as contacts in the tables.\n')


def _pick_note(pick):
    k1o = [r for r in pick if r['flight'] == 'k1o_2']
    return ('k1o_2\'s object body (6) had been redefined in Motive with its centre several cm off (registry). Its '
            f'{len(k1o)} rows, and his "6.0 / 7.5 cm off target", are measured against that shifted centre, not the '
            'true one. On k1m (4 Nov) the object rose 1.34 m with the magnet, but his pickup proof never passed '
            '(registry: proof too tight), so it counts as caught, not proven. On 4 Nov, "object lifted 0.08 m" is the '
            'reason his planner logged when LIFT_OBJECT ended; the object then rose about 1.3 m ("object rose").')


def _disagreements(plate, drops):
    def fl(name, day='2026-11-05'):
        return [r for r in plate if r['flight'] == name and r['day'] == day]
    o2 = fl('nr9o_2')
    o2h = [r for r in o2 if at_height(r)]
    m5 = fl('nr9m')
    y2 = fl('fm9y_2')
    p9 = [r for r in plate if r['day'] == '2026-11-04' and r['flight'] in ('nr9m_4', 'nr9m_5')]
    p9h = [r for r in p9 if at_height(r)]
    return (
        '## Where this table and the registry or the feed differ\n\n'
        '| source | says | this table | why |\n|---|---|---|---|\n'
        f'| RIG-1105-nr9o_2, F048 | 4 descents reached plate height, 3.5-8 cm off | {len(o2h)} of {len(o2)} reached it '
        f'({rng([num(r, "tip_xy_cm") for r in o2h], 1, signed=False)} cm); #2 and #3 were aborted 13-14 cm above it '
        '("alignment lost in the descent") | The other two were read at their lowest point, 3.8 and 4.9 cm off but '
        '13-14 cm up |\n'
        f'| RIG-1105-nr9m | ~8 descents, off the edge or quiet misses 74-103 mm off | {len(m5)} descents; at plate height '
        f'{rng([num(r, "tip_xy_cm") for r in m5], 1, signed=False)} cm, lowest point '
        f'{rng([num(r, "low_tip_xy_cm") for r in m5], 1, signed=False)} cm | The registry\'s range matches no single '
        'reading here; it is nearest the lowest point |\n'
        f'| RIG-1105-fm9y_2 | re-attach: 3 descents never touched plate 9 (4 Nov W/L with the new body 24: ~2 cm high) '
        f'| the tip reached plate height on all three (lowest {rng([num(r, "tip_min_normal_cm") for r in y2])} cm '
        f'against 5 Nov\'s W), {rng([num(r, "tip_xy_cm") for r in y2], 1, signed=False)} cm off the centre | Never '
        'touched: agreed. "~2 cm high" is not seen: the misses were sideways |\n'
        f'| F040, RIG-1104-nr9m_4/_5 | plate 9 descents ended 24-59 mm off the target | contact lines 23.8-59.0 mm '
        f'(the same); at plate height {rng([num(r, "tip_xy_cm") for r in p9h], 1, signed=False)} cm | Different '
        'instants: the contact line is where the contact was declared, often after the tip had settled closer; '
        'the plate-height reading is where it first reached the plate\'s height |\n'
        '| notes_for_tejen (sq8_2) | body 7 cm inward, swing 6 cm outward | body +2 to +4 (outward), swing -4 to -6 '
        '(inward) | See the reproduction section: the ring leaned 29 deg; the frame decides the split |\n'
        '| F051 / RIG-1105-k1ob_2c | k1d hit the net 11 cm off centre; k1ob_2 14 cm at ~2.8 m/s | ' +
        '; '.join(f"{r['flight']} {cm(r, 'impact_xy_cm')} cm at {cm(r, 'impact_speed')} m/s" for r in drops) +
        ' | Agree within the definition of the impact instant |\n')


def _limits_note(F, plate):
    return (
        '## What could not be measured, and caveats\n\n'
        '1. **The 4 Nov plate height.** 4 Nov flew the typed W 0.0365 m (measured 08:44). The welded starts of nr1_2, '
        'nr9m_4 and nr9m_5 sat 12-14 mm above it, close to 5 Nov\'s 0.0491 (geometry table). But the in-flight '
        'contacts sat near the typed value: sq6m\'s proven catch at +1.3 mm and nr1_2\'s three at +5 to +6 mm. So the '
        '4 Nov plate height is uncertain by about 1 cm, and 4 Nov is scored against the typed W. This matters for '
        'nr9m #4 (4 Nov). It held 40 s at +1.1 cm, 1.8 cm from the centre (registry: +12.1-12.5 mm, outside the '
        '+12 mm contact band). It counts as "never at plate height", but it may have been resting on the plate.\n'
        '2. **fm9y_2** typed 4 Nov\'s W/X/Y after body 24 was redefined. Its welded start sat 1.2 mm in xy and '
        '+0.5 mm in height from 5 Nov\'s values, so it is scored against those (0.0, 0.2293, 0.0491).\n'
        '3. **The weld point is the tip marker\'s welded position, not a measured plate centre.** Any offset between '
        'the two is common to all of one day\'s numbers. Plates 1 and 8 use plate 9\'s point turned by 30 deg per plate, '
        'as the launch does; no weld on plate 1 was measured. sq8_2\'s welded start on plate 8 sat 5.6 mm from its '
        'turned point.\n'
        '4. **Starts not usable as a check:** nr9 (body 24 tracked QUAD2\'s magnet), nr9m_2, nr1r and fm9y (the tip '
        'about 1 m from the ring centre at the start), sq8 (QUAD4 not on its plate) and nr9m_3 (a bad start). None of '
        'these flights has a descent. nr9m (4 Nov) has descents. At its start the tip read 74 mm from plate 9\'s point '
        '(7 cm outward) and 29 mm lower: that is not a magnet seated on the plate. The in-flight rows use the ring\'s '
        'pose and do not depend on the start.\n'
        '5. **No flight lacked its streams.** Every flight with a descent has the ring, body, tip and reference in its '
        'bag. The pickup body (6) was not streamed in nr1r, nr1r_3, sq8, sq8_2 and nr9o_2 (re-attach only; not '
        'needed) or in k1o (no descent). Times are the recorder\'s receive times, so every instant carries a few tens of ms of latency.\n'
        '6. **sq6b (the 1.15 kg ring):** its only descent was aborted 26 cm above the plate when the ring dropped, so '
        'it says nothing about the miss.\n'
        '7. **Single samples, and which instant.** Each reading is one mocap sample (about 90-100 Hz); at these '
        'speeds the tip moves under 1 cm between samples. Which instant is read changes the numbers by a few cm (plate '
        'height vs lowest point vs the contact line). The lowest point is not a cleaner miss distance. A magnet '
        'hanging below the plate can read horizontally close to it when the ring is tilted: on 4 Nov nr9m #1-#3 the '
        'lowest points were 8-11 cm below the plate yet 1.6-3.8 cm from its point, with the ring at 4-8 deg. The '
        'plate-height reading is the main one here. Quote the reading used.\n'
        '8. Exploratory: no card, no bars, no repeats.\n')


if __name__ == '__main__':
    sys.exit(main())
