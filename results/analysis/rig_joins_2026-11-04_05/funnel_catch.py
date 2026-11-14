#!/usr/bin/env python3
"""How many of the 4-5 Nov plate descents each magnet funnel (cad/funnel/funnel.py v1, v2) would have caught.

    source ~/ros2_humble/install/setup.bash && source ~/multi_drone_control/install/setup.bash
    JOINS_CACHE=<bag cache> nice -n 10 python3 results/analysis/rig_joins_2026-11-04_05/funnel_catch.py
    -> funnel_catch.md, funnel_catch.csv (run joins.py first: it reads joins.csv and flights.csv)

Geometry only. The funnel's mouth is an ellipse with half-axes a (radial) and b (tangential) at the cone's top, H above
the ring's top surface (funnel.py mouth(): a = catch_radial + disc_d/2 + margin, b likewise, H = bore_top +
(a - bore_r) tan(cone angle)). QUAD4's 64 mm disc sits 17 mm above the ring top when the magnet is on the steel
(steel 3 mm below the top + magnet 20 mm under the disc); the tip marker (body 24) is at about the disc's height, so
its welded position is the disc seated. The disc's bottom reaches the mouth plane when the tip is H - 17 mm above the
weld point, along the ring's normal (the funnel's axis). At the first sample of each descent at or below that height,
the tip's offset from the weld point is scored:
  (1) into the mouth: (dr/a)^2 + (dt/b)^2 <= 1        (2) fully inside: the same with a - 32, b - 32.
dr, dt are read two ways: in the ring's own plane (the funnel's frame: the mouth turns with the ring; the main
reading) and horizontally (the joins table's reading: the plate's radius projected on the floor).
A descent that started below the mouth plane (a re-descent after a failed proof) or never came down to it is listed,
not scored.
"""
import ast
import csv
import math
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as Rot

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import joins  # noqa: E402  (this folder's reader: bags, the cache, the plate frame)

FUNNEL = os.path.expanduser('~/multi_drone_control/cad/funnel/funnel.py')
DISC_SEATED_MM = 17.0     # the disc's bottom above the ring top with the magnet on the steel (funnel.py checks())


def funnel_variants():
    """P, VARIANTS and mouth() from funnel.py without importing it (it needs CadQuery)."""
    tree = ast.parse(open(FUNNEL).read())
    keep = [n for n in tree.body if (isinstance(n, ast.Assign) and any(getattr(t, 'id', '') in ('P', 'VARIANTS')
                                                                          for t in n.targets))
            or (isinstance(n, ast.FunctionDef) and n.name == 'mouth')]
    ns = {'math': math}
    exec(compile(ast.Module(body=keep, type_ignores=[]), FUNNEL, 'exec'), ns)
    out = {}
    for name, p in ns['VARIANTS'].items():
        a, b, h = ns['mouth'](p)
        disc_bottom = -p['steel_below_top'] + p['magnet_below_disc']
        out[name] = {'a': a, 'b': b, 'H': p['bore_top'] + h, 'disc_r': p['disc_d'] / 2.0, 'disc_bottom': disc_bottom,
                     'plane_mm': p['bore_top'] + h - disc_bottom}
    return out


def ring_plane(ring, ts, pts, weld):
    """Offsets of pts from the weld point in the ring's frame: (radial, tangential, normal), radial along the weld
    point's own direction in the ring frame."""
    k = np.clip(np.searchsorted(ring[:, 0], ts, side='right') - 1, 0, len(ring) - 1)
    rr = ring[k]
    rot = Rot.from_quat(rr[:, 4:8])
    rel = rot.inv().apply(pts - rr[:, 1:4]) - np.array(weld)
    u = np.array(weld[:2]) / np.hypot(*weld[:2])
    return np.stack([rel[:, :2] @ u, rel[:, :2] @ np.array([-u[1], u[0]]), rel[:, 2]], 1)


def main():
    V = funnel_variants()
    with open(os.path.join(HERE, 'flights.csv'), newline='') as f:
        flights = {(r['day'], r['flight']): r for r in csv.DictReader(f)}
    with open(os.path.join(HERE, 'joins.csv'), newline='') as f:
        rows = [r for r in csv.DictReader(f) if r['kind'] in ('re-attach', 'attach from the mat')]
    bags = {}
    out = []
    for r in rows:
        key = (r['day'], r['flight'])
        fl = flights[key]
        weld = tuple(float(x) for x in fl['target_xy'].split(',')) + (float(fl['target_z']),)
        if key not in bags:
            bags[key] = joins.read_bag(os.path.join(joins.RIG, r['day'], r['flight']))
        B = bags[key]
        t0 = B['t0'] + float(r['descent_start_s'])
        t1 = B['t0'] + float(r['descent_end_s'])
        tip, ring = B['tip'], B['ring']
        T = tip[(tip[:, 0] >= t0) & (tip[:, 0] <= t1)]
        rp = ring_plane(ring, T[:, 0], T[:, 1:4], weld)
        lvl, nrm, _ = joins.plate_level(ring, T[:, 0], T[:, 1:4], weld)
        row = {'day': r['day'], 'flight': r['flight'], 'plate': r['target'].split()[-1], 'formation': r['formation'],
               'n': r['n'], 'from': r['descent_from'], 'outcome': r['outcome'], 'start_mm': 1000 * nrm[0],
               'lowest_mm': 1000 * nrm.min(), 'miss_at_plate_cm': r['tip_xy_cm'],
               'at_height': r['eval'] == 'plate height',
               'plate_dr': 10 * float(r['tip_rad_cm']), 'plate_dt': 10 * float(r['tip_tan_cm']),
               'low_dr': 10 * float(r['low_tip_rad_cm']), 'low_dt': 10 * float(r['low_tip_tan_cm'])}
        for name, v in V.items():
            thr = v['plane_mm'] / 1000.0
            if nrm[0] <= thr:
                row[f'{name}_status'] = 'started below the mouth'
                # for the comparison with the earlier count only: the offsets at the descent's first sample
                dr, dt = 1000 * rp[0, 0], 1000 * rp[0, 1]
                row[f'{name}_start_mouth'] = (dr / v['a']) ** 2 + (dt / v['b']) ** 2 <= 1.0
                row[f'{name}_start_inside'] = ((dr / (v['a'] - v['disc_r'])) ** 2
                                               + (dt / (v['b'] - v['disc_r'])) ** 2 <= 1.0)
                continue
            hit = np.flatnonzero(nrm <= thr)
            if not len(hit):
                row[f'{name}_status'] = 'never came down to the mouth'
                continue
            k = int(hit[0])
            row[f'{name}_status'] = 'scored'
            row[f'{name}_t_s'] = T[k, 0] - B['t0']
            for frame, dr, dt in (('ring', rp[k, 0], rp[k, 1]), ('horiz', lvl[k, 0], lvl[k, 1])):
                dr, dt = 1000 * dr, 1000 * dt
                row[f'{name}_{frame}_dr_mm'] = dr
                row[f'{name}_{frame}_dt_mm'] = dt
                row[f'{name}_{frame}_mouth'] = (dr / v['a']) ** 2 + (dt / v['b']) ** 2 <= 1.0
                ai, bi = v['a'] - v['disc_r'], v['b'] - v['disc_r']
                row[f'{name}_{frame}_inside'] = (dr / ai) ** 2 + (dt / bi) ** 2 <= 1.0
            rk = joins.at(ring, T[k, 0])
            row[f'{name}_tilt_deg'] = math.degrees(math.acos(min(1.0, Rot.from_quat(rk[4:8]).apply([0, 0, 1])[2])))
        out.append(row)

    cols = ['day', 'flight', 'plate', 'formation', 'n', 'from', 'outcome', 'start_mm', 'lowest_mm', 'miss_at_plate_cm']
    for name in V:
        cols += [f'{name}_status', f'{name}_t_s', f'{name}_tilt_deg'] + [
            f'{name}_{fr}_{k}' for fr in ('ring', 'horiz') for k in ('dr_mm', 'dt_mm', 'mouth', 'inside')]
    with open(os.path.join(HERE, 'funnel_catch.csv'), 'w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore', lineterminator='\n')
        wr.writeheader()
        for r in out:
            wr.writerow({k: (f'{v:.1f}' if isinstance(v, float) else v) for k, v in r.items()})
    write_md(V, out)
    print(f'wrote funnel_catch.csv ({len(out)} plate descents) and funnel_catch.md')
    return 0


def _count(rows, name, frame):
    s = [r for r in rows if r.get(f'{name}_status') == 'scored']
    return (f"{sum(bool(r[f'{name}_{frame}_mouth']) for r in s)}/{len(s)}",
            f"{sum(bool(r[f'{name}_{frame}_inside']) for r in s)}/{len(s)}")


def write_md(V, out):
    L = ['# Funnel catch count: 4-5 Nov plate descents (geometry only, exploratory)\n',
         'Made by `funnel_catch.py` from `joins.csv`, `flights.csv`, the bags and `cad/funnel/funnel.py` (P, VARIANTS, '
         'mouth(); read, not run). Per-descent numbers are in `funnel_catch.csv`.\n',
         '| variant | mouth a x b (half-axes, mm) | mouth plane above the ring top H (mm) | fully-inside ellipse (mm) | '
         'tip height above the weld point at the mouth plane (mm) |', '|---|---|---|---|---|']
    for n, v in V.items():
        L.append(f"| {n} | {v['a']:.0f} x {v['b']:.0f} | {v['H']:.1f} | {v['a'] - v['disc_r']:.0f} x "
                 f"{v['b'] - v['disc_r']:.0f} | {v['plane_mm']:.1f} |")
    L.append('\nThe disc\'s bottom sits 17 mm above the ring top when seated, so it reaches the mouth plane when the '
             'tip is H - 17 mm above its welded position. The height is measured along the ring\'s normal. At the first '
             'sample at or below that height the tip\'s offset from the weld point is scored. "Into the mouth" means the '
             'disc centre is inside the a x b ellipse. "Fully inside" means it is inside the ellipse shrunk by the '
             'disc\'s 32 mm radius. **Ring frame** (the main reading) measures the offsets in the ring\'s own plane, '
             'which the funnel turns with. **Horizontal** is the joins table\'s reading. The two differ only where '
             'the ring leaned: sq8_2 at 29 deg, the rest at 0-8 deg.\n')

    # counts
    L.append('## Counts (scored descents: those that came down through the mouth plane)\n')
    groups = [('all', lambda r: True),
              ('4 Nov', lambda r: r['day'] == '2026-11-04'), ('5 Nov', lambda r: r['day'] == '2026-11-05'),
              ('triangle', lambda r: r['formation'] == 'triangle'), ('square (plate 8)', lambda r: r['formation'] == 'square'),
              ('triangle, plate 9', lambda r: r['formation'] == 'triangle' and r['plate'] == '9'),
              ('triangle, plate 9, 4 Nov', lambda r: r['formation'] == 'triangle' and r['plate'] == '9' and r['day'] == '2026-11-04'),
              ('triangle, plate 9, 5 Nov', lambda r: r['formation'] == 'triangle' and r['plate'] == '9' and r['day'] == '2026-11-05'),
              ('triangle, plate 1', lambda r: r['formation'] == 'triangle' and r['plate'] == '1')]
    head = ['descents']
    for n in V:
        head += [f'{n} into mouth', f'{n} fully inside', f'{n} (horizontal) mouth / inside']
    L.append('| ' + ' | '.join(head) + ' |')
    L.append('|' + '---|' * len(head))
    for label, pred in groups:
        g = [r for r in out if pred(r)]
        cells = [label]
        for n in V:
            m, i = _count(g, n, 'ring')
            mh, ih = _count(g, n, 'horiz')
            cells += [m, i, f'{mh} / {ih}']
        L.append('| ' + ' | '.join(cells) + ' |')
    # the earlier count's set: the 16 triangle descents of 5 Nov at plate height + sq8_2's catch
    prev = [r for r in out if (r['day'] == '2026-11-05' and r['flight'] in ('nr9m', 'nr9o_2', 'nr1r_3')
                               and r['at_height']) or (r['flight'] == 'sq8_2' and r['outcome'] == 'caught+proven')]
    L.append('')
    L.append('## Against the earlier count (a lost script, 17 descents)\n')
    L.append('The earlier set is taken to be the 16 triangle descents of 5 Nov that reached plate height (the notes\' '
             f'16) plus sq8_2\'s catch: {len(prev)} descents here.\n')
    L.append('| reading | v1 into mouth | v1 fully inside | v2 into mouth | v2 fully inside |')
    L.append('|---|---|---|---|---|')
    L.append('| earlier count | 10/17 | 5/17 | 8/17 | 0/17 |')
    for frame, lab in (('ring', 'this script, ring frame'), ('horiz', 'this script, horizontal')):
        cells = [lab]
        for n in V:
            cells += list(_count(prev, n, frame))
        L.append('| ' + ' | '.join(cells) + ' |')
    cells = ['ring frame, re-descents read at their start']
    for n in V:
        sc = [r for r in prev if r.get(f'{n}_status') == 'scored']
        sb = [r for r in prev if r.get(f'{n}_status') == 'started below the mouth']
        m = sum(bool(r[f'{n}_ring_mouth']) for r in sc) + sum(bool(r[f'{n}_start_mouth']) for r in sb)
        i = sum(bool(r[f'{n}_ring_inside']) for r in sc) + sum(bool(r[f'{n}_start_inside']) for r in sb)
        cells += [f'{m}/{len(sc) + len(sb)}', f'{i}/{len(sc) + len(sb)}']
    L.append('| ' + ' | '.join(cells) + ' |')
    for lab, kr, kt in (('joins table: offsets at plate height', 'plate_dr', 'plate_dt'),
                        ('joins table: offsets at the lowest point', 'low_dr', 'low_dt')):
        cells = [lab]
        for n, v in V.items():
            m = sum((r[kr] / v['a']) ** 2 + (r[kt] / v['b']) ** 2 <= 1 for r in prev)
            i = sum((r[kr] / (v['a'] - v['disc_r'])) ** 2 + (r[kt] / (v['b'] - v['disc_r'])) ** 2 <= 1 for r in prev)
            cells += [f'{m}/{len(prev)}', f'{i}/{len(prev)}']
        L.append('| ' + ' | '.join(cells) + ' |')
    L.append('')
    L.append('**It does not agree.** At the mouth plane this script finds more catches than the earlier count, for '
             'both variants and both scores. Reading the ring frame or the horizontal makes no difference here. No '
             'reading tried reproduces 10/5/8/0. The nearest is the lowest point of each descent (where a magnet that '
             'slid off the plate ended up), so the lost script probably scored that instant rather than the mouth '
             'plane. The question for the funnel is whether the disc came down through the mouth, and that is the '
             'mouth-plane reading. Two descents in this set (nr1r_3 #2 and #4) are re-descents after a failed proof '
             'that start below v1\'s plane. They are scored at their first sample only in the last ring-frame row.\n')
    # not scored
    L.append('## Not scored\n')
    L.append('| day | flight | plate | # | from | outcome | tip height at the descent\'s start (mm) | lowest (mm) | '
             + ' | '.join(f'{n}' for n in V) + ' |')
    L.append('|' + '---|' * (8 + len(V)))
    for r in out:
        st = [r.get(f'{n}_status') for n in V]
        if all(s == 'scored' for s in st):
            continue
        L.append(f"| {r['day'][8:10]} | {r['flight']} | {r['plate']} | {r['n']} | {r['from']} | {r['outcome']} | "
                 f"{r['start_mm']:.0f} | {r['lowest_mm']:.0f} | " + ' | '.join(s.replace('scored', 'scored') for s in st)
                 + ' |')
    L.append('\n"Started below the mouth": most are re-descents after a failed proof (from LIFT_PROOF), where the '
             'magnet had lifted only centimetres. With a funnel the disc would still be in the cone, so they are not a '
             'new entry. Some of these began below the plate because the magnet slid off during the failed proof '
             '(5 Nov nr1r_3 #4 at -10 mm). One is not a re-descent: 4 Nov nr9m #2 (from MATCH) began at -25 mm, with the '
             'magnet still below the plate beside it after the previous miss; that retry never lifted it clear. "Never came down '
             'to the mouth" rows were aborted higher up.\n')
    # per descent
    L.append('## Every scored descent (ring frame, mm; Y = yes)\n')
    hd = ['day', 'flight', 'plate', '#', 'outcome', 'miss at plate height (cm, joins)']
    for n in V:
        hd += [f'{n} dr', f'{n} dt', f'{n} mouth', f'{n} inside']
    hd += ['ring tilt at the mouth plane (deg)']
    L.append('| ' + ' | '.join(hd) + ' |')
    L.append('|' + '---|' * len(hd))
    for r in out:
        if not any(r.get(f'{n}_status') == 'scored' for n in V):
            continue
        c = [r['day'][8:10], r['flight'], r['plate'] + (' *' if r['plate'] == '9' and r['formation'] == 'triangle' else ''),
             r['n'], r['outcome'], r['miss_at_plate_cm'] and f"{float(r['miss_at_plate_cm']):.1f}"]
        for n in V:
            if r.get(f'{n}_status') == 'scored':
                c += [f"{r[f'{n}_ring_dr_mm']:+.0f}", f"{r[f'{n}_ring_dt_mm']:+.0f}",
                      'Y' if r[f'{n}_ring_mouth'] else '-', 'Y' if r[f'{n}_ring_inside'] else '-']
            else:
                c += ['', '', r[f'{n}_status'], '']
        tilt = r.get('v1_tilt_deg', r.get('v2_tilt_deg', math.nan))
        c.append(f'{tilt:.1f}' if math.isfinite(tilt) else '-')
        L.append('| ' + ' | '.join(str(x) for x in c) + ' |')
    L.append('\n`*` = plate 9 on the triangle, the funnel\'s intended place.\n')
    L.append(CAVEATS)
    with open(os.path.join(HERE, 'funnel_catch.md'), 'w') as f:
        f.write('\n'.join(L) + '\n')


CAVEATS = ('## Caveats\n\n'
           '1. **Geometry only.** A real cone changes the descent. The disc slides on the wall toward the bore, and the '
           'swinging magnet can hit the rim and bounce. QUAD4\'s controller reacts to all of this. A descent scored '
           '"into the mouth" is one whose disc would meet the cone, not a predicted catch. One scored outside would '
           'land on the rim or beside the funnel, not on the plate.\n'
           '2. **The mouth plane is read with the ring\'s own tilt.** The height and (main reading) the offsets are '
           'along the ring\'s normal and in its plane, as the funnel is bolted to it. sq8_2 leaned 29 deg (the T), so '
           'its horizontal and ring-frame offsets differ by about H x sin(29 deg). The rest leaned 0-8 deg.\n'
           '3. **The disc is the tip marker.** Body 24\'s point is 22.6 mm above the magnet face, about the disc\'s '
           'height. Its welded position is taken as the disc seated (17 mm above the ring top), and its xy as the '
           'disc centre. A tilted holder (the magnet swinging on its 1 m cable) puts the real disc edge a few mm off '
           'that.\n'
           '4. **The weld point** is the one each flight was scored against in joins.csv (5 Nov: 0.0, 0.2293, 0.0491; '
           '4 Nov: the typed 0.0039, 0.2271, 0.0365, uncertain by about 1 cm in height, see summary.md). Plates 1 '
           'and 8 use plate 9\'s point turned. A 1 cm height error moves the mouth-plane instant by about 0.1-0.2 s.\n'
           '5. Exploratory: the descents are the ones flown without a funnel, and no bars apply.\n')


if __name__ == '__main__':
    sys.exit(main())
