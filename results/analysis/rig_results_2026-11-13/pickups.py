#!/usr/bin/env python3
"""Q013 item 3, the pickups as counts: one row per Setup 3 flight (4 Nov on) with a pickup descent, from
results/analysis/pickup_seat_2026-11-13/tries.csv (every DESCEND_TO_PICKUP of every rig flight; its definitions:
lift_started = his LIFT_OBJECT after the descent, held = the object rose >= 3 cm with the tip in that lift). Writes
pickups.csv beside this file.

    cd ~/multi_drone_control && python3 results/analysis/rig_results_2026-11-13/pickups.py
"""
import csv
import os
import re

import common as C

TRIES = os.path.join(C.REPO, 'results', 'analysis', 'pickup_seat_2026-11-13', 'tries.csv')
FIRST = '11-04'
ONE_QUAD = ('q4p', 'q4d')


def registry_verdicts():
    out = {}
    with open(os.path.join(C.REPO, 'results', 'registry.csv'), newline='') as f:
        for r in csv.reader(f):
            if r and r[0].startswith('RIG-') and len(r) > 9:
                out[r[0]] = r[9]
    return out


def ring_during_pickup(day, name):
    if name.startswith(ONE_QUAD):
        return 'no carriers (one quad; the ring on the floor)'
    fn = os.path.join(C.RIG, day, name + '.log')
    txt = open(fn, errors='replace').read()
    traj = re.search(r'load_traj=(\w+)', txt)
    traj = traj.group(1) if traj else 'hover'
    if traj == 'hover':
        return 'hover'
    paused = re.search(r'attach pause: the orbit slows to rest .*his (APPROACH_ABOVE_PICKUP|SETTLE_ABOVE_PICKUP|'
                       r'DESCEND_TO_PICKUP)', txt)
    return f"{C.MOTION.get(traj, traj)}, {'paused for the pickup' if paused else 'running'}"


def main():
    regs = registry_verdicts()
    rows, order = {}, []
    with open(TRIES, newline='') as f:
        for r in csv.DictReader(f):
            md, name = r['flight'].split('/')
            if md < FIRST:
                continue
            day = f'2026-{md}'
            key = (day, name)
            if key not in rows:
                order.append(key)
                rid = f'RIG-{md[:2]}{md[3:]}-{name}'
                rows[key] = {'flight': name, 'day': day, 'registry_id': rid, 'object': r['object'],
                             'carriers': 'no' if name.startswith(ONE_QUAD) else 'yes',
                             'ring_during_pickup': ring_during_pickup(day, name), 'tries': 0, 'lift_started': 0, 'held': 0,
                             'held_then_fell': 0, 'try_held': '', 'ends': [], 'verdict': regs.get(rid, '')}
            row = rows[key]
            row['tries'] += 1
            if r['lifted'] == 'True':
                row['lift_started'] += 1
            if r['held'] == 'True':
                row['held'] += 1
                if not row['try_held']:
                    row['try_held'] = r['try_n']
                if r['outcome'].startswith('held, fell'):
                    row['held_then_fell'] += 1
            row['ends'].append(f"{r['try_n']}: {r['outcome']} ({r['end']})")
    out = []
    for k in order:
        r = rows[k]
        r['tries_detail'] = ' | '.join(r.pop('ends'))
        out.append(r)
    C.write_csv(os.path.join(C.HERE, 'pickups.csv'), out,
                ['flight', 'day', 'registry_id', 'object', 'carriers', 'ring_during_pickup', 'tries', 'lift_started', 'held',
                 'held_then_fell', 'try_held', 'verdict', 'tries_detail'])
    tot = {}
    for r in out:
        g = tot.setdefault(r['carriers'], [0, 0, 0, 0, 0])
        g[0] += 1
        g[1] += r['tries']
        g[2] += r['lift_started']
        g[3] += r['held']
        g[4] += 1 if r['held'] else 0
    for g, v in tot.items():
        print(f'carriers {g}: {v[0]} flights, {v[1]} tries, {v[2]} lifts started, {v[3]} held, {v[4]} flights with a held lift')


if __name__ == '__main__':
    main()
