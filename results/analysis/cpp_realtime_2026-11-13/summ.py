"""Summarise probe2 outputs per flight x mode x sweep."""
import csv
import math
import statistics as st
import sys

flights = ['fm9_5', 'fm9hc_2', 'fm9hc_4', 'fm9hc_5']
deltas = sys.argv[1].split(',') if len(sys.argv) > 1 else ['0', '0.5', '1', '1.5']
A = ('TRANSIT_TO_DROP_POINT', 'SETTLE_ABOVE_DROP_POINT')


def spans(ts, flags):
    out, start, prev = [], None, None
    for t, fl in zip(ts, flags):
        if fl and start is None:
            start = t
        if not fl and start is not None:
            out.append((start, prev))
            start = None
        prev = t
    if start is not None:
        out.append((start, prev))
    return out


for fl in flights:
    poses = {round(float(r['t']), 1): r for r in csv.DictReader(open(f'{fl}_poses.csv'))}
    inp = {}
    for line in open(f'{fl}_in_0.txt'):
        v = line.split()
        inp[v[0]] = v
    print(f'\n=== {fl}')
    for mode in ('octagon', 'legacy'):
        for d in deltas:
            try:
                rows = [l.split() for l in open(f'out/{fl}_{mode}_{d}.txt')][1:]
            except FileNotFoundError:
                continue
            t0 = float(rows[0][0])
            a = [r for r in rows if r[1] in A]
            b = [r for r in rows if r[1] == 'DESCEND_TO_DROP_HEIGHT']
            zq = {r[0]: float(inp[r[0]][4]) for r in rows}
            # QUAD4's distance from the ring centre (xy)
            dist = {r[0]: math.hypot(float(inp[r[0]][2]) - float(inp[r[0]][17]), float(inp[r[0]][3]) - float(inp[r[0]][18]))
                    for r in rows}
            ein = [r[2] == '1' for r in a]
            din = [r[4] == '1' for r in a]
            corr = [float(r[6]) + float(r[7]) for r in a if r[2] == '0']
            floors = [float(r[8]) for r in a]
            settle = [r for r in a if r[1] == 'SETTLE_ABOVE_DROP_POINT']
            sp = spans([float(r[0]) - t0 for r in a], ein)
            who = {}
            for r in a:
                if r[2] == '1':
                    who[r[3]] = who.get(r[3], 0) + 1
            narrow = [r for r in a if r[2] == '0' and float(r[6]) + float(r[7]) < 0.30]
            print(f'{mode:7s} d={d:>3s}: Q4 z {min(zq[r[0]] for r in a):.2f}-{max(zq[r[0]] for r in a):.2f}; '
                  f'drop pt inside {100*sum(din)/len(din):3.0f}% (floor med {st.median(floors):.2f} max {max(floors):.2f}'
                  f'{"; settle floor %.2f-%.2f at Q4 z %.2f-%.2f" % (min(float(r[8]) for r in settle), max(float(r[8]) for r in settle), min(zq[r[0]] for r in settle), max(zq[r[0]] for r in settle)) if settle else ""}); '
                  f'path inside {100*sum(ein)/len(ein):3.0f}% '
                  f'{["%.1f-%.1f" % s for s in sp]} {sorted(who.items(), key=lambda x: -x[1])[:3]}; '
                  f'corridor (free samples) min {min(corr) if corr else float("nan"):.2f} med {st.median(corr) if corr else float("nan"):.2f}, '
                  f'<0.30 m at {len(narrow)}/{len(a)}')
            if b:
                bz = [zq[r[0]] for r in b]
                bf = [float(r[8]) for r in b]
                print(f'          descent (Python): Q4 z {min(bz):.2f}-{max(bz):.2f}, drop-point floor {min(bf):.2f}-{max(bf):.2f}, '
                      f'drop pt inside {100*sum(r[4]=="1" for r in b)/len(b):.0f}%, path inside {100*sum(r[2]=="1" for r in b)/len(b):.0f}%')
            if mode == 'octagon' and d in ('0', '1'):
                # where the flown path was inside: relative time, Q4 height and distance from the ring centre
                for s0, s1 in sp:
                    rr = [r for r in a if s0 <= float(r[0]) - t0 <= s1]
                    print(f'          inside +{s0:.1f}..+{s1:.1f} s: Q4 z {min(zq[r[0]] for r in rr):.2f}-{max(zq[r[0]] for r in rr):.2f}, '
                          f'{min(dist[r[0]] for r in rr):.2f}-{max(dist[r[0]] for r in rr):.2f} m from the ring centre, {sorted(set(r[3] for r in rr))}')
