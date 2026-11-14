"""Ring-centred free-space maps (his hulls, octagon) at the settle instant; connectivity of the drop point.
usage: python3 maps.py run|analyse"""
import os
import subprocess
import sys

import numpy as np
from scipy import ndimage

flights = ['fm9_5', 'fm9hc_2', 'fm9hc_4', 'fm9hc_5']
HALF, STEP = 1.2, 0.02


def settle_line(fl, d):
    lines = [l for l in open(f'{fl}_in_{d:g}.txt')]
    for l in lines:
        if l.split()[1] == 'SETTLE_ABOVE_DROP_POINT':
            return l
    raise SystemExit(f'no settle in {fl}')


def settle_z(fl):
    zs = [float(l.split()[4]) for l in open(f'{fl}_in_0.txt') if l.split()[1] == 'SETTLE_ABOVE_DROP_POINT']
    return float(np.median(zs))


jobs = []
for fl in flights:
    for d in (0, 1.0):
        for z in (round(settle_z(fl), 2), 2.76, 2.90):
            for mode in ('octagon',):
                jobs.append((fl, d, z, mode))

if sys.argv[1] == 'run':
    os.makedirs('maps', exist_ok=True)
    procs = []
    for fl, d, z, mode in jobs:
        lf = f'maps/{fl}_{d:g}_line.txt'
        open(lf, 'w').write(settle_line(fl, d))
        env = dict(os.environ, MAPZ=str(z), MAPHALF=str(HALF), MAPSTEP=str(STEP))
        out = open(f'maps/{fl}_{mode}_{d:g}_{z:.2f}.txt', 'w')
        procs.append(subprocess.Popen(['nice', '-n', '10', '/dev/shm/rigcheck_build/probe3', mode, str(d), lf],
                                      stdout=out, env=env))
        if len(procs) >= 8:
            procs.pop(0).wait()
    for p in procs:
        p.wait()
else:
    for fl, d, z, mode in jobs:
        txt = open(f'maps/{fl}_{mode}_{d:g}_{z:.2f}.txt').read().splitlines()
        hdr = [l for l in txt if l.startswith('# t')][0]
        grid = np.array([[c == '#' for c in l] for l in txt if l and l[0] in '#.' and not l.startswith('# t')])
        free = ~grid
        n = grid.shape[0]
        ci = n // 2
        clear = ndimage.distance_transform_edt(free) * STEP      # distance to the nearest blocked cell (m)
        border = np.zeros_like(free)
        border[0, :] = border[-1, :] = border[:, 0] = border[:, -1] = True
        def connected(th):
            lab, _ = ndimage.label(clear >= th - 1e-9)
            l0 = lab[ci, ci]
            return l0 > 0 and bool(np.any(lab[border] == l0))
        best = 0.0
        if free[ci, ci]:
            for th in np.arange(STEP, 0.6, STEP / 2):
                if connected(th):
                    best = th
                else:
                    break
        # the free pocket around the drop point: the largest disc of free cells centred on it
        pocket = clear[ci, ci]
        within12 = free[ci - 6:ci + 7, ci - 6:ci + 7]
        print(f'{fl:8s} d={d:<3g} z={z:.2f} {hdr[2:40]}: drop pt {"free" if free[ci, ci] else "INSIDE"}, '
              f'clearance at the drop pt {pocket:.2f} m, free cells within 0.12 m {100*within12.mean():.0f}%, '
              f'connected to the map edge with clearance >= {best:.2f} m (corridor ~{2*best:.2f} m), '
              f'blocked share of map {100*grid.mean():.0f}%')
