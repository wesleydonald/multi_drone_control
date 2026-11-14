"""Align a replay's driver.csv with the flight's poses CSV (by the ring track) and report where the virtual QUAD4
flew relative to the measured carriers: highest body z, and the closest horizontal approach to a carrier body while
QUAD4 was above the carriers' tethers (z > carrier z)."""
import bisect, csv, math, sys
run, flight, req = sys.argv[1], sys.argv[2], float(sys.argv[3])
d = list(csv.DictReader(open(f'full/runs/{run}/driver.csv')))
p = list(csv.DictReader(open(f'{flight}_poses.csv')))
pt = [float(r['t']) for r in p]
def at(t):
    i = min(max(bisect.bisect_left(pt, t), 0), len(pt) - 1); return p[i]
dt0 = float(d[0]['t'])
best = None
for off in [req - 4.0 + x * 0.02 for x in range(250)]:
    err = 0.0
    for r in d[::20]:
        q = at(float(r['t']) - dt0 + off)
        err += math.hypot(float(r['ring_x']) - float(q['r_x']), float(r['ring_y']) - float(q['r_y']))
    if best is None or err < best[0]:
        best = (err, off)
off = best[1]
print(f'{run}: aligned at bag t {off:.2f} s (mean ring mismatch {best[0]/len(d[::20])*1000:.1f} mm)')
zmax = max(float(r['z']) for r in d)
closest = None
for r in d:
    if r['auth'] != '1':
        continue
    q = at(float(r['t']) - dt0 + off)
    x, y, z = float(r['x']), float(r['y']), float(r['z'])
    for i in range(3):
        h = math.hypot(x - float(q[f'd{i}_x']), y - float(q[f'd{i}_y']))
        if closest is None or h < closest[0]:
            closest = (h, i, z - float(q[f'd{i}_z']), float(r['t']) - dt0)
print(f'  highest body z {zmax:.2f} m; closest horizontal approach to a carrier body {closest[0]:.2f} m (drone_{closest[1]}) '
      f'with QUAD4 {closest[2]:+.2f} m above it at +{closest[3]:.1f} s')
