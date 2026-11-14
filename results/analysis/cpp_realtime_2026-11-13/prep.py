"""Make probe2 input from <flight>_poses.csv: TRANSIT_TO_DROP_POINT start to CLEAR_DROP_ZONE start, 5 Hz.
usage: python3 prep.py <flight> <delta_s> [drop_z_mode]  -> <flight>_in_<delta>.txt
drop point: body over the ring centre at QUAD4's height at that moment (K points over [t, t+delta])."""
import bisect
import os
import csv
import math
import sys

flight, delta = sys.argv[1], float(sys.argv[2])
rows = list(csv.DictReader(open(f'{flight}_poses.csv')))
T = [float(r['t']) for r in rows]
f = lambda r, k: float(r[k])


def at(t):
    i = min(max(bisect.bisect_left(T, t), 0), len(T) - 1)
    return rows[i]


t_start = next(float(r['t']) for r in rows if r['phase'] == 'TRANSIT_TO_DROP_POINT')
t_end = next(float(r['t']) for r in rows if r['phase'] == 'CLEAR_DROP_ZONE')


def quat_rot_T(r, v):
    """R(q)^T v: v in the ring frame."""
    w, x, y, z = f(r, 'r_qw'), -f(r, 'r_qx'), -f(r, 'r_qy'), -f(r, 'r_qz')
    vx, vy, vz = v
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return (vx + 2 * (w * cx + ccx), vy + 2 * (w * cy + ccy), vz + 2 * (w * cz + ccz))


# each carrier's plate (his numbering: counter-clockwise from the ring's +x, 30 deg apart), from the ring frame
plates = []
for i in range(3):
    votes = {}
    for r in rows:
        if t_start <= float(r['t']) <= t_end:
            d = quat_rot_T(r, (f(r, f'd{i}_x') - f(r, 'r_x'), f(r, f'd{i}_y') - f(r, 'r_y'), f(r, f'd{i}_z') - f(r, 'r_z')))
            p = int(round(math.degrees(math.atan2(d[1], d[0])) / 30.0)) % 12
            votes[p] = votes.get(p, 0) + 1
    plates.append(max(votes, key=votes.get))
yaws = [math.degrees(2 * math.atan2(f(r, 'r_qz'), f(r, 'r_qw'))) for r in rows if t_start <= float(r['t']) <= t_end]
print(f'{flight}: window {t_start:.2f}-{t_end:.2f} s, plates {plates}, ring yaw {min(yaws):.1f}..{max(yaws):.1f} deg',
      file=sys.stderr)

K = 5
import os
BRIDGE = bool(os.environ.get('BRIDGE'))
out = open(f'{flight}_in_{delta:g}{"_br" if BRIDGE else ""}{"_z" + os.environ["DROPZ"] if os.environ.get("DROPZ") else ""}.txt', 'w')
t = t_start
while t <= t_end:
    r0 = at(t)
    ts = [t + delta * k / (K - 1) for k in range(K)]
    rs = [at(x) for x in ts]
    ego = [(f(r, 'd3_x'), f(r, 'd3_y'), f(r, 'd3_z')) for r in rs]
    zq = float(os.environ['DROPZ']) if os.environ.get('DROPZ') else f(r0, 'd3_z')
    drop = [(f(r, 'r_x'), f(r, 'r_y'), zq) for r in rs]
    if delta > 0:
        r1 = rs[-1]
        vel = ((f(r1, 'r_x') - f(r0, 'r_x')) / delta, (f(r1, 'r_y') - f(r0, 'r_y')) / delta, 0.0)
    else:
        vel = (0.0, 0.0, 0.0)
    vals = [f'{t:.2f}', r0['phase']]
    for p in ego + drop:
        vals += [f'{x:.4f}' for x in p]
    vals += [f'{f(r0, k):.5f}' for k in ('r_x', 'r_y', 'r_z', 'r_qw', 'r_qx', 'r_qy', 'r_qz')]
    vals += [f'{x:.4f}' for x in vel]
    for i in range(3):
        if BRIDGE:
            # ring_bridge's carriers: ring + Rz(yaw) (0.725 at the plate's azimuth, attach_z + 0.866)
            yaw = math.atan2(2 * (f(r0, 'r_qw') * f(r0, 'r_qz') + f(r0, 'r_qx') * f(r0, 'r_qy')),
                             1 - 2 * (f(r0, 'r_qy') ** 2 + f(r0, 'r_qz') ** 2))
            a = yaw + math.radians(30 * plates[i])
            vals += [f'{f(r0, "r_x") + 0.725 * math.cos(a):.4f}', f'{f(r0, "r_y") + 0.725 * math.sin(a):.4f}',
                     f'{f(r0, "r_z") + 0.025 + 0.866:.4f}']
        else:
            vals += [f'{f(r0, f"d{i}_{k}"):.4f}' for k in ('x', 'y', 'z')]
    vals += [str(p) for p in plates]
    out.write(' '.join(vals) + '\n')
    t += 0.2
out.close()
