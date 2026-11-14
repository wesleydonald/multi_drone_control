"""Closest approach on the 12 Nov direct-route drop-offs (fm9hc_2/_4/_5), from mocap (20 Hz): QUAD4 body / its cable
(body 14 -> tip 24) / the object (body 6, minus its 0.0525 m half-size) against each carrier body and carrier cable
(body - 0.04 m -> its plate on the M2A ring, r 0.225, z 0.0175, the ring's measured attitude). Window: his TAKEOFF to
DROP_OBJECT (object), and his TAKEOFF to DROP_OBJECT + 3 s (body, cable). Centre lines, no radii off except the object."""
import csv, math, sys
import numpy as np
sys.path.insert(0, 'tools')
import m1_metrics as M
R_ATT, Z_ATT, PIV, OBJ_R = 0.225, 0.0175, 0.04, 0.0525
AZ_CW = (210.0, 330.0, 90.0)          # drone_0..2 measured at 151 / 30 / -90 deg ccw in the ring frame: plates 7, 11, 3
def rot(q, v):
    w, u = q[:, :1], q[:, 1:]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)
for f in sys.argv[1:]:
    rows = list(csv.DictReader(open(f'results/analysis/cpp_realtime_2026-11-13/{f}_poses.csv')))
    col = lambda k: np.array([float(r[k]) if r[k] not in ('', 'nan') else np.nan for r in rows])
    t = col('t'); ph = [r['phase'] for r in rows]
    tk = next(t[i] for i, p in enumerate(ph) if p == 'TAKEOFF')
    td = next(t[i] for i, p in enumerate(ph) if p == 'DROP_OBJECT')
    ring = np.stack([col('r_x'), col('r_y'), col('r_z')], 1); q = np.stack([col('r_qw'), col('r_qx'), col('r_qy'), col('r_qz')], 1)
    body = np.stack([col('d3_x'), col('d3_y'), col('d3_z')], 1); tip = np.stack([col('tip_x'), col('tip_y'), col('tip_z')], 1)
    obj = np.stack([col('o_x'), col('o_y'), col('o_z')], 1)
    ok_obj = np.linalg.norm(obj - tip, axis=1) < 0.15   # the object on the magnet; mocap jumps of body 6 dropped
    out = {}
    for i, az in enumerate(AZ_CW):
        cb = np.stack([col(f'd{i}_x'), col(f'd{i}_y'), col(f'd{i}_z')], 1)
        a = math.radians(-az)
        plate = ring + rot(q, np.broadcast_to(np.array([R_ATT * math.cos(a), R_ATT * math.sin(a), Z_ATT]), ring.shape))
        piv = cb - np.array([0, 0, PIV])
        pairs = {'QUAD4 body - carrier body': np.linalg.norm(body - cb, axis=1),
                 'QUAD4 body - carrier cable': M.seg_seg(body, body, piv, plate),
                 'QUAD4 cable - carrier body': M.seg_seg(body, tip, cb, cb),
                 'QUAD4 cable - carrier cable': M.seg_seg(body, tip, piv, plate),
                 'object - carrier cable': M.seg_seg(obj, obj, piv, plate) - OBJ_R,
                 'object - carrier body': np.linalg.norm(obj - cb, axis=1) - OBJ_R}
        for k, d in pairs.items():
            end = td if k.startswith('object') else td + 3.0
            m = (t >= tk) & (t <= end) & np.isfinite(d) & (ok_obj if k.startswith('object') else True)
            j = np.argmin(np.where(m, d, np.inf))
            if k not in out or d[j] < out[k][0]:
                out[k] = (float(d[j]), i + 1, float(t[j] - tk), ph[j])
    print(f'## {f}: his TAKEOFF +{tk:.1f} s, DROP_OBJECT +{td:.1f} s (from the log start)')
    for k, (d, c, tt, p) in sorted(out.items(), key=lambda kv: kv[1][0]):
        print(f'  {k}: {d:.2f} m (carrier {c}, {tt:.1f} s after his TAKEOFF, {p})')
