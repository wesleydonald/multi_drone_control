"""Moving-ring replay summary up to capture: capture time after authority, z max, closest (driver's modelled carriers)
body / cable-tether before capture, accepted share and LATE before capture. usage: python3 full/precap2.py <run>..."""
import csv
import sys
for run in sys.argv[1:]:
    d = list(csv.DictReader(open(f'full/runs/{run}/driver.csv')))
    auth = next((float(r['t']) for r in d if r['auth'] == '1'), None)
    cap = next((float(r['t']) for r in d if r['captured'] == '1'), None)
    end = cap or float(d[-1]['t'])
    pre = [r for r in d if auth and auth <= float(r['t']) <= end]
    rows = [r for r in csv.DictReader(open(f'full/runs/{run}/replans.csv')) if auth and auth <= float(r['request_ros_time']) <= end]
    f = lambda k: [float(r[k]) for r in pre if r[k] not in ('', 'nan')]
    acc = sum(r['status'].startswith('ACCEPTED') for r in rows)
    late = sum(r['candidate_late'] == '1' for r in rows)
    import numpy as np
    T = np.array([float(r['t']) for r in d]); RX = np.array([[float(r['ring_x']), float(r['ring_y'])] for r in d])
    V = np.r_[0, np.linalg.norm(np.diff(RX, axis=0), axis=1) / np.maximum(np.diff(T), 1e-3)]
    V = np.convolve(V, np.ones(25) / 25, mode='same')
    vcap = V[np.argmin(np.abs(T - cap))] if cap else float('nan')
    vmov = V[(T >= (auth or T[0])) & (T <= end)]
    print(f'{run}: ring speed at capture {vcap:.3f} m/s (during the approach median {np.median(vmov) if len(vmov) else float("nan"):.3f}); captured ' + (f'+{cap - auth:.1f} s after authority' if cap else f'NEVER ({end - auth:.0f} s)')
          + (f'; z max {max(f("z")):.2f}; closest body {min(f("min_body_m")):.2f} m, cable-tether {min(f("min_cable_tether_m")):.2f} m'
             if pre else '') + f'; replans {len(rows)}, accepted {acc}, late {late}')
