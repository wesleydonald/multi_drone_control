"""E5 (rejoin card, the leaning-pair neighbour): E2's metrics (as R2589/R2594's rows) plus the pair's signed lean toward
plate 3 over the 0.5 s before REJOIN_WELD (pair_lean.py's reading: + when plate 3's side is down)."""
import glob, re, sys
import numpy as np, pandas as pd
for rid in sys.argv[1:]:
    D = glob.glob(f'/home/wesley/multi_drone_control/results/2026-10-23/{rid}_*')[0]
    ev = pd.read_csv(D + '/logs/events.csv').dropna(subset=['sim_time']); E = {}
    for r in ev.itertuples(): E.setdefault(r.event, r.sim_time)
    run = pd.read_csv(D + '/logs/run.csv').drop_duplicates('t').reset_index(drop=True)
    t = run.t.values; tilt = run.payload_tilt_deg.values
    w_, x_, y_, z_ = (run[f'payload_q{c}'].to_numpy() for c in 'wxyz')
    toward3 = np.degrees(np.arcsin(np.clip(2 * (y_ * z_ + w_ * x_), -1, 1)))   # plate 3 at body -y
    toward6 = np.degrees(np.arcsin(np.clip(2 * (x_ * z_ - w_ * y_), -1, 1)))
    tw, tl, tb, tf = E['REJOIN_WELD'], E['LAND'], E['REJOIN_BLEND_OVER'], E['FOLD_IN']
    print(f'===== {rid} {D.split("_sim_gz_")[1]}')
    print('events', {k: round(v, 3) for k, v in E.items()})
    print('(a) %.2f  (b) %.3f  weld-start %.2f  blend %.2f' % (E['REJOIN_ON_PLATE'] - E['REJOIN_START'],
          E['JOINT_ATTACHED'] - E['REJOIN_ON_PLATE'], tw - E['REJOIN_START'], tb - tw))
    m = (t >= tw) & (t <= tw + 15); i = np.argmax(np.where(m, tilt, -1)); over = t[m & (tilt > 5)]
    print('tilt weld..+15 max %.2f at +%.2f; over 5: %s' % (tilt[i], t[i] - tw,
          ('%.2f..%.2f (%d rows)' % (over[0] - tw, over[-1] - tw, len(over))) if len(over) else 'never'))
    for a, b in ((0, 1.5), (1.5, 6), (6, 10.1), (10.1, 15)):
        mm = (t >= tw + a) & (t <= tw + b)
        print('  weld+%.1f..%.1f tilt max %.2f mean %.2f | toward3 mean %+.2f min %+.2f max %+.2f' % (
            a, b, tilt[mm].max(), tilt[mm].mean(), toward3[mm].mean(), toward3[mm].min(), toward3[mm].max()))
    ml = (t >= tl - 10) & (t < tl)
    print('lean last 10 s before LAND: tilt mean %.2f max %.2f | toward3 mean %+.2f' % (tilt[ml].mean(), tilt[ml].max(), toward3[ml].mean()))
    ma = (t >= E['ATTACH'] - 10) & (t < E['ATTACH'])
    print('pair 10 s before ATTACH: tilt mean %.2f max %.2f | toward3 mean %+.2f min %+.2f max %+.2f | toward6 mean %+.2f' % (
        tilt[ma].mean(), tilt[ma].max(), toward3[ma].mean(), toward3[ma].min(), toward3[ma].max(), toward6[ma].mean()))
    mw = (t >= tw - 0.5) & (t < tw)
    print('pair 0.5 s before WELD: toward3 mean %+.2f (min %+.2f max %+.2f), tilt mean %.2f' % (
        toward3[mw].mean(), toward3[mw].min(), toward3[mw].max(), tilt[mw].mean()))
    mwr = (t >= E['WRENCH'] + 5) & (t < E['ATTACH'])
    print('wrench+5 s..ATTACH: toward3 mean %+.2f (p-p %.2f)' % (toward3[mwr].mean(), np.ptp(toward3[mwr])))
    mpre = (t >= E['RELEASE'] + 5) & (t < E['WRENCH'])
    if mpre.any():
        print('release+5..wrench: toward3 mean %+.2f' % toward3[mpre].mean())
    mr = (t >= E['RELEASE']) & (t <= E['ATTACH']); print('release->attach tilt max %.2f' % tilt[mr].max())
    ms = (t >= E['REJOIN_START']) & (t < tw); print('rejoin start..weld tilt max %.2f' % tilt[ms].max())
    # whole-flight peak
    k = int(np.argmax(tilt)); print('whole-flight tilt max %.2f at %.2f (LIFTED %.2f)' % (tilt[k], t[k], E.get('LIFTED', np.nan)))
    # the rejoiner's reference through the blend (drone 1)
    rz, bz = run.d1_ref_z.values, run.d1_z.values
    iw = np.searchsorted(t, tw); ib = np.searchsorted(t, tb); mb = (t >= tw) & (t <= tb)
    print('ref z at weld %.3f (body %.3f); ref max in blend %.3f, min %.3f; at blend end %.3f; body max %.3f' % (
        rz[iw], bz[iw], rz[mb].max(), rz[mb].min(), rz[ib], bz[mb].max()))
    for dt in (0, 0.3, 0.5, 1.0, 2, 5, 9, 10.1, 10.5, 11.0, 11.5, 12, 13):
        j = np.searchsorted(t, tw + dt)
        if j < len(t):
            print('   +%4.1f: ref %.3f body %.3f tilt %.2f toward3 %+.2f | pair ref %.3f %.3f' % (
                dt, rz[j], bz[j], tilt[j], toward3[j], run.d0_ref_z.values[j], run.d2_ref_z.values[j]))
    # planner: the rejoiner's tension at the blend end vs the rider share before the release
    L = pd.read_csv(glob.glob(D + '/logs/dissipative_planner/*/log.csv')[0])
    n = L.n.values; up = np.where((n[1:] == 3) & (n[:-1] == 2))[0]; off = L.sim_time.values[up[-1] + 1] - tw
    ts = L.sim_time.values - off; s2d = str(L.slot2drone.iloc[up[-1] + 5]).split(); slot = s2d.index('1')
    j = np.argmin(abs(ts - tb)); T = L[f't{slot}'].values
    pre = (ts >= E['RELEASE'] - 10) & (ts < E['RELEASE'])
    share = L.t1.values[pre].mean()
    print('planner offset %.2f, rejoiner slot %d, T at blend end %.3f vs share %.3f (%+.1f %%); all three' % (
        off, slot, T[j], share, (T[j] / share - 1) * 100), [round(L[f't{k}'].values[j], 3) for k in range(3)])
    pl = (ts >= tl - 10) & (ts < tl); print('  last 10 s planned means', [round(L[f't{k}'].values[pl].mean(), 3) for k in range(3)])
    # disarm / landed
    arm = [c for c in run.columns if re.fullmatch(r'd\d+_armed', c)]
    dis = []
    for c in arm:
        a = run[c].values > 0.5; e = np.flatnonzero(a[:-1] & ~a[1:])
        dis.append((c, [round(t[o + 1], 2) for o in e], [round(run[c.replace('armed', 'z')].values[o + 1], 3) for o in e]))
    print('LANDED %.2f; disarm edges' % E['LANDED'], dis)
    end = run.iloc[-1]; print('end', [(i, round(end[f'd{i}_z'], 3), end[f'd{i}_armed']) for i in range(3)], 'payload z %.3f' % end.payload_z)
    log = open(D + '/logs/launch.log', errors='replace').read()
    for pat in ('REJOIN BLEND OVER', 'REJOIN_REFUSED', 'REJOIN REFUSED', 'REJOIN ABORT', 'DROP AND LAND', 'ENVELOPE FAULT',
                'FLEET ABORT', 'EMERGENCY', 'disarmed in flight', 'trajectory held 10.0 s'):
        hits = [l.split('] ', 2)[-1].split(']: ')[-1][:200] for l in log.splitlines() if pat in l]
        print(f'  [{pat}] {len(hits)}', hits[:1])
