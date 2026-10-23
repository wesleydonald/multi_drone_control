"""tools/catch_offset.py (sim U, card docs/experiments/2026-10-23_catch_offset.md) on synthetic run directories, and
n2_bands.path_metrics_clock_free's t_from."""
import json
import math
import os
import sys

import numpy as np
import pandas as pd
import pytest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import catch_offset as C  # noqa: E402
import n2_bands  # noqa: E402

P = np.array([0.05, 0.215, 0.045])           # the tip in the ring frame after the weld
RING = np.array([0.3, -0.2, 1.0])
YAW, ROLL = math.radians(10.0), math.radians(2.0)


def _ring_R():
    cz, sz, cx, sx = math.cos(YAW), math.sin(YAW), math.cos(ROLL), math.sin(ROLL)
    return np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])


def _ring_q():
    qz = np.array([math.cos(YAW / 2), 0.0, 0.0, math.sin(YAW / 2)])
    qx = np.array([math.cos(ROLL / 2), math.sin(ROLL / 2), 0.0, 0.0])
    w1, x1, y1, z1 = qz
    w2, x2, y2, z2 = qx
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def make_run(tmp_path, t_weld=60.0, t_path=100.0, descent=True, events=None, log=None, args=None, high_after_weld=0.03):
    """A 3+1 attach run: hover, the weld (a 5 cm datum step in payload_ref), the ring 3 cm high until the path, a
    figure-8 from t_path followed exactly, the planner's descent at t_path + 40, LAND 1 s later."""
    d = tmp_path / 'R9001_sim_gz_u_meas_t04'
    (d / 'logs').mkdir(parents=True)
    t = np.round(np.arange(0.0, 160.0, 0.02), 3)
    ref = np.tile(RING, (t.size, 1))
    ref[t >= t_weld, 0] += 0.05
    tau = np.clip(t - t_path, 0.0, 40.0)
    w = 2 * math.pi / 40.0
    ref[:, 0] += np.where(t >= t_path, 0.5 * np.sin(w * tau), 0.0)
    ref[:, 1] += np.where(t >= t_path, 0.25 * np.sin(2 * w * tau), 0.0)
    if descent:
        ref[:, 2] -= np.where(t > t_path + 40.0, 0.2 * (t - t_path - 40.0), 0.0)
    pay = ref.copy()
    pay[(t >= t_weld) & (t < t_path), 2] += high_after_weld
    R, q = _ring_R(), _ring_q()
    tip = pay + R @ P
    run = {'t': t, 'wall': t * 4.0}
    for i, a in enumerate('xyz'):
        run[f'payload_{a}'], run[f'payload_ref_{a}'], run[f'tip_{a}'] = pay[:, i], ref[:, i], tip[:, i]
    for i, a in enumerate('wxyz'):
        run[f'payload_q{a}'] = np.full(t.size, q[i])
    run['payload_tilt_deg'] = np.full(t.size, 2.0)
    for k in range(3):
        az = math.radians(90.0 + 120.0 * k)
        for i, a in enumerate('xyz'):
            run[f'd{k}_{a}'] = pay[:, i] + (0.8 * math.cos(az), 0.8 * math.sin(az), 0.9)[i]
        run[f'd{k}_tilt_deg'] = np.full(t.size, 3.0)
    for i, a in enumerate('xyz'):
        run[f'd3_{a}'] = tip[:, i] + (0.0, 0.0, 1.04)[i]
    run['d3_tilt_deg'] = np.full(t.size, 3.0)
    run['d3_thr'] = np.full(t.size, 0.4)
    run['d3_tension'] = np.full(t.size, 1.6)
    pd.DataFrame(run).to_csv(d / 'logs' / 'run.csv', index=False)
    if events is None:
        events = [(20.0, 'MAGNET', 'ON'), (t_weld, 'WELD', 'magnet/object_attached'), (t_weld + 0.15, 'FOLD_IN', '3'),
                  (t_path, 'PATH_START', 'fig_8'), (t_path + 41.0, 'LAND', ''), (t_path + 55.0, 'LANDED', '')]
    with open(d / 'logs' / 'events.csv', 'w') as fh:
        fh.write('sim_time,event,arg\n' + ''.join(f'{a:.3f},{b},{c}\n' for a, b, c in sorted(events)))
    if log is not None:
        (d / 'logs' / 'launch.log').write_text(log)
    la = {'num_drones': 3, 'attach_x_offset': 0.0, 'attach_y_offset': 0.225, 'weld_point_z': 0.04,
          'catch_offset_t': 0.04, 'catch_offset_r': 0.0, 'attach_point_source': 'measured', **(args or {})}
    (d / 'manifest.json').write_text(json.dumps({'launch_args': la, 'failures': []}))
    return str(d)


def test_the_true_catch_point_in_a_yawed_and_tilted_ring(tmp_path):
    r = C.analyse(make_run(tmp_path))
    assert np.allclose(r['rho_true'], P, atol=1e-9)
    assert r['d_true_m'] == pytest.approx(math.hypot(0.05, 0.01), abs=1e-9)
    assert r['d_rad_m'] == pytest.approx(math.hypot(0.05, 0.215) - 0.225, abs=1e-9)
    assert r['d_tan_m'] == pytest.approx(0.0513, abs=1e-4)                 # clockwise from plate 9: +
    assert r['dz_m'] == pytest.approx(0.005, abs=1e-9)
    assert r['aim'] == pytest.approx([0.0398, 0.2214], abs=1e-4)
    assert r['aim_err_m'] == pytest.approx(math.hypot(0.05 - r['aim'][0], 0.215 - r['aim'][1]), abs=1e-9)
    assert (r['arm'], r['direction'], r['d_cm']) == ('measured', 't', 4)
    assert C.verdict(r) == 'complete' and C.within_bar(r)
    assert r['fold_tilt_peak_deg'] == pytest.approx(2.0) and r['separation_min_m'] > 0.3


def test_the_windows_come_from_the_events_and_the_path_starts_at_the_path_not_the_weld(tmp_path):
    d = make_run(tmp_path)
    r = C.analyse(d)
    assert (r['t_weld'], r['t_fold'], r['t_path']) == (60.0, 60.15, 100.0)
    assert r['steady_s'] == pytest.approx(100.0 - 0.5 - 75.0)
    assert r['steady_err_z_m'] == pytest.approx(0.03, abs=1e-9)          # the ring 3 cm high in the 4-quad hover
    assert r['path_err_m'] == pytest.approx(0.0, abs=1e-6)               # the figure-8 followed exactly
    whole, _ = n2_bands.path_metrics_clock_free(d)                       # from the weld's datum step: the hover's 3 cm
    assert whole > 0.01
    assert n2_bands.path_metrics_clock_free(d, t_from=None) == n2_bands.path_metrics_clock_free(d)
    assert n2_bands.path_metrics_clock_free(d, t_from=99.5)[0] == pytest.approx(0.0, abs=1e-6)
    ev = [e for e in C.events(d) if e[1] != 'PATH_START']            # no PATH_START: the first ref step from weld + 5
    assert C.path_start(pd.read_csv(os.path.join(d, 'logs', 'run.csv')), ev, 60.0) == pytest.approx(100.02)


def test_failure_reasons(tmp_path):
    ev = [(20.0, 'MAGNET', 'ON'), (110.0, 'WELD_TIMEOUT', '90.00'), (141.0, 'LAND', ''), (155.0, 'LANDED', '')]
    r = C.analyse(make_run(tmp_path / 'a', events=ev))
    assert r['failures'][0].startswith('no weld') and C.verdict(r) == 'FAIL'
    log = ('[dissipative_planner-17] [ERROR] [dissipative_planner]: [dissipative] DROP AND LAND: tilt\n'
           '[dissipative_planner-17] [INFO] [dissipative_planner]: [planner] load trajectory complete\n')
    r = C.analyse(make_run(tmp_path / 'b', log=log))
    assert r['failures'] == ['DROP AND LAND']
    r = C.analyse(make_run(tmp_path / 'c', descent=False))
    assert r['failures'] == ['figure-8 not completed before LAND']
    r = C.analyse(make_run(tmp_path / 'd', events=[(20.0, 'MAGNET', 'ON'), (60.0, 'WELD', ''), (61.0, 'LAND', '')]))
    assert 'no fold-in within 2 s of the weld' in r['failures'] and 'not LANDED' in r['failures']


def _late(t_magnet):
    return [(t_magnet, 'MAGNET', 'ON'), (90.0, 'WELD', ''), (90.15, 'FOLD_IN', '3'), (100.0, 'PATH_START', 'fig_8'),
            (141.0, 'LAND', ''), (155.0, 'LANDED', '')]


def test_void_when_the_weld_is_late(tmp_path):
    r = C.analyse(make_run(tmp_path, t_weld=90.0, events=_late(50.0)))    # 40 s after MAGNET ON, 10 s before the path
    assert r['void'] and not r['failures'] and C.verdict(r) == 'VOID' and not C.within_bar(r)


def test_model_error_from_the_logged_attach_points(tmp_path):
    rho = [[0.19, -0.11, 0.02], [0.0, 0.225, 0.04]]
    log = ('[dissipative_planner-17] [INFO] [dissipative_planner]: [dissipative] RESIZE_INPUTS '
           + json.dumps({'what': 'attach', 'drone': 3, 'n': 4, 'rho': rho}) + '\n'
           '[dissipative_planner-17] [WARN] [dissipative_planner]: [dissipative] ATTACH drone 3 (OCP resize): n=4, '
           'ring gap 120 deg; point (+0.0000,+0.2250,+0.0400) m, slot assumed\n'
           '[dissipative_planner-17] [INFO] [dissipative_planner]: [planner] load trajectory complete\n')
    r = C.analyse(make_run(tmp_path, log=log, args={'attach_point_source': 'slot'}))
    assert r['e_model_xy_m'] == pytest.approx(math.hypot(0.05, 0.01), abs=1e-9)
    assert r['e_model_m'] == pytest.approx(math.hypot(0.05, 0.01, 0.005), abs=1e-9)
    assert r['arm'] == 'slot' and r['logged_point'] == 'slot' and not r['notes']


def _row(arm, direction, d, ok=True, peak=2.0, run='R1'):
    return {'arm': arm, 'direction': direction, 'd_cm': d, 'void': [], 'failures': [] if ok else ['DROP AND LAND'],
            'fold_tilt_peak_deg': peak, 'run': run}


def test_d_tol_needs_both_repeats_within_the_bar_up_to_d():
    rows = [_row(a, 'd0', 0) for a in ('measured', 'slot') for _ in range(2)]
    rows += [_row('measured', 't', 2), _row('measured', 't', 2), _row('measured', 't', 4),
             _row('measured', 't', 4, False), _row('measured', 't', 6), _row('measured', 't', 6)]
    rows += [_row('slot', 't', 2), _row('slot', 't', 2, peak=6.0), _row('measured', 'ri', 2)]
    t = C.table(rows)
    assert t['d_tol'] == {'measured': {'t': 2, 'ri': 0}, 'slot': {'t': 0}}
    assert t['cells']['measured']['t'][4]['n_ok'] == 1
    assert 'd_tol' in C.markdown(t)


def test_a_failure_takes_precedence_over_void(tmp_path):
    """Critic M1: a late weld that also dropped is a FAIL (counted in d_tol), not a VOID re-flown into a pass."""
    log = ('[dissipative_planner-17] [ERROR] [dissipative_planner]: [dissipative] DROP AND LAND: tilt\n'
           '[dissipative_planner-17] [INFO] [dissipative_planner]: [planner] load trajectory complete\n')
    r = C.analyse(make_run(tmp_path / 'a', t_weld=90.0, events=_late(50.0), log=log))
    assert r['void'] and r['failures'] == ['DROP AND LAND'] and C.verdict(r) == 'FAIL'
    r = C.analyse(make_run(tmp_path / 'b', t_weld=90.0, events=_late(20.0)))  # 70 s after MAGNET ON: failure 1 shows
    assert r['void'] and r['failures'] == ['no weld within 60 s of MAGNET ON'] and C.verdict(r) == 'FAIL'
    rows = [_row(a, 'd0', 0) for a in ('measured', 'slot') for _ in range(2)]
    late = dict(_row('measured', 't', 2), void=['weld late'])
    late_and_failed = dict(_row('measured', 't', 2, ok=False), void=['weld late'])
    t = C.table(rows + [_row('measured', 't', 2), late, late_and_failed])
    c = t['cells']['measured']['t'][2]
    assert (c['n'], c['void'], c['n_ok']) == (2, 1, 1) and t['d_tol']['measured']['t'] == 0


def test_the_figure_8_must_complete_before_the_planners_land(tmp_path):
    land = ("[dissipative_planner-17] [INFO] [dissipative_planner]: [planner] /fleet/command received: 'LAND' "
            "(phase=planner)\n")
    done = '[dissipative_planner-17] [INFO] [dissipative_planner]: [planner] load trajectory complete\n'
    assert not C.analyse(make_run(tmp_path / 'a', log=done + land))['failures']
    assert C.analyse(make_run(tmp_path / 'b', log=land + done))['failures'] == ['figure-8 not completed before LAND']
    assert C.analyse(make_run(tmp_path / 'c', log=land))['failures'] == ['figure-8 not completed before LAND']


def test_separation_is_scored_apart_as_d_sep(tmp_path):
    """Critic B1: crowding does not end d_tol (the arms are compared on the tilt bar and failures 1-6); it ends d_sep,
    and the task limit is the smaller."""
    r = C.analyse(make_run(tmp_path, args={'catch_offset_t': 0.16}))
    assert r['separation_min_m'] > C.SEPARATION_M and not r['crowded']
    rows = [dict(_row(a, 'd0', 0), separation_min_m=0.7) for a in ('measured', 'slot') for _ in range(2)]
    for d, sep in ((4, 0.6), (8, 0.45), (12, 0.33), (16, 0.22)):
        rows += [dict(_row('measured', 't', d), separation_min_m=sep, crowded=['x'] if sep < 0.3 else [])] * 2
        rows += [dict(_row('slot', 't', d), separation_min_m=0.7)] * 2
    t = C.table(rows)
    assert t['d_tol'] == {'measured': {'t': 16}, 'slot': {'t': 16}}
    assert t['d_sep'] == {'measured': {'t': 12}, 'slot': {'t': 16}}
    assert t['d_task'] == {'measured': {'t': 12}, 'slot': {'t': 16}}
    assert C.verdict(rows[-4]) == 'complete' and 'd_sep' in C.markdown(t)


def test_d_tol_on_the_true_offset_run_by_run():
    rows = [dict(_row('measured', 'd0', 0), d_true_m=0.012)] * 2
    rows += [dict(_row('measured', 't', 2), d_true_m=0.035), dict(_row('measured', 't', 2), d_true_m=0.004),
             dict(_row('measured', 't', 4), d_true_m=0.031), dict(_row('measured', 't', 4, peak=6.0), d_true_m=0.05)]
    t = C.table(rows)
    assert t['d_tol']['measured']['t'] == 2 and t['d_tol_true']['measured']['t'] == 3.5


def test_the_mechanism_bar():
    """Critic M4: the slot arm's excess tilt must grow past the d = 0 repeat spread at the two largest d."""
    def rows(slot_peak):
        out = []
        for a in ('measured', 'slot'):
            out += [dict(_row(a, 'd0', 0, peak=p), steady_tilt_mean_deg=1.0) for p in (2.0, 2.3)]
            for d in (4, 8, 16):
                p = 2.1 + (slot_peak(d) if a == 'slot' else 0.0)
                out += [dict(_row(a, 'ri', d, peak=p), steady_tilt_mean_deg=1.0)] * 2
        return out
    m = C.table(rows(lambda d: 0.1 * d))['mechanism']['ri']
    assert m['fold_tilt_peak_deg']['spread0'] == pytest.approx(0.3)
    assert m['fold_tilt_peak_deg']['shown'] and m['steady_tilt_mean_deg']['shown'] is False and m['shown']
    m = C.table(rows(lambda d: 0.05 if d == 16 else 0.0))['mechanism']['ri']
    assert m['shown'] is False
    assert 'the model point does not matter' in C.markdown(C.table(rows(lambda d: 0.0)))
    assert C.table(rows(lambda d: 0.0)[:2])['mechanism']['ri']['shown'] is None


def test_the_wrong_plate_chord_keeps_its_own_cell():
    assert C.direction_of({'catch_offset_t': 0.11647}) == ('t', 11.6)
    assert C.direction_of({'catch_offset_t': 0.12}) == ('t', 12)
    assert C.direction_of({'catch_offset_r': -0.04}) == ('ri', 4)
