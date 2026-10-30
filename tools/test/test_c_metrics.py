"""tools/c_metrics.py (card 2026-10-23_paper_method_compare) on synthetic run directories whose answers are known: the
hover window from the hand-over, the line's clock-free error, reach and lag, the arm from the parameters that flew,
the card's validity rule, the registry row and the table."""
import csv
import json
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import c_metrics as C  # noqa: E402

T_TOP = 20.0


def _shuttle(t, d, v):
    w = 2.0 * v / d
    return 0.5 * d * (1.0 - np.cos(w * np.maximum(t, 0.0)))


def _make(tmp, rid='R9001', arm='planner', case='line', v=0.4, d=0.9, cycles=2, lag=0.3, amp=1.0, shift=0.01,
          tilt_at=None, abort_at=None, log_lines=(), dip_mm=0.0, keep=1.0, path_event=True, yaw_rate=0.0,
          standing=0.0, handover_event=True, path_at=5.0, world=None):
    """One twin run: the lift tops out at 20 s, the network arm hands over 1.5 s later (shifting its datum, and the
    ring with it, by `shift` m in x), a line starts at 25 s for `cycles` whole cycles, the ring `lag` s late at `amp`
    of its length. `standing`: the ring that far off its target in x all along; the network's datum moves onto it at
    the hand-over, as dissipative_node does."""
    root = tmp / '2026-10-23' / f'{rid}_sim_gz_c_{arm}_{case}'
    (root / 'logs' / 'dissipative_planner' / '20261023_000000').mkdir(parents=True)
    net = arm != 'planner'
    t_ho, t_p0 = T_TOP + 1.5, T_TOP + path_at
    T = math.pi * d / v
    t_land = t_p0 + cycles * T + 0.05 if case == 'line' else t_ho + 63.0
    t = np.arange(0.0, t_land + 8.0, 0.02)
    ref = np.zeros((t.size, 3))
    ref[:, 2] = np.clip(0.02 + (t - 5.0) / (T_TOP - 5.0) * 0.98, 0.02, 1.0)
    if net:
        ref[t >= t_ho, 0] += shift
    act = ref.copy()
    act[:, 0] += standing
    if net:
        ref[t >= t_ho, 0] += standing
    if case == 'line':
        tick = np.floor((t - t_p0) / 0.1) * 0.1
        moving = t >= t_p0
        ref[moving, 0] += _shuttle(tick[moving], d, v)
        act[moving, 0] += amp * _shuttle(t[moving] - t_p0 - lag, d, v)
    act[(t >= t_ho) & (t < t_ho + 1.0), 2] -= dip_mm / 1000.0
    tilt = np.full(t.size, 1.0)
    if tilt_at is not None:
        tilt[(t >= tilt_at) & (t < tilt_at + 0.1)] = 35.0
    keep_rows = np.ones(t.size, bool)
    if keep < 1.0:
        gap = (t > t_p0 + 1.0) & (t < t_p0 + 1.0 + (1.0 - keep) * (t_land - t_p0) * 1.2)
        keep_rows &= ~gap
    yaw = np.radians(yaw_rate) * np.maximum(t - t_ho, 0.0)          # the ring turning from the hand-over on
    cols = {'t': t, 'payload_x': act[:, 0], 'payload_y': act[:, 1], 'payload_z': act[:, 2], 'payload_tilt_deg': tilt,
            'payload_qw': np.cos(yaw / 2), 'payload_qx': 0 * t, 'payload_qy': 0 * t, 'payload_qz': np.sin(yaw / 2),
            'payload_ref_x': ref[:, 0], 'payload_ref_y': ref[:, 1], 'payload_ref_z': ref[:, 2]}
    for i in range(4):
        cols.update({f'd{i}_x': act[:, 0], f'd{i}_tilt_deg': np.full(t.size, 5.0 + i),
                     f'd{i}_thr': np.full(t.size, 0.6)})
    with open(root / 'logs' / 'run.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for k in np.where(keep_rows)[0]:
            w.writerow([f'{cols[c][k]:.6f}' for c in cols])
    ev = [(3.0, 'ARM', ''), (6.0, 'TAKEOFF', ''), (t_land, 'LAND', '')]
    if net and handover_event:
        ev.append((t_ho + 0.02, 'NET_HANDOVER', ''))
    if case == 'line' and path_event:
        ev.append((t_p0, 'PATH_START', 'line_x'))
    if abort_at is not None:
        ev.append((abort_at, 'FLEET_ABORT', 'ENVELOPE FAULT'))
    with open(root / 'logs' / 'events.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['sim_time', 'event', 'arg'])
        for e in ev:
            w.writerow([f'{e[0]:.3f}', e[1], e[2]])
    mode = 'network' if net else 'dissipative'
    json.dump({'launch_args': {'mode': mode, 'load_traj': 'line_x' if case == 'line' else 'hover',
                               'traj_speed': v}, **({'world': world} if world else {})},
              open(root / 'manifest.json', 'w'))
    json.dump({'auto_network_handover': net, 'net_damp_relative': arm == 'paper', 'net_traj_lean': arm == 'paper',
               'load_traj': 'line_x' if case == 'line' else 'hover', 'traj_speed': v, 'traj_distance': d,
               'auto_handover_settle_s': 1.5, **({'net_law': 'quan'} if arm == 'quan' else {})},
              open(root / 'logs' / 'dissipative_planner' / '20261023_000000' / 'params.json', 'w'))
    with open(root / 'logs' / 'launch.log', 'w') as fh:
        fh.write(''.join(f'{x}\n' for x in log_lines))
    return str(root)


def test_a_late_line_is_charged_lag_not_clock_free_error(tmp_path):
    r = C.score(_make(tmp_path, lag=0.47))
    assert r['arm'] == 'planner' and r['case'] == 'linex_v400' and r['cycles'] == 2 and r['n_reversals'] == 3
    assert r['cf_rms_mm'] < 0.5 and abs(r['reach_mean_mm']) < 0.5
    assert r['lag_s'] == pytest.approx(0.47, abs=0.02)
    assert r['rmse_t_mm'] > 100.0                       # 0.47 s at up to 0.4 m/s
    assert r['headline_mm'] == pytest.approx(r['cf_rms_mm'] + r['reach_rms_mm'])
    assert r['valid'] and r['window'] == pytest.approx([25.0, 25.0 + 2 * math.pi * 0.9 / 0.4], abs=1e-6)


def test_a_line_flown_short_is_charged_reach(tmp_path):
    """92.5 % of the 0.9 m line: 6.75 cm short at both far ends, 0 at the near one; inside the segment throughout."""
    r = C.score(_make(tmp_path, arm='paper', lag=0.0, amp=0.925, shift=0.0))
    assert r['arm'] == 'paper' and r['cf_rms_mm'] < 0.5
    assert r['reach_absmax_mm'] == pytest.approx(67.5, abs=0.5)
    assert r['reach_mean_mm'] == pytest.approx(67.5 * 2 / 3, abs=0.5)
    assert r['headline_mm'] == pytest.approx(r['reach_rms_mm'], abs=0.5) and r['amp_ratio'] < 0.95


def test_the_network_arm_is_windowed_from_its_hand_over_and_scored_against_the_target_before_it(tmp_path):
    """The network shifts its datum 1 cm at the hand-over and the ring follows: against its own line that is no error,
    against the target before the hand-over (the card's critic, M1) the ends are 1 cm off; the hand-over is checked
    against the reference's top + 1.5 s."""
    r = C.score(_make(tmp_path, arm='port', lag=0.0, shift=0.01))
    assert r['arm'] == 'port' and r['t_handover'] == pytest.approx(T_TOP + 1.52, abs=1e-6)
    assert r['headline_own_mm'] < 0.5 and not r['notes']
    assert r['cf_rms_mm'] < 0.5 and r['reach_absmax_mm'] == pytest.approx(10.0, abs=0.5)
    assert r['datum_shift_mm'] == pytest.approx(10.0, abs=0.1) and r['path_start_s'] == pytest.approx(5.0, abs=0.03)


@pytest.mark.parametrize('case', ['hover', 'line'])
def test_a_standing_offset_counts_the_same_in_both_arms(tmp_path, case):
    """The ring 12 mm off its target in y all along: our planner keeps the target and is charged; the network moves its
    datum onto the ring at the hand-over, which its own-target number does not see and the common target does."""
    def go(rid, arm):
        root = _make(tmp_path, rid=rid, arm=arm, case=case, lag=0.0, shift=0.0)
        path = os.path.join(root, 'logs', 'run.csv')
        rows = list(csv.reader(open(path)))
        h = rows[0]
        iy, iry, it = h.index('payload_y'), h.index('payload_ref_y'), h.index('t')
        for r in rows[1:]:
            r[iy] = f'{float(r[iy]) + 0.012:.6f}'
            if arm != 'planner' and float(r[it]) >= T_TOP + 1.5:
                r[iry] = f'{float(r[iry]) + 0.012:.6f}'
        csv.writer(open(path, 'w', newline='')).writerows(rows)
        return C.score(root)
    pl, net = go('R9060', 'planner'), go('R9061', 'port')
    k, own = ('err_rms_mm', 'err_own_rms_mm') if case == 'hover' else ('headline_mm', 'headline_own_mm')
    assert pl[k] == pytest.approx(12.0, abs=0.3) and net[k] == pytest.approx(12.0, abs=0.3)
    assert pl[own] == pytest.approx(12.0, abs=0.3) and net[own] < 0.3


def test_a_path_that_starts_before_the_hand_over_is_scored_from_its_start(tmp_path):
    """An older run with no hold (the path 0.5 s after the top, before the hand-over's moment): the target is the one
    from before the path, not a point on the moving line."""
    r = C.score(_make(tmp_path, lag=0.0, path_at=0.5))
    assert r['headline_mm'] < 1.0 and r['headline_own_mm'] < 1.0


def test_an_overshoot_is_counted_once(tmp_path):
    """4 % past the far end (36 mm): the reach reads it, the clock-free error does not again (the critic, M2)."""
    r = C.score(_make(tmp_path, lag=0.0, amp=1.04))
    assert r['cf_rms_mm'] < 0.5 and r['reach_absmax_mm'] == pytest.approx(36.0, abs=0.5)
    assert r['headline_mm'] == pytest.approx(r['reach_rms_mm'], abs=0.5)


def test_without_a_path_start_the_path_starts_at_the_references_first_move(tmp_path):
    r = C.score(_make(tmp_path, arm='port', lag=0.0, path_event=False))
    assert r['window'][0] == pytest.approx(25.0, abs=0.06) and r['cf_rms_mm'] < 0.5
    assert any('no PATH_START' in n for n in r['notes'])


def test_the_hover_is_scored_from_the_hand_over_with_its_first_5_s_apart(tmp_path):
    """A 20 mm dip in the first second after the hand-over: in the first 5 s it is sqrt(1/5) of 20 mm RMS, over the
    60 s window sqrt(1/60) of it. Our planner's window starts where the network arm's would."""
    net = C.score(_make(tmp_path, rid='R9002', arm='port', case='hover', dip_mm=20.0, shift=0.0))
    pl = C.score(_make(tmp_path, rid='R9003', arm='planner', case='hover', dip_mm=20.0))
    for r in (net, pl):
        assert r['case'] == 'hover' and r['valid']
        assert r['err_first5_rms_mm'] == pytest.approx(20.0 * math.sqrt(1 / 5), rel=0.05)
        assert r['err_rms_mm'] == pytest.approx(20.0 * math.sqrt(1 / 60), rel=0.05)
        assert r['window'][1] - r['window'][0] == pytest.approx(60.0)
        assert r['handover_dip_mm'] == pytest.approx(20.0, abs=0.5)     # the event comes one sample into the dip
    assert pl['t_handover'] == pytest.approx(T_TOP + 1.5) and net['t_handover'] == pytest.approx(T_TOP + 1.52)


def test_the_rings_yaw_change_is_reported_from_the_windows_start(tmp_path):
    """The ring turning at 0.5 deg/s from the hand-over: 30 deg over the 60 s hover, through the wrap at 180 too."""
    assert C.score(_make(tmp_path, rid='R9005', case='hover', yaw_rate=0.5))['yaw_excursion_deg'] == pytest.approx(
        30.0, abs=0.1)
    assert C.score(_make(tmp_path, rid='R9006', case='hover', yaw_rate=5.0))['yaw_excursion_deg'] == pytest.approx(
        300.0, abs=0.5)
    assert C.score(_make(tmp_path, rid='R9007', case='hover'))['yaw_excursion_deg'] == pytest.approx(0.0, abs=1e-9)


def test_the_card_rule_scores_a_failure_before_the_window_ends_as_a_loss(tmp_path):
    """The critic's M3: a capsize, abort or drop is the method failing, a LOSS that stays in the table; VOID is for
    plumbing (rows missing with no loss, a network arm with no hand-over)."""
    ok = C.score(_make(tmp_path, rid='R9010', tilt_at=60.0))                # 35 deg after LAND: the network's landing
    assert ok['valid'] and not ok['lost']
    late = C.score(_make(tmp_path, rid='R9011', log_lines=["[planner] /fleet/command received: 'LAND'",
                                                           '[Drone 2] ENVELOPE FAULT: tilt 70 deg']))
    assert late['valid'] and not late['lost'] and any('after LAND: ENVELOPE FAULT' in n for n in late['notes'])
    for k, (kw, why) in enumerate(((dict(tilt_at=30.0), 'ring tilt over 30'), (dict(abort_at=30.0), 'fleet abort'),
                                   (dict(log_lines=['[dissipative] DROP AND LAND: drone 1: ring tilt 31 deg > 30']),
                                    'DROP AND LAND'),
                                   (dict(log_lines=['[dissipative] UNANNOUNCED DETACH: drone 1 cable 1.100 m']),
                                    'UNANNOUNCED DETACH in the log before LAND (the loss detector)'),
                                   (dict(abort_at=30.0, keep=0.9), 'fleet abort'))):
        r = C.score(_make(tmp_path, rid=f'R902{k}', **kw))
        assert r['valid'] and any(why in w for w in r['lost']), (kw, r['lost'])
    for k, (kw, why) in enumerate(((dict(keep=0.9), "of the window's rows"),
                                   (dict(arm='port', handover_event=False), 'no hand-over'))):
        r = C.score(_make(tmp_path, rid=f'R903{k}', **kw))
        assert not r['valid'] and not r['lost'] and any(why in w for w in r['why']), (kw, r['why'])


def test_one_tilt_sample_over_30_deg_is_not_a_drop(tmp_path):
    root = _make(tmp_path, rid='R9030')
    path = os.path.join(root, 'logs', 'run.csv')
    rows = list(csv.reader(open(path)))
    i = next(k for k, r in enumerate(rows[1:], 1) if float(r[0]) >= 30.0)
    rows[i][4] = '35.0'
    csv.writer(open(path, 'w', newline='')).writerows(rows)
    assert C.score(root)['valid']


def test_world_c_scores_the_band_rule_instead_of_the_tilt_rule_and_names_the_quan_arm(tmp_path):
    """Card 2026-10-23_quan_exact (Wesley 23 Oct): on world C the ring's tilt is free, so the tilt rule is off for both
    arms; the ring held more than 0.30 m off the path for 1 s after the hand-over is the loss instead."""
    c = C.TILT_FREE_WORLDS[0]
    tilted = C.score(_make(tmp_path, rid='R9050', tilt_at=30.0, world=c))
    assert tilted['valid'] and not tilted['lost'] and tilted['card'] == C.QUAN_CARD
    assert not C.score(_make(tmp_path, rid='R9051', case='hover', standing=0.25, world=c))['lost']
    off = C.score(_make(tmp_path, rid='R9052', case='hover', standing=0.35, world=c))
    assert off['valid'] and any('300 mm off the path' in w for w in off['lost'])
    assert any('ring tilt over 30' in w for w in C.score(_make(tmp_path, rid='R9053', tilt_at=30.0))['lost'])
    q = C.score(_make(tmp_path, rid='R9054', arm='quan', world=c))
    assert q['arm'] == 'quan' and q['valid'] and not q['lost']
    row = next(csv.reader([C.registry_row(q)]))
    assert row[3] == C.QUAN_CARD and row[5] == 'c_pl_hubcom_linex_v400_L100 (our planner)'
    assert row[4].startswith('Quan et al. as printed')
    assert "centroid" in row[6] and len(row[6].split(' / ')) == len(row[7].split(' / '))


def test_the_registry_row_is_one_lf_line_with_the_verdict(tmp_path):
    row = C.registry_row(C.score(_make(tmp_path, rid='R9040', arm='paper')))
    assert row.endswith('\n') and '\r' not in row and row.count('\n') == 1
    f = next(csv.reader([row]))
    assert len(f) == 12 and f[0] == 'R9040' and f[1] == '2026-10-23' and f[3] == C.CARD
    assert f[4].startswith("the network, the paper's form") and f[9] == 'valid (card rule)'
    assert f[5] == 'c_pl_linex_v400_L100 (our planner)'
    assert len(f[6].split(' / ')) == len(f[7].split(' / '))
    lost = next(csv.reader([C.registry_row(C.score(_make(tmp_path, rid='R9041', abort_at=30.0)))]))
    assert lost[9].startswith('LOSS (card rule: fleet abort at 30.0 s') and lost[5] == ''
    void = next(csv.reader([C.registry_row(C.score(_make(tmp_path, rid='R9042', keep=0.9)))]))
    assert void[9].startswith('VOID (card rule: ')


def test_the_table_says_which_arm_beats_which_only_when_every_run_does(tmp_path):
    dirs = [_make(tmp_path, rid=f'R91{k}0', arm='planner', lag=0.05, amp=1.0 - 0.01 * k) for k in range(3)]
    dirs += [_make(tmp_path, rid=f'R91{k}1', arm='port', lag=0.47, amp=0.9 - 0.01 * k) for k in range(3)]
    dirs += [_make(tmp_path, rid=f'R91{k}2', arm='paper', lag=0.1, amp=0.97 + 0.04 * k) for k in range(3)]
    cells, beats = C.table([C.score(d) for d in dirs])
    c = cells['linex_v400']
    assert set(c) == {'planner', 'port', 'paper'} and c['port']['headline_mm']['n'] == 3
    assert beats['linex_v400']['headline_mm planner vs port'] == 'planner'
    assert beats['linex_v400']['rmse_t_mm planner vs port'] == 'planner'
    assert beats['linex_v400']['headline_mm planner vs paper'] is None        # the ranges overlap
    out = tmp_path / 'c.json'
    assert C.main(['--table', *dirs, '--out', str(out)]) == 0
    j = json.load(open(out))
    assert j['card'] == C.CARD and len(j['runs']) == 9 and 'linex_v400' in j['cells']


def test_the_claim_reads_the_headline_only():
    """B1: a form ahead on the clock-free error alone (a late, short line wins it by construction) does not falsify."""
    cells = {'linex_v400': {'planner': {}, 'port': {}}, 'hover': {'planner': {}, 'port': {}}}
    beats = {'linex_v400': {'cf_rms_mm planner vs port': 'port', 'reach_rms_mm planner vs port': 'planner',
                            'headline_mm planner vs port': 'planner'},
             'hover': {'err_rms_mm planner vs port': None}}
    out = C.claim(cells, beats)
    assert out['linex_v400'] == {'port': 'confirms'}
    assert out['hover'] == {'port': 'no difference shown', 'paper': "no difference shown (the port's hover cell)"}


def test_the_claim_needs_the_minimum_effect_and_ranks_a_lost_run_last(tmp_path):
    """m4: every run ahead by less than the minimum effect is no difference; M3: a lost run ranks behind every run that
    was not lost, and stays in the cell's run count."""
    rows = [C.score(_make(tmp_path, rid=f'R94{k}0', arm='planner', lag=0.05, amp=0.97 - 0.0001 * k))
            for k in range(3)]
    rows += [C.score(_make(tmp_path, rid=f'R94{k}1', arm='port', lag=0.4, amp=0.9 - 0.01 * k, shift=0.0))
             for k in range(3)]
    rows += [C.score(_make(tmp_path, rid=f'R94{k}2', arm='paper', lag=0.0, amp=0.9715 - 0.0001 * k, shift=0.0))
             for k in range(3)]
    cells, beats = C.table(rows)
    out = C.claim(cells, beats)['linex_v400']
    assert out['port'] == 'confirms'
    c = cells['linex_v400']
    assert 0.0 < min(c['planner']['headline_mm']['values']) - max(c['paper']['headline_mm']['values']) < 1.0
    assert out['paper'] == 'no difference shown'
    lost = C.score(_make(tmp_path, rid='R9450', arm='planner', lag=0.05, amp=0.97, abort_at=30.0))
    cells, beats = C.table(rows[3:6] + [lost] + rows[1:3])
    assert cells['linex_v400']['planner']['runs'] == {'n': 3, 'lost': 1}
    assert C.claim(cells, beats)['linex_v400']['port'] == 'no difference shown'
    port_lost = [C.score(_make(tmp_path, rid=f'R946{k}', arm='port', lag=0.4, amp=0.9, shift=0.0, abort_at=30.0))
                 for k in range(3)]
    cells, beats = C.table(rows[:3] + port_lost)
    assert C.claim(cells, beats)['linex_v400']['port'] == 'confirms'
    assert cells['linex_v400']['port']['runs'] == {'n': 3, 'lost': 3}


def test_the_thesis_table_has_the_arms_as_columns_and_no_run_ids(tmp_path):
    dirs = [_make(tmp_path, rid=f'R92{k}{a}', arm=arm, lag=0.1 * a, amp=1.0 - 0.02 * k)
            for k in range(2) for a, arm in enumerate(('planner', 'port', 'paper'))]
    dirs += [_make(tmp_path, rid=f'R93{k}{a}', arm=arm, case='hover') for k in range(2)
             for a, arm in enumerate(('planner', 'port'))]
    cells, _ = C.table([C.score(d) for d in dirs])
    tex = C.latex(cells)
    assert "Our planner & Our port of Quan et al. & The paper's form" in tex
    assert '\\emph{Hover}' in tex and '\\emph{Line, 0.4\\,m/s}' in tex and '\\emph{Line, 0.2\\,m/s}' not in tex
    hover = next(line for line in tex.splitlines() if line.startswith('Error from the hand-over'))
    assert hover.count('--') == 3 and hover.count(' & ') == 3       # the port's hover stands for the paper's form
    assert 'R92' not in tex and 'R93' not in tex
    lags = [cells['linex_v400'][a]['lag_s']['mean'] for a in ('planner', 'port', 'paper')]
    assert lags == pytest.approx([0.0, 0.1, 0.2], abs=0.015)          # knots timed half a row early: +0.01 here
    lag = next(line for line in tex.splitlines() if line.startswith('Lag'))
    assert all(f'{v:.2f}' in lag for v in lags)
