"""tools/m1_metrics.py and tools/m1_score.py on synthetic runs with known answers, the tip_metrics torque fix, and the
registry numbers they must reproduce (R1573, R1579, R1581, R1615, R1624; the TB14 hand count 12 of 48), the last
skipped where the results tree is not on this machine.

    python3 -m pytest tools/test/test_m1_score.py -q
"""
import csv
import io
import json
import os
import sys
from contextlib import redirect_stdout

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import m1_metrics as M  # noqa: E402
import m1_score as S  # noqa: E402
import tip_metrics  # noqa: E402

DT = 0.02
EPOCH = 1791000000.0
AZ = (270.0, 30.0, 150.0)
CARRIERS = [np.array([0.5 * np.cos(np.radians(a)), 0.5 * np.sin(np.radians(a)), 1.5]) for a in AZ]
BTT = 0.59


def make_run(tmp_path, rid, events, q4, extra=None, log=(), t_end=90.0,
             world='three_rigid_ground_rig_attach_pickup.sdf', traj='hover'):
    """A run directory: three carriers still at r 0.5 / z 1.5, the ring at (0, 0, ring_z), QUAD4 (d3) from q4(t),
    its tip 0.59 m under its body unless extra sets tip_*; wall = 2 t; log lines (t, text) stamped on that clock."""
    d = tmp_path / 'results' / '2026-10-12' / f'{rid}_sim_gz_test'
    (d / 'logs').mkdir(parents=True)
    t = np.round(np.arange(0.0, t_end, DT), 3)
    rows = {'t': t, 'wall': 2.0 * t, 'step': np.arange(len(t)),
            'payload_x': 0 * t, 'payload_y': 0 * t, 'payload_z': np.where(t > 5, 1.0, 0.02),
            'payload_qw': 1 + 0 * t, 'payload_qx': 0 * t, 'payload_qy': 0 * t, 'payload_qz': 0 * t,
            'payload_tilt_deg': 1.0 + 0 * t}
    for i, c in enumerate(CARRIERS):
        rows.update({f'd{i}_x': c[0] + 0 * t, f'd{i}_y': c[1] + 0 * t, f'd{i}_z': c[2] + 0 * t,
                     f'd{i}_armed': 1 + 0 * t, f'd{i}_tilt_deg': 0 * t, f'd{i}_thr': 0.5 + 0 * t})
    p = np.array([q4(x) for x in t])
    rows.update({'d3_x': p[:, 0], 'd3_y': p[:, 1], 'd3_z': p[:, 2], 'd3_armed': 1 + 0 * t, 'd3_tilt_deg': 0.5 + 0 * t,
                 'd3_thr': 0.5 + 0 * t, 'd3_attached': 0 * t, 'd3_ref_x': p[:, 0],
                 'tip_x': p[:, 0], 'tip_y': p[:, 1], 'tip_z': p[:, 2] - BTT,
                 'ball_x': 1.0 + 0 * t, 'ball_y': 1.6 + 0 * t, 'ball_z': 0.08 + 0 * t})
    for k, v in (extra or {}).items():
        rows[k] = v(t, rows) if callable(v) else v
    pd.DataFrame(rows).to_csv(d / 'logs' / 'run.csv', index=False)
    ev = pd.DataFrame(events, columns=['sim_time', 'event', 'arg'])
    ev.to_csv(d / 'logs' / 'events.csv', index=False)
    stamp = lambda x: f'{EPOCH + 2.0 * x:.6f}'  # noqa: E731
    (d / 'logs' / 'launch.log').write_text(''.join(
        f'[dissipative_planner-17] [WARN] [{stamp(x)}] [dissipative_planner]: [dissipative] {txt}\n' for x, txt in log))
    (d / 'manifest.json').write_text(json.dumps({
        'scenario': f'/x/configs/experiments/{rid.lower()}_cfg.yaml', 'git_branch': 'm1-score',
        'git_sha': 'abc1234def', 'dirty_files': 0, 'world': world, 'lab': {'slot': 1}, 'harness': 'run_experiment.py',
        'launch_args': {'num_drones': 3, 'attach_azimuths_deg': '270,30,150', 'attach_radius': 0.225,
                        'attach_z': 0.02, 'load_traj': traj}}))
    return str(d)


def brute(p, q, a, b, n=600):
    """Closest distance of two segments by dense sampling: an independent reference."""
    s = np.linspace(0, 1, n)[:, None]
    P, Q = p + s * (q - p), a + s * (b - a)
    return float(np.min(np.linalg.norm(P[:, None, :] - Q[None, :, :], axis=-1)))


# ── geometry ─────────────────────────────────────────────────────────────────────────────────────────────────────

def test_seg_seg_matches_the_tb5_replay_and_brute_force():
    import tb5_replay as T
    rng = np.random.default_rng(3)
    P = rng.normal(size=(400, 4, 3))
    P[::7, 1] = P[::7, 0]                     # first segment a point
    P[::11, 3] = P[::11, 2]                   # second a point
    P[::13, 3] = P[::13, 2] + (P[::13, 1] - P[::13, 0])     # parallel
    got = M.seg_seg(P[:, 0], P[:, 1], P[:, 2], P[:, 3])
    want = np.array([T.seg_seg(*x) for x in P])
    assert np.allclose(got, want, atol=1e-9)
    for x in P[:20]:
        assert abs(M.seg_seg(*x) - brute(*x)) < 5e-3


# ── the full M1: keyed on REWELD ─────────────────────────────────────────────────────────────────────────────────

FULL_EVENTS = [(np.nan, 'WELD', 'magnet/object_attached'), (np.nan, 'MUX:ours', '/drone_3/mux_state'),
               (3.0, 'ARM', ''), (6.0, 'TAKEOFF', ''), (20.0, 'DETACH', '3'), (26.0, 'PARTNER_RELEASE', '3'),
               (26.3, 'PHASE:TAKEOFF', ''), (26.4, 'MUX:partner', ''), (30.0, 'PICKUP_JOINT', 'attached'),
               (30.0, 'PICKUP', ''), (40.0, 'PHASE:DROP_OBJECT', ''), (40.0, 'DROP', ''),
               (41.0, 'PHASE:TRANSIT_TO_REATTACH', ''), (42.0, 'PHASE:DESCEND_TO_ATTACHMENT', ''),
               (45.0, 'CONTACT', ''), (45.6, 'REWELD', 'magnet/object_attached'), (45.6, 'MUX:ours', ''),
               (45.7, 'FOLD_IN', '3'), (50.0, 'LAND', ''), (58.0, 'LANDED', '/fleet/landed')]


def full_run(tmp_path, rid='R9001'):
    # QUAD4 welded at plate 3 (body 0.5 m out) until 26 s, then free at (-1.0, 0.25, 2.3) until his descent at 42 s
    def q4(x):
        if x < 26.0 or x >= 45.0:
            return (0.0, 0.5, 1.5)
        if x < 42.0:
            return (-1.0, 0.25, 2.3)
        return (0.0, 0.5 + (45.0 - x) / 3.0 * 0.5, 1.5 + (45.0 - x) / 3.0 * 0.8)
    extra = {
        'd3_thr': lambda t, r: np.where(t < 45.6, 0.50, 0.52),
        'd3_tilt_deg': lambda t, r: np.where(np.abs(t - 45.6) < 1e-6, 0.7, 0.5),
        'payload_tilt_deg': lambda t, r: np.where((t > 46) & (t < 47), 3.0, np.where((t > 21) & (t < 22), 6.0, 1.0)),
        'payload_z': lambda t, r: np.where(t <= 5, 0.02, np.where((t >= 46.0) & (t < 46.5), 0.996, 1.0)),
        'ball_x': lambda t, r: np.where((t >= 30) & (t < 40), r['tip_x'], 1.0),
        'ball_y': lambda t, r: np.where((t >= 30) & (t < 40), r['tip_y'], 1.6),
        'ball_z': lambda t, r: np.where((t >= 30) & (t < 40), r['tip_z'] - 0.10, 0.08),
    }
    return make_run(tmp_path, rid, FULL_EVENTS, q4, extra, world='three_rigid_ground_rig_attach_welded_pickup.sdf')


def test_the_full_m1_is_keyed_on_reweld_not_the_startup_weld(tmp_path):
    r = M.Run(full_run(tmp_path))
    assert r.attach() == (45.6, 'REWELD')                  # the untimed startup WELD is not an attach
    c = M.contact(r)
    assert c['contact'] == 45.0 and c['contact_to_proof_s'] == 0.6 and c['descend_to_contact_s'] == 3.0
    assert c['proof_to_mux_s'] == 0.0
    assert c['thr_before'] == 0.5 and c['thr_after'] == 0.52 and c['thr_step_pct_hover'] == 4.0
    assert c['q4_tilt_at_confirm_deg'] == 0.7
    assert c['ring_tilt_peak_contact_to_confirm8_deg'] == 3.0 and c['dip_m'] == 0.004
    s = M.stages(r)
    assert s['welded_start'] and s['ring_tilt_detach_deg'] == 6.0
    assert M.stage_text(s) == ('welded start, DETACH 20.00 s, release 26.00 s, pickup (weld 30.00 s), drop 40.00 s, '
                               'contact 45.00 s, REWELD 45.60 s, fold-in')
    a = M.authority(r)
    assert a['attach'] == 45.6 and a['mux_ours'] == 45.6 and not a['mux_before_attach']


def test_his_flight_clearance_windows(tmp_path):
    r = M.Run(full_run(tmp_path))
    fl = M.flight(r, legacy=False)
    assert fl['from'] == 26.3 and fl['to'] == 45.6 and fl['rod_to'] == 45.0
    q4 = np.array([-1.0, 0.25, 2.3])
    # QUAD4's rod (body -> tip) against every carrier body and carrier rod (body -> plate), by brute force over the
    # rod window's rows (the hover, then the descent towards plate 3, which passes nearest)
    t = r.tt
    body, tip = r.xyz('d3_'), r.xyz('tip_')
    plates = [np.array([0.225 * np.cos(np.radians(a)), 0.225 * np.sin(np.radians(a)), 1.02]) for a in AZ]
    w = [i for i in np.where((t >= 26.3) & (t <= 45.0))[0] if t[i] >= 42.0 or i % 50 == 0]
    rod_ref = min(min(brute(body[i], tip[i], c, c, 2), brute(body[i], tip[i], c, p, 200))
                  for i in w for c, p in zip(CARRIERS, plates))
    assert abs(fl['rod_min'] - rod_ref) < 3e-3
    body_ref = min(np.linalg.norm(body[i] - c) for i in np.where((t >= 26.3) & (t <= 45.6))[0] for c in CARRIERS)
    assert abs(fl['body_min'] - body_ref) < 1e-6
    # the ball hangs 0.10 m under the tip while carried (30-40 s): its far side is 0.18 m under the tip
    tipb = q4 - [0, 0, BTT]
    want_ball = min(brute(tipb, tipb - [0, 0, 0.18], c, c) for c in CARRIERS)
    assert abs(fl['ball_min'] - want_ball) < 2e-3
    assert fl['overflight_s'] == 0.0


def test_full_row_columns(tmp_path):
    d = full_run(tmp_path)
    reg = tmp_path / 'registry.csv'
    reg.write_text('run,date,config,card,variable,baseline,metric,value,repeat_of,verdict,reviewer,notes\n'
                   'R9000,2026-10-11,"r9001_cfg (Gazebo, LAB PC, x; y)",c,a,b,m,v,,ok,,\n')
    before = reg.read_bytes()
    out = io.StringIO()
    with redirect_stdout(out):
        S.main(['full', d, '--rev', '10', '--registry', str(reg), '--verdict', 'supports "x"'])
    assert reg.read_bytes() == before                           # it prints; it never writes the registry
    line = out.getvalue().strip()
    assert '\n' not in line
    row = next(csv.reader([line]))
    assert len(row) == 12 and row[0] == 'R9001' and row[1] == '2026-10-12'      # the run's results/<date>/
    assert row[2] == 'r9001_cfg (Gazebo, LAB PC, rig-twin welded pickup world; branch m1-score abc1234)'
    assert row[3] == 'docs/experiments/2026-10-10_combined_m1_contact.md (claim rev 10, full M1 beside the table)'
    assert row[5] == 'R9000'                                    # its predecessor with the same config
    assert row[9] == "supports 'x'"
    val = row[7].split(' / ')
    assert len(val) == len(row[6].split(' / '))
    assert val[1] == '0.60' and val[2] == '3.00, 6.00' and val[3] == '0.0040' and val[5] == '0.500 -> 0.520'


# ── the claim (WELD) and the take-back ───────────────────────────────────────────────────────────────────────────

def tb_run(tmp_path, rid='R9002', land_tip=False):
    """No catch: QUAD4 hovers at (-1, 0.25, 2.3), gives up at 60 s; the take-back flies it at 0.1 m/s along y = 0.25
    over carrier 3 (x -0.433) to x = 0 at 70 s, where it is down (its log line)."""
    def q4(x):
        if x < 60.0:
            return (-1.0, 0.25, 2.3)
        return (-1.0 + 0.1 * min(x - 60.0, 10.0), 0.25, 2.3)
    ev = [(3.0, 'ARM', ''), (6.0, 'TAKEOFF', ''), (20.0, 'PHASE:TAKEOFF', ''),
          (41.0, 'PHASE:TRANSIT_TO_REATTACH', ''), (60.0, 'GIVE_UP', ''), (60.0, 'TAKE_BACK', '/drone_3/take_back'),
          (60.0, 'MUX:ours', ''), (80.0, 'LAND', ''), (88.0, 'LANDED', '/fleet/landed')]
    log = [(10.0, 'drone 4 (/drone_3) is down: before'),          # before the take-back: not its end
           (60.0, 'TAKE-BACK of drone 4 (/drone_3) (his give-up): our tracker takes it, zone c, moving with the ring'),
           (62.0, 'take-back of drone 4 (/drone_3): decoupled from the ring frame at [..]'),
           (70.0, 'departed drone 4 (/drone_3) is down: out of this flight')]
    extra = {'d3_tilt_deg': lambda t, r: np.where((t >= 60.04) & (t < 60.2), 3.5,
                                                  np.where((t >= 62.0) & (t < 62.1), 1.0, 0.5)),
             'payload_tilt_deg': lambda t, r: np.where((t > 64.9) & (t < 65.1), 2.5,
                                                       np.where((t > 41) & (t < 42), 1.8, 1.0))}
    return make_run(tmp_path, rid, ev, q4, extra, log=log)


def test_take_back_bars(tmp_path):
    r = M.Run(tb_run(tmp_path))
    o = M.take_back(r, tb5=False)
    assert o['zone'] == 'c' and o['decouple'] == 62.0 and o['down'] == 70.0 and o['down_s'] == 10.0
    assert o['down_within_60s']
    # straight over carrier 3: its tip 0.21 m above that carrier's body; bodies 0.8 m apart
    assert abs(o['rod_min'] - 0.21) < 1e-3 and o['rod_carrier'] == 3
    assert abs(o['body_min'] - 0.8) < 1e-3 and o['body_carrier'] == 3
    assert abs(o['att_step_take_back'] - 3.0) < 1e-9 and abs(o['att_step_decouple'] - 0.5) < 1e-9
    assert o['ring_tilt_max'] == 2.5
    # x in (-0.633, -0.233) at 0.1 m/s: 4 s over the disc, tip 0.21 m above it
    assert abs(o['overflight_s'] - 4.0) <= 0.04 and o['overflight_gap_min'] == 0.21
    assert o['bars'] == {'rod': 'confirms', 'body': 'confirms', 'step_take_back': 'between',
                         'step_decouple': 'confirms', 'ring_tilt': 'confirms', 'down_within_60s': True}
    c = M.contact(r)
    assert c['attach'] is None and c['ring_tilt_rejoin_deg'] == 1.8    # no confirmation: over his rejoin


def test_claim_row_tail_carries_the_take_back(tmp_path):
    d = tb_run(tmp_path)
    val, metric, data = S.claim_value(M.Run(d))
    assert len(val) == len(metric.split(' / ')) == 21
    assert val[12] == 'give-up 60.00, take-back 60.00' and val[13] == 'LANDED'
    assert val[15] == '0.210, 0.800' and val[16] == '3.00, 0.50' and val[18] == '10.0'
    assert val[20] == '1.80'


def test_overflight_counts_the_rod_not_only_the_body(tmp_path):
    # QUAD4 slanted: body 0.3 m beside carrier 3's xy, its tip over it; the tip 0.25 m above that carrier's body
    c = CARRIERS[2]

    def q4(x):
        return (c[0] - 0.3, c[1], 2.3)
    extra = {'tip_x': lambda t, r: c[0] + 0 * t, 'tip_z': lambda t, r: c[2] + 0.25 + 0 * t}
    ev = [(3.0, 'ARM', ''), (6.0, 'TAKEOFF', ''), (20.0, 'PHASE:TAKEOFF', ''), (60.0, 'TAKE_BACK', ''),
          (70.0, 'LANDED', '')]
    r = M.Run(make_run(tmp_path, 'R9003', ev, q4, extra, t_end=70.0))
    s, gap, body_only = M.overflight(r, (r.tt >= 60.0) & (r.tt < 65.0))
    assert abs(s - 5.0) <= 0.04 and gap == 0.25 and body_only == 0.0


# ── the tilt cut ─────────────────────────────────────────────────────────────────────────────────────────────────

def tilt_run(tmp_path, rid, inj):
    ev = [(3.0, 'ARM', ''), (6.0, 'TAKEOFF', ''), (20.0, 'PHASE:TAKEOFF', '')] + inj + [
        (30.24, 'TILT_FAULT', '3'), (30.24, 'FAULT_LAND', ''), (30.243, 'MUX:cut', '/drone_3/mux_state'),
        (30.243, 'PHASE:LANDED_DISARMED', ''), (40.0, 'LANDED', '/fleet/landed')]
    extra = {'d3_armed': lambda t, r: (t < 30.26).astype(int)}
    return M.Run(make_run(tmp_path, rid, ev, lambda x: (-1.0, 0.25, 2.3), extra, t_end=45.0))


def test_tilt_readouts_from_a_mocap_tilt(tmp_path):
    d = M.tilt(tilt_run(tmp_path, 'R9004', [(30.0, 'MOCAP_TILT', '3 90 2 0.3')]))
    assert d['injection'] == 'MOCAP_TILT' and d['injection_target'] == 3
    assert d['inj_to_tilt_fault_s'] == 0.24 and d['tilt_fault_to_cut_s'] == 0.003 and d['cut_to_d3_disarm_s'] == 0.017
    assert d['landed_disarmed'] and d['fault_land_count'] == 1 and d['land'] == 'LANDED'
    assert d['inj_to_fault_land_s'] == 0.24 and d['inj_to_d3_disarm_s'] == 0.26
    val, metric, _ = S.tilt_value(tilt_run(tmp_path, 'R9005', [(30.0, 'MOCAP_TILT', '3 90 2 0.3')]))
    assert val[:6] == ['0.240', '0.003', '0.017', 'yes', '1', 'LANDED'] and len(val) == len(metric.split(' / '))


def test_a_torque_only_wrench_is_the_injection_and_its_removal_is_not(tmp_path):
    link = 'x3_drone3::base_link'
    inj = [(29.0, 'WRENCH', f'0 0 0 0 0 0 {link}'), (30.0, 'WRENCH', f'0 0 0 0.15 0 0 {link}'),
           (30.3, 'WRENCH', f'0 0 0 0 0 0 {link}')]
    d = M.tilt(tilt_run(tmp_path, 'R9006', inj))
    assert d['injection'] == 'WRENCH' and d['injection_t'] == 30.0 and d['injection_target'] == 'x3_drone3::base_link'
    assert d['inj_to_tilt_fault_s'] == 0.24


def test_a_mocap_loss_is_an_injection(tmp_path):
    d = M.tilt(tilt_run(tmp_path, 'R9007', [(29.5, 'MOCAP_LOSS', '3 2')]))
    assert d['injection'] == 'MOCAP_LOSS' and d['inj_to_fault_land_s'] == 0.74 and d['inj_to_d3_disarm_s'] == 0.76


def test_tip_metrics_reads_a_torque_push(tmp_path):
    d = tmp_path / 'R9008_x'
    (d / 'logs' / 'dissipative_planner' / 'a').mkdir(parents=True)
    tn = np.round(np.arange(100.0, 110.0001, 0.1), 3)
    c = np.where(tn < 104.0, 0.0, np.minimum((tn - 104.0) / 2.0, 1.0) * 0.01)
    pd.DataFrame({'t': tn, 'tip_z': 0 * tn, 'setpoint_z': 0 * tn, 'valid': 1, 'hold': 1, 'c_z': c,
                  'reason': 'hold'}).to_csv(d / 'logs' / 'dissipative_planner' / 'a' / 'tip.csv', index=False)
    ev = lambda rows: pd.DataFrame(rows, columns=['sim_time', 'event', 'arg']).to_csv(  # noqa: E731
        d / 'logs' / 'events.csv', index=False)
    ev([(4.0, 'WRENCH', '0 0 0 0.15 0 0 x3_drone3::base_link'), (4.3, 'WRENCH', '0 0 0 0 0 0 x3_drone3::base_link'),
        (10.0, 'LAND', '')])
    assert tip_metrics.analyse(str(d))['push_dc'] == 0.01
    ev([(4.3, 'WRENCH', '0 0 0 0 0 0 x3_drone3::base_link'), (10.0, 'LAND', '')])
    assert tip_metrics.analyse(str(d))['push_dc'] is None
    assert tip_metrics.is_push('3 0 0') and not tip_metrics.is_push('0 0 0')
    assert not tip_metrics.is_push('0.0 0 0 0 0 0')


# ── TB14 ─────────────────────────────────────────────────────────────────────────────────────────────────────────

def landing_run(tmp_path, rid, rise_from=56.9, abort=57.35):
    """Welded at 40 s, LAND at 50 s: the ring down by 55 s, the tip on plate 3 (ring z + 0.037), QUAD4's body
    descending to 0.3 m by 57 s, its tilt rising 2 -> 70 deg from rise_from to 57.3 s, FLEET_ABORT at abort."""
    ring = lambda t: np.where(t < 50, 1.0, np.maximum(1.0 - (t - 50) * 0.196, 0.02))  # noqa: E731

    def q4(x):
        rz = float(ring(np.array([x])))
        return (0.0, 0.5, 0.1 if x < 6 else max(rz + 0.55 - max(x - 55.0, 0) * 0.14, 0.3))
    ev = [(3.0, 'ARM', ''), (6.0, 'TAKEOFF', ''), (40.0, 'WELD', ''), (40.0, 'MUX:ours', ''), (50.0, 'LAND', '')]
    if abort is not None:
        ev.append((abort, 'FLEET_ABORT', 'drone 4 disarmed unexpectedly'))
    extra = {'payload_z': lambda t, r: np.where(t <= 5, 0.02, ring(t)), 'tip_z': lambda t, r: ring(t) + 0.037,
             'd3_attached': lambda t, r: (t >= 40).astype(int),
             'd3_tilt_deg': lambda t, r: np.clip(2.0 + (t - rise_from) / (57.3 - rise_from) * 68.0, 2.0, 70.0)}
    return M.Run(make_run(tmp_path, rid, ev, q4, extra, t_end=(abort or 60.0) + 0.04))


def test_tb14_landing_tip_over(tmp_path):
    d = M.tb14(landing_run(tmp_path, 'R9010'))
    assert d['welded_at_land'] and d['abort_after_land_s'] == 7.35 and d['tb14']
    assert d['tilt_rise_s'] <= 1.0 and d['body_z_at_60'] > d['rest_z'] + 0.02
    assert not d['tb14_f10']                     # the welded tip never reaches 0.03 m
    assert d['unwind_tipover']                   # tilt card rev 4 9.2
    assert not M.tb14(landing_run(tmp_path, 'R9011', rise_from=55.5))['tb14']       # a slow lean: no
    assert not M.tb14(landing_run(tmp_path, 'R9012', abort=53.0))['tb14']           # abort < 5 s after LAND: no
    r = landing_run(tmp_path, 'R9013')
    assert M.land_text(r, M.tb14(r)) == 'tip-over (TB14)'


# ── the registry ─────────────────────────────────────────────────────────────────────────────────────────────────

def test_corr_rows_are_read_before_their_originals(tmp_path):
    p = tmp_path / 'registry.csv'
    p.write_text('run,date,config,card,variable,baseline,metric,value,repeat_of,verdict,reviewer,notes\n'
                 'R1,2026-10-11,"c",card (rev 9, full),"a",R0,"m","old",,"v",,""\n'
                 'R2,2026-10-11,"c",card,"a",R0,"m","two",,"v",,""\n'
                 'R1-corr,2026-10-11,"c","card (rev 9, full)","a","R0","m","new",,"v",,"correction"\n')
    reg = M.read_registry(str(p))
    assert reg['R1']['value'] == 'new' and reg['R1']['_corr'] and reg['R2']['value'] == 'two'


# ── the registry's own numbers (the results tree of this machine) ────────────────────────────────────────────────

def _have(*rids):
    try:
        for rid in rids:
            M.run_dir(rid)
        return M.registry_path() is not None
    except SystemExit:
        return False


def _planner():
    """TB5 replays the planner's own take-back: dissipative_node needs the sourced workspace (rclpy, interfaces)."""
    try:
        import tb5_replay
        tb5_replay.planner_module()
        return True
    except Exception:
        return False


@pytest.mark.skipif(not _have('R1573', 'R1579', 'R1581'), reason='the results tree is not on this machine')
@pytest.mark.parametrize('family,rid', [('full', 'R1573'), ('full', 'R1579'), ('tilt', 'R1581'),
                                        ('claim', 'R1565'), ('claim', 'R1568')])
def test_reproduces_the_registry(family, rid):
    reg = M.read_registry()
    r = M.Run(rid)
    val, _, _ = S.FAMILIES[family](r)
    assert [c for c in S.compare(val, reg[rid]['value']) if c[3] == 'DIFF'] == []


@pytest.mark.skipif(not _have('R1615', 'R1624'), reason='the results tree is not on this machine')
@pytest.mark.parametrize('rid', ['R1615', 'R1624'])
def test_reproduces_the_tb6_rows(rid):
    reg = M.read_registry()
    ok = _planner()
    val, _, _ = S.takeback_value(M.Run(rid), tb5=ok)
    skip = set() if ok else {1, 3}                        # the lap phase and TB5 need the planner module
    assert [c for c in S.compare(val, reg[rid]['value']) if c[3] == 'DIFF' and c[0] not in skip] == []


@pytest.mark.skipif(not _have('R1479', 'R1578'), reason='the results tree is not on this machine')
def test_tb14_hand_count():
    out = io.StringIO()
    tally = S.tb14_count(['R1479-R1578'], out=out)
    assert tally['subtest'][:2] == [12, 48]
    assert tally['subtest'][2] == ['R1479', 'R1486', 'R1502', 'R1518', 'R1529', 'R1531', 'R1534', 'R1547', 'R1548',
                                   'R1556', 'R1559', 'R1578']
    assert tally['full M1'][0] == 0
