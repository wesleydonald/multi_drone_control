"""The endurance relay's rota (tools/experiment/relay.py; card 2026-10-30_x4_endurance_relay): a config's relay: section
becomes, per lap end, TASK <leaver> (queued), ATTACH <last leaver>, DETACH <leaver> after a WAIT_LOG on 'lap k done';
the leaver is the oldest pack (a pad dwell resets it, an errand does not), never the one that has just come back; the
pads clear the lap's keep-out on a route the planner takes (the planner's own check agrees); the fixed team flies the
whole laps that land it by B; the config is refused when its laps, switches or duration do not fit the rota. Also the
runner's WAIT_LOG '#n' (the n-th matching line)."""
import os
import sys
from types import MethodType, SimpleNamespace

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from experiment import relay  # noqa: E402
from experiment.config import Event, ExperimentConfig, wait_log_arg, wait_log_spec  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
EXP = os.path.join(REPO, 'configs', 'experiments')
KNOBS = {'num_drones': 8, 'attach_azimuths_deg': '0,315,270,225,180,135,90,45', 'attach_radius': 0.5,
         'cable_len': 1.0, 'target_z': 1.0, 'attach_z': 0.02, 'pivot_offset_z': -0.04, 'load_traj': 'fig_8',
         'traj_speed': 0.2, 'fig8_size': 0.5, 'traj_lap_hold_s': 3.0, 'detach_unload_s': 3.0}
ERRAND = {'pickup': [1.0, 1.56, 1.926, 3.0], 'delivery': [-1.0, 1.56, 1.926, 3.0]}


def _plan(**spec):
    s = dict(leaves=10, first_lap=1, final_laps=1, dwell_s=8.0)
    s.update(spec)
    return relay.RelayPlan(s, KNOBS, 9.0)


def test_the_geometry_is_the_planners():
    g = relay.Geometry(dict(KNOBS, traj_laps=2))
    assert g.az == [45.0 * k for k in range(8)]                       # typed clockwise -> counter-clockwise
    assert g.r_stage == pytest.approx(0.5 + 0.5 + 0.3) and g.keep == pytest.approx(1.28)
    assert g.lim == pytest.approx(1.57) and g.lap_s == pytest.approx(19.2, abs=0.05)
    assert np.allclose(g.stage(2), [0.0, 1.3, g.z]) and g.z == pytest.approx(1.0 + 0.02 + np.sqrt(3) / 2 + 0.04)
    assert np.abs(g.lap[:, 0]).max() == pytest.approx(0.5, abs=1e-3) and np.abs(g.lap[:, 1]).max() <= 0.25 + 1e-9


def test_the_constants_are_the_planners():
    pytest.importorskip('acados_template')
    from dissipative_planner import dissipative_node as dn
    from dissipative_planner.approach import APPROACH_VEL
    assert relay.STAGE_OUT_M == dn.REJOIN_STAGE_OUT_M and relay.REACHED_M == dn.TAKE_BACK_REACHED_M
    assert relay.POSITION_LIMIT_M == dn.POSITION_LIMIT_M and relay.SAMPLE_M == dn.RELAY_SAMPLE_M
    assert relay.STAGE_INSET_M == pytest.approx(dn.STAGE_LIMIT_MARGIN_M + dn.STAGE_TRACK_M)
    assert relay.TASK_VEL == APPROACH_VEL and relay.TASK_ACCEL == dn.TAKE_BACK_ACCEL


def test_every_pad_is_clear_of_the_lap_inside_the_bound_and_on_a_route_the_planner_takes():
    p = _plan()
    g = p.geo
    for d, pad in p.pads.items():
        assert g.clear(pad, p.margin - 1e-6)
        assert g.route_refused(d, [(*pad, 8.0)]) is None
        assert np.linalg.norm(pad[:2] - g.stage(d)[:2]) < 1.0              # the nearest such point, not a far corner


def test_the_planners_own_check_takes_every_pad_and_the_errand():
    """dissipative_node._relay_route_refused on the 8-quad ring held at its lap end agrees with the generator's."""
    pytest.importorskip('acados_template')
    from dissipative_planner.dissipative_node import DissipativeController as D
    from mpc_planner.load_trajectory import LoadTrajectory
    p = _plan(errand_at=[2], errand=ERRAND)
    tr = LoadTrajectory('fig_8', 0.2, 1.0, 0.5, laps=12)
    f = SimpleNamespace(traj=tr, traj_t=tr.lap_s(), hover_xy=(0.0, 0.0), psi0=0.0, _traj_running=False,
                        _pos_limit_m=0.0, load_state=np.r_[0.0, 0.0, 1.0, 1.0, 0, 0, 0])
    for name in ('_stage_point', '_stage_lim', '_rejoin_limit', '_task_ring_points', '_relay_route', '_relay_keep',
                 '_relay_route_refused'):
        setattr(f, name, MethodType(getattr(D, name), f))
    for d in range(8):
        rj = {'r_stage': p.geo.r_stage, 'az': np.radians(p.geo.az[d]), 'z': p.geo.z}
        assert f._relay_route_refused(rj, [(*p.pads[d], 8.0)]) is None
    errand = next(lv for lv in p.leaves if lv.kind == 'errand')
    rj = {'r_stage': p.geo.r_stage, 'az': np.radians(p.geo.az[errand.d]), 'z': p.geo.z}
    assert f._relay_route_refused(rj, errand.wps) is None
    bad = {'r_stage': p.geo.r_stage, 'az': 0.0, 'z': p.geo.z}           # drone 0 can't do this errand ...
    assert f._relay_route_refused(bad, p._errand_wps(0)) is not None
    assert p.geo.route_refused(0, p._errand_wps(0)) is not None          # ... and the generator knows it


def test_the_rota_oldest_pack_first_one_off_at_a_time_and_the_last_leaver_comes_back_first():
    p = _plan(leaves=12)
    assert [lv.d for lv in p.leaves[:8]] == list(range(8))                # all fresh: by id, then by age
    assert [lv.d for lv in p.leaves[8:]] == [0, 1, 2, 3]
    for a, b in zip(p.leaves[:-1], p.leaves[1:]):
        assert b.returner == a.d and b.lap == a.lap + 1 and b.d != a.d
        assert b.t['release'] > a.t['blend_over']                          # never two off the ring
    assert p.laps == 1 + 12 + 1 and p.leaves[0].returner is None


def test_an_errand_takes_the_freshest_valid_carrier_and_does_not_reset_its_pack():
    p = _plan(leaves=12, errand_every=9, errand=ERRAND)
    lv = p.leaves[8]
    assert lv.kind == 'errand' and [w[3] for w in lv.wps] == [0.0, 3.0, 3.0]
    assert lv.d != p.leaves[7].d and p.geo.route_refused(lv.d, lv.wps) is None
    pads = [x for x in p.leaves if x.kind == 'pad']
    assert len(p.packs[lv.d]) == sum(1 for x in pads if x.d == lv.d) + 1  # one interval per recharge, + the landing
    with pytest.raises(ValueError, match='an errand needs'):
        _plan(errand_at=[1])


def test_the_events_per_lap_end():
    p = _plan(leaves=3)
    ev = [(e['do'], e.get('arg')) for e in p.events]
    assert ev[0] == ('WAIT_LOG', '240 \\] lap 1 done at t=')
    assert [d for d, _ in ev[:3]] == ['WAIT_LOG', 'TASK', 'DETACH']                      # no one away yet
    assert [d for d, _ in ev[3:7]] == ['WAIT_LOG', 'TASK', 'ATTACH', 'DETACH']
    assert ev[5] == ('ATTACH', p.leaves[0].d) and ev[6] == ('DETACH', p.leaves[1].d)
    assert ev[-4:-2] == [('WAIT_LOG', '240 \\] lap 4 done at t='), ('ATTACH', p.leaves[2].d)]   # the last one back
    assert ev[-2] == ('WAIT_TRAJ', f'lap {p.laps} 300') and ev[-1] == ('LAND', None)
    ts = [e['t'] for e in p.events]
    assert ts == sorted(ts) and ts[0] == 9.0
    for e in p.events:
        Event(e['t'], e['do'], e.get('arg'))                                               # every one is a valid event
    pat, timeout = wait_log_arg(p.events[0]['arg'])
    assert timeout == 240.0 and pat.search('[dissipative] lap 1 done at t=19.30 s')
    assert not pat.search('[dissipative] lap 11 done at t=19.30 s')


def test_the_projection_says_when_the_packs_outrun_b():
    p = _plan(leaves=26, errand_every=9, errand=ERRAND, budget_s=300.0)
    assert p.mission_s > 1700.0 and p.max_pack > 300.0 and set(p.over_budget()) == set(range(8))
    assert 'OVER B' in p.table()
    with pytest.raises(ValueError, match='keeps no pack inside B'):
        relay.expand(dict(leaves=26, errand_every=9, errand=ERRAND, budget_s=300.0, budget_check='refuse'), KNOBS, 9.0)
    q = _plan(leaves=26, errand_every=9, errand=ERRAND, budget_s=660.0)
    assert not q.over_budget() and 'every pack inside B' in q.table()


def test_the_fixed_team_flies_the_whole_laps_that_land_it_by_b():
    p = relay.RelayPlan({'leaves': 0, 'land_at_budget': True, 'budget_s': 300.0}, KNOBS, 9.0)
    assert p.laps == 11 and p.mission_s <= 300.0 and not p.leaves
    assert [e['do'] for e in p.events] == ['WAIT_TRAJ', 'LAND']
    longer = relay.RelayPlan({'leaves': 0, 'land_at_budget': True, 'budget_s': 600.0}, KNOBS, 9.0)
    assert longer.laps > p.laps and longer.mission_s <= 600.0
    with pytest.raises(ValueError, match='fixed team'):
        relay.RelayPlan({'leaves': 2, 'land_at_budget': True}, KNOBS, 9.0)


def test_unknown_keys_and_bad_values_are_refused():
    with pytest.raises(ValueError, match='unknown keys'):
        _plan(leaf=3)
    with pytest.raises(ValueError, match="budget_check"):
        _plan(budget_check='warn')
    with pytest.raises(ValueError, match='no pad'):
        _plan(pad_margin_m=0.5)


# ── the configs ──────────────────────────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize('name', ['x4_relay_short', 'x4_relay_30min', 'x4_relay_30min_m115', 'x4_fixed_B'])
def test_the_x4_configs_load(name):
    c = ExperimentConfig.from_yaml(os.path.join(EXP, name + '.yaml'))
    assert c.relay is not None and c.validate()
    assert c.duration_s >= 1.15 * c.relay.mission_s
    assert c.events[-1].do == 'LAND' and [e.do for e in c.events[:3]] == ['ARM', 'TAKEOFF', 'WAIT_LIFT']


def test_the_fixed_team_and_the_relay_differ_only_in_the_relay():
    def args(n):
        with open(os.path.join(EXP, n + '.yaml')) as fh:
            raw = yaml.safe_load(fh)
        raw['launch']['args'].pop('traj_laps')
        return raw['launch']['args'], raw['io_launch'], raw['world']
    assert args('x4_relay_30min') == args('x4_fixed_B')
    cell, full = args('x4_relay_30min_m115'), args('x4_relay_30min')
    diff = {k for k in full[0] if cell[0].get(k) != full[0][k]}
    assert diff == {'load_mass', 'load_ixx', 'load_izz'} and cell[2] == 'eight_rigid_ground_rig_R50_m115_L100.sdf'


def _doc(**over):
    with open(os.path.join(EXP, 'x4_relay_short.yaml')) as fh:
        raw = yaml.safe_load(fh)
    for k, v in over.items():
        sec, _, key = k.partition('.')
        if v is None:
            raw[sec].pop(key, None)
        else:
            raw[sec][key] = v
    return raw


@pytest.mark.parametrize('over, why', [
    ({'timing.duration_s': 300.0}, 'under 1.15 x the projected mission'),
])
def test_a_relay_that_does_not_fit_is_refused(tmp_path, over, why):
    p = tmp_path / 'x.yaml'
    p.write_text(yaml.safe_dump(_doc(**over), sort_keys=False))
    with pytest.raises(ValueError, match=why):
        ExperimentConfig.from_yaml(str(p))


@pytest.mark.parametrize('knob, value, why', [
    ('traj_laps', 9, 'the rota flies traj_laps 5'),
    ('relay_task', None, 'needs relay_task: true'),
    ('detach_unload_s', None, 'detach_unload_s > 0'),
    ('traj_lap_hold_s', 1.0, 'traj_lap_hold_s >= 2'),
])
def test_a_relay_whose_switches_do_not_fit_is_refused(tmp_path, knob, value, why):
    raw = _doc()
    if value is None:
        raw['launch']['args'].pop(knob)
    else:
        raw['launch']['args'][knob] = value
    p = tmp_path / 'x.yaml'
    p.write_text(yaml.safe_dump(raw, sort_keys=False))
    with pytest.raises(ValueError, match=why):
        ExperimentConfig.from_yaml(str(p))


def test_relay_task_without_a_rota_is_refused(tmp_path):
    raw = _doc()
    raw.pop('relay')
    p = tmp_path / 'x.yaml'
    p.write_text(yaml.safe_dump(raw, sort_keys=False))
    with pytest.raises(ValueError, match='without a relay: section'):
        ExperimentConfig.from_yaml(str(p))


# ── WAIT_LOG #n ──────────────────────────────────────────────────────────────────────────────────────────────────

def test_wait_log_nth():
    pat, timeout, nth = wait_log_spec('60 #3 TASK PARKED drone 2')
    assert (timeout, nth) == (60.0, 3) and pat.search('TASK PARKED drone 2 (/drone_1)')
    assert wait_log_spec('TASK DONE')[1:] == (None, 1) and wait_log_arg('60 #2 x')[1] == 60.0
    for bad in ('60 #0 x', '60 #3'):
        with pytest.raises(ValueError):
            Event(1.0, 'WAIT_LOG', bad)


def test_the_runner_waits_for_the_nth_line():
    pytest.importorskip('interfaces.msg')
    from test_runner_schedule import _waiting_runner
    r = _waiting_runner([Event(1.0, 'WAIT_LOG', '100 #2 TASK DONE drone 2'), Event(2.0, 'LAND')])
    r._planner_lines.append((0.5, '[dissipative] TASK DONE drone 2 (/drone_1): back'))
    for t in (1.0, 5.0, 20.0):
        r.now = t
        r.fire_due_events()
    assert [d for _, d in r.fired] == ['WAIT_LOG']                         # the first line does not count
    r._planner_lines.append((30.0, '[dissipative] TASK DONE drone 2 (/drone_1): back'))
    r.now = 30.0
    r.fire_due_events()
    r.now = 31.0
    r.fire_due_events()
    assert r.fired[-1] == (31.0, 'LAND')
