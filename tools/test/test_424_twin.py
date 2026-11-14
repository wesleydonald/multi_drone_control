"""The 4 -> 2 -> 4 twin (mdc_424 contract, results/analysis/424_design_2026-11-13/rig_and_twin.md 4): the pair2 world
(QUAD3 = drone 2 welded on plate 3 with its own ring weld, beside QUAD4 on 9, the pair on 0/6), the runner's set detach,
per-partner TAKEOFF gates and re-weld waits, its per-partner events, the T1-T7 configs and tools/score_424.py. A run
with one partner (every config before these) is wired exactly as before.

    python3 -m pytest tools/test/test_424_twin.py -q
"""
import csv
import json
import math
import pathlib
import re
import subprocess
import sys
import types
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

REPO = pathlib.Path(__file__).resolve().parents[2]
ASSETS = REPO / 'simulation_assets'
CONFIGS = REPO / 'configs' / 'experiments'
sys.path.insert(0, str(REPO / 'tools'))

import make_attach_layout_worlds_L100 as layout                    # noqa: E402
import make_attach_worlds as maw                                   # noqa: E402
from check_geometry import world_geometry                          # noqa: E402
from experiment.config import (Event, ExperimentConfig, drone_set_arg, instance_topic,  # noqa: E402
                               partner_go_topic, wait_reweld_spec)

WORLD = 'pair2_rigid_ground_rig_attach_welded_pickup_L100.sdf'
MODEL2 = 'x3_rig_drone2_magnet_welded_L100.sdf'
TWIN = {'t1_pair_hold': 'rig_424_t1_pair_hold_twin', 't3': 'rig_424_t3_set_detach_twin',
        't4': 'rig_424_t4_serial_twin', 't5': 'rig_424_t5_together_twin', 't7': 'rig_424_t7_land_descent_twin',
        # the critic's runs (13 Nov)
        't5_land': 'rig_424_t5_land_twin', 't5_giveup': 'rig_424_t5_giveup_twin',
        't8': 'rig_424_t8_carrier_loss_twin', 't9': 'rig_424_t9_q4_lost_twin', 't10': 'rig_424_t10_giveup_q3_twin',
        't11': 'rig_424_t11_land_unload_twin'}
PARTNER_RUNS = ('t3', 't4', 't5', 't7', 't5_land', 't5_giveup', 't8', 't9', 't10', 't11')


# ── the world ────────────────────────────────────────────────────────────────

def test_the_pair2_world_and_drone2_model_on_disk_are_current():
    r = subprocess.run([sys.executable, str(REPO / 'tools' / 'make_attach_layout_worlds_L100.py'), '--check'],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr


def _include(world, name):
    w = ET.parse(ASSETS / world).getroot().find('world')
    inc = [i for i in w.findall('include') if i.findtext('name') == name]
    assert len(inc) == 1, name
    return inc[0].findtext('uri'), [float(v) for v in inc[0].findtext('pose').split()]


def _tip(world, name):
    """Forward kinematics at spawn: body -> the arm's pivot -> the tip, 1 m along the arm's -z."""
    uri, (x, y, z, *_) = _include(world, name)
    model = ET.parse(ASSETS / uri).getroot().find('model')
    pose = [float(v) for v in model.find("link[@name='magnet_arm']").findtext('pose').split()]
    roll = pose[3]
    return np.array([x, y + 1.0 * math.sin(roll), z + pose[2] - 1.0 * math.cos(roll)])


def test_the_pair_carries_on_plates_0_and_6_and_each_partner_is_welded_on_its_own_plate():
    g = world_geometry(ASSETS / WORLD)
    assert g['num_drones'] == 2 and g['attach_azimuths'] == [0.0, 180.0]
    assert (g['load_mass'], g['cable_len'], g['attach_arm_len']) == (0.86, 1.0, 1.0)
    assert _include(WORLD, 'x3_drone2')[0] == f'models/{MODEL2}'
    assert _include(WORLD, 'x3_drone3')[0] == 'models/x3_rig_drone3_magnet_welded_pickup_L100.sdf'
    plate_top = maw.RING_THICKNESS
    assert np.allclose(_tip(WORLD, 'x3_drone2'), [0.0, -0.225, plate_top], atol=1e-4)     # plate 3, -y
    assert np.allclose(_tip(WORLD, 'x3_drone3'), [0.0, 0.225, plate_top], atol=1e-4)      # plate 9, +y
    # both on the carriers' ground-start circle, on opposite sides
    r = [math.hypot(*_include(WORLD, f'x3_drone{i}')[1][:2]) for i in (2, 3)]
    carrier = re.search(r'<name>x3_drone0</name>\s*<pose>(\S+) (\S+)', (ASSETS / WORLD).read_text())
    assert np.allclose(r, math.hypot(float(carrier[1]), float(carrier[2])), atol=1e-5)


def _joints(model_file):
    m = ET.parse(ASSETS / 'models' / model_file).getroot().find('model')
    return {p.findtext('child_model'): (p.findtext('attach_topic'), p.findtext('detach_topic'), p.findtext('output_topic'))
            for p in m.findall('plugin') if p.get('name') == 'gz::sim::systems::DetachableJoint'}


def test_each_partner_has_its_own_ring_weld_and_only_quad4_the_pickup_weld():
    assert _joints(MODEL2) == {'lift_system::payload': ('/drone_2/payload/attach', '/drone_2/payload/detach',
                                                        '/drone_2/payload/detachable_joint_state')}
    j3 = _joints('x3_rig_drone3_magnet_welded_pickup_L100.sdf')
    assert j3['lift_system::payload'] == ('/payload/attach', '/payload/detach', '/payload/detachable_joint_state')
    assert set(j3) == {'lift_system::payload', maw.BALL_MODEL}


def test_drone2s_model_is_quad4s_on_its_own_airframe_with_the_arm_swung_to_plus_y():
    """The same links, masses and joints (tip pose entry 0 on /model/x3_drone2/pose, as QUAD4's on its own topic)."""
    two = (ASSETS / 'models' / MODEL2).read_text()
    three = (ASSETS / 'models' / 'x3_rig_drone3_magnet_welded_L100.sdf').read_text()
    norm = lambda t, d: (t.replace(f'x3_drone{d}', 'x3_droneN').replace(f'/drone_{d}/payload', '/payload')  # noqa: E731
                         .replace(f'X3_{d}', 'X3_N'))
    code = lambda t: [ln for ln in t.splitlines() if not ('<!--' in ln or '-->' in ln or 'QUAD' in ln)]  # noqa: E731
    a, b = code(norm(two, 2)), code(norm(three, 3))
    diff = [(x, y) for x, y in zip(a, b) if x != y]
    assert len(a) == len(b) and len(diff) == 1                     # the arm's pose: the roll's sign
    assert diff[0][0].replace(' 1.550795 ', ' -1.550795 ') == diff[0][1]


def test_a_generation_leaves_the_one_partner_family_alone():
    keep = maw.EXTRA_PARTNERS
    layout.generate('pair2')
    assert maw.EXTRA_PARTNERS == keep == ()
    assert maw.spawn('welded') == maw.spawn('welded', 9)
    with pytest.raises(SystemExit, match='only rig plates 3 and 9'):
        maw.spawn('welded', 6)
    with pytest.raises(SystemExit, match='only QUAD4 has a free start'):
        maw.newcomer_model('free', 2, 3)


@pytest.mark.parametrize('extra, why', [(((1, 3),), 'after the carriers'), (((3, 3),), 'after the carriers'),
                                        (((2, 9),), "QUAD4's"), (((2, 3), (4, 3)), "another partner's")])
def test_a_partner_on_a_carrier_id_or_a_taken_plate_is_refused(monkeypatch, extra, why):
    monkeypatch.setattr(maw, 'EXTRA_PARTNERS', extra)
    monkeypatch.setattr(maw, 'CARRIER_AZIMUTHS', '0,180')
    carrier = re.search(r'<name>x3_drone0</name>\s*<pose>(\S+) (\S+) (\S+)',
                        '<name>x3_drone0</name> <pose>1.224800 0 0.1</pose>')
    with pytest.raises(SystemExit, match=why):
        maw.extra_partner_block(carrier, *extra[-1])


# ── the runner's events ──────────────────────────────────────────────────────

def test_the_contract_topic_rule():
    t = lambda topic, d=2: instance_topic(topic, d)    # noqa: E731
    assert t('/magnet/object_attached') == '/drone_2/magnet/object_attached'
    assert t('/magnet/command') == '/drone_2/magnet/command'
    assert t('/magnet_tip_pose') == '/drone_2/magnet_tip_pose'
    assert t('/partner/release') == '/drone_2/partner/release'
    assert t('/drone_3/take_back') == '/drone_2/take_back' and t('/drone_3/mux_state') == '/drone_2/mux_state'
    assert t('/join_planner/phase') == '/tejen_d2/join_planner/phase'
    assert t('/tejen/drone_command') == '/tejen_d2/drone_command'
    assert t('/fake_payload/pose') == '/tejen_d2/fake_payload/pose'
    assert t('/runner/partner_go') == '/tejen_d2/runner/partner_go'
    assert t('/payload/trajectory_state') == '/payload/trajectory_state'
    assert all(instance_topic(x, 3) == x for x in ('/magnet/contact', '/join_planner/phase', '/runner/partner_go'))
    # his TAKEOFF gate in the twin: a second instance takes a start gate of its own drone's only (partner_mission)
    assert partner_go_topic(2) == '/drone_2/runner/partner_go' and partner_go_topic(3) == partner_go_topic() == \
        '/runner/partner_go'


def test_the_new_event_args():
    assert Event(1, 'DETACH_SET', '2, 3').arg == '2,3' and Event(1, 'DETACH_SET', [3, 2]).arg == '3,2'
    assert Event(1, 'DETACH_SET', 3).arg == '3'
    assert Event(1, 'PARTNER_GO').arg is None and Event(1, 'PARTNER_GO', '2,3').arg == '2,3'
    for bad in (None, '', '2,2', 'x', '-1'):
        with pytest.raises(ValueError):
            Event(1, 'DETACH_SET', bad)
    assert wait_reweld_spec(None) == (None, None) and wait_reweld_spec(300) == (300.0, None)
    assert wait_reweld_spec('d2 300') == (300.0, 2) and wait_reweld_spec('d3') == (None, 3)
    for bad in ('60 9', 'd2 0', 'd2 1 2', 'dx 5'):
        with pytest.raises(ValueError):
            Event(1, 'WAIT_REWELD', bad)
    assert drone_set_arg('2,3', 'X') == (2, 3)


def _cfg(tmp_path, events, num_drones=2, n_total=4):
    doc = {'name': 'x', 'world': WORLD,
           'launch': {'package': 'bringup', 'file': 'sim_control_launch.py',
                      'args': {'num_drones': num_drones, 'reserved_attach': n_total - num_drones, 'mode': 'attach'}},
           'fleet': {'num_drones': num_drones, 'n_total': n_total}, 'timing': {'duration_s': 100.0},
           'events': [{'t': 3.0, 'do': 'ARM'}, {'t': 6.0, 'do': 'TAKEOFF'}] + events}
    p = tmp_path / 'x.yaml'
    p.write_text(yaml.safe_dump(doc))
    return ExperimentConfig.from_yaml(str(p))


@pytest.mark.parametrize('event, why', [({'do': 'PARTNER_GO', 'arg': '1,3'}, 'a partner is a reserved drone'),
                                        ({'do': 'WAIT_REWELD', 'arg': 'd0 50'}, 'a partner is a reserved drone'),
                                        ({'do': 'DETACH_SET', 'arg': '2,4'}, 'below n_total')])
def test_events_naming_a_drone_that_is_not_a_partner_are_refused(tmp_path, event, why):
    with pytest.raises(ValueError, match=why):
        _cfg(tmp_path, [dict(event, t=20.0)])
    assert _cfg(tmp_path, [{'t': 20.0, 'do': 'DETACH_SET', 'arg': '2,3'}, {'t': 21.0, 'do': 'PARTNER_GO', 'arg': '2,3'},
                           {'t': 22.0, 'do': 'WAIT_REWELD', 'arg': 'd2 50'}]).validate()


def _runner(events=(), n=4, num_drones=2):
    """The runner's methods on a stand-in: no rclpy node, a clock, a record of what fired and was published."""
    pytest.importorskip('interfaces.msg')
    from experiment.runner_node import ExperimentRunner, schedule
    log = types.SimpleNamespace(warn=lambda *a, **k: None, info=lambda *a, **k: None)
    r = types.SimpleNamespace(now=0.0, fired=[], event_log=[], sent=[], _pending=schedule(list(events)),
                              _planner_lines=[], _waiting_for_weld=False, _reweld_wait=None, _partner_release_wait=None,
                              _log_wait=None, _thrust_wait=None, _lift_wait=None, attached=False, payload=None,
                              t_partner_release=None, t_reweld=None, n=n, failures=[], _repeats=[],
                              _extra=list(range(num_drones, n - 1)), _partner_seen={}, _proof_d={}, _proven=[],
                              _PARTNER_EVENTS=ExperimentRunner._PARTNER_EVENTS)
    r._att = {d: {'attached': False, 'released': False, 't_weld': None, 't_reweld': None} for d in r._extra}
    r._rel = lambda: r.now
    r.get_logger = lambda: log
    r._log_event = lambda name, arg=None: r.event_log.append((round(r.now, 2), name, arg))
    for m in ('fire_due_events', '_partner_cb', '_partner_attached_cb', '_partner_proof_cb', '_publish'):
        setattr(r, m, types.MethodType(getattr(ExperimentRunner, m), r))
    r._fire_real = types.MethodType(ExperimentRunner._fire, r)
    r._fire = lambda ev: r.fired.append((round(r.now, 2), ev.do))
    return r


def _pub(r, topic):
    return types.SimpleNamespace(topic_name=topic, get_subscription_count=lambda: 1,
                                 publish=lambda m: r.sent.append((topic, m.data)))


def test_wait_reweld_of_one_partner_waits_for_its_own_weld_only():
    r = _runner([Event(5.0, 'WAIT_REWELD', 'd2 100'), Event(8.0, 'LAND')])
    msg = lambda v: types.SimpleNamespace(data=v)       # noqa: E731
    for t in (5.0, 6.0):
        r.now = t
        r.fire_due_events()
    r.t_reweld = 6.0                                    # QUAD4's weld does not end drone 2's wait
    r.now = 7.0
    r._partner_attached_cb(msg(True), 2)                # its start weld
    r.now = 20.0
    r._partner_attached_cb(msg(False), 2)               # released
    r.fire_due_events()
    assert [d for _, d in r.fired] == ['WAIT_REWELD']
    r.now = 40.0
    r._partner_attached_cb(msg(True), 2)                # re-welded
    r.fire_due_events()
    r.now = 43.0                                        # 8 s + the 35 s the wait took
    r.fire_due_events()
    assert r.fired[-1] == (43.0, 'LAND')
    names = [(n, a) for _, n, a in r.event_log]
    topic = '/drone_2/magnet/object_attached'
    assert names[:3] == [('WELD', topic), ('RELEASED', topic), ('REWELD', topic)]
    q = _runner([Event(5.0, 'WAIT_REWELD', 'd3 100'), Event(8.0, 'LAND')])     # QUAD4's: today's wait
    q.now = 5.0
    q.fire_due_events()
    q.t_reweld, q.now = 9.0, 9.0
    q.fire_due_events()
    q.now = 12.0
    q.fire_due_events()
    assert q.fired[-1] == (12.0, 'LAND')


def test_a_partners_state_changes_are_logged_once_on_its_own_topics():
    r = _runner()
    for what, v in (('contact', True), ('contact', True), ('contact', False), ('give_up', True), ('give_up', True),
                    ('take_back', True), ('mux', 'partner'), ('mux', 'partner'), ('phase', 'DESCEND_TO_ATTACHMENT'),
                    ('proof', 'prove'), ('proof', 'attached')):
        r._partner_cb(2, what, v)
    assert [(n, a) for _, n, a in r.event_log] == [
        ('CONTACT', '/drone_2/magnet/contact'), ('CONTACT_LOST', '/drone_2/magnet/contact'),
        ('GIVE_UP', '/tejen_d2/join_planner/give_up'), ('TAKE_BACK', '/drone_2/take_back'),
        ('MUX:partner', '/drone_2/mux_state'), ('PHASE:DESCEND_TO_ATTACHMENT', '/tejen_d2/join_planner/phase'),
        ('PROOF:prove', '/drone_2/magnet/attach_proof'), ('PROOF:attached', '/drone_2/magnet/attach_proof')]
    assert r._planner_lines == [(0.0, 'PHASE_D2:DESCEND_TO_ATTACHMENT'), (0.0, 'PROOF_D2:prove'),
                                (0.0, 'PROOF_D2:attached')]
    r._partner_cb(3, 'proof', 'attached')               # QUAD4's on its own (today's) topic
    assert r.event_log[-1][1:] == ('PROOF:attached', '/magnet/attach_proof')


def test_the_set_detach_and_both_takeoffs_publish_once_each_on_their_own_topics():
    r = _runner()
    r.detach_set_pub = _pub(r, '/fleet/detach_set')
    r.partner_go_pubs = {t: _pub(r, t) for t in ('/runner/partner_go', '/drone_2/runner/partner_go')}
    r._fire_real(Event(20.0, 'DETACH_SET', '2,3'))
    r._fire_real(Event(21.0, 'PARTNER_GO', '2,3'))
    assert r.sent == [('/fleet/detach_set', '2,3'), ('/drone_2/runner/partner_go', '{"running": true}'),
                      ('/runner/partner_go', '{"running": true}')]
    assert [t for _, p, _ in r._repeats for t in [p.topic_name]] == ['/drone_2/runner/partner_go', '/runner/partner_go']
    assert not r.failures


def test_his_land_per_instance_and_the_launch_check_after_every_launch_is_up(tmp_path):
    pytest.importorskip('interfaces.msg')
    from experiment.runner_node import his_land_topics
    import run_experiment
    assert his_land_topics(None, 3) == his_land_topics('3', 3) == ['/join_planner/land_now']
    assert his_land_topics('2,3', 3) == ['/tejen_d2/join_planner/land_now', '/join_planner/land_now']
    one = ExperimentConfig.from_yaml(str(CONFIGS / 'rig_nrtf_twin.yaml'))
    assert run_experiment.launches_up(one, 1)                       # one LAUNCH: checked after it, as always
    two = _load('t4')
    assert not run_experiment.launches_up(two, 1) and run_experiment.launches_up(two, 2)


def test_the_second_proof_and_a_give_up_of_the_partner_proven_last():
    r = _runner([Event(5.0, 'WAIT_LOG', '400 #2 PROOF_D\\d+:attached'), Event(5.2, 'OPERATOR_GIVE_UP', 'last_proven')])
    r._proven = []
    r.give_up_pubs = {t: _pub(r, t) for t in ('/join_planner/give_up', '/tejen_d2/join_planner/give_up')}
    r.get_subscriptions_info_by_topic = lambda topic: [types.SimpleNamespace(node_name='dissipative_planner')]
    r.get_name = lambda: 'experiment_runner'
    r._fire = r._fire_real
    r.now = 5.0
    r.fire_due_events()                                               # t=5: the wait starts
    for t, d in ((10.0, 2), (14.0, 3)):
        r.now = t
        r._partner_cb(d, 'proof', 'prove')
        r._partner_cb(d, 'proof', 'attached')
        r.fire_due_events()
    assert ('LOGGED', '9.00') in [(n, a) for _, n, a in r.event_log]       # the second proof, 9 s after the wait
    r.now = 14.2
    r.fire_due_events()
    assert r._proven == [2, 3] and r.sent == [('/join_planner/give_up', True)] and not r.failures
    assert ('GIVE_UP_SENT', '3') in [(n, a) for _, n, a in r.event_log]


def test_a_partners_release_is_its_ring_weld():
    from experiment.config import release_topic
    assert release_topic(0, 2) == '/drone_0/detach' and release_topic(1, 2) == '/drone_1/detach'
    assert release_topic(3, 2) == '/payload/detach' and release_topic(2, 2) == '/drone_2/payload/detach'
    assert release_topic(3, 4) == '/drone_3/detach'                 # a carrier of a four-carrier run, as always


def _wiring(cfg, monkeypatch):
    """(subscribed topics, published topics) of the runner node for a config, without rclpy."""
    pytest.importorskip('interfaces.msg')
    import rclpy.node
    from experiment.runner_node import ExperimentRunner
    monkeypatch.setattr(rclpy.node.Node, '__init__', lambda self, *a, **k: None)

    class Fake(ExperimentRunner):
        def create_subscription(self, typ, topic, cb, qos):
            self._w[0].append(topic)

        def create_publisher(self, typ, topic, qos):
            self._w[1].append(topic)
            return types.SimpleNamespace(topic_name=topic)

        def __init__(self, cfg):
            self._w = ([], [])
            ExperimentRunner.__init__(self, cfg, '/nonexistent')
    return Fake(cfg)._w


def test_a_one_partner_run_gets_no_partner_wiring_and_a_two_partner_run_gets_each_partners(monkeypatch):
    subs, pubs = _wiring(ExperimentConfig.from_yaml(str(CONFIGS / 'rig_nrtf_twin.yaml')), monkeypatch)
    partner2 = ('/drone_2/magnet', '/drone_2/take_back', '/drone_2/mux_state', '/drone_2/runner', '/tejen_d',
                '/fleet/detach_set')
    assert not [t for t in subs + pubs if t.startswith(partner2)]
    _, pubs = _wiring(_load('t9'), monkeypatch)
    assert '/payload/detach' in pubs                                  # RELEASE 3: QUAD4's ring weld
    _, pubs = _wiring(_load('t10'), monkeypatch)
    assert {'/join_planner/give_up', '/tejen_d2/join_planner/give_up'} <= set(pubs)
    subs, pubs = _wiring(ExperimentConfig.from_yaml(str(CONFIGS / f'{TWIN["t5"]}.yaml')), monkeypatch)
    for t in ('/drone_2/magnet/object_attached', '/drone_2/magnet/contact', '/drone_2/magnet/attach_proof',
              '/tejen_d2/join_planner/give_up', '/drone_2/take_back', '/drone_2/mux_state', '/tejen_d2/join_planner/phase',
              '/drone_2/magnet_tip_pose', '/magnet/object_attached', '/magnet/contact', '/drone_3/take_back'):
        assert t in subs, t
    assert {'/fleet/detach_set', '/runner/partner_go', '/drone_2/runner/partner_go'} <= set(pubs)


# ── the configs ──────────────────────────────────────────────────────────────

def _load(key):
    return ExperimentConfig.from_yaml(str(CONFIGS / f'{TWIN[key]}.yaml'))


def _raw(key):
    return yaml.safe_load((CONFIGS / f'{TWIN[key]}.yaml').read_text())


def _launches(cfg):
    return [dict(a.split(':=', 1) for a in e.arg.split()[2:]) for e in cfg.events if e.do == 'LAUNCH']


def test_the_partner_runs_differ_only_in_their_events_after_the_releases():
    """One base: the same world, launches and his two lines; T11 leaves at the set detach, every other run after
    both releases. The loss runs (T8, T9) keep recording after an abort."""
    skip = ('name', 'description', 'events', 'timing', 'criteria')
    base = {k: v for k, v in _raw('t4').items() if k not in skip}
    head = _raw('t4')['events'][:9]          # the launches to both releases (the 7th: the set detach)
    for key in PARTNER_RUNS:
        raw = _raw(key)
        assert {k: v for k, v in raw.items() if k not in skip} == base, key
        assert raw['events'][:9 if key != 't11' else 7] == head[:9 if key != 't11' else 7], key
        assert {k: v for k, v in raw['timing'].items() if k != 'duration_s'} == \
            {k: v for k, v in _raw('t4')['timing'].items() if k != 'duration_s'}
        assert raw['criteria'] == dict(_raw('t4')['criteria'], forbid_abort=key not in ('t8', 't9')), key


def test_the_partner_runs_fly_the_contract_args_and_his_default_contact():
    for key in PARTNER_RUNS:
        cfg = _load(key)
        la = cfg.launch_args
        assert (la['num_drones'], la['reserved_attach'], la['attach_azimuths_deg'], la['mode']) == (2, 2, '0,180', 'attach')
        assert la['params_file'].endswith('configs/rig/attach_424_pair.yaml')
        assert la['partners_file'].endswith('configs/twin/partners_424_twin.yaml')
        # the records come from the partners file only
        assert not {'weld_point_z', 'attach_x_offset', 'attach_y_offset', 'weld_radius', 'attach_cable_len',
                    'proof_taut_len'} & set(la)
        assert cfg.io_launch_args['attach_partners'] == 2 and cfg.world == WORLD
        a, b = _launches(cfg)
        assert 'instance' not in a and a['plate'] == '9' and a['start_gate_topic'] == '/runner/partner_go'
        assert (b['instance'], b['drone_id'], b['plate']) == ('d2', '2', '3')
        assert b['start_gate_topic'] == partner_go_topic(2) and b['start_gate_topic'].startswith('/drone_2/')
        assert 'confirm_pickup_release' not in b and a['confirm_pickup_release'] == 'true'   # B has no object
        assert b['mission_mode'] == 'floor_join' and a['ring_z'] == b['ring_z'] == str(la['target_z'])
        assert set(cfg.launch_expect_topics) == {'/join_planner/phase', '/fake_payload/pose',
                                                 '/tejen_d2/join_planner/phase', '/tejen_d2/fake_payload/pose'}
        # its marker: the standalone partner's tip, published as QUAD4's (pair2 has no lift_system tether for drone 2)
        assert (b['tip_topic'], b['tip_msg_type']) == (instance_topic('/magnet_tip_pose', 2), 'pose_stamped')
        for his in (a, b):
            assert 'contact_dz' not in his                             # the twin's rod misses at -0.03 (R4233-R4243)
            assert his['carrier_azimuths_deg'] == '[0.0,180.0]' and his['airborne_start'] == 'true'
            assert (his['match_xy'], his['descent_abort_xy']) == ('0.05', '0.05')
        assert [e.arg for e in cfg.events if e.do == 'DETACH_SET'] == ['2,3']
    gos = {k: [e.arg for e in _load(k).events if e.do == 'PARTNER_GO'] for k in PARTNER_RUNS}
    assert gos == {'t3': [], 't4': ['3', '2'], 't5': ['2,3'], 't7': ['3', '2'], 't5_land': ['2,3'],
                   't5_giveup': ['2,3'], 't8': ['2,3'], 't9': ['3'], 't10': ['3', '2'], 't11': []}
    assert [e.arg for e in _load('t7').events if e.do == 'WAIT_LOG'][-1] == '300 PHASE_D2:DESCEND_TO_ATTACHMENT'


def _after_release(key):
    """(do, arg) of the events after the base (the set detach and both release waits; T11: the set detach)."""
    return [(e.do, e.arg) for e in sorted(_load(key).events, key=lambda e: e.t)][9 if key != 't11' else 7:]


def test_the_critics_runs_do_what_was_asked():
    second_proof = '400 #2 PROOF_D\\d+:attached'
    assert _after_release('t5_land') == [('PARTNER_GO', '2,3'), ('WAIT_LOG', second_proof), ('LAND', None)]
    assert _after_release('t5_giveup') == [('PARTNER_GO', '2,3'), ('WAIT_LOG', second_proof),
                                           ('OPERATOR_GIVE_UP', 'last_proven'), ('LAND', None)]
    assert _after_release('t8') == [('PARTNER_GO', '2,3'), ('WAIT_LOG', '120 PHASE:APPROACH_ABOVE_TARGET'),
                                    ('RELEASE', 0), ('LAND', None)]
    assert _after_release('t9') == [('PARTNER_GO', '3'), ('WAIT_REWELD', 'd3 300'), ('RELEASE', 3), ('LAND', None)]
    assert _after_release('t10') == [('PARTNER_GO', '3'), ('WAIT_REWELD', 'd3 300'), ('PARTNER_GO', '2'),
                                     ('WAIT_LOG', '300 PHASE_D2:DESCEND_TO_ATTACHMENT'), ('OPERATOR_GIVE_UP', '2'),
                                     ('LAND', None)]
    assert _after_release('t11') == [('WAIT_LOG', '30 UNLOAD SET'), ('LAND', None)]
    t = {e.do: e.t for e in _load('t11').events if e.do in ('WAIT_LOG', 'LAND')}
    assert t['LAND'] - t['WAIT_LOG'] == pytest.approx(1.5) and _load('t11').launch_args['detach_unload_s'] == 3.0
    # the fold-ins start at their welds (no queue, Wesley 13 Nov): nothing waits on a queued fold-in
    for key in PARTNER_RUNS:
        assert not [e for e in _load(key).events if e.do == 'WAIT_LOG' and 'QUEUE' in str(e.arg).upper()], key


def test_his_launch_declares_every_argument_the_runs_give_it():
    text = (REPO / 'src' / 'tejen_mission' / 'launch' / 'partner_mission.launch.py').read_text()
    declared = set(re.findall(r"DeclareLaunchArgument\(\s*['\"](\w+)['\"]", text))
    for key in PARTNER_RUNS:
        for args in _launches(_load(key)):
            assert set(args) <= declared, sorted(set(args) - declared)


def test_the_twin_partner_records_match_the_world():
    recs = {r['drone']: r for r in yaml.safe_load((REPO / 'configs' / 'twin' / 'partners_424_twin.yaml').read_text())
            ['partners']}
    rig = {r['drone']: r for r in yaml.safe_load((REPO / 'configs' / 'rig' / 'partners_424.yaml').read_text())['partners']}
    assert set(recs) == set(rig) == {2, 3}
    for d, r in recs.items():
        assert set(r) == set(rig[d]) and r['plate'] == rig[d]['plate']
        tip = _tip(WORLD, f'x3_drone{d}')
        assert np.allclose([r['attach_x_offset'], r['attach_y_offset']], tip[:2], atol=1e-4)
        assert r['weld_point_z'] == pytest.approx(maw.RING_THICKNESS / 2 + maw.TIP_R, abs=1e-6)


def test_t1_flies_the_pair_on_todays_code():
    cfg = _load('t1_pair_hold')
    assert cfg.launch_args['mode'] == 'dissipative' and cfg.n_total == cfg.num_drones == 4
    assert [e.arg for e in cfg.events if e.do == 'DETACH'] == [1, 3]           # plates 9 and 3: the pair 0/6 stays
    assert not [e for e in cfg.events if e.do in ('LAUNCH', 'DETACH_SET', 'PARTNER_GO')]
    la = cfg.launch_args
    assert (la['drop_gap_deg'], la['min_survivors'], la['int_k_xy'], la['drop_tilt_deg']) == (200.0, 2, 0.0, 55.0)


# ── the scorer ───────────────────────────────────────────────────────────────

def _fake_run(tmp_path):
    """60 s of a 4 -> 2 -> 4: the pair on +-x, partner 2 on -y and 3 (QUAD4) on +y; a 5 deg roll of the ring about x
    at t=21.5; drone 1's throttle at the cap for 1 s; the partners 0.5 m apart at t=40."""
    run = tmp_path / 'R9999_sim_gz_rig_424_t5_together_twin'
    (run / 'logs').mkdir(parents=True)
    rows = []
    for k in range(601):
        t = k * 0.1
        roll = math.radians(5.0) * max(0.0, 1.0 - abs(t - 21.5) / 0.5)
        row = {'t': t, 'payload_x': 0.0, 'payload_y': 0.0, 'payload_z': 1.0 - (0.04 if 30 <= t <= 31 else 0.0),
               'payload_qw': math.cos(roll / 2), 'payload_qx': math.sin(roll / 2), 'payload_qy': 0.0,
               'payload_qz': 0.0, 'payload_tilt_deg': math.degrees(roll)}
        row.update(tip_x=0.0, tip_y=0.95, tip_z=1.0)
        pos = {0: (0.73, 0.0, 1.87), 1: (-0.73, 0.0, 1.87), 2: (0.0, -0.73, 1.87), 3: (0.0, 0.73, 1.87)}
        if k == 0:
            pos = {0: (1.2248, 0, 0.1), 1: (-1.2248, 0, 0.1), 2: (0, -1.2248, 0.1), 3: (0, 1.2248, 0.1)}
        if 39.95 <= t <= 40.05:
            pos[2], pos[3] = (0.0, -0.2, 2.0), (0.0, 0.3, 2.0)
        for i, (x, y, z) in pos.items():
            row.update({f'd{i}_x': x, f'd{i}_y': y, f'd{i}_z': z, f'd{i}_thr': 0.7, f'd{i}_attached': 0})
        if 35.0 <= t < 36.0:
            row['d1_thr'] = 0.95
        row.update(tip_d2_x=float('nan'), tip_d2_y=float('nan'), tip_d2_z=float('nan'))
        rows.append(row)
    with open(run / 'logs' / 'run.csv', 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    ev = [(20.0, 'DETACH_SET', '2,3'), (20.5, 'PLANNER', 'DETACH SET drones 2,3: one resize to n=2'),
          (23.0, 'RELEASED', '/drone_2/magnet/object_attached'), (23.1, 'RELEASED', 'magnet/object_attached'),
          (28.0, 'PARTNER_RELEASE', '2'), (28.2, 'PARTNER_RELEASE', '3'), (29.0, 'PARTNER_GO', '2,3'),
          (29.4, 'PHASE:TAKEOFF', '/join_planner/phase'), (29.5, 'PHASE:TAKEOFF', '/tejen_d2/join_planner/phase'),
          (45.0, 'CONTACT', '/magnet/contact'), (47.0, 'PROOF:attached', '/magnet/attach_proof'),
          (48.0, 'REWELD', 'magnet/object_attached'), (48.1, 'FOLD_IN', '3'),
          (50.0, 'CONTACT', '/drone_2/magnet/contact'), (51.0, 'CONTACT_LOST', '/drone_2/magnet/contact'),
          (51.5, 'CONTACT', '/drone_2/magnet/contact'), (52.0, 'REWELD', '/drone_2/magnet/object_attached'),
          (58.1, 'FOLD_IN', '2'), (59.0, 'LAND', '')]
    with open(run / 'logs' / 'events.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['sim_time', 'event', 'arg'])
        w.writerows([(f'{t:.3f}', e, a) for t, e, a in ev])
    (run / 'manifest.json').write_text(json.dumps({'launch_args': {
        'num_drones': 2, 'attach_radius': 0.225, 'attach_z': 0.02, 'throttle_max': 0.95, 'target_z': 1.0},
        'scenario': '/x/rig_424_t5_together_twin.yaml', 'passed': True, 'exit_reason': 'completed'}))
    return run


def test_the_scorer_reads_each_stage(tmp_path):
    import score_424
    s = score_424.score(str(_fake_run(tmp_path)))
    assert s['fleet']['pair'] == {0: 0, 1: 6} and s['fleet']['leavers'] == {2: 3, 3: 9}
    st = s['step_4_to_2']
    assert st['released'] == {2: 23.0, 3: 23.1} and st['tilt_peak_deg'] == pytest.approx(5.0, abs=0.01)
    assert st['t_tilt_peak'] == pytest.approx(21.5)
    pa = s['pair_alone']
    assert (pa['from'], pa['to']) == (23.1, 45.0)                    # the last release to the first contact
    assert pa['ring_z_min'] == pytest.approx(0.96) and pa['throttle'][1]['max'] == pytest.approx(0.95)
    assert pa['pct_at_cap'][1] == pytest.approx(100.0 * 10 / 220, abs=0.6) and pa['pct_at_cap'][0] == 0.0
    r3, r2 = s['returns'][3], s['returns'][2]
    assert (r3['his_takeoff'], r3['contact'], r3['proof'], r3['weld'], r3['fold_in']) == (29.0, 45.0, 47.0, 48.0, 48.1)
    assert (r2['contact'], r2['contacts'], r2['weld'], r2['fold_in'], r2['fold_in_wait_s']) == (50.0, 2, 52.0, 58.1, 6.1)
    assert r2['his_phase_takeoff'] == 29.5 and r3['stepped_out'] == 28.2
    c = s['clearance']['partners_2_3']
    assert c['body_min_m'] == pytest.approx(0.5) and c['t_body'] == pytest.approx(40.0)
    # QUAD4 over plate 9 at (0, 0.73): its body is 0.73 * sqrt(2) ~ 1.03 m from drone 0's and drone 1's bodies, nearer
    # their cables; its own cable (to the tip column) nearer still
    p = s['clearance']['partner_3_to_pair_cables']
    assert 0.0 < p['its_cable_min_m'] <= p['body_min_m'] < 1.04
    assert s['planner'] == [(20.5, 'DETACH SET drones 2,3: one resize to n=2')]
    assert ('LAND' in [e for _, e, _ in s['trouble']]) and s['drop'] is None
    assert 'rig_424_t5_together_twin' in score_424.report(s)
