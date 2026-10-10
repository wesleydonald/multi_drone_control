"""Bounded shared correction and opt-in pickup integration (no ROS runtime)."""
import ast
from pathlib import Path
from types import SimpleNamespace
import math

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
PLANNER = ROOT / 'tejen_mission/online_join_planner.py'


def test_shared_alignment_converges_and_m2_uses_same_implementation():
    from tejen_mission.measured_tip_alignment import TipAlignment
    from tejen_mission.m2b_attachment_mission import M2BIRLTipAlignment
    assert M2BIRLTipAlignment is TipAlignment
    a = TipAlignment()
    old = a.offset_xy.copy()
    for i in range(240):
        error = .054 - a.offset_xy[0]
        a.step(now_s=i/30, error_world_xy=np.array([error, 0]),
               xy_error_m=abs(error), normal_error_m=.2, relative_speed_mps=.01,
               fresh=True, magnet_on=False, phase='capture')
        assert np.linalg.norm(a.offset_xy-old) <= .025/30 + 1e-9
        assert np.linalg.norm(a.offset_xy) <= .08 + 1e-9
        old = a.offset_xy.copy()
    assert abs(.054-a.offset_xy[0]) < .005


def test_tiers_freeze_for_miss_stale_and_low_without_timeout_failure():
    from tejen_mission.measured_tip_alignment import TieredDescent, TipAlignment
    a = TipAlignment()
    d = TieredDescent(a, start_height_m=.20, floor_m=-.017, speed_mps=.04)
    def step(t, error=.06, normal=.2, fresh=True):
        return d.step(now_s=t, error_world_xy=np.array([error, 0]),
                      clearance_m=normal, relative_speed_mps=.01, fresh=fresh)
    step(0)
    for i in range(1,40): step(i/30)
    assert d.height_m == .20 and a.offset_xy[0] > 0
    before = a.offset_xy.copy()
    step(100, fresh=False)
    assert d.height_m == .20
    np.testing.assert_array_equal(a.offset_xy, before)
    step(200, error=.01)  # no catch-up after stale
    assert d.height_m == .20
    for i in range(1,160): step(200+i/30, error=.01, normal=d.height_m)
    assert d.band == 'LOW'
    frozen = a.offset_xy.copy()
    held = d.height_m
    step(206, error=.06, normal=.05)
    assert d.height_m == held
    np.testing.assert_array_equal(a.offset_xy, frozen)
    for i in range(1,200): step(206+i/30, error=.01, normal=d.height_m)
    assert d.complete and d.height_m == -.017
    np.testing.assert_array_equal(a.offset_xy, frozen)


def test_aligned_crossing_and_near_plate_vertical_return():
    from tejen_mission.measured_tip_alignment import TieredDescent, TipAlignment
    d = TieredDescent(TipAlignment(), start_height_m=.12, floor_m=-.017, speed_mps=.04)
    for i in range(40):
        d.step(now_s=i/30, error_world_xy=np.array([.01,0]),
               clearance_m=d.height_m, relative_speed_mps=.01, fresh=True)
    assert d.band == 'LOW'  # dwell earned above the boundary; no new stop
    d = TieredDescent(TipAlignment(), start_height_m=.07, floor_m=-.017, speed_mps=.04)
    for i in range(10):
        d.step(now_s=i/30, error_world_xy=np.array([.04,0]),
               clearance_m=.06, relative_speed_mps=.01, fresh=True)
    assert .07 < d.height_m <= .10
    np.testing.assert_array_equal(d.alignment.offset_xy, [0,0])
    assert d.band == 'HIGH'


def planner_method(name, **namespace):
    tree = ast.parse(PLANNER.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'OnlineJoinPlanner')
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    module = ast.Module(body=[fn], type_ignores=[])
    scope = dict(np=np, math=math, **namespace)
    exec(compile(ast.fix_missing_locations(module), str(PLANNER), 'exec'), scope)
    return scope[name]


def test_opt_in_reference_uses_authorised_height_for_entire_horizon():
    from tejen_mission.reference_generators import stationary_regulation
    fn = planner_method('build_tiered_pickup_reference', stationary_regulation=stationary_regulation)
    obj = SimpleNamespace(pickup_tier_descent=SimpleNamespace(height_m=.10),
        latched_pickup_surface=np.array([1.,2.,.04]),
        latched_pickup_marker_to_face=np.array([0.,0.,-.03]),
        quad_reference_offset_from_tip=lambda: np.array([.02,-.01,.53]),
        horizon_samples=61, min_reference_z=0.)
    r = fn(obj)
    np.testing.assert_allclose(r.positions, np.tile([1.02,1.99,.70], (61,1)))
    np.testing.assert_allclose(r.velocities, 0)


def test_mode_keeps_legacy_and_selects_low_entry_magnet_without_double_trim():
    from tejen_mission.mission_types import MissionPhase
    fn = planner_method('pickup_magnet_command', MissionPhase=MissionPhase)
    obj = SimpleNamespace(pickup_alignment_mode='legacy', phase=MissionPhase.DESCEND_TO_PICKUP,
                          ground_ring_exit=None, ground_ring_attach_trial=False)
    assert fn(obj, 'ON') == 'ON'
    obj.pickup_alignment_mode = 'measured_tip_tiered'
    obj.pickup_tier_descent = SimpleNamespace(band='HIGH')
    assert fn(obj, 'ON') == 'OFF'
    obj.pickup_tier_descent.band = 'LOW'
    assert fn(obj, 'ON') == 'ON'
    text = PLANNER.read_text()
    assert 'pickup_alignment_mode' in text.split('def pickup_compensation_applies',1)[1].split('def ',1)[0]
    assert 'pickup_tier_descent.complete' in text
    assert 'pickup_tier_descent.invalidate()' in text


def test_opt_in_launch_overlay_and_runner_preserve_legacy_default():
    cfg = yaml.safe_load((ROOT/'config/m1_measured_tip_pickup.yaml').read_text())['online_join_planner']['ros__parameters']
    assert cfg['pickup_alignment_gain_per_s'] == .8
    assert cfg['pickup_alignment_max_rate_mps'] == .025
    assert cfg['pickup_alignment_max_offset_m'] == .08
    assert not any('magnet_drop' in k or 'contact_face' in k for k in cfg)
    launch = (ROOT/'launch/m1_irl_virtual_ring.launch.py').read_text()
    assert "DeclareLaunchArgument('pickup_alignment_mode', default_value='legacy'" in launch
    assert "'pickup_alignment_mode': pickup_alignment_mode" in launch
    runner = (ROOT.parents[1]/'tools/sim_test/run_m1_irl_virtual_ring.sh').read_text()
    assert '${3:-legacy}' in runner and 'pickup_alignment_mode:=' in runner


def test_trim_is_exclusive_and_held_through_wait_lift_and_retry():
    from tejen_mission.mission_types import MissionPhase
    fn = planner_method('quad_reference_offset_from_tip', MissionPhase=MissionPhase)
    obj = SimpleNamespace(pickup_alignment_mode='measured_tip_tiered',
        pickup_tier_alignment=SimpleNamespace(offset_xy=np.array([.03,-.02])),
        pickup_tip_xy_compensation_filtered=np.array([.09,.09]),
        nominal_quad_from_tip_offset=lambda: np.array([0.,0.,.53]),
        pickup_compensation_applies=lambda: False)
    for phase in [MissionPhase.DESCEND_TO_PICKUP, MissionPhase.MAGNET_ATTACH_WAIT,
                  MissionPhase.LIFT_OBJECT, MissionPhase.APPROACH_ABOVE_PICKUP]:
        obj.phase = phase
        np.testing.assert_allclose(fn(obj), [.03,-.02,.53])
    obj.phase = MissionPhase.TRANSIT_TO_DROP_POINT
    np.testing.assert_allclose(fn(obj), [0,0,.53])
    obj.pickup_alignment_mode = 'legacy'
    obj.pickup_compensation_applies = lambda: True
    np.testing.assert_allclose(fn(obj), [.09,.09,.53])


def test_real_adapter_requires_live_pickup_and_two_measurements_not_cached_target():
    from tejen_mission.mission_types import MissionPhase
    from tejen_mission.measured_tip_alignment import TipAlignment
    from types import MethodType
    fn = planner_method('update_pickup_alignment', MissionPhase=MissionPhase)
    invalidate = planner_method('invalidate_pickup_alignment')
    clock = [1.]
    geom = SimpleNamespace(marker_to_contact_face_world=np.array([0.,0.,-.03]),
                           pickup_point_world=np.array([0.,0.,.04]))
    obj = SimpleNamespace(pickup_alignment_mode='measured_tip_tiered',
        phase=MissionPhase.SETTLE_ABOVE_PICKUP, _physical_now_s=lambda: clock[0],
        compute_pickup_target_geometry=lambda **kwargs: (geom,None,None,None,None),
        measured_magnet_tip_position=np.array([.04,0.,.30]),
        last_drone_time=1., last_magnet_tip_pose_time=1., last_pickup_object_pose_time=1.,
        input_timeout_s=.5, payload_pose_timeout_s=.25, use_static_pickup_object=False,
        pickup_alignment_last_sample=None, pickup_alignment_speed_ready=False,
        pickup_alignment_relative_velocity=np.zeros(3), pickup_tier_alignment=TipAlignment(),
        pickup_tier_descent=None, pickup_tier_decision=None)
    obj.invalidate_pickup_alignment = MethodType(invalidate,obj)
    fn(obj)
    assert not obj.pickup_alignment_speed_ready
    clock[0] = 1.04
    obj.last_magnet_tip_pose_time = 1.04
    fn(obj)
    assert obj.pickup_alignment_speed_ready
    assert obj.pickup_alignment_clearance_m == pytest.approx(.23)
    assert obj.pickup_tier_alignment.offset_xy[0] < 0
    saved = obj.pickup_tier_alignment.offset_xy.copy()
    clock[0] = 1.4  # geometry still returned, but live pickup is stale
    obj.last_drone_time = obj.last_magnet_tip_pose_time = 1.4
    fn(obj)
    assert obj.pickup_tier_decision is None
    np.testing.assert_array_equal(obj.pickup_tier_alignment.offset_xy,saved)


def test_descend_state_machine_does_not_complete_from_elapsed_time_in_opt_in():
    from tejen_mission.mission_types import MissionPhase
    fn = planner_method('update_state_machine', MissionPhase=MissionPhase)
    transitions=[]
    obj=SimpleNamespace(phase=MissionPhase.DESCEND_TO_PICKUP, is_m2b=False, ground_ring_exit=None,
        tracking_errors=lambda:(0.,0.,0.), drone_velocity=np.zeros(3),
        phase_elapsed=lambda:1000., pickup_alignment_mode='measured_tip_tiered',
        pickup_tier_descent=SimpleNamespace(complete=False),
        rate_profile=lambda:SimpleNamespace(complete=lambda elapsed:True),
        transition_to=lambda *args: transitions.append(args))
    fn(obj)
    assert not transitions
    obj.pickup_tier_descent.complete=True
    fn(obj)
    assert transitions[-1][0] == MissionPhase.MAGNET_ATTACH_WAIT
    obj.pickup_alignment_mode='legacy'
    obj.pickup_tier_descent.complete=False
    fn(obj)
    assert len(transitions)==2
