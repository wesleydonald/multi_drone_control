"""Local target trim only; no flight, ring proof or new progression gates."""
import ast
import math
from pathlib import Path
from types import MethodType, SimpleNamespace

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
PLANNER = ROOT / 'tejen_mission/online_join_planner.py'


def method(name, **extra):
    tree = ast.parse(PLANNER.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'OnlineJoinPlanner')
    fn = next((n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name), None)
    assert fn is not None, f'missing planner adapter: {name}'
    from tejen_mission.mission_types import MissionPhase, TargetSource
    scope = dict(np=np, math=math, MissionPhase=MissionPhase, TargetSource=TargetSource, **extra)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn], type_ignores=[])), str(PLANNER), 'exec'), scope)
    return scope[name]


def test_trim_converges_in_a_moving_frame_and_stays_bounded():
    from tejen_mission.measured_tip_alignment import TrackingTipAlignment
    trim = TrackingTipAlignment()
    old = trim.offset_xy.copy()
    for i in range(300):
        error = .06 - trim.offset_xy[0]
        # Both marker and target translate at .3 m/s: relative position stays small.
        trim.step(now_s=i/30, sample_stamp_s=i/30, relative_position=[-error, 0, .2],
                  error_world_xy=[error, 0], clearance_m=.2, fresh=True, learning=True)
        assert np.linalg.norm(trim.offset_xy-old) <= .025/30 + 1e-9
        assert np.linalg.norm(trim.offset_xy) <= .08 + 1e-9
        old = trim.offset_xy.copy()
    assert abs(error) < .005
    assert trim.relative_speed_mps < .08


def test_motion_stale_near_surface_and_frozen_phases_do_not_learn():
    from tejen_mission.measured_tip_alignment import TrackingTipAlignment
    trim = TrackingTipAlignment()
    for i in range(40):
        trim.step(now_s=i/30, sample_stamp_s=i/30, relative_position=[-.06, 0, .2],
                  error_world_xy=[.06, 0], clearance_m=.2, fresh=True, learning=True)
    saved = trim.offset_xy.copy()
    for i in range(40,70):
        trim.step(now_s=i/30, sample_stamp_s=i/30, relative_position=[-.06, 0, .2],
                  error_world_xy=[.06, 0], clearance_m=.04, fresh=True, learning=True)
    np.testing.assert_array_equal(trim.offset_xy, saved)
    trim.step(now_s=10, sample_stamp_s=10, relative_position=[0, 0, .2],
              error_world_xy=[.06, 0], clearance_m=.2, fresh=False, learning=True)
    trim.step(now_s=100, sample_stamp_s=100, relative_position=[-.06, 0, .2],
              error_world_xy=[.06, 0], clearance_m=.2, fresh=True, learning=True)
    np.testing.assert_array_equal(trim.offset_xy, saved)  # reacquire speed, no catch-up
    for i in range(1,20):
        trim.step(now_s=100+i/30, sample_stamp_s=100+i/30,
                  relative_position=[-.06+i/30, 0, .2], error_world_xy=[.06,0],
                  clearance_m=.2, fresh=True, learning=True)
    np.testing.assert_array_equal(trim.offset_xy, saved)
    trim.step(now_s=101, sample_stamp_s=101, relative_position=[-.06,0,.2],
              error_world_xy=[.06,0], clearance_m=.2, fresh=True, learning=False)
    np.testing.assert_array_equal(trim.offset_xy, saved)


def test_phase_coverage_separate_offsets_and_legacy_exclusion():
    from tejen_mission.mission_types import MissionPhase as P
    group = method('target_alignment_group')
    offset = method('target_alignment_offset')
    obj = SimpleNamespace(target_alignment_mode='measured_tip_trim', is_m2b=False,
        c1f2_exit_bridge_active=False, ground_ring_exit=None,
        target_drop_trim=SimpleNamespace(offset_xy=np.array([.03,-.01])),
        target_ring_trim=SimpleNamespace(offset_xy=np.array([-.02,.04])))
    obj.target_alignment_group = MethodType(group,obj)
    for phase in [P.SETTLE_ABOVE_DROP_POINT,P.DESCEND_TO_DROP_HEIGHT,P.DROP_OBJECT,P.CLEAR_DROP_ZONE]:
        obj.phase=phase
        np.testing.assert_array_equal(offset(obj),[.03,-.01])
    for phase in [P.MATCH_VELOCITY,P.DESCEND_TO_ATTACHMENT,P.ATTACH_READY,P.APPROACH_ABOVE_TARGET]:
        obj.phase=phase
        np.testing.assert_array_equal(offset(obj),[-.02,.04])
    for phase in [P.LIFT_OBJECT,P.TRANSIT_TO_DROP_POINT,P.TRANSIT_TO_REATTACH,P.LANDING]:
        obj.phase=phase
        np.testing.assert_array_equal(offset(obj),[0,0])
    obj.phase=P.MATCH_VELOCITY
    obj.target_alignment_mode='legacy'
    np.testing.assert_array_equal(offset(obj),[0,0])


def test_adapter_uses_surface_not_elevated_drop_goal_and_respects_ownership():
    from tejen_mission.measured_tip_alignment import TrackingTipAlignment
    from tejen_mission.mission_types import MissionPhase as P, TargetSource as T
    fn = method('update_target_alignment')
    clock=[1.]
    obj=SimpleNamespace(target_alignment_mode='measured_tip_trim', is_m2b=False,
        phase=P.SETTLE_ABOVE_DROP_POINT, target_drop_trim=TrackingTipAlignment(),
        target_ring_trim=TrackingTipAlignment(), ground_ring_exit=None,
        c1f2_exit_bridge_active=False,c1f2_cpp_authority_active=False,
        drone_position=np.zeros(3), drone_velocity=np.zeros(3),
        _physical_now_s=lambda:clock[0], input_timeout_s=.5, last_drone_time=1.,
        last_magnet_tip_pose_time=1., measured_magnet_tip_position=np.array([.06,0,.20]),
        last_payload_pose_time=1.,last_payload_twist_time=1.,payload_position=np.array([0,0,.05]),
        drop_point_source='payload',last_magnet_command='ON',ground_ring_attach_trial=False,
        magnet_marker_to_contact_face=np.array([0,0,-.0225]),
        current_phase_spec=lambda:SimpleNamespace(target_source=T.DROP_POINT),
        target_state=lambda source:SimpleNamespace(position=np.array([0,0,.20]),velocity=np.zeros(3)),
        desired_tip_offset=lambda:np.array([0,0,.3]))
    obj.target_alignment_group=MethodType(method('target_alignment_group'),obj)
    fn(obj)
    assert obj.target_drop_trim.clearance_m == pytest.approx(.1275)
    clock[0]=1.04; obj.last_magnet_tip_pose_time=1.04
    fn(obj)
    clock[0]=1.08; obj.last_magnet_tip_pose_time=1.08
    fn(obj)
    assert obj.target_drop_trim.offset_xy[0] < 0  # ON permitted for loaded drop
    saved=obj.target_drop_trim.offset_xy.copy()
    obj.drone_position[0]=float('nan')
    clock[0]=1.10;obj.last_magnet_tip_pose_time=1.10
    fn(obj)
    np.testing.assert_array_equal(obj.target_drop_trim.offset_xy,saved)
    obj.drone_position[:]=0
    obj.c1f2_exit_bridge_active=True; clock[0]=1.11;obj.last_magnet_tip_pose_time=1.11
    fn(obj)
    np.testing.assert_array_equal(obj.target_drop_trim.offset_xy,saved)
    obj.c1f2_exit_bridge_active=False; obj.phase=P.DROP_OBJECT
    clock[0]=1.12;obj.last_magnet_tip_pose_time=1.12
    fn(obj)
    np.testing.assert_array_equal(obj.target_drop_trim.offset_xy,saved)
    # Ring MATCH learns a separate correction while both move in world X.
    obj.phase=P.MATCH_VELOCITY; obj.last_magnet_command='OFF'
    obj.current_phase_spec=lambda:SimpleNamespace(target_source=T.ATTACHMENT_POINT)
    for i in range(12):
        clock[0]=1.2+i/30
        obj.last_drone_time=obj.last_magnet_tip_pose_time=clock[0]
        obj.last_attachment_pose_time=obj.last_attachment_twist_time=clock[0]
        obj.attachment_position=np.array([.3*i/30,0,.05])
        obj.measured_magnet_tip_position=obj.attachment_position+np.array([.06,0,.2])
        obj.target_state=lambda source:SimpleNamespace(position=obj.attachment_position,velocity=np.array([.3,0,0]))
        fn(obj)
    assert obj.target_ring_trim.offset_xy[0] < 0
    assert obj.target_ring_trim.relative_speed_mps == pytest.approx(0)
    np.testing.assert_array_equal(obj.target_drop_trim.offset_xy,saved)
    ring_saved=obj.target_ring_trim.offset_xy.copy()
    obj.phase=P.DESCEND_TO_ATTACHMENT; obj.last_magnet_command='ON'
    clock[0]+=1/30; obj.last_magnet_tip_pose_time=clock[0]
    fn(obj)
    np.testing.assert_array_equal(obj.target_ring_trim.offset_xy,ring_saved)


def test_offsets_reach_entire_reference_horizon_without_changing_target_motion():
    from tejen_mission.mission_types import MissionPhase as P
    from tejen_mission.reference_generators import target_relative_tracking,TargetState
    fn=method('quad_reference_offset_from_tip')
    obj=SimpleNamespace(phase=P.MATCH_VELOCITY,pickup_alignment_mode='measured_tip_tiered',
        nominal_quad_from_tip_offset=lambda:np.array([0,0,.575]),
        pickup_compensation_applies=lambda:False,
        target_alignment_offset=lambda:np.array([.03,-.02]))
    offset=fn(obj)
    np.testing.assert_allclose(offset,[.03,-.02,.575])
    target=TargetState(np.array([1.,2.,.05]),np.array([.2,-.1,0]),np.zeros(3))
    ref=target_relative_tracking(target,offset,dt=1/30,horizon_samples=61,
        lead_time=0,min_reference_z=0,max_reference_speed=1)
    times=np.arange(61)/30
    np.testing.assert_allclose(ref.positions,target.position+times[:,None]*target.velocity+offset)
    np.testing.assert_allclose(ref.velocities,np.tile(target.velocity,(61,1)))


def test_profile_launch_and_runners_are_opt_in_without_gate_or_hardware_overrides():
    path=ROOT/'config/m1_measured_tip_tracking.yaml'
    assert path.exists(),'missing opt-in tracking profile'
    cfg=yaml.safe_load(path.read_text())['online_join_planner']['ros__parameters']
    assert cfg['target_alignment_gain_per_s']==.8
    assert cfg['target_alignment_max_rate_mps']==.025
    assert cfg['target_alignment_max_offset_m']==.08
    assert all(k.startswith('target_alignment_') for k in cfg)
    # Adaptation speed is not a replacement progression guard.
    from tejen_mission.mission_types import MissionPhase as P
    fn=method('update_state_machine')
    transitions=[]; guard_speed=[.12]
    obj=SimpleNamespace(phase=P.MATCH_VELOCITY,ground_ring_exit=None,is_m2b=False,
        ground_ring_attach_trial=False,drone_velocity=np.zeros(3),phase_elapsed=lambda:0.,
        tracking_errors=lambda:(.01,guard_speed[0],0.),match_xy_threshold=.025,
        match_xy_velocity_threshold=.08,dwell_time_s=.5,auto_descend=True,
        condition_true_for=lambda condition,dwell:condition,
        target_alignment_mode='measured_tip_trim',
        target_ring_trim=SimpleNamespace(relative_speed_mps=0.),
        transition_to=lambda *args:transitions.append(args))
    fn(obj)
    assert not transitions
    guard_speed[0]=.01;obj.target_ring_trim.relative_speed_mps=.3
    fn(obj)
    assert transitions[-1][0]==P.DESCEND_TO_ATTACHMENT
    for launch in ['m1_irl_ground_ring.launch.py','m1_irl_virtual_ring.launch.py']:
        text=(ROOT/'launch'/launch).read_text()
        assert "DeclareLaunchArgument('target_alignment_mode', default_value='legacy'" in text
        assert 'm1_measured_tip_tracking.yaml' in text
        assert "'target_alignment_mode':" in text
    repo=ROOT.parents[1]
    assert '${3:-legacy}' in (repo/'tools/irl_test/run_m1_ground_ring.sh').read_text()
    assert '${4:-legacy}' in (repo/'tools/sim_test/run_m1_irl_virtual_ring.sh').read_text()
