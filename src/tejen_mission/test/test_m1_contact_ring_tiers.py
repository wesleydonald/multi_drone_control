"""Narrow M1 contact/tier contracts; no ROS stack or flight required."""
import ast
import math
from pathlib import Path
from types import MethodType, SimpleNamespace as NS

import numpy as np
import pytest

from tejen_mission.measured_tip_alignment import TrackingTipAlignment
from tejen_mission.m1_pickup_recovery import PickupProgressStall, early_candidate_ready
from tejen_mission.m1_ground_ring import GroundRingExit
from tejen_mission.mission_types import MissionPhase as P, TargetSource, ReferenceType
from tejen_mission.reference_generators import (
    TargetState, stationary_regulation, target_relative_tracking, rate_controlled_manoeuvre,
)

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE / 'tejen_mission/online_join_planner.py'


def method(name):
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'OnlineJoinPlanner')
    fn = next((n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name), None)
    assert fn is not None, f'missing focused M1 adapter: {name}'
    scope = dict(np=np, math=math, MissionPhase=P, TargetSource=TargetSource,
        ReferenceType=ReferenceType, PoseStamped=NS, Bool=NS, TrajectoryReference=NS,
        GroundRingExit=GroundRingExit,
        early_candidate_ready=early_candidate_ready, stationary_regulation=stationary_regulation,
        target_relative_tracking=target_relative_tracking, rate_controlled_manoeuvre=rate_controlled_manoeuvre,
        _vec3_from_position=lambda p:np.array([p.x,p.y,p.z]))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn], type_ignores=[])), str(SOURCE), 'exec'),scope)
    return scope[name]


def pickup(gap=.01):
    clock=[1.]; events=[]; retries=[]
    obj=NS(phase=P.DESCEND_TO_PICKUP, ground_ring_exit=None, is_m2b=False,
        drone_velocity=np.zeros(3), drone_position=np.array([0.,0.,.6]),
        phase_elapsed=lambda:100., tracking_errors=lambda:(.06,0.,0.),
        pickup_alignment_mode='measured_tip_tiered',pickup_proof_enabled=True,
        pickup_tier_descent=NS(complete=True,band='LOW',reason='contact_height_reached',height_m=-.028),
        pickup_alignment_speed_ready=True,pickup_alignment_xy_m=.06,
        pickup_alignment_clearance_m=gap, input_timeout_s=.5,payload_pose_timeout_s=.5,
        last_object_attached_time=1.,last_drone_time=1.,last_magnet_tip_pose_time=1.,
        last_pickup_object_pose_time=1., use_static_pickup_object=False,
        object_attached=True,last_magnet_command='ON',pickup_candidate_max_clearance_m=.005,
        measured_magnet_tip_position=np.array([0.,0.,gap+.0225]),
        compute_pickup_target_geometry=lambda **kw:(NS(marker_to_contact_face_world=np.array([0.,0.,-.0225]),
            pickup_point_world=np.zeros(3)),None,None,'fresh',''),
        last_reference=NS(positions=np.array([[0.,0.,.6]])),m1_candidate_hold_reference=None,
        pickup_retry_enabled=True,m1_pickup_release=NS(stage='idle'),
        m1_pickup_stall=PickupProgressStall(),pickup_lift_xy_tolerance=.08,
        pickup_lift_xy_dwell_s=.3,condition_start_time=None,attach_wait_start_time=None,
        attach_wait_timeout_s=0.,_physical_now_s=lambda:clock[0],
        begin_m1_pickup_retry=lambda reason:retries.append(reason))
    obj.condition_true_for=MethodType(method('condition_true_for'),obj)
    # Bind the consolidated evidence adapter when present, so pre-change tests
    # exercise the actual bypass rather than fail because a fixture lacks it.
    if 'def pickup_contact_evidence(' in SOURCE.read_text():
        obj.pickup_contact_evidence=MethodType(method('pickup_contact_evidence'),obj)
    def transition(phase,reason):
        events.append((phase,reason));obj.phase=phase;obj.condition_start_time=None
    obj.transition_to=transition
    def tick(t,fresh=True):
        clock[0]=t
        if fresh:
            obj.last_object_attached_time=obj.last_drone_time=obj.last_magnet_tip_pose_time=t
            obj.last_pickup_object_pose_time=t
        method('update_state_machine')(obj)
    return obj,tick,events,retries


def test_nominal_completion_cannot_bypass_contact_but_existing_xy_dwell_is_preserved():
    obj,tick,events,_=pickup()
    tick(1.);tick(1.1);tick(1.5)
    assert not any(p==P.LIFT_OBJECT for p,_ in events), '10 mm face gap must not start lift'
    assert obj.last_magnet_command=='ON'
    obj.measured_magnet_tip_position[2]=.0265  # 4 mm face gap; XY remains 60 mm < existing80 mm
    tick(2.);tick(2.2)
    assert obj.phase!=P.LIFT_OBJECT
    tick(2.31)
    assert obj.phase==P.LIFT_OBJECT


def test_ground_pounce_candidate_cannot_end_stroke_early_but_virtual_candidate_still_can():
    obj,tick,events,_=pickup(.004)
    obj.ground_ring_attach_trial=True
    obj.pickup_tier_descent.complete=False
    tick(1.)
    assert not events, 'ground pickup must finish its bounded stroke before candidate hold'
    assert obj.last_magnet_command=='ON'
    obj.pickup_tier_descent.complete=True
    tick(1.1)
    assert obj.phase==P.MAGNET_ATTACH_WAIT
    tick(1.2);tick(1.51)
    assert obj.phase==P.LIFT_OBJECT
    obj,tick,events,_=pickup(.004)
    obj.ground_ring_attach_trial=False
    obj.pickup_tier_descent.complete=False
    tick(1.)
    assert obj.phase==P.MAGNET_ATTACH_WAIT  # virtual/default early candidate unchanged


def test_contact_wait_retries_despite_coarse_true_flag_and_resets_on_improvement_or_stale():
    obj,tick,_,retries=pickup(.016)
    obj.phase=P.MAGNET_ATTACH_WAIT
    tick(1.);tick(11.01)
    assert len(retries)==1, 'coarse true proximity must not suppress no-contact retry'
    obj,tick,_,retries=pickup(.016);obj.phase=P.MAGNET_ATTACH_WAIT
    tick(1.);obj.measured_magnet_tip_position[2]-=.006;tick(10.)
    tick(11.01)
    assert not retries  # meaningful approach reset the timer
    tick(20.,fresh=False)
    assert not retries and obj.m1_pickup_stall.since is None


def test_buffered_pose_timestamps_do_not_reset_ring_velocity_to_zero():
    clock=[1.01]
    obj=NS(ground_ring_attach_trial=True,ground_ring_last_tip_sample=None,
        ground_ring_last_tip_receipt_time=None,ground_ring_tip_velocity=np.zeros(3),
        ground_ring_tip_velocity_ready=False,input_timeout_s=.5,_physical_now_s=lambda:clock[0])
    def send(stamp,x,receipt):
        clock[0]=receipt
        method('magnet_tip_pose_callback')(obj,NS(header=NS(stamp=NS(
            sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9))),pose=NS(position=NS(x=x,y=0.,z=.2))))
    send(1.,0.,1.01);send(1.002,.002,1.0101)
    assert obj.ground_ring_tip_velocity[0]>0, 'buffered valid motion cannot become zero'
    saved=obj.ground_ring_tip_velocity.copy()
    send(1.002,.002,1.0102);send(1.001,.001,1.0103)
    np.testing.assert_array_equal(obj.ground_ring_tip_velocity,saved)
    send(0.,.01,2.)  # unstamped, large receipt gap => reacquire
    assert not obj.ground_ring_tip_velocity_ready
    send(0.,.02,2.01)
    assert obj.ground_ring_tip_velocity_ready and obj.ground_ring_tip_velocity[0]>0


def test_ring_tiers_reuse_bounded_trim_pause_and_freeze_low():
    trim=TrackingTipAlignment()
    trim.start_descent(start_height_m=.10,floor_m=-.01,speed_mps=.08,high_xy_m=.05)
    old=trim.offset_xy.copy()
    for i in range(100):
        trim.step_descent(now_s=i/30,sample_stamp_s=i/30,relative_position=[.04,0,.10],
            error_world_xy=[-.04,0],clearance_m=.10,fresh=True)
        assert np.linalg.norm(trim.offset_xy-old)<=.025/30+1e-9
        assert np.linalg.norm(trim.offset_xy)<=.08
        assert trim.descent.height_m==pytest.approx(.10)  # 40 mm miss cannot cross tier
        old=trim.offset_xy.copy()
    for i in range(100,180):
        trim.step_descent(now_s=i/30,sample_stamp_s=i/30,relative_position=[0,0,.09],
            error_world_xy=[0,0],clearance_m=.09,fresh=True)
        if trim.descent.band=='LOW': break
    assert trim.descent.band=='LOW'
    frozen=trim.offset_xy.copy();height=trim.descent.height_m
    trim.step_descent(now_s=20.,sample_stamp_s=20.,relative_position=[.035,0,.07],
        error_world_xy=[-.035,0],clearance_m=.07,fresh=True)
    np.testing.assert_array_equal(trim.offset_xy,frozen)
    assert trim.descent.height_m==height
    trim.step_descent(now_s=21.,sample_stamp_s=21.,relative_position=[0,0,.07],
        error_world_xy=[0,0],clearance_m=.07,fresh=False)
    assert trim.descent.height_m==height


def ring_obj():
    trim=TrackingTipAlignment()
    obj=NS(ground_ring_attach_trial=True,ring_descent_mode='measured_tip_tiered',
        target_alignment_mode='measured_tip_trim',ground_ring_exit=None,
        ground_ring_config={'marker_to_face_m':.0225},target_ring_trim=trim,
        phase=P.DESCEND_TO_ATTACHMENT,ground_ring_last_tip_sample=None,
        current_phase_spec=lambda:NS(target_source=TargetSource.ATTACHMENT_POINT),
        target_state=lambda source:TargetState(np.array([.2,0.,.05]),np.zeros(3),np.zeros(3)),
        approach_offset_x=0.,approach_offset_y=0.,final_attach_offset_z=.0125,
        quad_reference_offset_from_tip=lambda:np.array([0.,0.,.575]),
        horizon_samples=61,dt=1/30,min_reference_z=.58,max_reference_speed=1.)
    obj.ground_ring_tiered_active=MethodType(method('ground_ring_tiered_active'),obj)
    trim.start_descent(start_height_m=.1,floor_m=-.01,speed_mps=.08,high_xy_m=.05)
    return obj


def test_ring_horizon_respects_authorised_height_and_magnet_latches_only_in_low():
    obj=ring_obj()
    obj.desired_tip_offset=MethodType(method('desired_tip_offset'),obj)
    obj.target_alignment_group=MethodType(method('target_alignment_group'),obj)
    obj.is_m2b=False;obj.drone_position=np.array([.2,0.,.7]);obj.drone_velocity=np.zeros(3)
    obj.attachment_position=np.array([.2,0.,.05]);obj.last_magnet_command='OFF'
    obj.c1f2_exit_bridge_active=obj.c1f2_cpp_authority_active=False
    obj.input_timeout_s=.5
    for i in range(15):
        now=1.+i/30
        obj._physical_now_s=lambda:now
        obj.last_drone_time=obj.last_magnet_tip_pose_time=now
        obj.last_attachment_pose_time=obj.last_attachment_twist_time=now
        obj.ground_ring_tip_stamp_s=now
        obj.measured_magnet_tip_position=np.array([.24,0.,.1725])
        method('update_target_alignment')(obj)
    assert obj.target_ring_trim.offset_xy[0]<0  # actual HIGH adapter owns trim
    assert obj.target_ring_trim.descent.height_m==pytest.approx(.10)
    ref=method('build_tiered_ring_reference')(obj)
    np.testing.assert_allclose(ref.positions[:,2],.05+.0225+.1+.575)
    assert method('pickup_magnet_command')(obj,'OFF')=='OFF'
    obj.target_ring_trim.descent.band='LOW'
    assert method('pickup_magnet_command')(obj,'OFF')=='ON'
    obj.ground_ring_exit=NS()
    assert method('pickup_magnet_command')(obj,'ON')=='OFF'  # LAND release wins
    # Both internal tiers route the unchanged manual LAND release; no phase enum
    # or second descent owner can hide the operator command.
    for band in ('HIGH', 'LOW'):
        obj=ring_obj();commands=[]
        obj.target_ring_trim.descent.band=band
        obj.is_m2b=False;obj.drone_position=np.array([.2,0.,.6])
        obj.takeoff_start_position=np.array([.8,0.,.06])
        obj._physical_now_s=lambda:1.
        obj.cancel_c1f2_reference_recovery=lambda:None  # existing LAND adapter prerequisite
        obj.publish_magnet_command=commands.append
        obj.ground_ring_config.update(release_dwell_s=.25,separation_gap_m=.08,
            separation_dwell_s=.12,rise_limit_m=.20,rise_rate_mps=.03)
        method('external_landing_callback')(obj,NS(data=True))
        assert commands==['OFF'] and obj.ground_ring_exit.stage=='release_wait'


def test_tier_completion_not_wall_time_and_existing_reapproach_are_authoritative():
    obj=ring_obj();events=[]
    obj.is_m2b=False;obj.drone_velocity=np.zeros(3)
    obj.tracking_errors=lambda:(0.,0.,0.);obj.phase_elapsed=lambda:1000.
    obj.ground_ring_xy_m=.02;obj.ground_ring_relative_speed_mps=0.
    obj.ground_ring_evidence=lambda:True
    obj.descend_abort_xy_threshold=.05;obj.descend_abort_xy_velocity_threshold=.15
    obj.rate_profile=lambda:NS(complete=lambda elapsed:True)
    obj.transition_to=lambda phase,reason:events.append(phase)
    method('update_state_machine')(obj)
    assert not events, '1000 s wall time cannot bypass a HIGH hold'
    obj.ground_ring_xy_m=.051
    method('update_state_machine')(obj)
    assert not events, 'approved HIGH correction waits instead of restarting above 50 mm'
    obj.target_ring_trim.descent.band='LOW'
    method('update_state_machine')(obj)
    assert events==[P.APPROACH_ABOVE_TARGET]
    events.clear();obj.target_ring_trim.descent.band='HIGH'
    obj.ground_ring_xy_m=.02;obj.ground_ring_relative_speed_mps=.151
    method('update_state_machine')(obj)
    assert events==[P.APPROACH_ABOVE_TARGET]  # existing velocity reapproach retained


def test_ground_launch_and_runner_expose_legacy_default_opt_in_without_mpc_changes():
    launch=(PACKAGE/'launch/m1_irl_ground_ring.launch.py').read_text()
    runner=(PACKAGE.parents[1]/'tools/irl_test/run_m1_ground_ring.sh').read_text()
    assert "DeclareLaunchArgument('ring_descent_mode', default_value='legacy'" in launch
    assert 'RING_DESCENT_MODE="${5:-legacy}"' in runner
    assert 'ring_descent_mode:="$RING_DESCENT_MODE"' in runner
    assert 'ring_descent_mode' not in (PACKAGE/'launch/m1_irl_virtual_ring.launch.py').read_text()
