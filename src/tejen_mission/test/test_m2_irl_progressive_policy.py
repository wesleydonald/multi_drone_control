"""Compact behavioural checks for measured-contact 15-degree policy."""
import numpy as np
from tejen_mission.m2_irl_attachment_policy import ContactProofEvidence, ProofArc, RetryBudget

def sample(d, t, contact=(0,0,-.001), anchor=(0,0,.5565), **kw):
    return d.update(now=t, contact=np.array(contact), anchor=np.array(anchor),
                    speed=kw.pop("speed",0.0), fresh=kw.pop("fresh",True),
                    magnet_on=kw.pop("magnet_on",True), **kw)

def test_quiet_miss_tracker_progress_resets_and_one_shot_expiry():
    from tejen_mission.m2_irl_attachment_policy import QuietMissTracker
    q = QuietMissTracker(wait_s=10., improvement_m=.005)
    def step(t, xy=.061, eligible=True, key=("LOW", 1)):
        return q.step(now=t, xy_error_m=xy, eligible=eligible, key=key)
    assert not step(0) and not step(9.9)
    assert step(10) and not step(11)
    assert not step(12, key=("LOW", 2))
    assert not step(21, xy=.055, key=("LOW", 2))  # meaningful improvement
    assert not step(30, xy=.055, key=("LOW", 2))
    assert step(31, xy=.055, key=("LOW", 2))
    assert not step(32, eligible=False) and q.elapsed_s == 0
    assert not step(33) and not step(42)
    assert not step(43, xy=float("nan")) and q.elapsed_s == 0
    assert not step(44) and not step(53, key=("SEARCH", 1))


def test_quiet_miss_adapter_reuses_release_and_excludes_motion_contact_and_other_phases():
    from tejen_mission.m2_irl_attachment_policy import QuietMissTracker
    from tejen_mission.online_join_planner import MissionPhase
    p = planner_adapter()
    p.m2b_irl_quiet_miss_enabled = True
    p.m2b_irl_quiet_miss_relative_speed_mps = .03
    p.m2b_quiet_miss = QuietMissTracker(wait_s=10., improvement_m=.005)
    p.m2b_attachment_diagnostics.update(xy_error_m=.061, relative_speed_mps=.001)
    def tick(t):
        p.last_drone_time=t
        p._physical_now_s=lambda:t
        return p.update_m2b_quiet_miss(t)
    for phase in (MissionPhase.M2_SETTLE_CAPTURE, MissionPhase.M2_ATTACH_APPROACH_HIGH,
                  MissionPhase.M2_ATTACH_PROOF, MissionPhase.M2_ATTACHED_HOLD):
        p.phase=phase
        assert not tick(0) and not tick(20)
    for field, value in (("fresh", False), ("initial_contact", True), ("relative_speed_mps", .04)):
        p.phase=MissionPhase.M2_ATTACH_APPROACH_LOW
        original=p.m2b_attachment_diagnostics[field]
        p.m2b_attachment_diagnostics[field]=value
        assert not tick(0) and not tick(20)
        p.m2b_attachment_diagnostics[field]=original
    p.drone_velocity=np.array([.09,0,0])
    assert not tick(0) and not tick(20)
    p.drone_velocity=np.zeros(3)
    p.m2d_enabled=True; p.m2d_mission_permission=False
    assert not tick(0) and not tick(20)
    p.m2d_enabled=False
    for phase in (MissionPhase.M2_ATTACH_APPROACH_LOW, MissionPhase.M2_ATTACH_CAPTURE_WAIT):
        p.phase=phase
        assert not tick(0) and not tick(9.99)
        assert tick(10)
        assert p.phase == MissionPhase.M2_DETACHED_RETREAT
        assert p.m2b_retreat_stage == "release" and not p.m2b_magnet_on_requested
        assert np.allclose(p.progressive_target[:2], p.last_reference.positions[0,:2])
        assert not tick(11)
    assert p.m2b_progressive_budget.attempt == 3
    # The main tick can reject stale inputs before calling the mission adapter.
    p.m2b_quiet_miss.reset()
    p.m2b_quiet_miss.step(now=0., xy_error_m=.061, eligible=True, key=("LOW",3))
    p.m2b_quiet_miss.step(now=9., xy_error_m=.061, eligible=True, key=("LOW",3))
    p._last_planner_callback_start=None
    p.planner_callback_budget_ms=float("inf")
    p.update_m2b_bootstrap=lambda:None
    p.publish_m2b_control_contracts=lambda:None
    p.publish_committed_trajectory=p.m2c_ground_reference_enabled=False
    p.inputs_ready=lambda:False
    p.timer_callback()
    assert p.m2b_quiet_miss.elapsed_s == 0, "skipped stale tick retained quiet-miss time"

def test_sim_contact_allowance_starts_proof_without_relaxing_irl_or_confirmation():
    from dataclasses import replace
    from tejen_mission.attachment_detection import AttachmentDetector, MagnetContactCalibration
    from tejen_mission.cooperative_trajectory import RingNetGeometry
    from tejen_mission.m2a_attachment_runtime import default_detector_config
    cfg = replace(default_detector_config(), attachment_policy="measured_contact_15deg",
                  candidate_normal_m=.015, candidate_dwell_s=.3, proof_speed_mps=.03,
                  initial_contact_max_normal_m=.010)
    evidence = AttachmentDetector(geometry=RingNetGeometry(), assigned_plate_id=0,
        calibration=MagnetContactCalibration(np.zeros(3), np.eye(3)), config=cfg).progressive
    for t in (0., .31):
        sample(evidence, t, contact=(.0022, 0., .00499))
    assert evidence.initial_contact and not evidence.confirmed
    end = ProofArc(evidence.contact_seed, evidence.anchor_seed, evidence.anchor_seed).position(100.)
    for t in (1., 1.51):
        sample(evidence, t, contact=(.0022, 0., .00499), anchor=end,
               proving=True, motion_complete=True)
    assert evidence.confirmed
    strict = ContactProofEvidence()
    for t in (0., .31, 10.):
        sample(strict, t, contact=(.0022, 0., .00499))
    assert not strict.initial_contact and not strict.confirmed
    evidence = ContactProofEvidence(evidence.config)
    for t in (0., .31):
        sample(evidence, t, contact=(0., 0., .0101))
    assert not evidence.initial_contact


def test_contact_allowances_remain_separate_for_simulation_and_commissioned_irl():
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    sim = (root / "launch/m2d_four_drone_sequential.launch.py").read_text()
    opt_in = sim.split("if policy == POLICY:", 1)[1].split("    drone_share", 1)[0]
    assert 'progressive["observer"]["initial_contact_max_normal_m"] = 0.010' in opt_in
    irl = (root / "launch/m2_irl_two_drone.launch.py").read_text()
    # Existing Oct-2 lab calibration; this retry/display increment must preserve it.
    assert '"initial_contact_max_normal_m": 0.008' in irl


def contact_ready():
    d=ContactProofEvidence()
    sample(d,0); sample(d,.31)
    assert d.initial_contact
    return d

def test_contact_dwell_not_timeout_and_stale_is_unknown():
    d=ContactProofEvidence()
    for t in [0,10,100]:
        sample(d,t,contact=(0,0,.005))
    assert not d.initial_contact and not d.lost
    sample(d,101); sample(d,101.31)
    assert d.initial_contact
    sample(d,102,fresh=False)
    assert d.initial_contact and not d.lost and not d.confirmed

def test_taut_arc_confirms_without_upward_excitation():
    d=contact_ready(); arc=ProofArc(d.contact_seed,d.anchor_seed,d.anchor_seed)
    start=arc.position(0); end=arc.position(arc.duration)
    assert end[2]<start[2]
    assert np.isclose(np.linalg.norm(end-d.contact_seed),np.linalg.norm(start-d.contact_seed))
    sample(d,1,anchor=end,proving=True,motion_complete=True)
    sample(d,1.51,anchor=end,proving=True,motion_complete=True)
    assert d.confirmed and not d.lost
    p=np.array([arc.position(t) for t in np.linspace(0,arc.duration,1001)])
    assert np.max(np.linalg.norm(np.diff(p,axis=0),axis=1)/(arc.duration/1000))<=.03001

def test_motion_complete_and_fixed_contact_are_required():
    d=contact_ready(); end=ProofArc(d.contact_seed,d.anchor_seed,d.anchor_seed).position(20)
    for t in [1,2]:
        sample(d,t,anchor=end,proving=True,motion_complete=False)
    assert not d.confirmed
    for t in [3,4]:
        sample(d,t,contact=(.02,0,-.001),anchor=end,proving=True,motion_complete=True)
    assert not d.confirmed and not d.lost

def test_loss_requires_fresh_separation_dwell():
    d=contact_ready()
    sample(d,1,contact=(.06,0,0),fresh=False,proving=True)
    assert not d.lost
    sample(d,2,contact=(.06,0,0),proving=True)
    sample(d,2.13,contact=(.06,0,0),proving=True)
    assert d.lost and not d.confirmed

def test_arc_bridge_is_continuous_and_retains_measured_radius():
    d=contact_ready(); ref=d.anchor_seed+np.array([.003,0,-.012])
    arc=ProofArc(d.contact_seed,d.anchor_seed,ref)
    assert np.allclose(arc.position(0),ref)
    assert np.allclose(arc.position(arc.bridge_duration),d.anchor_seed)
    assert np.allclose(arc.position(arc.duration),arc.position(arc.duration+100))

def test_progressive_fleet_stale_peer_waits_without_revoking_active():
    from tejen_mission.m2d_sequential_supervision import M2DSequentialSupervisorCore
    s=M2DSequentialSupervisorCore(vehicle_ids=("drone_0","drone_1"), progressive_policy=True)
    s.freeze_assignment({"drone_0":0,"drone_1":6}); s.update_m2c_ready(True)
    s.update_phase("drone_0","M2_ATTACHED_HOLD")
    s.update_attachment_evidence("drone_0",confirmed=True,lost=False,stamp_s=0.)
    assert s.step(0.).active_vehicle_id=="drone_1"
    out=s.step(2.)
    assert not out.aborted and out.permissions["drone_1"]
    assert out.waiting_reason
    s.update_phase("drone_1","LANDING")
    s.update_attachment_evidence("drone_0",confirmed=True,lost=False,stamp_s=2.)
    assert not s.step(2.).aborted

def test_release_stale_evidence_breaks_separation_dwell_and_freezes_reference():
    from types import SimpleNamespace
    from tejen_mission.online_join_planner import OnlineJoinPlanner, MissionPhase
    p=object.__new__(OnlineJoinPlanner)
    p.m2b_progressive=True
    p.phase=MissionPhase.M2_DETACHED_RETREAT
    p.m2b_retreat_stage="release"
    p.m2b_attachment_diagnostics={"fresh":False, "geometry_separated":True}
    p.m2b_attachment_evidence_fresh=lambda now:False
    p.progressive_separation_since=0.
    p.progressive_target=np.array([0.,0.,1.])
    p.drone_position=p.progressive_target.copy()
    p.drone_velocity=np.zeros(3)
    p.last_reference=SimpleNamespace(positions=np.array([[0.,0.,.9]]))
    p.horizon_samples=3
    p.min_reference_z=0.
    assert p.update_m2b_progressive(5.)
    assert p.progressive_separation_since is None
    p.m2b_retreat_stage="return"
    # A stale pause must not advance the lateral transfer generator.
    p.transfer_generator=SimpleNamespace(generate=lambda **kw: (_ for _ in ()).throw(AssertionError("advanced stale return")))
    ref=p.build_m2b_b1_reference()
    assert np.allclose(ref.positions[0], p.last_reference.positions[0])

def test_land_is_idempotent_while_release_waits_for_separation():
    from types import SimpleNamespace
    from tejen_mission.online_join_planner import OnlineJoinPlanner, MissionPhase
    p=object.__new__(OnlineJoinPlanner)
    p.is_m2b=p.m2b_progressive=True
    p.phase=MissionPhase.M2_LANDING_STAGE
    p.ground_ring_exit=None
    p.drone_position=np.zeros(3)
    p.external_landing_topic="land_now"
    calls=[]
    p.transition_to=lambda *args:calls.append(args)
    p.external_landing_callback(SimpleNamespace(data=True))
    assert not calls, "repeated LAND bypassed measured release/clearance"

def test_progressive_profile_is_validated_and_has_no_legacy_release_exception():
    from pathlib import Path
    import copy, pytest, yaml
    from tejen_mission.m2_irl_launch_config import progressive_ros_parameters
    payload=yaml.safe_load((Path(__file__).resolve().parents[1]/"config/m2_irl_two_drone.yaml").read_text())
    result=progressive_ros_parameters(payload)
    assert result["observer"]["candidate_speed_mps"] == .03
    assert result["observer"]["candidate_dwell_s"] == .30
    assert result["observer"]["proof_dwell_s"] == .50
    assert result["planner"]["attachment_policy"] == "measured_contact_15deg"
    broken=copy.deepcopy(payload)
    del broken["m2_irl"]["gate_profiles"]["progressive_attachment"]
    with pytest.raises(ValueError, match="progressive_attachment"):
        progressive_ros_parameters(broken)

def planner_adapter():
    from types import SimpleNamespace
    from tejen_mission.online_join_planner import OnlineJoinPlanner, MissionPhase
    from tejen_mission.m2b_attachment_mission import M2BConfig
    p=object.__new__(OnlineJoinPlanner)
    p.m2b_progressive=p.is_m2b=p.m2b_controller_armed=True
    p.m2d_enabled=False
    p.ground_ring_exit=None; p.ground_ring_attach_trial=False
    p.phase=MissionPhase.M2_ATTACH_CAPTURE_WAIT
    p.m2b_config=M2BConfig(progressive_waits=True, contact_overtravel_m=.01)
    p.drone_position=np.array([0.,0.,.5575]); p.drone_velocity=np.zeros(3)
    p.last_reference=SimpleNamespace(positions=np.array([p.drone_position.copy()]))
    p.m2b_attachment_diagnostics=dict(fresh=True, initial_contact=False, lost=False,
        confirmed=False, normal_error_m=.005, xy_error_m=.01, geometry_separated=False)
    p.m2b_attachment_evidence_fresh=lambda now:p.m2b_attachment_diagnostics["fresh"]
    p._physical_now_s=lambda:0.
    p.dt=.1
    p.m2b_capture_wait_contact_height_m=-.01
    p.m2b_irl_contact_correction_m=0.
    p.m2b_irl_max_contact_correction_m=.03
    p.m2b_irl_contact_correction_rate_mps=.01
    p.progressive_arc=None; p.progressive_proof_time=0.
    p.progressive_arc_duration_s=10.; p.progressive_arc_speed_mps=.03
    p.m2b_tether_anchor_body=np.zeros(3)
    p.m2b_plate_pose_world=lambda:(np.zeros(3), np.eye(3))
    p.m2b_body_rotation_world=lambda:np.eye(3)
    p.transition_to=lambda phase, reason:setattr(p, "phase", phase)
    p.m2b_progressive_budget=RetryBudget()
    p.m2b_require_recent_link_statistics=False
    p.last_drone_time=0.; p.input_timeout_s=.5
    p.progressive_release_dwell_s=.25; p.progressive_release_speed_mps=.03
    p.progressive_loss_dwell_s=.12
    p.transfer_generator=SimpleNamespace(reset=lambda:None)
    p.m2b_request_detector_reset=lambda:None
    p.m2b_capture_body_target=lambda:np.array([.1,0.,.9])
    return p

def test_adapter_contact_before_floor_proof_wait_and_vertical_release():
    from tejen_mission.online_join_planner import MissionPhase
    p=planner_adapter()
    # Search does not time out, even with no initial contact.
    for t in range(1000):
        p.update_m2b_progressive(float(t))
    assert p.phase==MissionPhase.M2_ATTACH_CAPTURE_WAIT and p.m2b_magnet_on_requested
    assert p.m2b_irl_contact_correction_m<=.03
    p.m2b_attachment_diagnostics.update(initial_contact=True,
        contact_seed_plate=[0.,0.,-.001], anchor_seed_plate=[0.,0.,.5575])
    p.update_m2b_progressive(1001.)
    assert p.phase==MissionPhase.M2_ATTACH_PROOF
    for t in range(300):
        p.update_m2b_progressive(1002.+t)
    assert p.phase==MissionPhase.M2_ATTACH_PROOF and p.m2b_magnet_on_requested
    p.m2b_attachment_diagnostics["fresh"]=False
    elapsed=p.progressive_proof_time
    p.update_m2b_progressive(2000.)
    assert p.progressive_proof_time==elapsed and p.m2b_magnet_on_requested
    p.m2b_attachment_diagnostics.update(fresh=True,lost=True)
    p.update_m2b_progressive(2001.)
    assert p.phase==MissionPhase.M2_DETACHED_RETREAT and not p.m2b_magnet_on_requested
    assert p.m2b_progressive_budget.attempt==2
    origin=p.progressive_target.copy()
    for t in range(300):
        now=2002.+t; p.last_drone_time=now
        p.update_m2b_progressive(now)
    assert np.allclose(p.progressive_target[:2],origin[:2])
    assert np.isclose(p.progressive_target[2],p.drone_position[2]+.2)
    assert p.m2b_retreat_stage=="release" and p.phase==MissionPhase.M2_DETACHED_RETREAT

def test_low_inconclusive_depth_waits_without_releasing_or_descending():
    from types import SimpleNamespace
    from tejen_mission.online_join_planner import MissionPhase
    p=planner_adapter()
    p.phase=MissionPhase.M2_ATTACH_APPROACH_LOW
    p.m2b_commissioning_stage="b1"
    p.m2b_irl_tip_alignment_enabled=False
    p.m2b_magnet_on_requested=True
    p.m2b_attachment_diagnostics["normal_error_m"]=-.03  # uncertain, inside 45 mm loss boundary
    p.m2b_descent_progress=SimpleNamespace(
        target_contact_height_m=-.005,
        step=lambda **kw:(_ for _ in ()).throw(AssertionError("descended during uncertainty")))
    p.m2b_approach_evidence_gate=SimpleNamespace(step=lambda **kw:None)
    p.update_m2b_b1()
    assert p.phase==MissionPhase.M2_ATTACH_APPROACH_LOW
    assert p.m2b_magnet_on_requested
    assert p.m2b_progressive_budget.attempt==1

def test_aligned_high_crossing_reaches_low_with_measured_height_below_command():
    from tejen_mission.online_join_planner import MissionPhase
    from tejen_mission.m2b_attachment_mission import (
        M2BIRLTipAlignment, M2BDescentProgress, M2BEvidenceFreshnessGate,
    )
    p = planner_adapter()
    p.phase = MissionPhase.M2_ATTACH_APPROACH_HIGH
    p.m2b_commissioning_stage = "b1"
    p.m2b_irl_tip_alignment_enabled = True
    p.m2b_irl_tip_alignment = M2BIRLTipAlignment()
    p.m2b_ring_pose_source = "fixed"
    p.m2b_magnet_on_requested = False
    p.m2b_approach_evidence_gate = M2BEvidenceFreshnessGate(p.m2b_config)
    p.m2b_descent_progress = M2BDescentProgress(p.m2b_config, initial_contact_height_m=.15)
    p.m2b_attachment_diagnostics.update(radial_error_m=.002, tangential_error_m=0.,
                                       xy_error_m=.002, relative_speed_mps=.01)
    for n in range(300):
        p._physical_now_s = lambda: n * p.dt
        # Logged failure: excellent XY, actual face below its commanded height.
        p.m2b_attachment_diagnostics["normal_error_m"] = (
            p.m2b_descent_progress.target_contact_height_m - .025)
        p.update_m2b_irl_tip_alignment()
        if p.m2b_attachment_diagnostics["normal_error_m"] < .08:
            assert not p.m2b_irl_tip_decision.updating  # no near-plate lateral trim
        p.update_m2b_b1()
        if p.phase == MissionPhase.M2_ATTACH_APPROACH_LOW:
            break
    assert p.phase == MissionPhase.M2_ATTACH_APPROACH_LOW


def test_release_dwell_precedes_return_even_if_already_separated():
    p=planner_adapter()
    p.progressive_begin_release("operator RETRY")
    p.m2b_attachment_diagnostics["geometry_separated"]=True
    p.update_m2b_progressive(0.)
    p.update_m2b_progressive(.13)
    assert p.m2b_retreat_stage=="release"
    p.update_m2b_progressive(.26)
    assert p.m2b_retreat_stage=="return"

def test_local_land_routes_to_explicit_release_and_retry_rejections():
    from types import SimpleNamespace
    from tejen_mission.online_join_planner import MissionPhase
    p=planner_adapter()
    calls=[]
    p.progressive_begin_release=lambda reason, **kw:calls.append((reason,kw))
    local=[MissionPhase.M2_SETTLE_CAPTURE, MissionPhase.M2_ATTACH_APPROACH_HIGH,
        MissionPhase.M2_ATTACH_APPROACH_LOW, MissionPhase.M2_ATTACH_CAPTURE_WAIT,
        MissionPhase.M2_ATTACH_PROOF, MissionPhase.M2_DETACHED_RETREAT,
        MissionPhase.M2_ATTACHED_HOLD]
    for phase in local:
        p.phase=phase; p.m2b_retreat_stage="await_operator"
        p.external_landing_callback(SimpleNamespace(data=True))
        assert calls[-1]==("operator LAND", {"landing":True})
        assert p.progressive_retry_available() == (phase!=MissionPhase.M2_ATTACHED_HOLD)
    for phase in [MissionPhase.M2_TAKEOFF_HOVER, MissionPhase.M2_CPP_TRANSIT_CAPTURE,
                  MissionPhase.LANDING, MissionPhase.M2_LANDING_STAGE]:
        p.phase=phase
        assert not p.progressive_retry_available()
    p.phase=MissionPhase.M2_ATTACH_PROOF; p.m2b_controller_armed=False
    assert not p.progressive_retry_available()

def test_two_measured_detector_proofs_reach_persistent_fleet_hold_then_land():
    from dataclasses import replace
    from tejen_mission.attachment_detection import AttachmentDetector, AttachmentObservation, MagnetContactCalibration
    from tejen_mission.cooperative_trajectory import RingNetGeometry
    from tejen_mission.m2a_attachment_runtime import default_detector_config
    from tejen_mission.m2d_sequential_supervision import M2DSequentialSupervisorCore
    cfg=replace(default_detector_config(), attachment_policy="measured_contact_15deg",
        candidate_dwell_s=.3, proof_dwell_s=.5, proof_max_contact_drift_m=.008,
        proof_speed_mps=.03, magnet_off_counts_as_loss=False)
    geometry=RingNetGeometry()
    fleet=M2DSequentialSupervisorCore(vehicle_ids=("drone_0","drone_1"),
        progressive_policy=True, operator_hold_after_goal=True)
    fleet.freeze_assignment({"drone_0":0,"drone_1":6}); fleet.update_m2c_ready(True)
    now=0.
    for vehicle,plate_id in [("drone_0",0),("drone_1",6)]:
        detector=AttachmentDetector(geometry=geometry, assigned_plate_id=plate_id,
            calibration=MagnetContactCalibration(np.zeros(3),np.eye(3)), config=cfg)
        plate=geometry.plate_body_offset(plate_id)
        rotation=geometry.plate_rotation_world(rotation_world_from_ring=np.eye(3),plate_index=plate_id)
        contact=np.array([0.,0.,-.001]); anchor=np.array([0.,0.,.5565])
        arc=ProofArc(contact,anchor,anchor)
        for t in np.arange(0.,arc.duration+1.2,.05):
            now+=.05
            body=plate+rotation@arc.position(max(0.,t-.5))
            out=detector.update(AttachmentObservation(time_s=now,
                ring_position_world=np.zeros(3),rotation_world_from_ring=np.eye(3),ring_time_s=now,
                magnet_position_world=plate+rotation@contact, rotation_world_from_magnet=np.eye(3),magnet_time_s=now,
                drone_position_world=body,drone_time_s=now,anchor_position_world=body,
                magnet_on=True,proof_requested=t>=.5,proof_motion_complete=t>=arc.duration+.5))
        assert out.confirmed and not out.lost and out.proof_excitation_m==0.
        fleet.update_phase(vehicle,"M2_ATTACHED_HOLD")
        for attached in (*fleet.attached,vehicle):
            fleet.update_attachment_evidence(attached,confirmed=True,lost=False,stamp_s=now)
        state=fleet.step(now)
    assert state.success_hold
    for attached in fleet.attached:
        fleet.update_attachment_evidence(attached,confirmed=True,lost=False,stamp_s=now+100.)
    assert fleet.step(now+100.).success_hold
    assert fleet.request_landing()
    for vehicle in ("drone_1","drone_0"):
        fleet.update_phase(vehicle,"M2_LANDING_STAGE")
        assert not fleet.step(now+100.).aborted
        fleet.update_phase(vehicle,"LANDED_DISARMED")
        fleet.step(now+100.)
    assert fleet.snapshot().landed

def test_three_failures_wait_and_manual_extra_does_not_erase_count():
    b=RetryBudget()
    assert b.fail() and b.attempt==2
    assert b.fail() and b.attempt==3
    assert not b.fail() and b.attempt==3
    b.manual_retry()
    assert b.attempt==4
    assert not b.fail()
