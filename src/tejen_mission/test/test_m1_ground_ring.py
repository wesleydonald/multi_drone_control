"""Focused ground-ring geometry, authority and release-order contract."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from tejen_mission.m1_ground_ring import (
    GroundRingExit,
    MeasuredRingSource,
    minimum_face_gap,
    plate0_world,
)
from tejen_mission.fake_cooperative_transport_world import FakeCooperativeTransportWorld


ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / "src/tejen_mission"


def test_visual_labels_cannot_move_published_plate_or_obstacle_geometry():
    world = SimpleNamespace(
        append_ring_markers=lambda markers, payload_state, stamp: None,
        make_marker_common=lambda marker_id, namespace, kind, stamp: Marker(),
        get_float_param=lambda name: 0.06,
        ring_attachment_plate_index=0,
        attachment_trail=[],
    )
    plate_pose = PoseStamped()
    plate_pose.pose.position.z = 0.05
    obstacle = SimpleNamespace(
        state=SimpleNamespace(x=0.0, y=0.0, z=0.4, yaw=0.0),
        radius=0.10,
        safety_radius=0.15,
    )
    world.make_pose = FakeCooperativeTransportWorld.make_pose.__get__(world)
    markers = FakeCooperativeTransportWorld.make_markers(
        world, None, None, plate_pose, [obstacle], Time()
    ).markers
    assert np.isclose(plate_pose.pose.position.z, 0.05)
    assert np.isclose(markers[0].pose.position.z, 0.05)
    assert np.isclose(markers[1].pose.position.z, 0.17)
    assert np.isclose(markers[2].pose.position.z, 0.4)
    assert np.isclose(markers[3].pose.position.z, 0.4)
    assert np.isclose(markers[4].pose.position.z, 0.63)


def test_measured_stationary_source_has_no_origin_or_stale_future():
    source = MeasuredRingSource(timeout_s=.5, max_tilt_deg=5.)
    assert source.state(0.) is None
    assert source.ingest([.3, -.2, .05], [0., 0., 0., 1.], now_s=1.)
    state = source.state(1.2)
    assert state is not None
    np.testing.assert_allclose(state.position, [.3, -.2, .05])
    np.testing.assert_allclose(state.velocity, [0., 0., 0.])
    assert state.yaw_rad == 0.
    assert source.state(1.6) is None
    assert not source.ingest([.3, -.2, .05], [.2, 0., 0., .98], now_s=2.)
    assert source.state(2.) is None


def test_same_measured_pose_gives_one_plate_and_constant_future():
    source = MeasuredRingSource(timeout_s=.5, max_tilt_deg=5.)
    assert source.ingest([.1, .2, .05], [0., 0., 0., 1.], now_s=1.)
    state = source.state(1.2)
    assert state is not None
    np.testing.assert_allclose(state.plate0, [.32375, .2, .05])
    np.testing.assert_allclose(state.velocity, [0., 0., 0.])
    for point in state.future_points(8):
        np.testing.assert_allclose(point, state.position)


def test_plate_zero_and_ground_floor_reachability():
    centre = np.array([1.0, 2.0, 0.05])
    np.testing.assert_allclose(
        plate0_world(centre, np.pi / 2, 0.4475), [1.0, 2.22375, 0.05], atol=1e-9
    )
    assert np.isclose(minimum_face_gap(.05, .60, .53, .0225), -.0025)
    assert minimum_face_gap(0.0, .60, .53, .0225) > .005


def test_manual_land_releases_then_rises_before_any_lateral_return():
    exit = GroundRingExit([.22, 0., .60], [.60, 0.], started_s=0.)
    for time_s in (0., .24):
        decision = exit.step(time_s, face_gap_m=.0, fresh=True,
                             body_position=[.22, 0., .60], body_speed_mps=0.)
        assert decision.stage == "release_wait"
        assert not decision.magnet_on
        np.testing.assert_allclose(decision.target_position[:2], [.22, 0.])
    decision = exit.step(.26, face_gap_m=.0, fresh=True,
                         body_position=[.22, 0., .60], body_speed_mps=0.)
    assert decision.stage == "rise"
    np.testing.assert_allclose(decision.target_position[:2], [.22, 0.])
    exit.step(.30, face_gap_m=.081, fresh=True,
              body_position=[.22, 0., .61], body_speed_mps=0.)
    decision = exit.step(.43, face_gap_m=.081, fresh=True,
                         body_position=[.22, 0., .61], body_speed_mps=0.)
    assert decision.stage == "return"
    np.testing.assert_allclose(decision.target_position[:2], [.60, 0.])
    decision = exit.step(.50, face_gap_m=.10, fresh=True,
                         body_position=[.60, 0., .61], body_speed_mps=0.)
    assert decision.ready_to_land


def test_stale_or_unseparated_ring_never_authorises_lateral_motion():
    exit = GroundRingExit([.22, 0., .60], [.60, 0.], started_s=0.)
    decision = exit.step(1.0, face_gap_m=.10, fresh=False,
                         body_position=[.22, 0., .60], body_speed_mps=0.)
    assert decision.stage != "return"
    np.testing.assert_allclose(decision.target_position[:2], [.22, 0.])
    decision = exit.step(8.0, face_gap_m=.0, fresh=True,
                         body_position=[.22, 0., .80], body_speed_mps=0.)
    assert decision.stage == "blocked"
    assert not decision.ready_to_land
    np.testing.assert_allclose(decision.target_position[:2], [.22, 0.])


def test_verified_separation_survives_return_gap_dip_and_stale_return_resumes():
    exit = GroundRingExit([.22, 0., .60], [.60, 0.], started_s=0.)
    exit.step(.30, face_gap_m=.10, fresh=True,
              body_position=[.22, 0., .60], body_speed_mps=0.)
    assert exit.step(.43, face_gap_m=.10, fresh=True,
                     body_position=[.22, 0., .60], body_speed_mps=0.).stage == "return"
    decision = exit.step(.50, face_gap_m=.02, fresh=True,
                         body_position=[.40, 0., .65], body_speed_mps=0.)
    assert decision.stage == "return"  # clearance is an entry certificate, not a return gate
    np.testing.assert_allclose(decision.target_position[:2], [.60, 0.])
    stale = exit.step(.55, face_gap_m=float('nan'), fresh=False,
                      body_position=[.40, 0., .65], body_speed_mps=0.)
    assert stale.stage == "return" and not stale.ready_to_land
    np.testing.assert_allclose(stale.target_position[:2], [.40, 0.])
    later = exit.step(.60, face_gap_m=.02, fresh=True,
                      body_position=[.43, 0., .65], body_speed_mps=0.)
    np.testing.assert_allclose(later.target_position[:2], [.60, 0.])



def test_ground_profile_keeps_virtual_m1_default_and_ring_8_opt_in():
    config = PACKAGE / "config/m1_ground_ring.yaml"
    launch = PACKAGE / "launch/m1_irl_ground_ring.launch.py"
    mocap = PACKAGE / "tejen_mission/motion_capture_publisher_irl.py"
    world = PACKAGE / "tejen_mission/fake_cooperative_transport_world.py"
    planner = PACKAGE / "tejen_mission/online_join_planner.py"
    assert "ring_rigid_body_id: \"8\"" in config.read_text()
    assert "measured_stationary" in config.read_text()
    assert "ground_ring" in launch.read_text()
    assert "ring_pose_publisher.publish" in mocap.read_text()
    assert "measured_stationary" in world.read_text()
    assert "ground_ring_attach_trial" in planner.read_text()


def test_ground_launch_owns_one_udp_reader_and_one_measured_commitment():
    launch = (PACKAGE / "launch/m1_irl_ground_ring.launch.py").read_text()
    backend = (ROOT / "src/tejen_dynamic_planner/launch/"
               "c1f_irl_m1_virtual_ring_backend.launch.py").read_text()
    assert "'stopped_continuation_enabled': 'true'" in launch
    assert "'c1f2_stopped_continuation_enabled': True" in launch
    assert "'external_reference_validate_timing': True" in launch
    assert launch.count("executable='motion_capture_publisher_irl'") == 1
    assert "m1_ground_ring.yaml" in launch
    assert "ring_commitment_topic" in launch
    assert "/irl/ring/committed_trajectory" in launch
    assert "ring_commitment_topic" in backend
    assert "executable='fake_cooperative_transport_world'" in launch
    assert "name='fake_cooperative_transport_world'" in launch
    assert "executable='elrs_interface_irl'" in launch
    assert "executable='online_join_planner'" in launch
    assert "executable='main'" in launch
    assert "executable='motion_capture_publisher_irl'" in launch
    assert "ros2 topic pub" not in launch


def test_ground_trial_never_claims_proven_handoff_and_land_owns_release():
    planner = (PACKAGE / "tejen_mission/online_join_planner.py").read_text()
    assert "ground_ring_attach_trial" in planner
    assert "ground_ring_face_gap_m" in planner
    assert "ground_ring_exit" in planner
    assert "GroundRingExit(" in planner
    assert "self.publish_magnet_command('OFF')" in planner
    assert "not self.ground_ring_attach_trial" in planner
    assert "MissionPhase.DESCEND_TO_ATTACHMENT" in planner
    assert "MissionPhase.ATTACH_READY" in planner
    assert "minimum_face_gap(" in (ROOT / "tools/irl_test/check_m1_ground_ring.py").read_text()
    assert "self.ground_ring_preflight(" not in planner
    checker = (ROOT / "tools/irl_test/check_m1_ground_ring.py").read_text()
    assert "same_side_pad(" not in checker
    assert "unverified" in planner.lower()
    assert "self.ground_ring_exit is not None" in planner
