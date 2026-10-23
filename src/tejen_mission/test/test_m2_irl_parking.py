"""Bounded parking geometry and real local-adapter contracts; no ROS flight."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml


def make_arc(**kwargs):
    from tejen_mission.m2_irl_attachment_policy import ParkingArc, ParkingConfig
    contact = np.array([0., 0., .058])
    offset = np.array([0., 0., -.04])
    start = contact + .557 * np.array([np.sin(np.deg2rad(15)), 0., np.cos(np.deg2rad(15))]) - offset
    return ParkingArc(contact, start, offset, [1., 0., 0.],
                      measured_reach_m=.553, resting_body_z=.08,
                      config=ParkingConfig(**kwargs))


def test_parking_arc_continuity_analytic_pva_reach_and_speed_bounds():
    arc = make_arc()
    times = np.linspace(0., arc.duration, 4001)
    samples = [arc.sample(t) for t in times]
    p, v, a = map(np.array, zip(*samples))
    np.testing.assert_allclose(p[0], arc.start_body)
    np.testing.assert_allclose(v[[0, -1]], 0., atol=1e-10)
    np.testing.assert_allclose(a[[0, -1]], 0., atol=1e-10)
    assert np.all(np.diff(p[:, 2]) <= 1e-10)
    radius = np.linalg.norm(p + arc.body_to_anchor - arc.contact, axis=1)
    assert np.max(radius) <= .557 + 1e-10
    assert radius[-1] == pytest.approx(.50)
    assert p[-1, 2] == pytest.approx(.08)
    assert np.max(np.linalg.norm(v, axis=1)) <= .10 + 1e-9
    low = p[:, 2] <= arc.cutoff_z + .15
    assert np.max(np.linalg.norm(v[low], axis=1)) <= .04 + 1e-9
    t = arc.duration * .4
    h = 1e-3
    before, after = arc.sample(t-h), arc.sample(t+h)
    np.testing.assert_allclose((after[0]-before[0])/(2*h), arc.sample(t)[1], atol=1e-7)
    np.testing.assert_allclose((after[1]-before[1])/(2*h), arc.sample(t)[2], atol=1e-7)


def test_parking_tuning_caps_reach_and_rejects_impossible_geometry():
    arc = make_arc(max_radius_m=.8, slack_margin_m=.01, disarm_height_above_rest_m=.03)
    assert arc.end_radius == pytest.approx(.543)
    assert arc.cutoff_z == pytest.approx(.11)
    with pytest.raises(ValueError):
        make_arc(max_radius_m=.001)


def test_irl_motion_trial_is_60_percent_faster_without_relaxing_proof_evidence():
    from tejen_mission.m2_irl_launch_config import parking_ros_parameters, progressive_ros_parameters
    for count in ('two', 'four'):
        payload = yaml.safe_load((Path(__file__).parents[1]/f'config/m2_irl_{count}_drone.yaml').read_text())
        params = parking_ros_parameters(payload, 'attach_park')
        assert params['parking_speed_mps'] == pytest.approx(.16)
        assert params['parking_final_speed_mps'] == pytest.approx(.064)
        proof = progressive_ros_parameters(payload)
        assert proof['planner']['progressive_arc_duration_s'] == pytest.approx(6.25)
        assert proof['planner']['progressive_arc_speed_mps'] == pytest.approx(.048)
        assert proof['observer']['proof_speed_mps'] == pytest.approx(.03)
        assert proof['observer']['proof_dwell_s'] == pytest.approx(.50)
        faster = make_arc(speed_mps=params['parking_speed_mps'], final_speed_mps=params['parking_final_speed_mps'])
        slower = make_arc()
        assert faster.duration == pytest.approx(slower.duration/1.6)
        np.testing.assert_allclose(faster.sample(faster.duration*.5)[0], slower.sample(slower.duration*.5)[0])


def test_parking_yaml_parameters_are_explicit_and_legacy_defaults_remain_off():
    from tejen_mission.m2_irl_launch_config import parking_ros_parameters
    for name in ('two', 'four'):
        payload = yaml.safe_load((Path(__file__).resolve().parents[1] /
                                 f'config/m2_irl_{name}_drone.yaml').read_text())
        assert parking_ros_parameters(payload)['m2b_completion_mode'] == 'airborne_hold'
        params = parking_ros_parameters(payload, 'attach_park')
        assert params['parking_max_radius_m'] == .50
        assert params['parking_slack_margin_m'] == .02
        assert params['parking_disarm_height_above_rest_m'] == .02
        broken = copy.deepcopy(payload)
        broken['m2_irl']['parking']['max_radius_m'] = float('nan')
        with pytest.raises(ValueError):
            parking_ros_parameters(broken, 'attach_park')
        with pytest.raises(ValueError):
            parking_ros_parameters(payload, 'typo')


def parking_adapter():
    from test_m2_irl_progressive_policy import planner_adapter
    from tejen_mission.m2_irl_attachment_policy import ParkingConfig
    from tejen_mission.mission_types import MissionPhase
    p = planner_adapter()
    p.m2b_commissioning_stage = 'b2'
    p.m2b_completion_mode = 'attach_park'
    p.m2b_parking_config = ParkingConfig()
    p.parking_arc = None
    p.parking_time = 0.
    p.parking_last_step_s = None
    p.parking_disarm_requested = False
    p.m2b_last_arming_feedback_s = 0.
    p.phase = MissionPhase.M2_ATTACHED_HOLD
    p.drone_position = make_arc().start_body.copy()
    p.last_reference = SimpleNamespace(positions=p.drone_position[None, :])
    p.takeoff_start_position = np.array([0., 0., .08])
    p.m2b_tether_anchor_body = np.array([0., 0., -.04])
    p.m2b_plate_pose_world = lambda: (np.array([0., 0., .058]), np.eye(3))
    p.m2b_attachment_diagnostics.update(confirmed=True, radial_error_m=0.,
        tangential_error_m=0., normal_error_m=0.)
    p.horizon_samples = 61
    p.dt = 1./30
    p.min_reference_z = .60
    p.cancel_c1f2_reference_recovery = lambda: None
    p.drone_command_publisher = SimpleNamespace(publish=lambda msg: p.commands.append(msg.data))
    p.commands = []
    p.magnets = []
    p.publish_magnet_command = p.magnets.append
    return p


def test_adapter_parks_without_floor_clamp_keeps_on_and_waits_for_disarm_ack():
    from tejen_mission.mission_types import MissionPhase
    p = parking_adapter()
    p.update_m2b_progressive(0.)
    assert p.phase == MissionPhase.M2_PARKING_ATTACHED
    p.m2d_enabled = True
    p.m2d_assignment_received = p.m2d_mission_permission = True
    p.m2b_bootstrap_arm_permitted = True
    assert not p.m2b_arm_permitted()
    assert p.m2b_magnet_command() == 'ON'
    p.parking_time = p.parking_arc.duration
    p.drone_position = p.parking_arc.sample(p.parking_time)[0]
    p.last_drone_time = .1
    p.update_m2b_progressive(.1)
    assert p.commands == ['DISARM']

    assert p.phase == MissionPhase.M2_PARKING_ATTACHED
    ref = p.build_m2b_b1_reference()
    assert np.max(ref.positions[:, 2]) < .60
    p._physical_now_s = lambda: .2
    p.m2b_arming_state_callback(SimpleNamespace(data=False))
    p.last_drone_time = .2
    p.update_m2b_progressive(.2)
    assert p.phase == MissionPhase.M2_PARKED_ATTACHED
    assert p.m2b_magnet_command() == 'ON'
    p.external_landing_callback(SimpleNamespace(data=True))
    assert not p.m2b_controller_armed
    assert p.m2b_magnet_command() == 'OFF'
    assert p.commands == ['DISARM']
    assert p.magnets == ['OFF']


def test_parking_stale_or_lost_pauses_and_land_retains_release_order():
    from tejen_mission.mission_types import MissionPhase
    p = parking_adapter()
    p.update_m2b_progressive(0.)
    p.last_drone_time = .1
    p.update_m2b_progressive(.1)
    progress = p.parking_time
    p.m2b_attachment_diagnostics['fresh'] = False
    p.update_m2b_progressive(10.)
    assert p.parking_time == progress and p.m2b_magnet_command() == 'ON'
    p.m2b_attachment_diagnostics['fresh'] = True
    p.last_drone_time = 10.
    p.update_m2b_progressive(10.)
    assert p.parking_time == progress  # Reacquisition cannot credit the stale interval.
    p.m2b_controller_armed = False
    p.update_m2b_progressive(10.)
    assert p.phase == MissionPhase.M2_PARKING_ATTACHED and not p.commands
    p.m2b_controller_armed = True
    p.m2b_attachment_diagnostics.update(fresh=True, lost=True, confirmed=False)
    p.last_drone_time = 10.
    p.update_m2b_progressive(10.)
    assert p.parking_time == progress and not p.commands
    p.external_landing_callback(SimpleNamespace(data=True))
    assert p.phase == MissionPhase.M2_LANDING_STAGE
    assert p.m2b_retreat_stage == 'release' and p.m2b_magnet_command() == 'OFF'


def parked_core():
    from tejen_mission.m2d_sequential_supervision import M2DSequentialSupervisorCore
    core = M2DSequentialSupervisorCore(vehicle_ids=('drone_0', 'drone_1'),
        progressive_policy=True, operator_hold_after_goal=True, completion_mode='attach_park')
    core.freeze_assignment({'drone_0': 0, 'drone_1': 6})
    core.update_m2c_ready(True)
    return core


def fresh_contact(core, now):
    for vehicle in core.vehicle_ids:
        core.update_attachment_evidence(vehicle, confirmed=True, lost=False, stamp_s=now)


def test_fleet_retains_active_through_proof_parking_and_fresh_disarm_ack():
    core = parked_core()
    for phase in ('M2_ATTACHED_HOLD', 'M2_PARKING_ATTACHED'):
        core.update_phase('drone_0', phase)
        fresh_contact(core, 1.)
        assert core.step(1.).active_vehicle_id == 'drone_0'
        assert core.snapshot().permissions['drone_0']
    core.update_phase('drone_0', 'M2_PARKED_ATTACHED')
    core.update_arming_state('drone_0', False, stamp_s=0.)
    assert core.step(1.).active_vehicle_id == 'drone_0'
    core.update_arming_state('drone_0', True, stamp_s=1.)
    assert core.step(1.).active_vehicle_id == 'drone_0'
    core.update_arming_state('drone_0', False, stamp_s=1.)
    assert core.step(1.).active_vehicle_id == 'drone_1'
    assert core.snapshot().parked_vehicle_ids == ('drone_0',)
    core.update_phase('drone_1', 'M2_PARKED_ATTACHED')
    core.update_arming_state('drone_1', False, stamp_s=1.1)
    fresh_contact(core, 1.1)
    assert core.step(1.1).state == 'M2D_PARKED'
    assert not any(core.snapshot().permissions.values())
    assert core.request_landing()
    for vehicle in ('drone_1', 'drone_0'):
        assert core.snapshot().landing_vehicle_id == vehicle
        core.update_phase(vehicle, 'LANDED_DISARMED')
        fresh_contact(core, 1.2)
        core.step(1.2)
    assert core.snapshot().state == 'M2D_LANDED'
    assert not core.snapshot().parked_vehicle_ids


def test_parked_peer_loss_blocks_progress_and_manual_release_withdraws_success():
    core = parked_core()
    for vehicle in core.vehicle_ids:
        core.update_phase(vehicle, 'M2_PARKED_ATTACHED')
        core.update_arming_state(vehicle, False, stamp_s=1.)
        fresh_contact(core, 1.)
        core.step(1.)
    core.update_attachment_evidence('drone_0', confirmed=False, lost=True, stamp_s=1.1)
    assert core.step(1.1).state == 'M2D_OPERATOR_WAIT'
    assert not core.snapshot().aborted
    fresh_contact(core, 1.2)
    core.update_phase('drone_0', 'LANDED_DISARMED')
    assert core.step(1.2).state != 'M2D_PARKED'
    assert 'drone_0' not in core.snapshot().parked_vehicle_ids


def test_parking_launch_runners_and_existing_fleet_land_button_contract():
    repo = Path(__file__).resolve().parents[3]
    launch = (repo / 'src/tejen_mission/launch/m2_irl_two_drone.launch.py').read_text()
    assert 'parking_ros_parameters(payload, completion_mode)' in launch
    assert '**parking_params' in launch
    assert '"completion_mode": completion_mode' in launch
    assert 'DeclareLaunchArgument("completion_mode", default_value="airborne_hold"' in launch
    for count in ('two', 'four'):
        runner = (repo / f'tools/irl_test/run_m2_irl_{count}_drone.sh').read_text()
        assert 'completion_mode="${5:-airborne_hold}"' in runner
        assert 'completion_mode:="${completion_mode}"' in runner
        assert 'ros2 topic pub' not in runner and 'ros2 service call' not in runner
    panel = (repo / 'src/drone_visualisation/src/m2_fleet_panel.cpp').read_text()
    assert panel.count('fleet_state_ == "M2D_PARKED"') >= 2
    wrapper = (repo / 'src/tejen_mission/tejen_mission/m2d_fleet_supervisor.py').read_text()
    assert 'f"{ns}/arming_state_feedback"' in wrapper
    assert '"parked_vehicle_ids"' in wrapper


def override_adapter():
    p = parking_adapter()
    p.update_m2b_progressive(0.)
    p.events = []
    p.progressive_event_publisher = SimpleNamespace(publish=lambda msg: p.events.append(json.loads(msg.data)))
    p.get_logger = lambda: SimpleNamespace(info=lambda value: None)
    p.enable_csv_logging = False
    p._physical_now_s = lambda: .1
    p.last_drone_time = .1
    p.m2d_enabled = p.m2d_mission_permission = True
    return p


def test_manual_accept_parked_bypasses_only_height_and_waits_for_ack():
    from tejen_mission.mission_types import MissionPhase
    p = override_adapter()
    assert p.drone_position[2] > p.parking_arc.cutoff_z
    assert p.parking_override_available(.1)
    p.parking_override_callback(SimpleNamespace(data=True))
    assert p.commands == ['DISARM'] and p.events[-1]['accepted']
    assert p.phase == MissionPhase.M2_PARKING_ATTACHED and p.m2b_magnet_command() == 'ON'
    assert not p.parking_override_available(.1)
    assert p.build_m2b_b1_reference().velocities.max() == 0.
    p.parking_override_callback(SimpleNamespace(data=True))
    assert p.commands == ['DISARM'] and not p.events[-1]['accepted']
    p.m2b_controller_armed = False
    p.update_m2b_progressive(.1)  # Negative feedback from BEFORE request cannot acknowledge it.
    assert p.phase == MissionPhase.M2_PARKING_ATTACHED
    p._physical_now_s = lambda: .2
    p.m2b_arming_state_callback(SimpleNamespace(data=False))
    p.last_drone_time = .2
    p.update_m2b_progressive(.2)
    assert p.phase == MissionPhase.M2_PARKED_ATTACHED and p.m2b_magnet_command() == 'ON'


def test_manual_accept_parked_rejects_wrong_phase_authority_or_unproven_stale_contact():
    from tejen_mission.mission_types import MissionPhase
    for change in ('phase', 'mode', 'owner', 'proof', 'lost', 'stale', 'body', 'link', 'armed'):
        p = override_adapter()
        if change == 'phase': p.phase = MissionPhase.M2_ATTACH_PROOF
        if change == 'mode': p.m2b_completion_mode = 'airborne_hold'
        if change == 'owner': p.m2d_mission_permission = False
        if change == 'proof': p.m2b_attachment_diagnostics['confirmed'] = False
        if change == 'lost': p.m2b_attachment_diagnostics['lost'] = True
        if change == 'stale': p.m2b_attachment_diagnostics['fresh'] = False
        if change == 'body': p.last_drone_time = -1.
        if change == 'link':
            p.m2b_require_recent_link_statistics = True
            p.m2b_last_link_statistics_s = None
            p.m2b_link_statistics_timeout_s = .75
        if change == 'armed': p.m2b_controller_armed = False
        p.parking_override_callback(SimpleNamespace(data=True))
        assert not p.commands and not p.events[-1]['accepted'], change
    repo = Path(__file__).resolve().parents[3]
    panel = (repo/'src/drone_visualisation/src/m2_fleet_panel.cpp').read_text()
    assert 'ACCEPT PARKED ACTIVE' in panel and 'QMessageBox::question' in panel
    assert '/join_planner/accept_parked' in panel
