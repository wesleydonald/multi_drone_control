import csv
import json
from pathlib import Path

import numpy as np

from tejen_mission.attachment_detection import AttachmentEvidenceState
from tejen_mission.m2a_attachment_runtime import (
    default_calibration,
    default_detector_config,
    observation_from_csv_row,
    quaternion_xyzw_to_rotation,
    reconstruct_relative_pose_world,
    detector_config_to_dict,
    detector_config_from_dict,
    parse_gz_detachable_joint_state_line,
)
from tejen_mission.m2a_attachment_replay import replay_attachment_csv


def _quat_z(yaw):
    return np.array([0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)])


def test_quaternion_rotation_and_relative_pose_reconstruction():
    q = _quat_z(np.pi / 2.0)
    R = quaternion_xyzw_to_rotation(q)
    np.testing.assert_allclose(R @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-12)

    p_world, R_world = reconstruct_relative_pose_world(
        parent_position_world=np.array([1.0, 2.0, 3.0]),
        parent_quaternion_xyzw=q,
        child_position_parent=np.array([1.0, 0.0, 0.0]),
        child_quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    np.testing.assert_allclose(p_world, [1.0, 3.0, 3.0], atol=1e-12)
    np.testing.assert_allclose(R_world, R, atol=1e-12)


def test_detector_config_roundtrip_is_json_serializable():
    cfg = default_detector_config()
    payload = detector_config_to_dict(cfg)
    json.dumps(payload)
    recovered = detector_config_from_dict(payload)
    assert recovered.candidate_xy_m == cfg.candidate_xy_m
    assert recovered.loss_xy_m == cfg.loss_xy_m
    np.testing.assert_allclose(recovered.proof_direction_plate, cfg.proof_direction_plate)


def test_default_calibration_places_contact_below_magnet_center():
    calibration = default_calibration()
    np.testing.assert_allclose(calibration.contact_position_magnet, [0.0, 0.0, -0.025])


def test_csv_observation_does_not_accept_joint_truth_as_detector_input():
    row = {
        'ros_time_s': '1.0',
        'ring_time_s': '1.0',
        'magnet_time_s': '1.0',
        'drone_time_s': '1.0',
        'ring_x': '0', 'ring_y': '0', 'ring_z': '0.1',
        'ring_qx': '0', 'ring_qy': '0', 'ring_qz': '0', 'ring_qw': '1',
        'magnet_x': '0.25', 'magnet_y': '0', 'magnet_z': '0.125',
        'magnet_qx': '0', 'magnet_qy': '0', 'magnet_qz': '0', 'magnet_qw': '1',
        'drone_x': '0.25', 'drone_y': '0', 'drone_z': '0.615',
        'magnet_on': 'true',
        'proof_requested': 'false',
        'joint_detached_truth': 'false',
    }
    obs = observation_from_csv_row(row)
    assert not hasattr(obs, 'joint_detached_truth')
    assert obs.magnet_on is True
    assert obs.proof_requested is False


def test_replay_can_confirm_synthetic_attached_trace_without_using_truth(tmp_path):
    input_csv = tmp_path / 'attachment.csv'
    output_csv = tmp_path / 'replay.csv'
    metadata = tmp_path / 'metadata.json'
    cfg = default_detector_config()
    metadata.write_text(json.dumps({
        'assigned_plate_id': 0,
        'detector_config': detector_config_to_dict(cfg),
        'calibration': {
            'contact_position_magnet': [0.0, 0.0, -0.025],
            'rotation_magnet_from_contact': np.eye(3).tolist(),
        },
    }))

    fieldnames = [
        'ros_time_s','ring_time_s','magnet_time_s','drone_time_s',
        'ring_x','ring_y','ring_z','ring_qx','ring_qy','ring_qz','ring_qw',
        'magnet_x','magnet_y','magnet_z','magnet_qx','magnet_qy','magnet_qz','magnet_qw',
        'drone_x','drone_y','drone_z','magnet_on','proof_requested','joint_detached_truth',
    ]
    with input_csv.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        # Contact point is magnet centre - 25 mm, so centre z=0.125 puts contact on ring z=0.1.
        for k in range(30):
            t = 0.05 * k
            proof = k >= 10
            # During proof, move the drone along configured proof direction while magnet stays fixed.
            dx = 0.0
            dz = 0.0
            if proof:
                distance = min(0.03, 0.003 * (k - 10))
                dx = distance * cfg.proof_direction_plate[0]
                dz = distance * cfg.proof_direction_plate[2]
            writer.writerow({
                'ros_time_s': t, 'ring_time_s': t, 'magnet_time_s': t, 'drone_time_s': t,
                'ring_x': 0, 'ring_y': 0, 'ring_z': 0.1,
                'ring_qx': 0, 'ring_qy': 0, 'ring_qz': 0, 'ring_qw': 1,
                'magnet_x': 0.25, 'magnet_y': 0, 'magnet_z': 0.125,
                'magnet_qx': 0, 'magnet_qy': 0, 'magnet_qz': 0, 'magnet_qw': 1,
                'drone_x': 0.25 + dx, 'drone_y': 0, 'drone_z': 0.615 + dz,
                'magnet_on': 'true', 'proof_requested': str(proof).lower(),
                # Deliberately contradictory at first: replay result must not be driven by this field.
                'joint_detached_truth': 'true' if k < 20 else 'false',
            })

    summary = replay_attachment_csv(input_csv=input_csv, output_csv=output_csv, metadata_path=metadata)
    assert summary['rows'] == 30
    assert summary['ever_confirmed'] is True
    rows = list(csv.DictReader(output_csv.open()))
    assert any(r['replay_state'] == AttachmentEvidenceState.CONFIRMED.value for r in rows)


def test_gazebo_detachable_joint_string_state_parser_preserves_detached_semantics():
    assert parse_gz_detachable_joint_state_line('data: "detached"') is True
    assert parse_gz_detachable_joint_state_line('data: "attached"') is False
    assert parse_gz_detachable_joint_state_line('header { stamp { sec: 1 } }') is None
