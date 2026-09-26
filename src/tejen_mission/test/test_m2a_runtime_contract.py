from pathlib import Path
import ast
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]


def _text(rel):
    return (ROOT / rel).read_text()


def test_observer_has_no_trajectory_authority_and_logs_truth_only():
    text = _text('src/tejen_mission/tejen_mission/m2_attachment_observer.py')
    assert '/join_planner/reference' not in text
    assert 'MultiDOFJointTrajectory' not in text
    assert 'joint_detached_truth' in text
    assert 'AttachmentObservation(' in text
    observation_block = text.split('AttachmentObservation(', 1)[1].split(')', 1)[0]
    assert 'joint_detached_truth' not in observation_block


def test_observer_defaults_are_restored_to_real_x3_named_frames():
    text = _text('src/tejen_mission/tejen_mission/m2_attachment_observer.py')
    assert '"/model/x3/pose"' in text
    assert '"base_link"' in text
    assert '"magnet_tip_link"' in text
    assert 'kinematic carrier anchor' not in text


def test_physical_capture_manager_is_independent_of_software_detector():
    text = _text('src/tejen_mission/tejen_mission/m2a_physical_capture_manager.py')
    tree = ast.parse(text)
    imported = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            imported.append(node.module)
        elif isinstance(node, ast.Import):
            imported.extend(alias.name for alias in node.names)
    assert not any(name.endswith('attachment_detection') for name in imported)
    assert 'physical_capture_xy_m' in text
    assert 'physical_pose_timeout_s' in text
    assert '/payload/attach' in text
    assert '/payload/detach' in text


def test_joint_truth_bridge_republishes_latest_truth_for_late_observers():
    text = _text('src/tejen_mission/tejen_mission/m2a_gazebo_joint_truth_bridge.py')
    assert '/payload/detachable_joint_state' in text
    assert '/m2a/sim/joint_detached_truth' in text
    assert 'self.last_detached' in text
    assert 'self.pub.publish(msg)' in text
    assert 'attachment_detection' not in text


def test_probe_is_test_only_and_has_no_reference_authority():
    text = _text('tools/sim_test/m2a_attachment_probe.py')
    assert 'ros_string("/join_planner/reference"' not in text
    assert 'ros_bool("/join_planner/reference"' not in text
    assert '/magnet/command' in text
    assert '/m2a/attachment/proof_requested' in text
    assert '/world/quadcopter/set_pose' in text
    assert 'm2a_x3_support' in text


def test_supervised_runner_uses_m2a_log_folder_and_no_controller():
    text = _text('tools/sim_test/run_m2a_attachment_foundations.sh')
    assert 'logs/m2_attachment' in text
    assert 'world_m2a_attachment_foundations.sdf' in text
    assert 'm2_attachment_observer.py' in text
    assert 'm2a_physical_capture_manager.py' in text
    assert 'm2a_gazebo_joint_truth_bridge.py' in text
    assert 'ros2 run tejen_mpc' not in text
    assert 'ros2 run tejen_mission online_join_planner' not in text
    assert 'timeout 5 ros2 topic echo /model/x3/pose --once' in text
    assert 'timeout 5 ros2 topic echo /m2a/sim/joint_detached_truth --once' in text


def test_fixture_plate_top_surface_is_ring_plane():
    path = ROOT / 'simulation_assets/tejen/m2a_ring_fixture.sdf'
    tree = ET.parse(path)
    for collision in tree.findall('.//collision'):
        if collision.attrib.get('name', '').startswith('plate_'):
            pose = [float(v) for v in collision.findtext('pose').split()]
            length = float(collision.findtext('geometry/cylinder/length'))
            assert abs((pose[2] + 0.5 * length) - 0.0) < 1e-12


def test_m2a_shell_helpers_do_not_enable_nounset_or_pipefail():
    for rel in (
        'tools/sim_test/validate_m2a_runtime_increment.sh',
        'tools/sim_test/run_m2a_attachment_foundations.sh',
    ):
        command_lines = [
            line.strip() for line in _text(rel).splitlines()
            if line.strip().startswith('set ')
        ]
        assert command_lines[0] == 'set -e'
        assert set(command_lines).issubset({'set -e', 'set +e'})
