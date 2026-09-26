from pathlib import Path
import math
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
FIXTURE = ROOT / 'simulation_assets' / "tejen" / 'm2a_ring_fixture.sdf'
SUPPORT = ROOT / 'simulation_assets' / "tejen" / 'm2a_x3_support.sdf'
WORLD = ROOT / 'simulation_assets' / "tejen" / 'world_m2a_attachment_foundations.sdf'
RUNNER = ROOT / 'tools' / 'sim_test' / 'run_m2a_attachment_foundations.sh'
PROBE = ROOT / 'tools' / 'sim_test' / 'm2a_attachment_probe.py'


def _pose_xyz(text):
    values = [float(v) for v in text.split()]
    return values[:3]


def test_ring_fixture_is_static_circular_support_without_square_slab():
    root = ET.parse(FIXTURE).getroot()
    model = root.find('.//model')
    assert model is not None
    assert model.findtext('static', default='false').strip().lower() == 'true'

    names = [e.attrib.get('name', '') for e in root.findall('.//collision')]
    assert 'ring_support_collision' not in names

    segments = [e for e in root.findall('.//collision') if e.attrib.get('name', '').startswith('ring_segment_')]
    assert len(segments) >= 24
    for segment in segments:
        size = [float(v) for v in segment.findtext('geometry/box/size').split()]
        assert max(size[:2]) < 0.10
        x, y, _ = _pose_xyz(segment.findtext('pose'))
        assert abs(math.hypot(x, y) - 0.25) < 1e-9


def test_fixture_has_twelve_plate_centres_on_025m_pitch_circle_and_plate_top_at_zero():
    root = ET.parse(FIXTURE).getroot()
    plates = [e for e in root.findall('.//collision') if re.fullmatch(r'plate_\d+_collision', e.attrib.get('name', ''))]
    assert len(plates) == 12
    for plate in plates:
        x, y, z = _pose_xyz(plate.findtext('pose'))
        assert abs(math.hypot(x, y) - 0.25) < 1e-9
        length = float(plate.findtext('geometry/cylinder/length'))
        assert abs((z + 0.5 * length) - 0.0) < 1e-12


def test_plate_zero_is_visually_distinct():
    root = ET.parse(FIXTURE).getroot()
    plate0 = next(v for v in root.findall('.//visual') if v.attrib.get('name') == 'plate_0_visual')
    ambient = [float(v) for v in plate0.findtext('material/ambient').split()]
    assert ambient[0] > ambient[1]


def test_external_support_is_invisible_kinematic_and_holds_real_x3_base():
    root = ET.parse(SUPPORT).getroot()
    model = root.find('.//model')
    assert model is not None and model.attrib.get('name') == 'm2a_x3_support'
    link = model.find('link')
    assert link is not None and link.attrib.get('name') == 'support_link'
    assert link.findtext('kinematic', default='false').strip().lower() == 'true'
    assert link.find('visual') is None
    assert link.find('collision') is None

    plugin = next(p for p in model.findall('plugin') if 'DetachableJoint' in p.attrib.get('name', ''))
    assert plugin.findtext('parent_link') == 'support_link'
    assert plugin.findtext('child_model') == 'x3'
    assert plugin.findtext('child_link') == 'X3/base_link'
    assert plugin.findtext('detach_topic') == '/m2a/support/detach'
    assert plugin.findtext('output_topic') == '/m2a/support/state'


def test_world_restores_real_m2_x3_and_does_not_reference_placeholder_carrier():
    text = WORLD.read_text(encoding='utf-8')
    root = ET.parse(WORLD).getroot()
    includes = root.findall('.//include')

    x3_include = next(i for i in includes if i.findtext('uri') == 'modelLargeM2BallMagnet.sdf')
    x, y, z = _pose_xyz(x3_include.findtext('pose'))
    assert abs(x - 0.55) < 1e-12
    assert abs(y) < 1e-12
    assert abs(z - 0.855) < 1e-12

    support_include = next(i for i in includes if i.findtext('uri') == 'm2a_x3_support.sdf')
    sx, sy, sz = _pose_xyz(support_include.findtext('pose'))
    assert (sx, sy, sz) == (x, y, z)

    assert 'm2a_kinematic_carrier.sdf' not in text
    assert 'm2a_carrier' not in text
    assert 'm2a_ring_fixture.sdf' in text
    assert 'PosePublisher' in text


def test_runner_bootstraps_detachable_joint_while_paused_then_unpauses():
    text = RUNNER.read_text(encoding='utf-8')
    assert 'start_bg gz sim "$REPO/simulation_assets/tejen/world_m2a_attachment_foundations.sdf"' in text
    assert 'gz sim -r' not in text
    assert "--req 'multi_step: 1'" in text
    assert '/payload/detach' in text
    assert 'wait_for_gz_state "$LOG_DIR/payload_joint_bootstrap.log" "attached"' in text
    assert 'wait_for_gz_state "$LOG_DIR/payload_joint_bootstrap.log" "detached"' in text
    assert 'wait_for_gz_state "$LOG_DIR/support_joint_bootstrap.log" "attached"' in text
    assert "--req 'pause: false'" in text
    assert '/model/x3/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' in text
    assert 'force_initial_detach:=false' in text
    assert 'carrier_frame_leaf:=base_link' in text
    assert 'm2a_kinematic_carrier' not in text


def test_runner_retains_indefinite_sanity_mode_without_flight_stack():
    text = RUNNER.read_text(encoding='utf-8')
    assert 'sanity' in text
    assert 'SANITY MODE ACTIVE' in text
    assert 'wait_for_interrupt' in text
    assert 'ros2 run tejen_mpc' not in text
    assert 'ros2 run tejen_mission online_join_planner' not in text


def test_probe_moves_only_external_support_and_uses_real_x3_geometry_heights():
    text = PROBE.read_text(encoding='utf-8')
    assert 'm2a_x3_support' in text
    assert 'm2a_carrier' not in text
    assert 'HORIZONTAL_APPROACH' in text
    assert 'CONTACT_DESCENT' in text
    assert 'PROOF' in text
    assert 'start-z' in text and 'default=0.855' in text
    assert 'precontact-z' in text and 'default=0.715' in text
    assert 'contact-z' in text and 'default=0.675' in text
    assert 'ros_string("/join_planner/reference"' not in text
    assert 'ros_bool("/join_planner/reference"' not in text
