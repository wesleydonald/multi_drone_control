"""tools/experiment/record.py: the `record:` block, the recording world copy, and the gz command."""
import math
import os
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, TOOLS)

from experiment import record as R                    # noqa: E402
from experiment.config import ExperimentConfig        # noqa: E402

REPO = os.path.dirname(TOOLS)
M1_WORLD = os.path.join(REPO, 'simulation_assets', 'three_rigid_ground_rig_attach_welded_pickup.sdf')


def test_off_unless_asked():
    assert R.parse(None) is None and R.parse(False) is None


def test_true_takes_every_default():
    s = R.parse(True)
    assert s['period_s'] == 0.01 and s['fps'] == 30 and s['render'] is True
    assert set(s['cameras']) == {'wide', 'ring', 'plate9'}
    assert s['cameras']['wide']['size'] == [1920, 1080]


def test_named_default_cameras_and_overrides():
    s = R.parse({'cameras': ['wide']})
    assert list(s['cameras']) == ['wide']
    s = R.parse({'fps': 25, 'cameras': {'wide': {'hfov_deg': 80},
                                        'top': {'pos': [0, 0, 6], 'look_at': [0.01, 0, 0]}}})
    assert s['cameras']['wide']['hfov_deg'] == 80.0
    assert s['cameras']['wide']['pos'] == R.DEFAULT_CAMERAS['wide']['pos']
    assert s['cameras']['top']['size'] == [1920, 1080]


@pytest.mark.parametrize('raw', [
    {'perid_s': 0.01},                                    # typo
    {'period_s': 0.05},                                   # coarser than a 30 fps frame
    {'cameras': ['nosuch']},
    {'cameras': {'x': {'pos': [1, 0, 1]}}},               # no look_at
    {'cameras': {'x': {'pos': [1, 0, 1], 'look_at': [0, 0, 1], 'size': [1279, 720]}}},
    {'cameras': {'bad name': {'pos': [1, 0, 1], 'look_at': [0, 0, 1]}}},
    {'cameras': {}},
    'yes',
])
def test_bad_blocks_are_refused(raw):
    with pytest.raises(ValueError):
        R.parse(raw)


def test_camera_pose_points_at_the_target():
    assert R.camera_pose([0, 0, 1], [2, 0, 1])[3:] == pytest.approx((0, 0, 0))
    x, y, z, roll, pitch, yaw = R.camera_pose([1, 1, 2], [0, 0, 2 - math.sqrt(2)])
    assert (pitch, yaw) == pytest.approx((math.pi / 4, -3 * math.pi / 4))   # down 45 deg, toward -x-y


def test_config_block_is_parsed(tmp_path):
    doc = {'name': 'unit', 'world': 'three_attach.sdf', 'fleet': {'num_drones': 3, 'n_total': 3},
           'launch': {'args': {'num_drones': 3}}, 'timing': {'duration_s': 10.0}}
    p = tmp_path / 'a.yaml'
    p.write_text(yaml.safe_dump(doc))
    assert ExperimentConfig.from_yaml(str(p)).record is None
    p.write_text(yaml.safe_dump(dict(doc, record={'cameras': ['plate9']})))
    assert list(ExperimentConfig.from_yaml(str(p)).record['cameras']) == ['plate9']


WORLD = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="w">
    <plugin filename="ignition-gazebo-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <model name="m"><link name="l"><sensor name="imu" type="imu"/></link></model>
  </world>
</sdf>
'''


def test_recording_world_adds_cameras_and_drops_the_sensors_system():
    out = R.recording_world(WORLD, R.parse(True))
    world = ET.fromstring(out.split('?>', 1)[1]).find('world')
    plugins = [p.get('name') for p in world.findall('plugin')]
    assert plugins == ['gz::sim::systems::Physics']
    cams = {m.get('name'): m for m in world.findall('model') if m.get('name').startswith('mdc_rec_')}
    assert set(cams) == {'mdc_rec_wide', 'mdc_rec_ring', 'mdc_rec_plate9'}
    s = cams['mdc_rec_wide'].find('link/sensor')
    assert s.get('type') == 'camera' and s.find('topic').text == R.TOPIC_PREFIX + 'wide'
    assert s.find('update_rate').text == '30'
    assert cams['mdc_rec_wide'].find('static').text == 'true'
    assert world.find("model[@name='m']") is not None


def test_a_world_with_a_camera_is_refused():
    with pytest.raises(ValueError):
        R.recording_world(WORLD.replace('type="imu"', 'type="camera"'), R.parse(True))


@pytest.mark.skipif(not os.path.exists(M1_WORLD), reason='M1 world not generated')
def test_the_m1_world_copy_leaves_the_original_alone(tmp_path):
    before = open(M1_WORLD).read()
    dst = R.write_recording_world(M1_WORLD, str(tmp_path / 'rec' / 'w.sdf'), R.parse(True))
    assert open(M1_WORLD).read() == before
    out = open(dst).read()
    assert 'systems::Sensors' not in out and out.count('<sensor name="camera" type="camera">') == 3
    # the copy differs from the original only by the Sensors line and the camera models
    assert out.replace(R.recording_world(before, R.parse(True)), '') == ''
    ET.fromstring(out.split('?>', 1)[1])


def test_gz_command_records_the_copy(tmp_path, monkeypatch):
    import run_experiment as RE
    cfg = ExperimentConfig('unit')
    cfg.world = M1_WORLD
    cfg.record = R.parse(True)
    seen = {}

    class P:
        def __init__(self, cmd, **kw):
            seen['cmd'] = cmd
    monkeypatch.setattr(RE.subprocess, 'Popen', P)
    monkeypatch.setattr(RE.shutil, 'which', lambda _: '/usr/bin/gz')
    (tmp_path / 'logs').mkdir()
    RE.start_gazebo(cfg, str(tmp_path / 'logs' / 'gazebo.log'), nice=10)
    cmd = seen['cmd']
    i = cmd.index('--record-path')
    assert cmd[i + 1] == str(tmp_path / 'gz_record')
    assert cmd[cmd.index('--record-period') + 1] == '0.01'
    assert cmd[-1] == str(tmp_path / 'record' / os.path.basename(M1_WORLD))
    assert os.path.exists(cmd[-1])

    cfg.record = None                                    # a normal run: no recording, its own world
    RE.start_gazebo(cfg, str(tmp_path / 'logs' / 'gazebo.log'), nice=10)
    assert '--record-path' not in seen['cmd'] and seen['cmd'][-1] == M1_WORLD


def test_view_moves_only_the_named_camera_in_a_log_copy(tmp_path):
    sm = pytest.importorskip('gz.msgs10.serialized_map_pb2')
    import sqlite3
    import render_video as RV
    name_t, pose_t = RV.fnv1a64('gz_sim_components.Name'), RV.fnv1a64('gz_sim_components.Pose')
    state = sm.SerializedStateMap()
    for eid, name in ((1, 'mdc_rec_ring'), (2, 'mdc_rec_wide'), (3, 'x3_drone0')):
        e = state.entities[eid]
        e.id = eid
        for t, v in ((name_t, name.encode()), (pose_t, b'0 0 0 0 0 0')):
            c = e.components[t - 2**64 if t >= 2**63 else t]     # the map key is an int64
            c.type, c.component = t, v
    src = tmp_path / 'src'
    src.mkdir()
    db = sqlite3.connect(str(src / 'state.tlog'))
    db.execute('CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT, message_type_id INTEGER)')
    db.execute('CREATE TABLE messages (id INTEGER PRIMARY KEY, time_recv INTEGER, topic_id INTEGER, '
               'message BLOB)')
    db.execute("INSERT INTO topics VALUES (1, '/gz_record/sdf', 1), (2, '/world/w/changed_state', 2)")
    db.execute('INSERT INTO messages VALUES (1, 1000000, 1, ?), (2, 1000000, 2, ?)',
               (b'<sdf/>', state.SerializeToString()))
    db.commit()
    db.close()
    name, pose, _ = RV.parse_view('ring:2,0,1:0,0,1')
    assert name == 'ring' and pose.split()[:3] == ['2', '0', '1']
    RV.repose_log(str(src), str(tmp_path / 'dst'), {name: pose})
    out = sm.SerializedStateMap()
    out.ParseFromString(sqlite3.connect(str(tmp_path / 'dst' / 'state.tlog')).execute(
        'SELECT message FROM messages WHERE id = 2').fetchone()[0])
    poses = {eid: next(c.component for c in e.components.values() if c.type == pose_t)
             for eid, e in out.entities.items()}
    assert poses[1] == pose.encode() and poses[2] == poses[3] == b'0 0 0 0 0 0'
    with pytest.raises(SystemExit):
        RV.repose_log(str(src), str(tmp_path / 'dst2'), {'plate9': pose})
