"""tools/rig_to_run.py on a synthetic rig flight: two tracker logs, a bag, a T2 console log, all on epoch time."""
import math
import os
import sys

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
T0 = 1790600000.0


def _tracker(path, i, t):
    n = len(t)
    d = pd.DataFrame({'step': range(n), 'sim_time': t, 'u0': 0, 'u1': 0, 'u2': 0.45 + 0.01 * i, 'u3': 0,
                      'pose_x': float(i), 'pose_y': 0.0, 'pose_z': 0.5, 'pose_qw': 1.0, 'pose_qx': 0.0,
                      'pose_qy': 0.0, 'pose_qz': 0.0, 'ref_x': float(i), 'ref_y': 0.0, 'ref_z': 0.6,
                      'payload_x': 0.0, 'payload_y': 0.0, 'payload_z': 0.3,
                      'payload_ref_x': 0.0, 'payload_ref_y': 0.0, 'payload_ref_z': 0.6})
    os.makedirs(path)
    d.to_csv(os.path.join(path, 'log.csv'), index=False)


def _bag(path, t):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from rclpy.serialization import serialize_message
    from interfaces.msg import ELRSCommand, MotionCaptureState
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topics = {'/payload/motion_capture_state': 'interfaces/msg/MotionCaptureState',
              '/drone_0/motion_capture_state': 'interfaces/msg/MotionCaptureState',
              '/drone_1/motion_capture_state': 'interfaces/msg/MotionCaptureState',
              '/drone_0/ELRSCommand': 'interfaces/msg/ELRSCommand'}
    for name, typ in topics.items():
        w.create_topic(rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format='cdr'))
    half = math.radians(10.0) / 2          # the ring rolled 10 deg
    for tk in t:
        m = MotionCaptureState()
        m.pose.position.z = 0.6
        m.pose.orientation.w, m.pose.orientation.x = math.cos(half), math.sin(half)
        w.write('/payload/motion_capture_state', serialize_message(m), int(tk * 1e9))
        for i in range(2):
            d = MotionCaptureState()
            d.pose.position.x, d.pose.position.z = float(i), 0.9
            d.pose.orientation.w = 1.0
            w.write(f'/drone_{i}/motion_capture_state', serialize_message(d), int(tk * 1e9))
        w.write('/drone_0/ELRSCommand', serialize_message(ELRSCommand(armed=True)), int(tk * 1e9))
    del w


def test_converts_a_synthetic_rig_flight(tmp_path):
    t = T0 + np.arange(0, 10, 0.02)
    trk = [str(tmp_path / f'planner_drone{i}') for i in range(2)]
    for i, p in enumerate(trk):
        _tracker(p, i, t)
    bag = str(tmp_path / 'bag')
    _bag(bag, t)
    t2 = tmp_path / 't2.log'
    t2.write_text(f"[main-1] [INFO] [{T0 + 1.0:.6f}] [fleet_manager]: Fleet command received: 'ARM'\n"
                  f"[main-1] [INFO] [{T0 + 3.0:.6f}] [fleet_manager]: Fleet command received: 'TAKEOFF'\n"
                  f"[main-1] [INFO] [{T0 + 9.0:.6f}] [fleet_manager]: Landed - disarming the fleet.\n")
    import rig_to_run
    man, n, ne = rig_to_run.convert(trk, str(tmp_path / 'run'), bag=bag, t2_log=str(t2))
    d = pd.read_csv(tmp_path / 'run' / 'logs' / 'run.csv')
    assert n == len(d) and abs(d.t.iloc[-1] - 9.98) < 0.05
    assert d.payload_tilt_deg.iloc[10:].between(9.9, 10.1).all()          # ring orientation from the bag
    assert (d.d0_z.iloc[10:] == 0.9).all() and (d.d1_x.iloc[10:] == 1.0).all()   # drone pose from the bag
    assert d.d1_thr.iloc[-1] == pytest.approx(0.46) and (d.d0_armed.iloc[10:] == 1.0).all()
    ev = pd.read_csv(tmp_path / 'run' / 'logs' / 'events.csv')
    assert list(ev.event) == ['ARM', 'TAKEOFF', 'LANDED']
    assert ev.sim_time.tolist() == pytest.approx([1.0, 3.0, 9.0])
    assert man['sources']['payload_orientation'] == 'bag mocap'


def test_tracker_only_flight_has_no_ring_tilt(tmp_path):
    t = T0 + np.arange(0, 4, 0.02)
    trk = [str(tmp_path / f'planner_drone{i}') for i in range(3)]
    for i, p in enumerate(trk):
        _tracker(p, i, t)
    import rig_to_run
    man, n, ne = rig_to_run.convert(trk, str(tmp_path / 'run'))
    d = pd.read_csv(tmp_path / 'run' / 'logs' / 'run.csv')
    assert d.payload_tilt_deg.isna().all() and (d.payload_z == 0.3).all() and (d.d2_x == 2.0).all()
    assert man['sources']['drone_pose'] == 'tracker log'
