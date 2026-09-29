"""r0b_mocap_report on a synthetic rest bag: 120 Hz with jittered stamps, two w glitches."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


def _write_bag(path, n=1200, glitches=(300, 900)):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from rclpy.serialization import serialize_message
    from interfaces.msg import MotionCaptureState
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topic = '/drone_0/motion_capture_state'
    w.create_topic(rosbag2_py.TopicMetadata(name=topic, type='interfaces/msg/MotionCaptureState',
                                            serialization_format='cdr'))
    rng = np.random.default_rng(0)
    t = 100.0
    for k in range(n):
        t += 1 / 120 + rng.uniform(-0.002, 0.002)
        m = MotionCaptureState()
        m.header.stamp.sec, m.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
        m.pose.position.z = 0.1 + rng.normal(0, 2e-4)
        m.pose.orientation.w = 1.0
        m.twist.linear.x = rng.normal(0, 0.005)
        m.twist.angular.y = 2.0 if k in glitches else rng.normal(0, 0.02)
        w.write(topic, serialize_message(m), int(t * 1e9))
    del w


def test_report_counts_rest_noise_and_glitches(tmp_path):
    bag = str(tmp_path / 'rest')
    _write_bag(bag)
    import r0b_mocap_report as r
    a = r.read_bag(bag)['/drone_0/motion_capture_state']
    s = r.summarise(a, window=5)
    assert s['n'] == 1200 and abs(s['rate_hz'] - 120) < 1
    assert 0.5 < s['dt_ms_std'] < 1.5 and s['dt_ms_min'] > 6.0
    assert s['w_max'] == pytest.approx(2.0)
    span = a[-1, 0] - a[0, 0]
    assert s['w_glitch_per100s'] == pytest.approx(2 * 100 / span)
    assert s['v_glitch_per100s'] == 0
    assert s['v_lsq_p99'] < 0.1                 # the LSQ slope of a still body stays small


def test_lsq_velocity_recovers_a_constant_speed():
    import r0b_mocap_report as r
    t = np.cumsum(np.full(50, 1 / 120))
    pos = np.column_stack([0.3 * t, np.zeros(50), np.zeros(50)])
    v = r.lsq_velocity(t, pos, 5)
    np.testing.assert_allclose(v[4:, 0], 0.3, atol=1e-9)
    assert np.isnan(v[:4]).all()
