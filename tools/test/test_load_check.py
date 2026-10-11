"""load_check: the F1 gap statistics, windows, log lines, and a synthetic rig bag scored end to end."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import load_check as lc  # noqa: E402

T0 = 1791600000.0     # an epoch time, as the rig bag and the T2 console stamp


def _stream(hz, seconds, t0=T0, holes=()):
    """Arrival times at hz for seconds, with each (start, length) hole removed."""
    t = t0 + np.arange(0.0, seconds, 1.0 / hz)
    for s, d in holes:
        t = t[(t < t0 + s) | (t >= t0 + s + d)]
    return t


def test_a_steady_stream_passes_and_one_hole_fails():
    s = lc.topic_stats(_stream(50, 60), lc.FAST_BAR_S)
    assert s['verdict'] == 'PASS' and s['n'] == 3000
    assert s['max_gap_s'] == pytest.approx(0.02, abs=1e-6)
    assert s['rate_hz'] == pytest.approx(50.0, rel=1e-3)
    holed = lc.topic_stats(_stream(50, 60, holes=[(30.0, 0.18)]), lc.FAST_BAR_S)
    assert holed['verdict'] == 'FAIL' and holed['over_bar'] == 1
    assert holed['max_gap_s'] == pytest.approx(0.20, abs=0.021)
    assert holed['max_gap_at'] - T0 == pytest.approx(29.98, abs=0.021)
    assert holed['p99_gap_s'] == pytest.approx(0.02, abs=1e-6)     # one gap in 3000 is not the 99th percentile


def test_the_bar_is_strict_and_the_reference_has_its_own():
    # 0.15 s exactly is not under the bar; 0.29 s passes the reference's 0.3 s
    assert lc.topic_stats([T0, T0 + 0.15, T0 + 0.30], lc.FAST_BAR_S)['verdict'] == 'FAIL'
    assert lc.topic_stats(_stream(30, 20, holes=[(10.0, 0.25)]), lc.REF_BAR_S)['verdict'] == 'PASS'
    assert lc.topic_stats(_stream(30, 20, holes=[(10.0, 0.30)]), lc.REF_BAR_S)['verdict'] == 'FAIL'


def test_a_window_counts_its_edges_the_whole_recording_does_not():
    t = _stream(30, 60)                      # the stream stops at +60 s
    assert lc.topic_stats(t, lc.FAST_BAR_S)['verdict'] == 'PASS'
    late = lc.topic_stats(t, lc.FAST_BAR_S, window=(T0 + 10.0, T0 + 61.0))
    assert late['verdict'] == 'FAIL' and late['max_gap_s'] == pytest.approx(1.0, abs=0.04)
    inside = lc.topic_stats(t, lc.FAST_BAR_S, window=(T0 + 10.0, T0 + 50.0))
    assert inside['verdict'] == 'PASS' and inside['n'] == pytest.approx(1200, abs=1)


def test_no_messages_fail():
    assert lc.topic_stats([], lc.FAST_BAR_S)['verdict'] == 'FAIL'
    assert lc.topic_stats([], lc.FAST_BAR_S, window=(T0, T0 + 5))['verdict'] == 'FAIL'
    assert lc.topic_stats([], None)['verdict'] == ''          # a reported topic has no verdict


def test_bag_metadata_times_read_in_either_form():
    import datetime
    from types import SimpleNamespace
    assert lc._seconds(SimpleNamespace(nanoseconds=1_500_000_000)) == 1.5      # rclpy Time / Duration (Humble)
    assert lc._seconds(datetime.timedelta(seconds=2)) == 2.0
    assert lc._seconds(None) is None


def test_the_f1_topics_are_the_ones_the_graph_publishes():
    topics = [t for t, _, _ in lc.f1_bars()]
    assert topics == ['/drone_3/ELRSCommand_tejen', '/drone_3/ELRSCommand', '/drone_3/ELRSCommand_diss',
                      '/drone_0/ELRSCommand', '/drone_1/ELRSCommand', '/drone_2/ELRSCommand',
                      '/join_planner/reference']
    assert dict((t, b) for t, b, _ in lc.f1_bars())['/join_planner/reference'] == lc.REF_BAR_S
    assert [t for t, _, _ in lc.f1_bars(0)] == topics[:3] + topics[-1:]


LOG = f"""\
[fleet_manager-5] [INFO] [{T0 + 1:.9f}] [fleet_manager]: Fleet command received: 'ARM'
[main-31] [INFO] [{T0 + 20:.9f}] [tejen.tejen_mpc]: Takeoff requested
[tracker-2] [ERROR] [{T0 + 30:.9f}] [tracker_1]: [Drone 2] Pose timeout (0.31s) - disarming.
[main-31] [ERROR] [{T0 + 40:.9f}] [tejen.tejen_mpc]: External-reference fault latched after TAKEOFF: stale. Holding
[elrs_mux-12] [INFO] [{T0 + 50:.9f}] [elrs_mux_3]: [elrs_mux] -> DISSIPATIVE tracker
"""


def test_log_lines_are_counted_and_windowed():
    got = {label: (bar, n) for label, bar, n, _ in lc.scan_log(LOG)}
    assert got["his MPC 'External-reference fault latched'"] == (0, 1)
    assert got["our trackers' 'Pose timeout'"] == (0, 1)
    windowed = {label: n for label, _, n, _ in lc.scan_log(LOG, window=(T0 + 35, T0 + 60))}
    assert windowed["our trackers' 'Pose timeout'"] == 0
    assert windowed["his MPC 'External-reference fault latched'"] == 1


def test_the_flight_window_is_his_takeoff_to_the_switch_or_the_take_back():
    a, b, note = lc.flight_window(LOG)
    assert (a, b) == (pytest.approx(T0 + 20), pytest.approx(T0 + 50)) and 'switch' in note
    take_back = LOG.replace('-> DISSIPATIVE tracker', 'TAKE-BACK: our tracker takes the drone (no attach)')
    assert lc.flight_window(take_back)[1] == pytest.approx(T0 + 50)
    no_end = LOG.replace('-> DISSIPATIVE tracker', 'something else')
    assert lc.flight_window(no_end)[:2] == (pytest.approx(T0 + 20), None)
    assert lc.flight_window('nothing')[0] is None


def _write_bag(path, hole_topic=None, hole=(100.0, 0.2), seconds=180.0):
    """A rig-like bag: the F1 topics at their rates (his MPC 30 Hz, our trackers 50 Hz, his reference 30 Hz)."""
    rosbag2_py = pytest.importorskip('rosbag2_py')
    pytest.importorskip('interfaces.msg')
    from rclpy.serialization import serialize_message
    from interfaces.msg import ELRSCommand
    from trajectory_msgs.msg import MultiDOFJointTrajectory
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    rates = {'/drone_3/ELRSCommand_tejen': 30, '/drone_3/ELRSCommand': 30, '/drone_3/ELRSCommand_diss': 50,
             '/drone_0/ELRSCommand': 50, '/drone_1/ELRSCommand': 50, '/drone_2/ELRSCommand': 50,
             '/join_planner/reference': 30}
    msgs = []
    for topic, hz in rates.items():
        typ = lc.REF_TYPE if topic == lc.REFERENCE else lc.ELRS_TYPE
        w.create_topic(rosbag2_py.TopicMetadata(name=topic, type=typ, serialization_format='cdr'))
        holes = [hole] if topic == hole_topic else []
        for t in _stream(hz, seconds, holes=holes):
            msgs.append((t, topic))
    for t, topic in sorted(msgs):
        m = MultiDOFJointTrajectory() if topic == lc.REFERENCE else ELRSCommand(armed=topic != '/drone_0/ELRSCommand')
        w.write(topic, serialize_message(m), int(round(t * 1e9)))
    del w


def test_a_synthetic_rig_bag_passes_end_to_end(tmp_path, capsys):
    bag = str(tmp_path / 'load')
    _write_bag(bag)
    log = tmp_path / 'load.log'
    log.write_text(LOG.replace('External-reference', 'Ext-ref').replace('Pose timeout', 'Pose ok'))
    assert lc.main([bag, '--log', str(log)]) == 0
    out = capsys.readouterr().out
    assert 'F1 load bars: PASS' in out
    assert 'disarmed' in out.split('/drone_0/ELRSCommand')[1].splitlines()[0]   # the last command's armed flag
    assert lc.main([bag]) == 2                                                  # no log: incomplete


def test_a_synthetic_rig_bag_with_a_hole_fails_and_the_window_can_exclude_it(tmp_path, capsys):
    bag = str(tmp_path / 'load')
    _write_bag(bag, hole_topic='/drone_3/ELRSCommand_tejen', hole=(100.0, 0.2))
    log = tmp_path / 'clean.log'
    log.write_text('nothing\n')
    assert lc.main([bag, '--log', str(log)]) == 1
    assert 'FAIL (1 failing: /drone_3/ELRSCommand_tejen)' in capsys.readouterr().out
    assert lc.main([bag, '--log', str(log), '--start', '110', '--end', '170']) == 0
    # the load test's window: his TAKEOFF (+20 s) from the log to a typed epoch end before the hole (+90 s)
    log.write_text(LOG.replace('External-reference', 'Ext-ref').replace('Pose timeout', 'Pose ok'))
    assert lc.main([bag, '--log', str(log), '--flight-window', '--end', f'{T0 + 90:.3f}']) == 0
    out = capsys.readouterr().out
    assert 'window: +20.0 s -> +90.0 s (70.0 s; his TAKEOFF to the typed end)' in out
    assert lc.main([bag, '--log', str(log), '--flight-window', '--end', f'{T0 + 110:.3f}']) == 1
    capsys.readouterr()
    # a window clear of the hole with the fault and pose-timeout lines inside it fails on the lines alone
    log.write_text(LOG)
    assert lc.main([bag, '--log', str(log), '--start', '25', '--end', '60']) == 1
    out = capsys.readouterr().out
    assert "1 failing: his MPC 'External-reference fault latched'" not in out
    assert "FAIL (2 failing: his MPC 'External-reference fault latched', our trackers' 'Pose timeout')" in out
