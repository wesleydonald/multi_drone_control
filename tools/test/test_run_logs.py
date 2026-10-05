"""run_logs: the same run reads the same in the pre-rename and the post-rename folder layout."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import run_logs  # noqa: E402


def _touch(root, *parts):
    d = os.path.join(root, *parts)
    os.makedirs(d, exist_ok=True)
    open(os.path.join(d, 'log.csv'), 'w').close()
    return os.path.join(d, 'log.csv')


def test_old_layout(tmp_path):
    r = str(tmp_path)
    p = _touch(r, 'logs', 'controller_quad_load', 'load_planner_20261001_100000')
    t0 = _touch(r, 'logs', 'controller_quad_load', 'planner_drone0_20261001_100001')
    _touch(r, 'logs', 'controller_quad_load', 'planner_drone1_20261001_100000')
    t1 = _touch(r, 'logs', 'controller_quad_load', 'planner_drone1_20261001_100500')
    d = _touch(r, 'logs', 'controller_quad_load', 'dissipative_controller_20261001_100000')
    assert run_logs.node_csvs(r, 'mpc_planner') == [p]
    assert run_logs.node_csvs(r, 'dissipative_planner') == [d]
    assert run_logs.trackers(r) == {0: t0, 1: t1}
    # a rig <flight>_logs/logs/controller_quad_load folder passed directly
    flat = os.path.join(r, 'logs', 'controller_quad_load')
    assert run_logs.trackers(flat) == {0: t0, 1: t1}
    assert run_logs.trackers(run_logs.run_root(p)) == {0: t0, 1: t1}


def test_new_layout(tmp_path):
    r = str(tmp_path)
    p = _touch(r, 'logs', 'mpc_planner', '20261001_100000')
    t0 = _touch(r, 'logs', 'tracker', 'drone0_20261001_100001')
    t3 = _touch(r, 'logs', 'tracker', 'drone3_20261001_100002')
    d = _touch(r, 'logs', 'dissipative_planner', '20261001_100000')
    assert run_logs.node_csvs(r, 'mpc_planner') == [p]
    assert run_logs.node_csvs(r, 'dissipative_planner') == [d]
    assert run_logs.trackers(r) == {0: t0, 3: t3}
    assert run_logs.trackers(os.path.join(r, 'logs')) == {0: t0, 3: t3}
    assert run_logs.trackers(os.path.join(r, 'logs', 'tracker')) == {0: t0, 3: t3}
    assert run_logs.trackers(run_logs.run_root(p)) == {0: t0, 3: t3}
    assert run_logs.trackers(run_logs.run_root(d)) == {0: t0, 3: t3}
    assert run_logs.drone_of(t3) == 3 and run_logs.stamp_of(p) == '20261001_100000'
