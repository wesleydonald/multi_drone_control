"""tools/yaw_track.py --plan: the plan's delay behind the desired at nodes 0/1/5, from the planner log's yaw columns
(23 Oct), on a synthetic line_spin whose ring is 0.5 s late and whose plan is 0.3 s late at node 1, 0.1 s at node 5."""
import json
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), 'src', 'mpc_planner'))
import yaw_track  # noqa: E402
from mpc_planner.load_trajectory import LoadTrajectory  # noqa: E402

P = {'load_traj': 'line_spin', 'traj_speed': 0.2, 'traj_distance': 1.0, 'traj_radius': 0.5}
DATUM = 0.3             # rad: the ring's yaw at the path start
T0 = 10.0               # s: the path starts (run.csv time)


def _quat(yaw):
    return np.stack([np.cos(yaw / 2), 0 * yaw, 0 * yaw, np.sin(yaw / 2)], axis=1)


def _run(tmp_path, plan_cols=True):
    traj = LoadTrajectory.from_params(P)
    T = traj._line_spin(0.0)[4]
    traj_t = lambda t: np.clip(t - T0, 0.0, T)
    des = lambda t: DATUM + np.array([traj.yaw_at(v)[0] for v in traj_t(np.atleast_1d(t))])
    z = lambda t: np.where(t > 1.995, 0.6, 0.1)       # lifts on a sample of both clocks
    t = np.arange(0.0, T0 + T + 8.0, 0.01)
    run = tmp_path / 'R9999_spin'
    (run / 'logs').mkdir(parents=True)
    q = _quat(des(t - 0.5))
    pd.DataFrame({'t': t, 'payload_z': z(t), 'payload_qw': q[:, 0], 'payload_qx': q[:, 1], 'payload_qy': q[:, 2],
                  'payload_qz': q[:, 3]}).to_csv(run / 'logs' / 'run.csv', index=False)
    tl = np.arange(0.0, T0 + T + 8.0, 0.1)
    ql = _quat(des(tl - 0.5))
    L = {'sim_time': tl + 500.0, 'load_z': z(tl), 'traj_t': traj_t(tl),
         'qw': ql[:, 0], 'qx': ql[:, 1], 'qy': ql[:, 2], 'qz': ql[:, 3]}
    if plan_cols:
        wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi
        L.update({'yaw_ref0': des(tl), 'plan_yaw1': wrap(des(tl + 0.1 - 0.3)), 'plan_yaw5': wrap(des(tl + 0.5 - 0.1))})
        L['yaw_ref0'][tl < 1.0] = np.nan              # creep ticks: no plan
    sess = run / 'logs' / 'mpc_planner' / '20261023_120000'
    sess.mkdir(parents=True)
    pd.DataFrame(L).to_csv(sess / 'log.csv', index=False)
    (sess / 'params.json').write_text(json.dumps(P))
    return str(run)


def test_the_plan_delay_at_each_node(tmp_path):
    r = yaw_track.track(_run(tmp_path))
    assert abs(r['delay_s'] - 0.5) < 0.015
    pl = r['plan']
    for k, want in (('ref', 0.0), ('node0', 0.5), ('node1', 0.3), ('node5', 0.1)):
        assert abs(pl[k][0] - want) < 0.015, (k, pl[k])
        assert pl[k][1] < 0.5, (k, pl[k])


def test_an_older_log_has_no_plan_and_the_option_says_so(tmp_path, capsys):
    run = _run(tmp_path, plan_cols=False)
    assert yaw_track.track(run)['plan'] is None
    yaw_track.main([run, '--plan'])
    assert 'no plan yaw columns' in capsys.readouterr().out


def test_the_option_prints_each_node(tmp_path, capsys):
    yaw_track.main([_run(tmp_path), '--plan'])
    out = capsys.readouterr().out
    assert 'node1 +0.30' in out and 'node5 +0.10' in out and 'ref +0.00' in out
