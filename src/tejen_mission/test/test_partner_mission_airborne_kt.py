"""The full M1 (our welded start, released in the air): his MPC must learn kT there too.

His MPC re-latches its kT estimator's start height on every disarmed tick and learns only
thrust_ratio_ukf_min_height_above_start (0.20 m) above it. Armed at QUAD4's step-out point (1.847 m in
R1503), his mission rose at most 0.14 m before the pickup descent: the estimator never started, kT stayed at
its nominal 22 (the subtest converged to ~21.1 from the mat) and his MPC, with no z integral, flew ~11 cm
under its reference, so the tip reached the floor. partner_mission.launch.py lets it learn from where it arms
when the start is in the air (airborne_start 'auto': the gate is /partner/release), where his kT feedback flies.
His gate is re-checked every tick: 0.0 started it at his takeoff but paused it ('waiting_for_height') as soon as
QUAD4 went below the arm height, his whole pickup (R1505); the launch passes -10 m, which never gates.
"""
import ast
import math
from pathlib import Path
import time
import types

import numpy as np
import pytest

from test_partner_mission_geometry import _params

MPC_SOURCE = Path(__file__).resolve().parents[2] / 'tejen_mpc' / 'tejen_mpc' / 'main.py'
KEY = 'thrust_ratio_ukf_min_height_above_start'


def _ukf_method():
    """His Controller.run_full_model_thrust_ratio_ukf, executed from the source (no rclpy, no acados)."""
    tree = ast.parse(MPC_SOURCE.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Controller')
    fns = [n for n in cls.body if isinstance(n, ast.FunctionDef)
           and n.name in ('run_full_model_thrust_ratio_ukf', '_empty_thrust_ratio_ukf_result')]
    for fn in fns:
        fn.decorator_list = []
    ns = {'math': math, 'np': np, 'time': time}
    exec(compile(ast.fix_missing_locations(ast.Module(body=fns, type_ignores=[])), str(MPC_SOURCE), 'exec'), ns)
    return ns['run_full_model_thrust_ratio_ukf'], ns['_empty_thrust_ratio_ukf_result']


class _Estimator:
    def __init__(self):
        self.initialized = False

    def initialize(self, kt):
        self.initialized = True
        return {'status': 'initialized', 'initialized': True, 'updated': False, 'update_count': 0}

    def snapshot(self, status):
        return {'status': status, 'initialized': True, 'updated': False}


def _mpc(min_height, arm_z):
    run, empty = _ukf_method()
    log = types.SimpleNamespace(info=lambda *a, **k: None, error=lambda *a, **k: None)
    m = types.SimpleNamespace(
        enable_thrust_ratio_ukf=True, enable_thrust_ratio_feedback=True,
        thrust_ratio_full_model_ukf=_Estimator(), thrust_ratio_ukf_last_result=None,
        current_pose=np.r_[0.0, 1.0, arm_z, 1.0, np.zeros(9)], thrust_ratio_ukf_initial_z=arm_z,
        takeoff_requested=True, thrust_ratio_ukf_min_height_above_start=min_height, est_params=np.array([22.0]),
        thrust_ratio_full_model_start_state=None, thrust_ratio_full_model_start_time_s=None,
        _empty_thrust_ratio_ukf_result=empty, _node_now_s=lambda: 0.0,
        _resync_full_model_kt_interval=lambda now: None, get_logger=lambda: log)
    m.run = types.MethodType(run, m)
    return m


def _fly(m, zs):
    statuses = []
    for z in zs:
        m.current_pose[2] = z
        statuses.append(m.run()['status'])
    return statuses


R1503_ZS = np.r_[np.linspace(1.847, 1.987, 15), np.linspace(1.987, 0.58, 30)]   # armed in the air, then the pickup
LEARN_IN_AIR = -10.0      # the launch's gate when QUAD4 arms in the air


def test_armed_in_the_air_his_default_gate_never_starts_the_kt_estimator():
    m = _mpc(0.20, arm_z=1.847)
    assert set(_fly(m, R1503_ZS)) == {'waiting_for_height'}
    assert not m.thrust_ratio_full_model_ukf.initialized and m.est_params[0] == 22.0


def test_with_no_climb_required_it_starts_where_he_takes_off():
    m = _mpc(LEARN_IN_AIR, arm_z=1.847)
    assert _fly(m, R1503_ZS[:1]) == ['initialized'] and m.thrust_ratio_full_model_ukf.initialized
    g = _mpc(0.20, arm_z=0.10)                            # the subtest: armed on the mat, the climb starts it
    statuses = _fly(g, np.linspace(0.10, 0.70, 30))
    assert statuses[0] == 'waiting_for_height' and 'initialized' in statuses


def test_a_descent_below_the_arm_height_keeps_learning():
    """R1505: with the gate at 0.0 the estimator paused at the first tick below the arm height (1.85 m), so
    through his whole pickup (~0.7 m); at -10 m it keeps running."""
    paused = _fly(_mpc(0.0, arm_z=1.847), R1503_ZS)
    assert paused[0] == 'initialized' and 'waiting_for_height' in paused[15:]
    m = _mpc(LEARN_IN_AIR, arm_z=1.847)
    statuses = _fly(m, R1503_ZS)
    assert statuses[0] == 'initialized' and 'waiting_for_height' not in statuses
    assert all(s == 'resynced_interval' for s in statuses[1:])        # past the gate on every tick


@pytest.mark.parametrize('overrides, learn_in_air', [
    ({'start_gate_topic': '/partner/release'}, True),                        # the full M1 (auto)
    ({}, False),                                                              # the subtest: from the mat
    ({'start_gate_topic': '/partner/release', 'airborne_start': 'false'}, False),
    ({'airborne_start': 'true'}, True),
])
def test_the_launch_lets_him_learn_from_an_airborne_arm(overrides, learn_in_air):
    mpc = _params('rig_twin', **overrides)['tejen_mpc']
    assert (mpc.get(KEY) == LEARN_IN_AIR) is learn_in_air
    assert learn_in_air or KEY not in mpc                  # otherwise his own default (0.20)


def test_legacy_geometry_is_unchanged():
    """The old partner world flies a fixed kT (no feedback): nothing changes there, gate or not."""
    assert KEY not in _params('legacy', start_gate_topic='/partner/release')['tejen_mpc']
