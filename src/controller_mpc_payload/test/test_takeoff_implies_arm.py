"""The approach MPC on a manager-commanded drone (three_attach drone 3): the manager's
accepted TAKEOFF on the drone's own command topic arms and starts it (one command path,
2026-09-29). Checked on a stand-in object: the node itself needs an acados build."""
import types

import pytest

main = pytest.importorskip('controller_mpc_payload.main')


def _node(pose=True, armed=False):
    n = types.SimpleNamespace(current_pose=object() if pose else None, armed=armed,
                              takeoff_requested=False, logs=[])
    n.get_logger = lambda: types.SimpleNamespace(info=n.logs.append, warn=n.logs.append)
    return n


def _cls():
    return next(c for c in vars(main).values()
                if isinstance(c, type) and hasattr(c, '_takeoff_implies_arm'))


def test_takeoff_arms_and_starts():
    n = _node()
    _cls()._takeoff_implies_arm(n, types.SimpleNamespace(data='TAKEOFF'))
    assert n.armed and n.takeoff_requested


@pytest.mark.parametrize('text', ['ARM', 'DISARM', 'LAND', 'ESTOP'])
def test_other_commands_do_nothing(text):
    n = _node()
    _cls()._takeoff_implies_arm(n, types.SimpleNamespace(data=text))
    assert not n.armed and not n.takeoff_requested


def test_no_pose_no_arm():
    n = _node(pose=False)
    _cls()._takeoff_implies_arm(n, types.SimpleNamespace(data='TAKEOFF'))
    assert not n.armed and not n.takeoff_requested
