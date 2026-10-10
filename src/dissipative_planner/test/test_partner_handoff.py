"""Partner hand-off (M1 with Tejen's mission): when his planner hands QUAD4 back at ATTACH_READY, our tracker
flies the last descent, so the approach must start there. A partner run has no approach hold, so the magnet ON
of the hand-off never starts it (R1470: no reference, ref stale, fleet abort 4 s after the hand-off)."""
from types import MethodType, SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import DissipativeController as D  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402


class _Log:
    def info(self, *a, **k):
        pass

    warn = warning = error = info


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m.data)


def _partner_node():
    f = SimpleNamespace(
        _partner_handed=False, _attach_approach=False, attach_pending=[True], reserved_attach=1, _n_carry0=3,
        attach_pos=[np.array([0.0, 0.3, 1.2])], _approach={}, _newcomer_out={}, _departed_hold={3: object()},
        _approach_hold_on=False, _newcomer_lifecycle=False, _land_to_ground=False, _dropped=None,
        _attach_body_to_tip=0.49, _attach_seek_m=0.0, _attach_approach_direct=True, _tip_log=False,
        _tip_feedback=False, _tip_dz=0.0, _magnet_cmd_pub=_Pub(), phase='planner')
    f.get_logger = lambda: _Log()
    for name in ('_partner_handoff_cb', '_start_approach', '_magnet_cmd_cb'):
        setattr(f, name, MethodType(getattr(D, name), f))
    return f


def test_the_hand_off_starts_the_approach_for_the_returned_drone():
    f = _partner_node()
    f._partner_handoff_cb(Bool(data=True))
    assert f._attach_approach and f._partner_handed
    assert 0 in f._approach                     # its reference comes from the approach from now on
    assert 3 not in f._departed_hold            # no longer the released drone's fixed hold


def test_a_second_hand_off_or_none_pending_changes_nothing():
    f = _partner_node()
    f._partner_handoff_cb(Bool(data=True))
    prof = f._approach[0]
    f._partner_handoff_cb(Bool(data=True))
    assert f._approach[0] is prof
    g = _partner_node()
    g.attach_pending = [False]
    g._partner_handoff_cb(Bool(data=True))
    assert not g._partner_handed and g._approach == {}
