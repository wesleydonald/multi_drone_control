"""multi_drone_control 4 Nov (Te-Jen's review of RIG-1104-k1m_3): his OCP's 0.60 throttle bound, raised at run time on
stages 1..N-1 with the attitude-rate bounds kept; outside [0.60, 0.95] it is refused."""
from types import SimpleNamespace

import numpy as np
import pytest

from tejen_mpc.throttle_bound import THROTTLE_STATE_MAX, set_throttle_state_max


class FakeSolver:
    def __init__(self, n=20):
        self.acados_ocp = SimpleNamespace(solver_options=SimpleNamespace(N_horizon=n))
        self.calls = []

    def constraints_set(self, stage, field, value):
        self.calls.append((stage, field, np.asarray(value)))


def test_every_stage_but_the_first_gets_the_new_bound_and_the_rate_bounds_stay():
    s = FakeSolver()
    assert set_throttle_state_max(s, 0.8) == 19
    assert [c[0] for c in s.calls] == list(range(1, 20)) and {c[1] for c in s.calls} == {'ubx'}
    assert all(np.allclose(c[2], [0.8, 1.0, 1.0, 1.0]) for c in s.calls)


@pytest.mark.parametrize('bad', [0.5, 0.96, 1.2])
def test_outside_the_range_is_refused(bad):
    with pytest.raises(ValueError):
        set_throttle_state_max(FakeSolver(), bad)
    assert THROTTLE_STATE_MAX == 0.6


def test_battery_offset_is_the_rig_voltage_term_capped_and_off_by_default():
    from tejen_mpc.throttle_bound import battery_throttle_offset as off
    assert off(0.0, 23.5, 21.0) == 0.0 and off(0.022, 23.5, None) == 0.0 and off(0.022, 23.5, 24.2) == 0.0
    assert np.isclose(off(0.022, 23.5, 21.5), 0.044)
    assert off(0.022, 23.5, 15.0) == 0.10


def test_battery_offset_reaches_the_channel_but_never_a_zero_throttle_command():
    from tejen_mpc.throttle_bound import battery_compensated_channel as ch
    assert ch(0.2, 0.044) == round((0.6 + 0.044) * 2 - 1, 3)        # 0.60 -> 0.644
    assert ch(-1.0, 0.05) == -1.0 and ch(0.2, 0.0) == 0.2 and ch(0.95, 0.10) == 1.0


def test_the_solvers_own_state_bounds_are_kept():
    """Te-Jen's review 11 Nov: only the throttle state's bound changes; the other state bounds stay as built."""
    s = FakeSolver(n=4)
    s.acados_ocp.constraints = SimpleNamespace(ubx=np.array([0.6, 3.0, 2.0, 1.5]), idxbx=np.array([15, 13, 14, 16]))
    set_throttle_state_max(s, 0.8)
    assert all(np.allclose(c[2], [0.8, 3.0, 2.0, 1.5]) for c in s.calls)
    s = FakeSolver(n=4)
    s.acados_ocp.constraints = SimpleNamespace(ubx=np.array([2.0, 0.6]), idxbx=np.array([13, 15]))
    set_throttle_state_max(s, 0.9)
    assert all(np.allclose(c[2], [2.0, 0.9]) for c in s.calls)
