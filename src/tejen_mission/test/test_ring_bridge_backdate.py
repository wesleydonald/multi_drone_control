"""ring_bridge shared_trajectory_backdate_s: off, the carriers' shared commitments start at the refresh time (his
timing); on, they start that long before it, so his backend's start-skew check (transfer_backend_node.cpp: a
commitment may not start after a carrier's latest state) passes when the refresh lands ahead of the same tick's states
(R2854, R2855). The refresh period is unchanged."""
import types

import pytest

from tejen_mission.ring_bridge import RingBridge

TICK = 0.02          # the bridge's 50 Hz timer (his default publish_rate_hz; the launch sets none)


def _stub(back):
    params = {} if back is None else {'shared_trajectory_backdate_s': back}
    built = []
    s = types.SimpleNamespace(
        _param=lambda name, default: params.get(name, default),
        last_shared_commitment_publish_s=None,
        build_shared_obstacle_splines=lambda t: built.append(t) or [],
        build_shared_payload_spline=lambda t: None,
        publish_shared_commitments=lambda: None)
    return s, built


def _start_skew(start, state_rx):
    """His backend's test: start_skew when the commitment starts after the carrier's latest state."""
    return start - state_rx > 1e-6


@pytest.mark.parametrize('back', [None, 0.0])
def test_off_is_his_timing(back):
    s, built = _stub(back)
    RingBridge.refresh_shared_commitments(s, 100.0)
    assert built == [100.0] and s.last_shared_commitment_publish_s == 100.0
    # his race: the refresh at tick T, the carriers' latest state still from T - TICK
    assert _start_skew(built[0], 100.0 - TICK)


def test_on_starts_before_the_last_tick_and_keeps_the_period():
    s, built = _stub(0.04)
    RingBridge.refresh_shared_commitments(s, 100.0)
    assert built == [pytest.approx(99.96)]
    assert s.last_shared_commitment_publish_s == 100.0
    assert not _start_skew(built[0], 100.0 - TICK)
    assert not _start_skew(built[0], 100.0 - 2 * TICK)
    assert _start_skew(built[0], 100.0 - 3 * TICK)     # a tick late by more than one: his check again


# ---- the launch: commit_backdate, typed only, both paths ----------------------------------------------------------

def test_launch_twin_off_by_default_and_typed_on():
    from test_partner_mission_geometry import _params
    ring = 'fake_cooperative_transport_world'
    assert 'shared_trajectory_backdate_s' not in _params('rig_twin_L100_p115')[ring]
    assert _params('rig_twin_L100_p115', commit_backdate='0.04')[ring]['shared_trajectory_backdate_s'] == 0.04


def test_launch_rig_off_by_default_and_typed_on():
    from test_partner_mission_rig import _nodes
    ring = 'fake_cooperative_transport_world'
    assert 'shared_trajectory_backdate_s' not in _nodes()[0][ring]['params']
    assert _nodes(commit_backdate='0.04')[0][ring]['params']['shared_trajectory_backdate_s'] == 0.04
