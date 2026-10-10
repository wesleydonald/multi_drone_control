"""ring_bridge must keep the parent's marker builder: its label marker once aliased the
published attachment pose and raised /fake_attachment_point/pose by 0.12 m."""

from tejen_mission.fake_cooperative_transport_world import FakeCooperativeTransportWorld
from tejen_mission.ring_bridge import RingBridge


def test_ring_bridge_uses_the_fixed_marker_builder():
    assert RingBridge.make_markers is FakeCooperativeTransportWorld.make_markers
    assert RingBridge.timer_callback is FakeCooperativeTransportWorld.timer_callback
