"""The magnet aux value lands in the CRSF slot the manager and the mux agree on, and
survives the idle-packet reset: channel_6 (AUX4, the default since 2026-09-16) is
packet[7], channel_10 is packet[11], never the arm switch."""
import numpy as np
import pytest
from drone_communication.elrs_interface import apply_magnet, magnet_packet_index


def test_default_channel_6_is_packet_slot_7():
    assert magnet_packet_index(6) == 7


def test_channel_10_is_packet_slot_11():
    assert magnet_packet_index(10) == 11


def test_low_channels_map_one_to_one_and_skip_the_arm_slot():
    assert [magnet_packet_index(c) for c in range(0, 6)] == [0, 1, 2, 3, 5, 6]


def test_out_of_range_channel_is_refused():
    with pytest.raises(ValueError):
        magnet_packet_index(11)


def test_on_and_off_values_hit_the_ends_of_the_range():
    idle, rng = 993, 820
    p = np.full(16, idle, dtype=np.uint16)
    apply_magnet(p, 6, 1.0, idle, rng)
    assert p[7] == idle + rng
    apply_magnet(p, 6, -1.0, idle, rng)
    assert p[7] == idle - rng
    assert p[4] == idle and p[2] == idle            # arm switch and throttle untouched


def test_latch_logs_changes_only():
    """The magnet manager re-sends the newcomer's latch at 2 Hz; only a change is logged."""
    from types import SimpleNamespace
    from std_msgs.msg import String
    from drone_communication.elrs_interface import ELRSInterface
    lines = []
    log = SimpleNamespace(info=lines.append, warn=lines.append)
    f = SimpleNamespace(magnet_on_value=1.0, magnet_off_value=-1.0, magnet_value=-1.0,
                        magnet_channel=6, get_logger=lambda: log)
    for word in ('OFF', 'OFF', 'ON', 'ON', 'OFF'):
        ELRSInterface.magnet_callback(f, String(data=word))
    assert f.magnet_value == -1.0 and len(lines) == 2
