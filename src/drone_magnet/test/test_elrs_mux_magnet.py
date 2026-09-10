"""The mux is the only path to the newcomer's radio, so the magnet aux channel has to
ride on the command it forwards (hardware); with no magnet command seen it must not
touch the message (sim)."""
import pytest

pytest.importorskip('interfaces.msg')
from interfaces.msg import ELRSCommand
from drone_magnet.elrs_mux import merge_magnet_channel


def test_no_magnet_command_leaves_the_message_alone():
    m = ELRSCommand()
    m.channel_10 = 0.25
    assert merge_magnet_channel(m, 10, None).channel_10 == 0.25


def test_magnet_value_is_written_and_clamped():
    m = ELRSCommand()
    assert merge_magnet_channel(m, 10, 1.0).channel_10 == 1.0
    assert merge_magnet_channel(m, 10, -7.0).channel_10 == -1.0


def test_unknown_channel_is_ignored():
    m = ELRSCommand()
    merge_magnet_channel(m, 99, 1.0)      # no channel_99 field: no exception
