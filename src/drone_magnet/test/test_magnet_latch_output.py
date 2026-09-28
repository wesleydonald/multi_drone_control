"""The rig's one magnet path for the newcomer (ladder P10): the manager's ON/OFF decisions go
out as the String radio latch /drone_<d>/magnet of that drone's elrs_interface. Off by
default (magnet_latch_topic ''), so the sim's ELRSCommand output is unchanged."""
from types import SimpleNamespace

import pytest

pytest.importorskip('interfaces.msg')
from std_msgs.msg import String  # noqa: E402
from drone_magnet.magnet_attachment_manager import MagnetAttachmentManager as M, latch_word  # noqa: E402


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class _Log:
    def info(self, *a, **k):
        pass

    warn = error = info


def _manager(latch, elrs, attached_at_start=False):
    f = SimpleNamespace(
        enable_elrs_magnet_output=elrs,
        magnet_latch_topic='/drone_3/magnet' if latch else '',
        magnet_latch_pub=_Pub() if latch else None,
        elrs_magnet_pub=_Pub(), attach_pub=_Pub(), detach_pub=_Pub(),
        elrs_magnet_repeat_rate_hz=2.0, elrs_magnet_channel=6,
        elrs_magnet_on_value=1.0, elrs_magnet_off_value=-1.0,
        _last_elrs_magnet_state=None, _last_elrs_publish_time=0.0,
        magnet_on=attached_at_start, _ever_magnet_on=attached_at_start,
        attached=attached_at_start, attach_command_sent=False, detach_command_sent=False,
        attach_condition_start=None, last_magnet_command_time=None,
        detach_when_magnet_off=True, command_backend='none')
    f.get_logger = lambda: _Log()
    for name in ('_set_elrs_channel', 'publish_elrs_magnet_command', 'command_attach_once',
                 'command_detach_once', 'magnet_command_callback'):
        setattr(f, name, getattr(M, name).__get__(f))
    return f


def _words(f):
    return [m.data for m in f.magnet_latch_pub.msgs]


def test_latch_word():
    assert latch_word(True) == 'ON' and latch_word(False) == 'OFF'


def test_rig_capture_weld_release_is_on_the_latch_only():
    f = _manager(latch=True, elrs=False)
    f.magnet_command_callback(String(data='ON'))          # ATTACH: magnet on for the capture
    assert _words(f) == ['ON']
    f.command_attach_once('d=0.02 m')                     # weld declared: stays ON
    assert _words(f)[-1] == 'ON' and f.attached
    f.magnet_command_callback(String(data='OFF'))         # detach of the newcomer: release
    assert _words(f)[-1] == 'OFF' and not f.attached
    assert not f.elrs_magnet_pub.msgs                     # the merge path is dead on the rig


def test_rig_welded_start_keepalive_is_on():
    f = _manager(latch=True, elrs=False, attached_at_start=True)
    f.publish_elrs_magnet_command(f.magnet_on)            # the 30 Hz timer's keepalive
    f.publish_elrs_magnet_command(f.magnet_on)            # within the 0.5 s repeat period
    assert _words(f) == ['ON']


def test_keepalive_repeats_after_the_period():
    f = _manager(latch=True, elrs=False)
    f.publish_elrs_magnet_command(False)
    f._last_elrs_publish_time -= 0.6
    f.publish_elrs_magnet_command(False)
    assert _words(f) == ['OFF', 'OFF']


def test_sim_default_publishes_the_elrs_command_only():
    f = _manager(latch=False, elrs=True)
    f.magnet_command_callback(String(data='ON'))
    assert len(f.elrs_magnet_pub.msgs) == 1
    assert f.elrs_magnet_pub.msgs[0].channel_6 == 1.0 and f.elrs_magnet_pub.msgs[0].channel_10 == 1.0


def test_no_output_configured_publishes_nothing():
    f = _manager(latch=False, elrs=False)
    f.magnet_command_callback(String(data='ON'))
    assert not f.elrs_magnet_pub.msgs and f._last_elrs_magnet_state is None
