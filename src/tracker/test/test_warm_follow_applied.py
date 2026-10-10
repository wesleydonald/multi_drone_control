"""Combined M1 contact attach, our drone-3 tracker (card 2026-10-10_combined_m1_contact, critic round 2 G and round
3 finding 1): while the mux forwards his stream, the u-state is the mux output (all four channels, the throttle in
model units through the thrust map), there is no takeoff spool, and at the switch to ours the solve restarts from the
measured state with the heading the drone has then. Off (warm_follow_applied false) the tracker is unchanged."""
import math
import types
from collections import deque

import numpy as np
import pytest

pytest.importorskip('interfaces.msg')
from interfaces.msg import ELRSCommand  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from tracker import tracker_node as cm  # noqa: E402
from tracker.tracker_node import Controller  # noqa: E402

OFF = 0.185


def _node(follow=True, yaw_deg=90.0):
    sent = []
    h = math.radians(yaw_deg) / 2.0
    c = types.SimpleNamespace(
        thrust_offset=OFF, thrust_offset_v_slope=0.022, thrust_v_ref=23.5, battery_voltage=23.5,
        throttle_max=0.8, _model_thr_max=0.8 - OFF, takeoff_requested=True, takeoff_spool_s=0.5,
        _takeoff_step=None, drone_id=3, last_cmd_throttle=None, _yaw_hold=False,
        warm_follow_applied=follow, _in_authority=not follow, _applied_cmd=None, _applied_u=np.zeros(4),
        first_solve=False, _heading_datum=0.0,
        current_pose=np.r_[0.0, 0.6, 1.6, math.cos(h), 0.0, 0.0, math.sin(h), np.zeros(6)],
        _vel_hist=deque([(1.0, np.zeros(3), np.zeros(3))]), _acc_lpf=np.ones(3), _thr_w_lpf=np.ones(3),
        _acc_t=1.0, _applied_cable_vec=np.ones(3))
    c.cb = types.SimpleNamespace(cmd_publisher_=types.SimpleNamespace(publish=sent.append))
    c.sent = sent
    log = types.SimpleNamespace(debug=lambda *a, **k: None, info=lambda *a, **k: None, warn=lambda *a, **k: None)
    c.get_logger = lambda: log
    for name in ('_thrust_off', '_output_throttle', '_realised_model_throttle', '_pin_applied',
                 '_applied_cmd_cb', '_mux_state_cb', '_measured_heading'):
        setattr(c, name, types.MethodType(getattr(Controller, name), c))
    c.publish = lambda u: Controller._publish_channels(c, np.asarray(u, float), np.zeros(4))
    return c


def _his(thr=0.5, armed=True, rates=(0.2, -0.3, 0.1)):
    return ELRSCommand(armed=armed, channel_0=rates[0], channel_1=rates[1], channel_2=2.0 * thr - 1.0,
                       channel_3=rates[2])


def test_while_his_stream_flies_the_u_state_is_the_mux_output_in_model_units():
    c = _node()
    c._applied_cmd_cb(_his(thr=0.5))
    c.publish([0.05, 0.04, 0.4, 0.0])                          # our own (unforwarded) command
    assert np.allclose(c._applied_u, [0.2, -0.3, 0.5 - OFF, 0.1])
    assert c.last_cmd_throttle == pytest.approx(0.5 - OFF)
    assert c.sent[-1].channel_2 == pytest.approx(2.0 * (0.4 + OFF) - 1.0, abs=1e-3)   # we still publish ours
    c._applied_cmd_cb(_his(thr=0.6, rates=(0.0, 0.1, -0.2)))   # a newer command between our ticks
    assert np.allclose(c._applied_u, [0.0, 0.1, 0.6 - OFF, -0.2])


def test_nothing_sent_yet_or_disarmed_is_no_thrust():
    c = _node()
    c.publish([0.05, 0.04, 0.4, 0.0])
    assert np.allclose(c._applied_u, 0.0) and c.last_cmd_throttle is None
    c._applied_cmd_cb(_his(thr=0.5, armed=False))
    assert np.allclose(c._applied_u, 0.0) and c.last_cmd_throttle is None


def test_no_takeoff_spool_once_it_has_a_reference():
    c = _node()
    c.publish([0.0, 0.0, 0.4, 0.0])
    assert c._thr_out == pytest.approx(0.4 + OFF)              # full command on its first solve
    assert c._takeoff_step >= int(c.takeoff_spool_s * cm.FREQUENCY_HZ)
    d = _node(follow=False)
    d.publish([0.0, 0.0, 0.4, 0.0])
    assert d._thr_out < 0.4 + OFF - 0.1                        # off: the spool as before


def test_at_the_switch_the_solve_restarts_from_the_measured_state_and_heading():
    c = _node(yaw_deg=90.0)
    c._applied_cmd_cb(_his(thr=0.5))
    c.publish([0.05, 0.04, 0.4, 0.0])
    c._mux_state_cb(String(data='partner'))
    assert not c.first_solve and c._heading_datum == 0.0
    c._mux_state_cb(String(data='ours'))
    assert c._in_authority and c.first_solve
    assert c._heading_datum == pytest.approx(math.pi / 2)      # the heading it has now, not the mat's
    assert np.allclose(c._applied_u, [0.2, -0.3, 0.5 - OFF, 0.1])   # its first x0 carries his last command
    assert len(c._vel_hist) == 0 and c._acc_lpf is None and c._applied_cable_vec is None
    c.first_solve = False
    c._mux_state_cb(String(data='ours'))
    assert not c.first_solve                                   # once
    c._applied_cmd_cb(_his(thr=0.7))
    c.publish([0.05, 0.04, 0.4, 0.0])
    assert np.allclose(c._applied_u, [0.05, 0.04, 0.4, 0.0])   # ours now: our own command books


def test_off_the_tracker_books_its_own_command_as_before():
    c = _node(follow=False)
    c._takeoff_step = 10 ** 6
    c.publish([0.05, 0.04, 0.4, 0.0])
    assert np.allclose(c._applied_u, [0.05, 0.04, 0.4, 0.0]) and c.last_cmd_throttle == pytest.approx(0.4)


def test_the_full_m1_mux_is_ours_from_boot_then_his_after_the_release_then_ours_at_the_attach():
    """The welded start (mux start_ours): our command books while QUAD4 carries, his after /partner/release,
    and the switch back at the attach re-seeds from the measured state."""
    c = _node(yaw_deg=0.0)
    c._mux_state_cb(String(data='ours'))                       # latched at boot
    assert c._in_authority
    c.first_solve = False
    c._applied_cmd_cb(_his(thr=0.9))                           # whatever the mux output shows: not ours to book
    c.publish([0.05, 0.04, 0.4, 0.0])
    assert np.allclose(c._applied_u, [0.05, 0.04, 0.4, 0.0])   # a carrier on its own command
    c._mux_state_cb(String(data='partner'))                    # released to his stream
    assert not c._in_authority and not c.first_solve
    c._applied_cmd_cb(_his(thr=0.55))
    c.publish([0.05, 0.04, 0.4, 0.0])
    assert np.allclose(c._applied_u, [0.2, -0.3, 0.55 - OFF, 0.1])
    c.current_pose[3:7] = [math.cos(math.radians(20.0)), 0.0, 0.0, math.sin(math.radians(20.0))]
    c._mux_state_cb(String(data='ours'))                       # the attach (or a take-back)
    assert c._in_authority and c.first_solve and c._heading_datum == pytest.approx(math.radians(40.0))
