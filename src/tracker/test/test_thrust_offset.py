"""The affine thrust map (rig identification 2026-09-30): model throttle above the offset,
the offset (+ a per-volt term) added at the output, capped, eased in with the spool."""
import types

from tracker.tracker_node import Controller


def _c(off, slope=0.022, v=23.5, cap=0.8):
    log = types.SimpleNamespace(warn=lambda *a, **k: None)
    c = types.SimpleNamespace(thrust_offset=off, thrust_offset_v_slope=slope, thrust_v_ref=23.5,
                              thrust_v_full=25.2, _last_good_v=None, drone_id=0, get_logger=lambda: log,
                              battery_voltage=v, throttle_max=cap)
    c._thrust_off = types.MethodType(Controller._thrust_off, c)
    return c


def out(c, thr, spool=1.0):
    return Controller._output_throttle(c, thr, spool)


def test_zero_offset_is_the_old_map():
    assert out(_c(0.0), 0.47) == 0.47


def test_hover_and_carry_match_the_fit():
    kt = 9.81 / (0.507 * 0.55)                       # 35.2, the gain above the offset
    assert abs(out(_c(0.185), 9.81 / kt) - 0.464) < 0.002          # 0.55 kg hover
    assert abs(out(_c(0.185), 9.81 * 0.765 / 0.55 / kt) - 0.573) < 0.002   # + a quarter ring


def test_weaker_pack_needs_more_throttle():
    assert abs(out(_c(0.185, v=22.5), 0.3) - (0.3 + 0.185 + 0.022)) < 1e-9
    full = 0.185 + 0.022 * (23.5 - 25.2)
    assert abs(out(_c(0.185, v=None), 0.3) - (0.3 + full)) < 1e-9   # no telemetry: a full pack (errs low)
    assert abs(out(_c(0.185, v=0.0), 0.3) - (0.3 + full)) < 1e-9    # the radio's 0.0 before any frame


def test_a_lost_reading_keeps_the_last_good_voltage():
    c = _c(0.185, v=22.5)
    out(c, 0.3)
    c.battery_voltage = 0.0
    assert abs(out(c, 0.3) - (0.3 + 0.185 + 0.022)) < 1e-9


def test_cap_and_spool():
    assert out(_c(0.185), 0.7) == 0.8
    assert abs(out(_c(0.185), 0.0, spool=0.5) - 0.0925) < 1e-9


class _Ocp:
    def __init__(self):
        self.ub = None

    def constraints_set(self, k, field, val):
        self.ub = float(val[0])


def _cap_node(v):
    c = _c(0.185, v=v)
    c.ocp, c.N, c._model_cap_t, c.velocity_loop = _Ocp(), 5, None, None
    c._model_thr_max = 0.8 - 0.185
    return c


def test_voltage_change_moves_the_model_cap():
    c = _cap_node(v=23.5)
    Controller._refresh_model_cap(c, now=0.0)
    assert abs(c._model_thr_max - 0.615) < 1e-9 and c.ocp.ub is None   # unchanged, not re-set
    c.battery_voltage = 22.5
    Controller._refresh_model_cap(c, now=0.5)
    assert abs(c._model_thr_max - 0.615) < 1e-9                         # at most once a second
    Controller._refresh_model_cap(c, now=1.1)
    assert abs(c._model_thr_max - (0.8 - 0.207)) < 1e-9
    assert abs(c.ocp.ub - c._model_thr_max) < 1e-9


def test_applied_is_output_minus_offset():
    c = _cap_node(v=22.5)
    o = out(c, 0.3)
    assert abs(Controller._realised_model_throttle(c, o) - 0.3) < 1e-9
    # spooled: the offset is not realised in full, so neither is the model throttle
    o = out(c, 0.5 * 0.4, spool=0.5)                                # _publish_channels spools thr first
    assert abs(Controller._realised_model_throttle(c, o) - (0.5 * (0.4 + 0.207) - 0.207)) < 1e-9
    assert Controller._realised_model_throttle(c, out(c, 0.0, spool=0.5)) == 0.0


def test_capped_output_realises_the_model_cap():
    c = _cap_node(v=23.5)
    assert out(c, 0.7) == 0.8
    assert abs(Controller._realised_model_throttle(c, out(c, 0.7)) - 0.615) < 1e-9


def test_proportional_map_is_untouched():
    c = _c(0.0)
    assert Controller._realised_model_throttle(c, 0.47) == 0.47


def test_w1_log_values_match_the_header_and_tolerate_gaps():
    from tracker.tracker_node import TRACKER_W1_COLS
    c = types.SimpleNamespace(_thr_out=0.5, _thr_off=0.2)          # everything else missing
    vals = Controller._w1_log_values(c)
    assert len(vals) == len(TRACKER_W1_COLS)
    assert vals[:2] == [0.5, 0.2] and all(v != v for v in vals[2:])


def _spool_node(v=23.5):
    import numpy as np
    from tracker import tracker_node as cm
    sent = []
    c = _cap_node(v)
    c.takeoff_requested, c.takeoff_spool_s, c._takeoff_step = True, 0.5, None
    c.drone_id, c.last_cmd_throttle, c._yaw_hold = 0, None, False
    c.cb = types.SimpleNamespace(cmd_publisher_=types.SimpleNamespace(publish=sent.append))
    c.get_logger = lambda: types.SimpleNamespace(debug=lambda *a, **k: None)
    c._output_throttle = types.MethodType(Controller._output_throttle, c)
    c._realised_model_throttle = types.MethodType(Controller._realised_model_throttle, c)
    c.publish = lambda thr: Controller._publish_channels(c, np.array([0.0, 0.0, thr, 0.0]), np.zeros(4))
    return c, cm


def test_publish_channels_books_the_realised_throttle_through_the_spool():
    c, cm = _spool_node()
    spool = int(c.takeoff_spool_s * cm.FREQUENCY_HZ)
    applied = []
    for _ in range(spool + 5):
        c.publish(0.3)
        assert abs(c._applied_u[2] - max(0.0, c._thr_out - 0.185)) < 1e-12
        assert c.last_cmd_throttle == c._applied_u[2]
        applied.append(c._applied_u[2])
    # the offset is eased in with the spool, so the realised throttle starts well below u
    assert applied[0] < 0.1 and all(b >= a - 1e-12 for a, b in zip(applied, applied[1:]))
    assert abs(applied[-1] - 0.3) < 1e-12


def test_recovering_pack_cannot_book_above_the_model_cap():
    c, cm = _spool_node(v=23.5)
    c._takeoff_step = 10 ** 6                                       # spool done
    c.publish(0.7)
    assert c._thr_out == 0.8 and abs(c._applied_u[2] - 0.615) < 1e-12
    c.battery_voltage = 24.5                                       # offset falls, cap not yet refreshed
    c.publish(0.7)
    assert c._thr_out == 0.8 and abs(c._applied_u[2] - c._model_thr_max) < 1e-12


def test_arming_state_exists_before_the_arming_service():
    """The callback manager's arming service is live while the tracker waits for its first pose: an ARM/DISARM then
    read takeoff_requested before it existed and killed the node (RIG-1014-lt_9)."""
    import inspect
    src = inspect.getsource(Controller.__init__)
    assert src.index('self.takeoff_requested = False') < src.index('CallbackManagerMulti(')
