"""The affine thrust map (rig identification 2026-09-30): model throttle above the offset,
the offset (+ a per-volt term) added at the output, capped, eased in with the spool."""
import types

from controller_quad_load.controller_mpc import Controller


def _c(off, slope=0.022, v=23.5, cap=0.8):
    return types.SimpleNamespace(thrust_offset=off, thrust_offset_v_slope=slope, thrust_v_ref=23.5,
                                 battery_voltage=v, throttle_max=cap)


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
    assert out(_c(0.185, v=None), 0.3) == 0.3 + 0.185            # no telemetry: nominal


def test_cap_and_spool():
    assert out(_c(0.185), 0.7) == 0.8
    assert abs(out(_c(0.185), 0.0, spool=0.5) - 0.0925) < 1e-9
