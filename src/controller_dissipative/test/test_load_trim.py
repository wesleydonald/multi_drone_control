"""Common-mode load trim (docs/design/velocity_loop.md §11).

One z-only integrator on the measured load error, its output added IDENTICALLY to
every attached node's a_ff. Each test names the property it protects: the whole point
of the design is that the trim can shift the fleet's thrust but can NEVER differ
between drones (a differential integral bias is a moment on the load — §10's measured
divergence), and that ki_load=0 leaves every verified path byte-unchanged.
"""
import numpy as np
import pytest

from controller_dissipative.dissipative_network import (DissipativeNetwork,
                                                        DissipativeParams)

G = 9.81


def _net(**kw):
    n = kw.pop('n', 3)
    params = DissipativeParams(**kw)
    rho = [np.array([0.25 * np.cos(a), 0.25 * np.sin(a), 0.0])
           for a in np.radians([0, 120, 240])[:n]]
    net = DissipativeNetwork(n, rho, cable_len=0.5, drone_mass=0.6,
                             load_mass=0.4, g=G, params=params)
    p_des = np.array([0.0, 0.0, 0.6])
    seeds = [p_des + np.array([0.35 * np.cos(a), 0.35 * np.sin(a), 0.35])
             for a in np.radians([0, 120, 240])[:n]]
    net.seed(seeds)
    return net, p_des


LOAD_Q = np.array([1.0, 0.0, 0.0, 0.0])


def _settle(net, p_des, load_pos, ticks=30, **step_kw):
    for _ in range(ticks):
        net.step(load_pos, LOAD_Q, np.zeros(3), p_des, 0.1, **step_kw)


def test_ki_zero_is_byte_inert():
    """Default off: with ki_load=0 the references are bit-identical to a network that
    has never heard of the trim — this is what keeps every verified config unchanged."""
    net_a, p_des = _net()                      # defaults: ki_load=0
    net_b, _ = _net(ki_load=0.0)
    sagging = p_des - np.array([0.0, 0.0, 0.07])
    _settle(net_a, p_des, sagging)
    _settle(net_b, p_des, sagging)
    for i in range(3):
        ra, rb = net_a.reference(i, LOAD_Q, p_des), net_b.reference(i, LOAD_Q, p_des)
        for xa, xb in zip(ra, rb):
            assert np.array_equal(xa, xb)
    assert np.array_equal(net_a._a_trim, np.zeros(3))


def test_sag_below_target_produces_upward_trim():
    """The steady symptom the trim exists for: load hanging below target charges a
    positive-z trim, and it appears in a_ff."""
    net, p_des = _net(ki_load=1.0)
    base = [net.reference(i, LOAD_Q, p_des)[2].copy() for i in range(3)]
    _settle(net, p_des, p_des - np.array([0.0, 0.0, 0.07]))
    assert net._a_trim[2] > 0.0
    for i in range(3):
        a_ff = net.reference(i, LOAD_Q, p_des)[2]
        assert a_ff[2] > base[i][2]


def test_trim_is_identical_across_all_nodes_and_modes():
    """The core moment argument: every attached node's a_ff shifts by the SAME vector,
    so relative geometry (what sets differential tension) is untouched. Checked in the
    equal-share mode AND the balanced-tensions mode, since they build a_ff separately."""
    for balanced in (False, True):
        net0, p_des = _net(balanced_tensions=balanced)
        net1, _ = _net(balanced_tensions=balanced, ki_load=1.0)
        sagging = p_des - np.array([0.0, 0.0, 0.07])
        _settle(net0, p_des, sagging)
        _settle(net1, p_des, sagging)
        shifts = []
        for i in range(3):
            a0 = net0.reference(i, LOAD_Q, p_des)[2]
            a1 = net1.reference(i, LOAD_Q, p_des)[2]
            shifts.append(a1 - a0)
        for s in shifts[1:]:
            assert np.allclose(s, shifts[0], atol=1e-12), \
                f'trim differs between nodes (balanced={balanced})'
        assert shifts[0][2] > 0.0


def test_z_only_by_default_xy_error_never_charges():
    """An XY common-mode integral would charge on sweep tracking lag and distort phase
    (§11.2); by default only the vertical error integrates."""
    net, p_des = _net(ki_load=1.0)
    offset_xy = p_des - np.array([0.10, -0.08, 0.0])   # pure horizontal error
    _settle(net, p_des, offset_xy)
    assert np.array_equal(net._a_trim[:2], np.zeros(2))
    net_xyz, _ = _net(ki_load=1.0, i_load_xyz=True)
    _settle(net_xyz, p_des, offset_xy)
    assert float(np.linalg.norm(net_xyz._a_trim[:2])) > 0.0


def test_contribution_clamp_is_gain_independent():
    """The bound is on ki_load * integral, not the raw integral, so authority stays
    a_i_load_max whatever the gain."""
    for ki in (0.5, 5.0):
        net, p_des = _net(ki_load=ki, a_i_load_max=1.5)
        _settle(net, p_des, p_des - np.array([0.0, 0.0, 0.5]), ticks=400)
        assert float(np.linalg.norm(net._a_trim)) <= 1.5 + 1e-9


def test_freeze_holds_and_reset_zeroes():
    """integrate_trim=False freezes without zeroing (LAND); reset_load_trim zeroes
    (network-phase entry)."""
    net, p_des = _net(ki_load=1.0)
    _settle(net, p_des, p_des - np.array([0.0, 0.0, 0.07]))
    held = net._a_trim.copy()
    assert held[2] > 0.0
    _settle(net, p_des, p_des - np.array([0.0, 0.0, 0.5]), integrate_trim=False)
    assert np.array_equal(net._a_trim, held)
    net.reset_load_trim()
    assert np.array_equal(net._a_trim, np.zeros(3))


def test_horizon_rollout_is_trim_side_effect_free():
    """horizon_references predicts future ticks; prediction must not charge the trim,
    and the trim it bakes into the horizon must be the live one on every node."""
    net, p_des = _net(ki_load=1.0)
    # long settle: the horizon-equals-live property below holds at equilibrium
    _settle(net, p_des, p_des - np.array([0.0, 0.0, 0.07]), ticks=600)
    before = net.i_load.copy()
    trim = net._a_trim.copy()
    horizon = net.horizon_references(LOAD_Q, [p_des] * 5, 0.1)
    assert np.array_equal(net.i_load, before)
    # constant target => every horizon node equals the live reference, trim included
    for i in range(3):
        live = net.reference(i, LOAD_Q, p_des)[2]
        for _, _, a_ff, _ in horizon[i]:
            assert np.allclose(a_ff, live, atol=1e-6)
    assert trim[2] > 0.0


def test_detach_keeps_the_trim():
    """The droop the trim holds does not vanish when a drone leaves; a detach must not
    step every survivor's reference by dumping the integral."""
    net, p_des = _net(ki_load=1.0)
    _settle(net, p_des, p_des - np.array([0.0, 0.0, 0.07]))
    held = net._a_trim.copy()
    net.detach(1)
    assert np.array_equal(net._a_trim, held)
    a_ff = net.reference(0, LOAD_Q, p_des)[2]
    net.reset_load_trim()
    a_ff_no = net.reference(0, LOAD_Q, p_des)[2]
    assert a_ff[2] > a_ff_no[2]


def test_fly_away_reference_gets_no_trim():
    """A departed drone is no longer coupled to the load; its free-flight reference
    must not carry the carrying fleet's thrust trim."""
    net, p_des = _net(ki_load=1.0)
    _settle(net, p_des, p_des - np.array([0.0, 0.0, 0.07]))
    assert net._a_trim[2] > 0.0
    net.detach(1)
    _, _, a_ff, a_cable = net.fly_away_reference(1)
    assert np.array_equal(a_ff, np.array([0.0, 0.0, G]))
    assert np.array_equal(a_cable, np.zeros(3))


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-q']))


def test_share_weighted_trim_scales_with_the_solved_tension():
    """On the uneven four-point rim (4/12/8 o'clock plus the newcomer at 6) the moment
    balance loads the 12 o'clock drone about twice as much as each of the others; with
    trim_share_weighted its trim is larger in that ratio, and with the flag off every
    node gets the same trim. (Three drones at 4/12/8 alone are symmetric: equal shares.)"""
    from controller_load_mpc.geometry import attach_points
    import math
    az = [330.0, 90.0, 210.0, 270.0]
    rho = attach_points(4, 0.25, 0.025, azimuths_deg=az)
    out, shares = {}, None
    for flag, ki in ((None, 0.0), (False, 1.0), (True, 1.0)):
        net = DissipativeNetwork(4, rho, cable_len=0.5, drone_mass=0.6, load_mass=0.6, g=9.81,
                                 params=DissipativeParams(balanced_tensions=True, ki_load=ki,
                                                          trim_share_weighted=bool(flag)))
        load = np.array([0.0, 0.0, 0.6]); el = math.radians(45.0)
        net.seed([load + rho[k] + 0.5 * np.array([math.cos(el) * math.cos(math.radians(a)),
                                                  math.cos(el) * math.sin(math.radians(a)),
                                                  math.sin(el)]) for k, a in enumerate(az)])
        for _ in range(20):                      # load 10 cm low -> the trim winds up
            net.step(load - [0, 0, 0.1], LOAD_Q, np.zeros(3), load, 0.1)
        out[flag] = [float(net.reference(k, LOAD_Q, load)[2][2]) for k in range(4)]
        shares = net._solve_tensions(np.eye(3), load)
    assert shares[1] > 1.6 * shares[0] and abs(shares[0] - shares[2]) < 1e-6
    # the trim is what the integrator ADDS over the ki_load=0 network
    off = [out[False][k] - out[None][k] for k in range(4)]
    on = [out[True][k] - out[None][k] for k in range(4)]
    assert max(off) - min(off) < 1e-9 and off[0] > 0.05
    assert on[1] > 1.5 * on[0] and on[1] > 1.5 * on[3] and abs(on[0] - on[2]) < 1e-6
