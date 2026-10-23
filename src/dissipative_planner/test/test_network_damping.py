"""damp_relative (card 2026-10-23_paper_method_compare): the paper's friction relative to the moving formation (Quan
et al. Eq. 9) in place of the port's damping against still pinned nodes. Off, the network is the one flown before
the switch, bit for bit; on, a node follows a moving target with only the 10 Hz target's own sampling lag."""
import numpy as np

from mpc_planner.geometry import attach_points
from dissipative_planner.dissipative_network import DissipativeNetwork, DissipativeParams

LEVEL = np.array([1.0, 0.0, 0.0, 0.0])
P0 = np.array([0.0, 0.0, 1.0])
V = np.array([0.4, 0.0, 0.0])


def _net(central=False, **kw):
    """The C geometry: 4 quads, ring radius 0.225, 1 m rods at 60 deg, 0.55 kg quads, the 0.86 kg ring."""
    net = DissipativeNetwork(4, attach_points(4, 0.225, 0.02), 1.0, 0.55, 0.86, 9.81,
                             DissipativeParams(elev_deg=60.0, ki_load=1.0, **kw))
    net.seed([net.cone_target(i, LEVEL, P0) + np.array([0.03, -0.02, 0.01]) for i in range(4)])
    if central:
        net.detach(3)
        net.attach(3, P0 + np.array([0.0, 0.0, 1.05]), central=True)
    return net


def _fly(net, ticks=40, vel=None):
    for k in range(ticks):
        pd = P0 + 0.1 * k * V
        net.step(pd - np.array([0.01, 0.0, 0.02]), LEVEL, np.zeros(3), pd, 0.1,
                 load_accel=np.array([0.1, 0.0, 0.0]), p_des_vel=vel)
    return net


# node 0's position and velocity after _fly with the network as it was before the switch (scratchpad golden_net.py
# ran the pre-switch module on this scenario): the off path must reproduce them
GOLDEN = {False: ([2.212490533684315, -0.00011430212969308704, 1.9176258928125143],
                  [0.3608470229596272, 0.00010498493393208372, -0.009279202639061686]),
          True: ([2.134841566754768, -0.28955345265404386, 1.9074384929941115],
                 [0.3635816192147996, 0.009910570388749176, -0.006621005416463888])}


def test_off_reproduces_the_network_before_the_switch():
    for central in (False, True):
        q0, qd0 = GOLDEN[central]
        for vel in (None, V):                            # a target velocity is ignored while the switch is off
            net = _fly(_net(central), vel=vel)
            assert np.allclose(net.q[0], q0, rtol=0, atol=1e-12) and np.allclose(net.qd[0], qd0, rtol=0, atol=1e-12)


def test_on_without_a_target_velocity_is_the_port():
    a, b = _fly(_net()), _fly(_net(damp_relative=True))
    assert np.array_equal(a.q, b.q) and np.array_equal(a.qd, b.qd)


def _steady_lag(rel):
    net = DissipativeNetwork(4, attach_points(4, 0.225, 0.02), 1.0, 0.55, 0.86, 9.81,
                             DissipativeParams(elev_deg=60.0, damp_relative=rel))
    net.seed([net.cone_target(i, LEVEL, P0) for i in range(4)])
    lag = []
    for k in range(200):
        pd = P0 + 0.1 * k * V
        net.step(pd, LEVEL, V, pd, 0.1, p_des_vel=V)
        lag.append(max(float(np.linalg.norm(net.q[i] - net.cone_target(i, LEVEL, pd))) for i in range(4)))
    return max(lag[-50:])


def test_a_moving_target_is_followed_with_only_its_sampling_lag():
    """0.4 m/s: the port's nodes sit ~15 cm behind their cone slots (the line_x formation 0.47 s late offline); damped
    against the target they trail it by the half tick its 10 Hz steps imply (2 cm), no more."""
    port, paper = _steady_lag(False), _steady_lag(True)
    assert port > 0.10
    assert paper < 0.5 * 0.4 * 0.1 + 0.01 and paper < 0.25 * port


def test_the_offline_gate_holds_at_cs_geometry(monkeypatch):
    """verify_dissipative's A-D (hold and lift, outward hold, a 4->3 detach, the inward guard) at 1 m and 60 deg."""
    from dissipative_planner import verify_dissipative as V
    for k, v in V.GEOMETRY_C.items():
        monkeypatch.setattr(V, k, v)          # restored after the test: the module's own run keeps its geometry
    assert V.main_geometry_c()


def test_the_horizon_takes_the_target_velocity_and_restores_the_live_state():
    net = _fly(_net(damp_relative=True), vel=V)
    q, qd, ho = net.q.copy(), net.qd.copy(), net.handout.copy()
    pd = P0 + 4.0 * V
    seq = [pd + 0.1 * j * V for j in range(8)]
    still = net.horizon_references(LEVEL, seq, 0.1)
    moving = net.horizon_references(LEVEL, seq, 0.1, p_des_vel_seq=[V] * 8)
    assert np.array_equal(net.q, q) and np.array_equal(net.qd, qd) and np.array_equal(net.handout, ho)
    for i in range(4):
        assert np.allclose(moving[i][0][0], still[i][0][0])            # node 0 is the live state in both
        assert moving[i][-1][0][0] > still[i][-1][0][0] + 0.01          # the predicted nodes keep up with the target
