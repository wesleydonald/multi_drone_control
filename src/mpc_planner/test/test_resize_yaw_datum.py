"""resize_fleet keeps the placement yaw datum: the rebuilt ReferenceBuilder starts at
yaw 0, so without the carry-over a ring placed rotated is pulled toward world yaw 0 at
every OCP detach/attach (found 2026-09-28 while writing the control-math page)."""
import types

import numpy as np

from mpc_planner.planner_node import LoadPlanner
from mpc_planner.geometry import attach_points


class _Log:
    def warn(self, *_):
        pass

    info = error = warn


class _Solver:
    N, dt = 20, 0.1

    def set_geometry(self, rho, cable_lengths):
        pass


def _planner(n_old, n_new, psi0, latched):
    rho_new = attach_points(n_new, 0.25, 0.025)
    dyn = types.SimpleNamespace(rho=rho_new, m=0.86)
    p = types.SimpleNamespace(
        _solvers={n_new: (dyn, _Solver(), rho_new)}, n=n_old, cable_len=0.5,
        cable_len_i=[0.5] * n_old, cable_elev_deg=45.0, dt=0.1, traj=None,
        psi0=psi0, _yaw_datum_latched=latched, get_logger=lambda: _Log())
    return p, rho_new


def test_resize_keeps_the_latched_yaw_datum():
    psi0 = np.radians(37.0)
    p, rho = _planner(4, 3, psi0, latched=True)
    assert LoadPlanner.resize_fleet(p, 3, [0, 1, 2], rho=rho)
    assert abs(p.refs.psi0 - psi0) < 1e-12


def test_resize_before_the_latch_leaves_the_builder_at_zero():
    p, rho = _planner(4, 3, 0.0, latched=False)
    assert LoadPlanner.resize_fleet(p, 3, [0, 1, 2], rho=rho)
    assert p.refs.psi0 == 0.0


def test_reference_publish_delay_holds_each_message_until_due():
    """Card 2026-10-08_lag_h1: ref_publish_delay_s queues each reference for that long of node
    time; 0 publishes at once."""
    sent, now = [], [10.0]
    pub = types.SimpleNamespace(publish=lambda m: sent.append(m))
    p = types.SimpleNamespace(N=1, dt=0.1, ref_pub=[pub], _ref_queue=[], _ref_delay_s=0.035,
                              get_clock=lambda: types.SimpleNamespace(
                                  now=lambda: types.SimpleNamespace(nanoseconds=int(now[0] * 1e9))))
    node = [(np.zeros(3),) * 4] * 2
    LoadPlanner._publish_ref(p, 0, node)
    assert not sent and len(p._ref_queue) == 1
    now[0] = 10.03
    LoadPlanner._flush_delayed_refs(p)
    assert not sent
    now[0] = 10.036
    LoadPlanner._flush_delayed_refs(p)
    assert len(sent) == 1 and not p._ref_queue
    p._ref_delay_s = 0.0
    LoadPlanner._publish_ref(p, 0, node)
    assert len(sent) == 2
