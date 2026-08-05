"""
Regression tests for the mid-flight ATTACH path through DissipativeNetwork.

WHAT THESE COVER, HONESTLY
--------------------------
The 2026-08-05 SIL run found the planner process DYING on the first plan tick after a
ring attach: dissipative_node._network_plan built its taut-gate list with range(self.n)
(the TETHERED count, 3) while the network it indexes has n_net nodes (4), so
horizon_references raised IndexError, rclpy killed the node, every drone lost its
reference, the newcomer went armed-idle and dropped onto the load, and the payload
capsized into an envelope fault ~0.2 s later.

These tests pin the NETWORK side of that seam: that horizon_references covers every
node after an attach, and that a wrongly-sized gate list now fails BY NAME instead of
as an IndexError inside a list comprehension.

They do NOT catch the original defect. That was in the node's glue -- the caller's
range() -- and verify_dissipative drives DissipativeNetwork directly without ever
instantiating DissipativeNode, so the offline gate is structurally blind to it. Only
the SIL bench (tools/sil_bench.py, which runs the real node) closes that hole.
"""
import math

import numpy as np
import pytest

from controller_load_mpc.geometry import attach_points
from controller_dissipative.dissipative_network import (
    DissipativeNetwork, DissipativeParams)

N_TETHERED = 3
N_NET = 4                      # 3 tethered + 1 reserved attach slot
CABLE_LEN = 0.5
DRONE_MASS = 0.6
LOAD_MASS = 0.4
G = 9.81
LEVEL = np.array([1.0, 0.0, 0.0, 0.0])


def _net():
    """A network in the exact shape three_attach_launch flies: n_net nodes, the
    reserved slot detached (inert) until it welds on."""
    rho = attach_points(N_NET, 0.08, 0.025)
    net = DissipativeNetwork(N_NET, rho, CABLE_LEN, DRONE_MASS, LOAD_MASS, G,
                             DissipativeParams())
    for k in range(N_TETHERED, N_NET):
        net.detach(k)
    load = np.array([0.0, 0.0, 0.6])
    elev = math.radians(45.0)
    seed = []
    for k in range(N_NET):
        az = 2 * math.pi * k / N_TETHERED
        seed.append(load + np.array([CABLE_LEN * math.cos(elev) * math.cos(az),
                                     CABLE_LEN * math.cos(elev) * math.sin(az),
                                     CABLE_LEN * math.sin(elev)]))
    net.seed(seed)
    return net, load


def test_horizon_references_covers_every_node_after_a_ring_attach():
    """THE regression. After the reserved slot welds on, the horizon must produce a
    reference for all n_net nodes -- this is the call that killed the planner."""
    net, load = _net()
    net.attach(3, load + np.array([-0.08, 0.0, 0.55]), handout=True)
    assert net.n_attached() == 4

    p_seq = [load] * 5
    out = net.horizon_references(LEVEL, p_seq, 0.1,
                                 taut_gates=[1.0] * N_NET)
    assert len(out) == N_NET, 'one entry per NETWORK node, not per tethered drone'
    for i, per_node in enumerate(out):
        assert len(per_node) == len(p_seq)
        for p_ref, v_ref, a_ff, a_cable in per_node:
            for name, vec in (('p_ref', p_ref), ('v_ref', v_ref),
                              ('a_ff', a_ff), ('a_cable', a_cable)):
                assert np.all(np.isfinite(vec)), f'node {i} {name} not finite'


def test_a_gate_list_sized_to_the_tethered_count_fails_by_name():
    """The exact mistake that crashed the planner: 3 gates for a 4-node network. It
    must say so, not die as an IndexError deep inside a comprehension."""
    net, load = _net()
    net.attach(3, load + np.array([-0.08, 0.0, 0.55]), handout=True)
    with pytest.raises(ValueError, match=r'3 entries but the network has 4'):
        net.horizon_references(LEVEL, [load] * 3, 0.1,
                               taut_gates=[1.0] * N_TETHERED)


def test_horizon_references_defaults_to_fully_taut_gates():
    """taut_gates=None must still cover every node (the default path)."""
    net, load = _net()
    net.attach(3, load + np.array([-0.08, 0.0, 0.55]), handout=True)
    out = net.horizon_references(LEVEL, [load] * 3, 0.1)
    assert len(out) == N_NET


def test_horizon_references_leaves_the_live_network_state_untouched():
    """It rolls a COPY forward. If it mutated the live state, the attach transient
    would be applied twice per tick -- once by step(), once by the preview."""
    net, load = _net()
    net.attach(3, load + np.array([-0.08, 0.0, 0.55]), handout=True)
    q0, qd0, ho0 = net.q.copy(), net.qd.copy(), net.handout.copy()
    net.horizon_references(LEVEL, [load + np.array([0.1 * k, 0.0, 0.0])
                                   for k in range(6)], 0.1,
                           taut_gates=[1.0] * N_NET)
    assert np.allclose(net.q, q0)
    assert np.allclose(net.qd, qd0)
    assert np.allclose(net.handout, ho0)
