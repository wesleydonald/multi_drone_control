"""The ring's inertia as a planner parameter (load_ixx / load_izz, card 2026-10-23_scalability): untyped it is the
M2A ring's per kg at load_mass, as before; typed it is the bigger ring's own; one without the other is refused; and a
different inertia is a different compiled solver (the cache signature), so S's ring never reuses the rig ring's."""
import types

import numpy as np
import pytest

from mpc_planner.geometry import attach_points
from mpc_planner.load_cable_dynamics import LoadCableDynamics
from mpc_planner.params import PlannerConfig, load_inertia
from mpc_planner.planner_solver import _ocp_signature


class _Node:
    def __init__(self, **typed):
        self.typed = typed

    def declare_parameter(self, name, default):
        return types.SimpleNamespace(value=self.typed.get(name, default))


def test_untyped_is_the_m2a_ring_per_kg():
    assert PlannerConfig(_Node(load_mass=0.9)).load_inertia == load_inertia(0.9)
    assert PlannerConfig(_Node(load_mass=0.9, load_ixx=0.0, load_izz=0.0)).load_inertia == load_inertia(0.9)


def test_typed_is_the_rings_own():
    cfg = PlannerConfig(_Node(load_mass=0.9, load_ixx=0.113025, load_izz=0.22581))
    assert cfg.load_inertia == [0.113025, 0.113025, 0.22581]


@pytest.mark.parametrize('typed', [dict(load_ixx=0.11), dict(load_izz=0.22), dict(load_ixx=-0.1, load_izz=0.2)])
def test_one_without_the_other_is_refused(typed):
    with pytest.raises(ValueError, match='load_ixx'):
        PlannerConfig(_Node(**typed))


def test_a_different_inertia_is_a_different_solver():
    rho = attach_points(8, 0.5, 0.02)
    sig = [_ocp_signature(LoadCableDynamics(8, 0.9, J, [1.0] * 8, rho, 0.55), 20, 2.0)
           for J in (load_inertia(0.9), [0.113025, 0.113025, 0.22581])]
    assert sig[0] != sig[1]
    assert np.allclose(load_inertia(0.86), [2.733e-02, 2.733e-02, 5.452e-02])
