"""Q9 floor-start M1: drone 3, welded on plate 3 at the start (start_attached), joins the
creep as the 4th carrier. The resize to n=4 happens before TAKEOFF with the PLATE as its
attach point, and the creep is resized with it. Evidence of the gap: R0732 (drone 3 left
out of the creep, folded in at the hand-over with the body-minus-arm point)."""
import re
from types import MethodType, SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from controller_dissipative.dissipative_node import DissipativeController as D  # noqa: E402
from controller_load_mpc.creep_controller import CreepController  # noqa: E402
from controller_load_mpc.geometry import attach_points  # noqa: E402
from controller_load_mpc.planner_node import LoadPlanner  # noqa: E402

R_RING, Z_ATT, CABLE, ARM = 0.25, 0.025, 0.5, 0.49


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m, *a, **k):
        self.lines.append(m)

    warn = error = info


class _Solver:
    N, dt = 20, 0.1

    def __init__(self, dyn):
        self.dyn, self.last_X = dyn, None

    def set_geometry(self, rho=None, cable_lengths=None):
        if rho is not None:
            self.dyn.rho = rho


class _Clock:
    def now(self):
        return 0.0


def _node(start_taut=False, takeoff_seen=False, reconfig='ocp', start_attached=True):
    rho3 = attach_points(3, R_RING, Z_ATT, [150.0, 270.0, 30.0])
    dyn4 = SimpleNamespace(rho=attach_points(4, R_RING, Z_ATT), m=0.86)
    load = np.array([0.0, 0.0, 0.012, 1.0, 0.0, 0.0, 0.0])
    # carriers resting on their rods at ~7.5 deg; drone 3 on the floor at 90 deg, r 0.745
    drones = [load[0:3] + r + CABLE * np.array([np.cos(np.radians(7.5)) * r[0] / R_RING,
                                                np.cos(np.radians(7.5)) * r[1] / R_RING,
                                                np.sin(np.radians(7.5))]) for r in rho3]
    p3 = np.array([0.0, 0.745, 0.115])
    log = _Log()
    f = SimpleNamespace(
        phase='creep', start_taut=start_taut, takeoff_seen=takeoff_seen,
        _start_attached=start_attached, _reconfig_mode=reconfig,
        n=3, n_net=4, _n_carry0=3, reserved_attach=1, slot2drone=[0, 1, 2],
        rho=list(rho3), cable_len=CABLE, cable_len_i=[CABLE] * 3, _attach_cable_len=ARM,
        attach_radius=R_RING, attach_z=Z_ATT, cable_elev_deg=45.0, dt=0.1, traj=None,
        psi0=0.0, _yaw_datum_latched=False, load_state=load,
        drone_pos=list(drones), drone_vel=[np.zeros(3)] * 3, attach_pos=[p3],
        attach_pending=[True], _approach={}, _ocp_attached={}, _weld_time={},
        detached=[False] * 4, _reconfig_hold_left=0.0,
        refs=SimpleNamespace(_t_nom=[0.86 * 9.81 / 3 / np.sin(np.radians(45.0))] * 3))
    f._solvers = {4: (dyn4, _Solver(dyn4), dyn4.rho)}
    f.get_logger = lambda: log
    f.get_clock = lambda: _Clock()
    f.creep = CreepController(3, f.rho, CABLE, 20, 0.1, 9.81, 45.0, 10.0,
                              lambda i: f.drone_pos[f.slot2drone[i]], lambda i, nodes: None, log)
    for name in ('_do_attach', '_attach_ocp', '_start_weld_joins_creep', '_reserved_index',
                 '_capture_weld_rho'):
        setattr(f, name, MethodType(getattr(D, name), f))
    f.resize_fleet = MethodType(LoadPlanner.resize_fleet, f)
    return f, log


def test_floor_start_weld_resizes_to_four_on_the_plate_before_takeoff():
    f, log = _node()
    f.creep.step(f.load_state, f.drone_pos, [(0.0, 0.0)] * 3, False)   # latched for n=3
    f._do_attach(3)
    assert f.n == 4 and f.slot2drone == [0, 1, 2, 3]
    # the PLATE at 90 deg on the ring, not the body-minus-arm point (0.4 m under the ring)
    assert np.allclose(f.rho[3], [0.0, R_RING, Z_ATT], atol=1e-6)
    assert f.cable_len_i == [CABLE, CABLE, CABLE, ARM]
    assert f.creep.n == 4 and np.allclose(f.creep.rho[3], f.rho[3])
    assert f.creep.cable_lens == [CABLE, CABLE, CABLE, ARM]
    assert f.creep.arc_anchor is None                 # re-latched on the next creep step
    assert f.attach_pending == [False] and f._ocp_attached == {3: True}
    assert not f.refs.blend_active() and f._reconfig_hold_left == 0.0
    assert any('welded at start: plate at 90' in m for m in log.lines)
    # the runner's FOLD_IN edge (runner_node _ROSOUT) keys on this line, as in the air start
    assert any(re.search(r'\] ATTACH drone 3 \(start weld', m) for m in log.lines)
    # next creep tick latches four anchors, drone 3 from its own resting pose
    f.creep.step(f.load_state, f.drone_pos, [(0.0, 0.0)] * 4, False)
    assert len(f.creep.arc_anchor) == 4
    d = f.drone_pos[3] - f.creep.arc_anchor[3][0]
    assert f.creep.arc_theta0[3] == pytest.approx(np.arctan2(d[2], np.hypot(d[0], d[1])))


@pytest.mark.parametrize('kw', [dict(start_taut=True), dict(takeoff_seen=True),
                                dict(reconfig='network'), dict(start_attached=False)])
def test_other_paths_keep_refusing_an_attach_in_the_creep(kw):
    f, log = _node(**kw)
    f._do_attach(3)
    assert f.n == 3 and f.creep.n == 3 and f.attach_pending == [True]
    assert any('attach ignored - not flying yet' in m for m in log.lines)


def test_no_fold_in_once_the_sweep_has_started():
    f, log = _node()
    f.creep.lifted_off = True
    f._do_attach(3)
    assert f.n == 3 and f.creep.n == 3 and f.attach_pending == [True]
