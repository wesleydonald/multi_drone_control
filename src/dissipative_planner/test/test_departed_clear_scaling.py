"""A freed drone lands DEPARTED_CLEAR_R from the ring centre, further out only by how far the formation reaches past
the 1 m rig ring's (Wesley 22 Oct: 1.2 m stays for the rig ring; card 2026-10-23_scalability): exactly 1.2 m for the
rig ring at 0.55 and 1 m and the legacy ring, 1.475 m for S's R 0.5 m ring on 1 m cables at 60 deg; typed values,
so measured rods never move it; and a drop on S's ring steps every drone out to it."""
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import (DEPARTED_CLEAR_R, DissipativeController as D,  # noqa: E402
                                                  _departed_clear_r)


@pytest.mark.parametrize('radius,cable,elev', [(0.225, 0.55, 60.0), (0.225, 1.0, 60.0), (0.25, 0.5, 45.0),
                                               (0.225, 0.55, 45.0)])
def test_the_rig_and_legacy_rings_keep_1p2(radius, cable, elev):
    node = SimpleNamespace(attach_radius=radius, cable_len=cable, cable_elev_deg=elev, cable_len_i=[1.08] * 4)
    assert _departed_clear_r(node) == DEPARTED_CLEAR_R == 1.2


def test_the_r50_ring_moves_it_out_by_its_extra_reach():
    node = SimpleNamespace(attach_radius=0.5, cable_len=1.0, cable_elev_deg=60.0)
    assert _departed_clear_r(node) == pytest.approx(1.475, abs=1e-12)


def test_a_stub_without_the_geometry_keeps_1p2():
    assert _departed_clear_r(SimpleNamespace()) == DEPARTED_CLEAR_R


class _Pub:
    def publish(self, m):
        pass


def test_a_drop_on_the_r50_ring_steps_every_drone_out_to_it():
    az = np.radians(np.arange(8) * 45.0)
    f = SimpleNamespace(
        attach_radius=0.5, cable_len=1.0, cable_elev_deg=60.0, detached=[False] * 8, _dropped=None,
        _drop_t=None, _drop_centre=None, load_state=np.r_[0.2, -0.1, 1.0, 1, 0, 0, 0, np.zeros(6)],
        drone_pos=[np.array([0.2 + np.cos(a), -0.1 + np.sin(a), 1.5]) for a in az], drone_vel=[np.zeros(3)] * 8,
        _departed_hold={}, _departed_step={}, _departed_next={}, _departed_land=False,
        detach_pub=[_Pub() for _ in range(8)], _dropped_pub=_Pub(), reserved_attach=0)
    f.get_logger = lambda: SimpleNamespace(error=lambda *a, **k: None, info=lambda *a, **k: None)
    f.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0))
    f._cancel_unload_quietly = lambda: None
    f._release_all_magnets = lambda: None
    D._drop_and_land(f, 'test')
    for d in range(8):
        step = f._departed_step[d]
        assert np.linalg.norm(step[:2] - [0.2, -0.1]) == pytest.approx(1.475, abs=1e-9)
