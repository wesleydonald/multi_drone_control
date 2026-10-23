"""The freed and taken-back drones' clearances on 1 m cables (Wesley 22 Oct): the path-clear and take-back zone
distances grow with a cable's reach past 0.55 m (legacy and 0.55 m geometry fly them as before), the landing clearance
only past the rig ring's reach at 1 m (S's bigger ring); and
with the position limit on (pos_limit_m) a freed or dropped quad's landing point stays POS_LIMIT_LAND_MARGIN_M inside
it, while with it off nothing moves."""
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner import dissipative_node as dn  # noqa: E402
from dissipative_planner.dissipative_node import DissipativeController as D  # noqa: E402
from test_drop_and_land import _node as _drop_node  # noqa: E402
from test_soft_detach import _departed_fake, _Log  # noqa: E402


def _geo(cable, radius=0.225, elev=60.0):
    return SimpleNamespace(cable_len=cable, attach_radius=radius, cable_elev_deg=elev)


def test_on_0p55m_cables_the_clearances_are_the_flown_constants():
    for g in (_geo(0.55), SimpleNamespace()):                 # the rig ring at 60 deg; a stub with no geometry
        assert (dn._path_clear_m(g), dn._zone_rise_far_m(g), dn._departed_clear_r(g)) == (1.0, 0.9, 1.2)


def test_the_legacy_geometry_keeps_the_clearances_as_flown():
    g = _geo(0.5, radius=0.25, elev=45.0)                     # sim_legacy: reach 0.604, past the 0.55 m ring's 0.5
    assert (dn._path_clear_m(g), dn._zone_rise_far_m(g), dn._departed_clear_r(g)) == (1.0, 0.9, 1.2)


def test_on_1m_cables_the_path_and_zone_grow_with_the_reach_and_the_landing_clearance_does_not():
    g = _geo(1.0)                                             # reach 0.225 + 1.0 cos 60 = 0.725
    assert dn._path_clear_m(g) == pytest.approx(1.225) and dn._zone_rise_far_m(g) == pytest.approx(1.125)
    assert dn._departed_clear_r(g) == 1.2
    s = _geo(1.0, radius=0.5)                                 # S's ring R 0.5: reach 1.0
    assert dn._departed_clear_r(s) == pytest.approx(1.475)
    f = SimpleNamespace(traj=SimpleNamespace(kind='orbit', radius=0.5), hover_xy=(0.0, 0.0), cable_len=1.0,
                        attach_radius=0.225, cable_elev_deg=60.0)
    p = D._path_clear_point(f, np.array([0.0, 1.0, 1.5]), np.array([0.0, 1.0]))
    assert np.allclose(p, [0.0, 0.5 + 0.5 + 1.225, 1.5])        # the orbit's centre (0, 0.5) + r 0.5 + 1.225


def test_the_landing_margin_leaves_every_point_alone_when_the_limit_is_off():
    p = np.array([2.0, -2.5, 1.0])
    assert dn._land_inside(SimpleNamespace(), p) is p and dn._land_inside(SimpleNamespace(_pos_limit_m=0.0), p) is p
    q = dn._land_inside(SimpleNamespace(_pos_limit_m=1.75), p)
    assert np.allclose(q, [1.65, -1.65, 1.0]) and np.allclose(p, [2.0, -2.5, 1.0])


def _land_step_out(limit, hold=(1.5, 0.0, 1.5), ring=(0.9, 0.0)):
    """LAND with a freed quad at hold from a ring at a lobe tip (x 0.9): it steps out to the landing clearance,
    then lands. Returns (the published references, the node)."""
    published = []
    f = SimpleNamespace(dyn=SimpleNamespace(g=9.81), _land_to_ground=True, _departed_land=False,
                        _departed_landing=set(), _departed_td={}, _departed_clear={},
                        _departed_land_vel=0.0, land_vel=0.15, N=20,
                        load_state=np.r_[ring[0], ring[1], 1.0, 1.0, 0.0, 0.0, 0.0, np.zeros(6)],
                        _departed_hold={2: np.array(hold, float)}, _departed_step={}, _departed_next={},
                        drone_pos=[None] * 3, cable_len=1.0, attach_radius=0.225, cable_elev_deg=60.0,
                        _pos_limit_m=limit)
    f.get_logger = lambda: _Log()
    f._publish_ref = lambda d, nodes: published.append(nodes[0][0].copy())
    _departed_fake(f)
    for _ in range(120):
        D._publish_departed_refs(f)
    return published, f


def _first_down(refs, z0):
    """The reference where the descent starts (the first below the hold height)."""
    return next(q for q in refs if q[2] < z0 - 1e-6)


def test_a_land_step_out_near_the_limit_lands_inside_it_and_clear_of_the_ring():
    free, _ = _land_step_out(0.0)
    assert max(q[0] for q in free) == pytest.approx(0.9 + 1.2, abs=0.035)     # out to the clearance radius (-3 cm)
    held, _ = _land_step_out(1.75)
    assert max(max(abs(q[0]), abs(q[1])) for q in held) <= 1.65 + 1e-9        # the landing margin, 10 cm inside
    down = _first_down(held, 1.5)
    assert np.hypot(down[0] - 0.9, down[1]) >= 1.2 - 0.03                      # not down inside the clearance
    assert down[0] == pytest.approx(1.65, abs=0.02) and held[-1][2] < 1.0      # on the circle at the margin, down


def test_a_release_at_a_lobe_tip_lands_on_the_clearance_circle_nearest_its_bearing():
    """Critic M1/m1 (22 Oct): released at (1.53, 0.36) off a ring at x 0.9 (bearing 30 deg) with the limit at 1.75,
    it used to land 0.96 m from the ring (x and y clamped apart, sideways at 63 deg) over its own tether. Now it lands
    on the 1.2 m circle where that meets the 1.65 m margin, at the bearing nearest its own (51.3 deg); with the limit
    off, straight out at 30 deg."""
    free, _ = _land_step_out(0.0, hold=(1.53, 0.36, 1.5))
    d0 = _first_down(free, 1.5)
    assert np.degrees(np.arctan2(d0[1], d0[0] - 0.9)) == pytest.approx(29.7, abs=0.5)
    refs, f = _land_step_out(1.75, hold=(1.53, 0.36, 1.5))
    down = _first_down(refs, 1.5)
    assert np.hypot(down[0] - 0.9, down[1]) >= 1.2 - 0.03
    assert np.degrees(np.arctan2(down[1], down[0] - 0.9)) == pytest.approx(51.3, abs=1.0)
    assert max(max(abs(q[0]), abs(q[1])) for q in refs) <= 1.65 + 1e-9 and refs[-1][2] < 1.0
    assert f._departed_clear[2] and not getattr(f, '_held_unclear', {})


def test_no_landing_point_clear_of_the_ring_inside_the_limit_holds_the_quad_at_height():
    """A limit too tight for the clearance circle (0.9 m: the margin's corners are 1.13 m from a ring at the origin,
    inside its 1.2 m circle): the quad stays where it is, at height, and the status line says so."""
    refs, f = _land_step_out(0.9, hold=(0.6, 0.0, 1.5), ring=(0.0, 0.0))
    assert all(np.allclose(q, [0.6, 0.0, 1.5]) for q in refs)
    assert not f._departed_clear.get(2, False) and 'HELD' in f._held_unclear[2]
    f.phase, f.load_state, f._reconfig_hold_left, f.n = 'planner', np.r_[0.0, 0.0, 1.0, 1.0, np.zeros(9)], 0.0, 2
    status = []
    f.status_pub = SimpleNamespace(publish=lambda m: status.append(m.data))
    D._publish_status(f)
    assert 'drone 3 (/drone_2) HELD' in status[-1]


def test_a_drop_near_the_limit_lands_every_quad_inside_it():
    """The ring dropped at a lobe tip (x 1.0): its quads step out 1.2 m from where it was, inside the limit."""
    for limit, inside in ((0.0, False), (1.75, True)):
        f = _drop_node()
        f.load_state[0] += 1.0
        for p in f.drone_pos:
            p[0] += 1.0
        f._pos_limit_m = limit
        f._drop_and_land('test')
        steps = [f._departed_step[d] for d in range(4) if d in f._departed_step]
        assert steps and (max(max(abs(s[0]), abs(s[1])) for s in steps) <= 1.65 + 1e-9) == inside
        assert all(np.hypot(*(s[:2] - f._drop_centre)) >= 1.2 - 0.03 for s in steps)   # every one clear of the ring
