"""drop_release_xy_m / drop_release_wait_s (13 Nov 2026, Te-Jen at the rig, RIG-1113-fm9hc: released 0.16 m off centre on
the moving circle, the object bounced off the ring edge): typed > 0, the release at the end of the lowering waits for the
magnet within the gate of the drop point, but never longer than the wait (it always releases). Off (0) as before."""
from types import SimpleNamespace

from tejen_mission.mission_types import MissionPhase as P
from test_drop_on_arrival import _transit
from test_m1_contact_attach import tick


def lowered(gate, wait=3.0):
    p = _transit(on=False)
    p.phase = P.DESCEND_TO_DROP_HEIGHT
    p.rate_profile = lambda *a, **k: SimpleNamespace(complete=lambda e: True)
    p.drop_release_xy_m, p.drop_release_wait_s, p.drop_release_wait_start = gate, wait, None
    return p


def test_off_releases_at_the_end_of_the_lowering():
    p = lowered(0.0)
    p.errors = (0.16, 0.0, 0.0)
    tick(p, 1 / 30)
    assert p.phase == P.DROP_OBJECT


def test_on_waits_while_off_centre_and_releases_once_inside():
    p = lowered(0.06)
    p.errors = (0.16, 0.0, 0.0)
    for _ in range(30):
        tick(p, 1 / 30)
    assert p.phase == P.DESCEND_TO_DROP_HEIGHT        # 1 s off centre: held
    p.errors = (0.04, 0.0, 0.0)
    tick(p, 1 / 30)
    assert p.phase == P.DROP_OBJECT


def test_on_always_releases_after_the_wait():
    p = lowered(0.06, wait=3.0)
    p.errors = (0.16, 0.0, 0.0)
    tick(p, 1 / 30)
    assert p.phase == P.DESCEND_TO_DROP_HEIGHT
    for k in range(120):
        tick(p, 1 / 30)
        if p.phase != P.DESCEND_TO_DROP_HEIGHT:
            break
    assert p.phase == P.DROP_OBJECT and 2.9 <= k / 30 <= 3.2      # released at the 3 s wait, off centre


def test_launch_switches_typed_only_on_both_paths():
    from test_return_after_drop import _sim, _rig_planner
    assert _rig_planner()['c1f2_prepare_settle_speed_mps'] == 0.12
    rig = _rig_planner(prepare_settle_speed='0.20', drop_release_xy='0.06', drop_release_wait='3')
    assert rig['c1f2_prepare_settle_speed_mps'] == 0.20
    assert rig['drop_release_xy_m'] == 0.06 and rig['drop_release_wait_s'] == 3.0
    assert 'drop_release_xy_m' not in _rig_planner()
    assert _sim(prepare_settle_speed='0.2')['c1f2_prepare_settle_speed_mps'] == 0.2
