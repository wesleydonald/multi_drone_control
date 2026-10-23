"""The fleet manager's position limit (Wesley 22 Oct; pos_limit_m, off until the critic, card
2026-10-22_position_limit): an airborne drone's x or y past +-pos_limit_m on POS_LIMIT_SAMPLES poses in a row lands
the fleet (one LAND a flight, never the emergency stop), with the reason on the status line; a drone on the floor, one
bad pose, or the limit off change nothing."""
from types import SimpleNamespace

from std_msgs.msg import String

import fleet_manager.fleet_manager_node as m
from test_mux_abort_manager import _Rec, _fake, sync  # noqa: F401
from utility_objects.position_limit import POS_LIMIT_SAMPLES


def _mgr(limit=1.75):
    f = _fake()
    f.pos_limit_m, f._limit_count = limit, {}
    f.tilt_fault_land = f.landed_partner_cut = f.fc_disarm_land = False
    f._pose_z, f._rest_z, f._fc_flew, f._fc_landed_for = {}, {i: 0.08 for i in range(4)}, set(), set()
    f._pose_speed, f._fc_armed, f._fault_land_sent = {}, {}, False
    f.status_pub = _Rec()
    C = m.CentralController
    for name in ('_tilt_pose_callback', '_limit_check', '_fault_land', '_command_callback', '_land_fleet'):
        setattr(f, name, getattr(C, name).__get__(f))
    f._arm_fleet = lambda: None
    return f


def _pose(x, y, z):
    return SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=z)),
                           twist=SimpleNamespace(linear=SimpleNamespace(x=0.0, y=0.0, z=0.0)))


def _lands(f):
    return [x.data for x in f.fleet_cmd_pub.msgs].count('LAND')


def _poses(f, d, xyz, k):
    for _ in range(k):
        f._tilt_pose_callback(_pose(*xyz), d)


def test_an_airborne_drone_past_the_limit_lands_the_fleet_once(sync):
    f = _mgr()
    f.flying = True
    _poses(f, 2, (0.4, 1.76, 1.2), POS_LIMIT_SAMPLES - 1)
    assert _lands(f) == 0                                    # not yet: one bad mocap frame is not a breach
    _poses(f, 2, (0.4, 1.76, 1.2), 1)
    assert _lands(f) == 1 and f.abort_pub.msgs == [] and f.service_disarms == []
    assert 'POSITION LIMIT: drone 3' in f.status_pub.msgs[-1].data
    _poses(f, 2, (0.4, 1.8, 1.2), 10)
    _poses(f, 1, (-1.9, 0.0, 1.0), 10)                       # another drone past it: still one LAND
    assert _lands(f) == 1


def test_inside_the_limit_on_the_floor_or_off_nothing_happens(sync):
    f = _mgr()
    f.flying = True
    _poses(f, 0, (1.74, -1.74, 1.5), 10)                     # inside (not beyond 1.75)
    _poses(f, 1, (2.5, 0.0, 0.12), 10)                       # past it, but on the floor (rest 0.08 + 0.10)
    f.flying = False
    _poses(f, 2, (2.5, 0.0, 1.0), 10)                        # past it before TAKEOFF
    assert _lands(f) == 0
    g = _mgr(limit=0.0)
    g.flying = True
    _poses(g, 2, (3.0, 3.0, 1.0), 10)
    assert _lands(g) == 0


def test_a_short_excursion_resets_and_arm_starts_a_new_flight(sync):
    f = _mgr()
    f.flying = True
    for xy in ((1.8, 0.0), (1.8, 0.0), (1.0, 0.0)) * 3:      # never POS_LIMIT_SAMPLES in a row
        f._tilt_pose_callback(_pose(xy[0], xy[1], 1.0), 0)
    assert _lands(f) == 0
    _poses(f, 0, (1.8, 0.0, 1.0), POS_LIMIT_SAMPLES)
    assert _lands(f) == 1
    f._command_callback(String(data='ARM'))
    assert f._limit_count == {} and not f._fault_land_sent


def test_a_drone_the_partner_flies_does_not_trip_it_until_we_fly_it(sync):
    """QUAD4 before we take it over is his (mux 'partner', or 'cut'): his pickup may lie outside the limit; once the
    mux hands it to us ('ours') the limit covers it like any carrier."""
    f = _mgr()
    f.flying = True
    f.mux_state = {3: 'partner'}
    _poses(f, 3, (1.6, 2.0, 1.0), 10)
    f.mux_state[3] = 'cut'
    _poses(f, 3, (1.6, 2.0, 1.0), 10)
    assert _lands(f) == 0
    f.mux_state[3] = 'ours'
    _poses(f, 3, (1.6, 2.0, 1.0), POS_LIMIT_SAMPLES)
    assert _lands(f) == 1
