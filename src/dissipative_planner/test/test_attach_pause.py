"""attach_pause (Wesley 4 Nov, the orbit re-attach): once his TAKEOFF follows a release, the path clock's rate slews to
0 over attach_pause_ramp_s and holds while QUAD4 is pending; on the weld (or a take-back, LAND, drop) it slews back to
1. Off, or in hover, nothing changes; his bridge reads the pause as an open-ended hold."""
from types import SimpleNamespace

from dissipative_planner.dissipative_node import DissipativeController, PLANNER_HZ


def _node(**kw):
    f = object.__new__(DissipativeController)
    lines = []
    log = SimpleNamespace(warn=lines.append, info=lines.append)
    f.get_logger = lambda: log
    f.lines = lines
    base = dict(_attach_pause=True, _attach_pause_ramp_s=3.0, _partner_contact=True, load_traj='orbit', traj_t=5.0,
                descending=False, _land_to_ground=False, _dropped=None, _taken_back=False, attach_pending=[True],
                _his_phase='TAKEOFF', _traj_rate=1.0, _attach_paused=False, _pause_hold_s=0.0)
    base.update(kw)
    for k, v in base.items():
        setattr(f, k, v)
    return f


def _run(f, seconds):
    for _ in range(round(seconds * PLANNER_HZ)):
        DissipativeController._attach_pause_tick(f)


def test_his_takeoff_after_the_release_slows_the_path_to_rest_and_the_weld_resumes_it():
    f = _node()
    _run(f, 1.5)
    assert 0.45 < f._traj_rate < 0.55 and f._pause_hold_s == 60.0
    _run(f, 2.0)
    assert f._traj_rate == 0.0 and any('slows to rest' in ln for ln in f.lines)
    f.attach_pending = [False]                      # folded in
    _run(f, 3.1)
    assert f._traj_rate == 1.0 and f._pause_hold_s == 0.0 and any('speeds back up' in ln for ln in f.lines)


def test_no_pause_before_his_takeoff_in_hover_off_or_after_a_take_back():
    for kw in (dict(_his_phase='WAIT_FOR_TAKEOFF'), dict(load_traj='hover'), dict(_attach_pause=False),
               dict(_taken_back=True), dict(_his_phase='TAKEN_BACK'), dict(_land_to_ground=True),
               dict(attach_pending=[False])):
        f = _node(**kw)
        _run(f, 2.0)
        assert f._traj_rate == 1.0, kw


def test_reattach_mode_circles_on_through_the_pickup_and_drop_and_pauses_for_the_reattach():
    f = _node(_attach_pause_at='reattach', _reattach_leg=False)
    for phase in ('TAKEOFF', 'APPROACH_ABOVE_PICKUP', 'DESCEND_TO_PICKUP', 'LIFT_OBJECT', 'TRANSIT_TO_DROP_POINT',
                  'DROP_OBJECT', 'CLEAR_DROP_ZONE'):
        f._his_phase = phase
        _run(f, 1.0)
        assert f._traj_rate == 1.0, phase
    f._his_phase = 'TRANSIT_TO_REATTACH'
    _run(f, 3.1)
    assert f._traj_rate == 0.0 and any('slows to rest' in ln for ln in f.lines)
    f._his_phase = 'ATTACH_READY'                   # later phases of the leg keep it paused
    _run(f, 1.0)
    assert f._traj_rate == 0.0
    f.attach_pending = [False]                      # folded in: back up, and the leg forgotten for a next release
    _run(f, 3.1)
    assert f._traj_rate == 1.0 and f._reattach_leg is False


def test_reattach_mode_without_a_pickup_pauses_at_the_approach():
    f = _node(_attach_pause_at='reattach', _reattach_leg=False, _his_phase='APPROACH_ABOVE_TARGET')
    _run(f, 3.1)
    assert f._traj_rate == 0.0


def test_land_during_the_pause_speeds_the_clock_back_up():
    f = _node()
    _run(f, 4.0)
    assert f._traj_rate == 0.0
    f._land_to_ground = True
    _run(f, 3.1)
    assert f._traj_rate == 1.0


def test_pickup_reattach_mode_pauses_for_the_pickup_resumes_for_the_drop_and_pauses_for_the_reattach():
    f = _node(_attach_pause_at='pickup_reattach', _reattach_leg=False, _his_phase='TAKEOFF')
    _run(f, 1.0)
    assert f._traj_rate == 1.0
    for phase in ('APPROACH_ABOVE_PICKUP', 'DESCEND_TO_PICKUP', 'MAGNET_ATTACH_WAIT'):
        f._his_phase = phase
        _run(f, 1.1)
    assert f._traj_rate == 0.0                        # at rest for the pickup
    for phase in ('LIFT_OBJECT', 'TRANSIT_TO_DROP_POINT', 'DROP_OBJECT', 'CLEAR_DROP_ZONE'):
        f._his_phase = phase
        _run(f, 1.0)
    assert f._traj_rate == 1.0                        # circling again for the drop
    f._his_phase = 'TRANSIT_TO_REATTACH'
    _run(f, 3.1)
    assert f._traj_rate == 0.0                        # at rest for the re-attach



def test_through_landing_his_land_keeps_the_ring_at_rest_until_he_is_down():
    """13 Nov (Te-Jen's Codex): his LAND mid-mission must not restart the ring while QUAD4 descends over it."""
    f = _node(_attach_pause_through_landing=True)
    _run(f, 4.0)
    assert f._traj_rate == 0.0
    f._his_phase = 'LANDING'
    _run(f, 4.0)
    assert f._traj_rate == 0.0                      # still at rest while he lands
    f._his_phase = 'LANDED_DISARMED'
    _run(f, 3.1)
    assert f._traj_rate == 1.0
    g = _node()                                     # off: his LANDING ends the pause, as before
    _run(g, 4.0)
    g._his_phase = 'LANDING'
    _run(g, 3.1)
    assert g._traj_rate == 1.0
