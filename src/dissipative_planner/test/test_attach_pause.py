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


def test_land_during_the_pause_speeds_the_clock_back_up():
    f = _node()
    _run(f, 4.0)
    assert f._traj_rate == 0.0
    f._land_to_ground = True
    _run(f, 3.1)
    assert f._traj_rate == 1.0
