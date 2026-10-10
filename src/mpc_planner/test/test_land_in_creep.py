"""LAND in the floor creep ends the creep with no lift, then descends (Wesley 9 Oct: LAND lands every
drone, whatever it is doing; R1303 ignored it and the ring lifted)."""
import types

from mpc_planner.planner_node import LoadPlanner


def _fake(phase='creep', takeoff_seen=True, start_taut=False):
    log = types.SimpleNamespace(lines=[])
    f = types.SimpleNamespace(
        phase=phase, takeoff_seen=takeoff_seen, start_taut=start_taut, descending=False,
        _land_to_ground=False, _landed=False, _lift_refused=False, entered=[],
        _touchdown=types.SimpleNamespace(reset=lambda: None),
        get_logger=lambda: types.SimpleNamespace(info=log.lines.append, warn=log.lines.append))
    f.log = log

    def enter(reason, measure=True):
        f.entered.append((reason, measure))
        f.phase = 'planner'
    f._enter_planner_phase = enter
    return f


def _land(f):
    LoadPlanner._fleet_command_cb(f, types.SimpleNamespace(data='LAND'))


def test_land_in_the_creep_ends_it_without_a_lift_and_descends():
    f = _fake()
    _land(f)
    assert f.entered == [('LAND in the creep', False)]      # no rod measure on a slack creep
    assert f._lift_refused and f.descending and f._land_to_ground


def test_land_before_takeoff_is_still_ignored_by_the_planner():
    f = _fake(takeoff_seen=False)
    _land(f)
    assert f.entered == [] and not f.descending and f.phase == 'creep'


def test_an_air_start_has_no_creep_to_end():
    f = _fake(start_taut=True)
    _land(f)
    assert f.entered == [] and not f.descending
