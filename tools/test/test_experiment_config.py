"""
Unit tests for tools/experiment/config.py — THESIS_PLAN §9.1.

The config is the only part of the Gazebo runner that can be tested without Gazebo, and
it is where the cheap failures live: a launch arg that does not match the fleet size, an
event scheduled after the run ends, a bool rendered as Python's `True` instead of ROS's
`true`. Each of those produces a run that completes and means nothing, so they are worth
catching before a 60-second simulation rather than after it.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from experiment.config import Criteria, Event, ExperimentConfig  # noqa: E402

import yaml  # noqa: E402


def write(tmp_path, doc):
    p = tmp_path / 'exp.yaml'
    p.write_text(yaml.safe_dump(doc, sort_keys=False))
    return str(p)


BASE = {
    'name': 'unit',
    'world': 'three_attach.sdf',
    'launch': {'package': 'controller_quad_load', 'file': 'three_attach_launch.py',
               'args': {'num_drones': 3, 'reserved_attach': 1,
                        'attach_central': False}},
    'fleet': {'num_drones': 3, 'n_total': 4},
    'timing': {'duration_s': 60.0},
    'events': [{'t': 5.0, 'do': 'ARM'}, {'t': 8.0, 'do': 'TAKEOFF'},
               {'t': 20.0, 'do': 'MAGNET', 'arg': 'ON'}],
}


def test_a_valid_config_loads(tmp_path):
    c = ExperimentConfig.from_yaml(write(tmp_path, BASE))
    assert c.num_drones == 3 and c.n_total == 4
    assert [e.do for e in c.events] == ['ARM', 'TAKEOFF', 'MAGNET']


def test_booleans_render_as_ros_lowercase(tmp_path):
    """`attach_central:=False` is not an error in ROS launch — the arg falls through to
    its DEFAULT, so the run silently flies the opposite configuration to the one the
    YAML asked for. This is the single cheapest way to invalidate an experiment."""
    c = ExperimentConfig.from_yaml(write(tmp_path, BASE))
    argv = c.launch_argv()
    assert 'attach_central:=false' in argv
    assert not any('True' in a or 'False' in a for a in argv)


def test_fleet_size_must_match_the_launch_args(tmp_path):
    """If these disagree the runner watches the wrong number of mocap topics, so its
    pose watchdog can never fire and a dead drone looks like a healthy run."""
    doc = {**BASE, 'fleet': {'num_drones': 4, 'n_total': 5}}
    with pytest.raises(ValueError, match='num_drones'):
        ExperimentConfig.from_yaml(write(tmp_path, doc))


def test_n_total_must_equal_drones_plus_reserved(tmp_path):
    doc = {**BASE, 'fleet': {'num_drones': 3, 'n_total': 3}}
    with pytest.raises(ValueError, match='n_total'):
        ExperimentConfig.from_yaml(write(tmp_path, doc))


def test_an_event_after_the_end_of_the_run_is_rejected(tmp_path):
    """A weld scheduled at t=90 in a 60 s run produces a complete, plausible-looking
    result directory in which the thing being studied never happened."""
    doc = {**BASE, 'events': BASE['events'] + [{'t': 90.0, 'do': 'LAND'}]}
    with pytest.raises(ValueError, match='after duration_s'):
        ExperimentConfig.from_yaml(write(tmp_path, doc))


def test_takeoff_before_arm_is_rejected(tmp_path):
    doc = {**BASE, 'events': [{'t': 5.0, 'do': 'TAKEOFF'}, {'t': 8.0, 'do': 'ARM'}]}
    with pytest.raises(ValueError, match='before ARM'):
        ExperimentConfig.from_yaml(write(tmp_path, doc))


def test_magnet_without_takeoff_is_rejected(tmp_path):
    doc = {**BASE, 'events': [{'t': 5.0, 'do': 'ARM'},
                              {'t': 20.0, 'do': 'MAGNET', 'arg': 'ON'}]}
    with pytest.raises(ValueError, match='nothing is flying'):
        ExperimentConfig.from_yaml(write(tmp_path, doc))


def test_magnet_on_survives_yaml_boolean_coercion(tmp_path):
    """YAML 1.1 parses an unquoted ON as the BOOLEAN True. Left alone, the runner
    publishes "TRUE" to /magnet/command, the magnet never switches on, the approach
    never welds — and the run completes looking completely normal, which is the worst
    possible way for this to fail."""
    doc = {**BASE, 'events': [{'t': 5.0, 'do': 'ARM'}, {'t': 8.0, 'do': 'TAKEOFF'},
                              {'t': 20.0, 'do': 'MAGNET', 'arg': True}]}
    c = ExperimentConfig.from_yaml(write(tmp_path, doc))
    assert [e.arg for e in c.events if e.do == 'MAGNET'] == ['ON']
    assert Event(1.0, 'MAGNET', False).arg == 'OFF'
    assert Event(1.0, 'MAGNET').arg == 'ON'
    assert Event(1.0, 'MAGNET', 'on').arg == 'ON'
    with pytest.raises(ValueError, match='MAGNET arg'):
        Event(1.0, 'MAGNET', 'MAYBE')


def test_attach_needs_a_drone_id():
    with pytest.raises(ValueError, match='needs a drone id'):
        Event(1.0, 'ATTACH')
    assert Event(1.0, 'ATTACH', '3').arg == 3


def test_unknown_event_names_are_rejected():
    """Typos must not be silently ignored: an event that never fires is the failure
    mode where the run looks fine and the interesting moment never happened."""
    with pytest.raises(ValueError, match='unknown event'):
        Event(1.0, 'ATATCH', 3)


def test_weld_relative_criteria_need_a_weld(tmp_path):
    doc = {**BASE,
           'events': [{'t': 5.0, 'do': 'ARM'}, {'t': 8.0, 'do': 'TAKEOFF'}],
           'criteria': {'window_from': 'weld', 'window_s': 20.0,
                        'max_payload_tilt_deg': 40.0}}
    with pytest.raises(ValueError, match='no weld event'):
        ExperimentConfig.from_yaml(write(tmp_path, doc))


def test_unknown_criteria_keys_are_rejected():
    """A misspelled threshold would otherwise be silently dropped and the run would
    report PASS without ever checking the thing it was named for."""
    with pytest.raises(ValueError, match='unknown criteria keys'):
        Criteria(max_payload_tilt_degrees=40.0)


def test_criteria_defaults_forbid_aborts():
    assert Criteria().forbid_abort is True


def test_the_shipped_experiment_configs_are_valid():
    """Every config in configs/experiments/ must load and validate, so a broken one is
    caught by the gate rather than at the start of a Gazebo session."""
    repo = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))          # tools/test/ -> tools/ -> repo
    d = os.path.join(repo, 'configs', 'experiments')
    if not os.path.isdir(d):
        pytest.skip('no experiment configs yet')
    found = [f for f in sorted(os.listdir(d)) if f.endswith(('.yaml', '.yml'))]
    assert found, 'configs/experiments/ exists but is empty'
    for f in found:
        ExperimentConfig.from_yaml(os.path.join(d, f))
