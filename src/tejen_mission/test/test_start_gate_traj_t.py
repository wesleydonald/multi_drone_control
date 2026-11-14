"""start_gate_min_traj_t (13 Nov 2026): the twin's start gate waits for the path clock, so his TAKEOFF comes while the
ring already flies its path, as on the rig. 0 (the default) keeps the gate opening on "running" alone."""
import json
from types import SimpleNamespace

from tejen_mission.simulation_test_supervisor import SimulationTestSupervisor


def gate(min_t):
    g = SimpleNamespace(start_gate_open=False, start_gate_min_traj_t=min_t, start_gate_topic='/payload/trajectory_state',
                        get_logger=lambda: SimpleNamespace(info=lambda *a, **k: None))
    feed = lambda **m: SimulationTestSupervisor._start_gate(g, SimpleNamespace(data=json.dumps(m)))
    return g, feed


def test_default_opens_on_running():
    g, feed = gate(0.0)
    feed(running=False, traj_t=0.0)
    assert not g.start_gate_open
    feed(running=True, traj_t=0.0)
    assert g.start_gate_open


def test_waits_for_the_path_clock():
    g, feed = gate(10.0)
    feed(running=True, traj_t=0.0)
    feed(running=True, traj_t=9.9)
    assert not g.start_gate_open
    feed(running=True, traj_t=10.0)
    assert g.start_gate_open


def test_bad_messages_are_ignored():
    g, feed = gate(10.0)
    SimulationTestSupervisor._start_gate(g, SimpleNamespace(data='not json'))
    feed(running=True, traj_t=None)
    assert not g.start_gate_open
