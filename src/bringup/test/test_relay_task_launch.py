"""relay_task (twin only, off by default; card 2026-10-30_x4_endurance_relay) reaches the twin's mode:=dissipative
planner with rejoin_task; unset, no planner gets it; refused on the rig and without rejoin_task."""
import pytest

pytest.importorskip('launch_ros')
from test_real_mode_launches import _by_name, _planner, rig, sim  # noqa: E402


def test_relay_task_reaches_the_twin_planner_only_when_set():
    p = _planner(_by_name(sim('dissipative', num_drones=8, rejoin_departed=True, departed_land=False, rejoin_task=True,
                              relay_task=True, load_traj='fig_8', fig8_size=0.5, traj_laps=5, traj_lap_hold_s=3.0)))
    assert p['relay_task'] is True and p['rejoin_task'] is True
    for g in (sim('dissipative'), sim('dissipative', num_drones=4, rejoin_departed=True, departed_land=False,
                                      rejoin_task=True), rig('dissipative', thrust_ratio=35.2)):
        assert 'relay_task' not in _planner(_by_name(g))
    for launch in (lambda: sim('dissipative', num_drones=4, rejoin_departed=True, departed_land=False, relay_task=True),
                   lambda: sim('mpc', relay_task=True),
                   lambda: rig('dissipative', thrust_ratio=35.2, relay_task=True)):
        with pytest.raises(RuntimeError, match='relay_task flies only'):
            launch()
