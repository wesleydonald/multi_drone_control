"""tools/fleet_monitor.py: the explanations and the state it shows, without a screen or ROS."""
import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import fleet_monitor as fm  # noqa: E402


def test_known_messages_get_an_explanation():
    assert 'mocap' in fm.explain('[Drone 1] Pose timeout (0.26s) - disarming.')
    assert 'one-sided' in fm.explain('Drone 1 disarmed before TAKEOFF: fleet disarmed, TAKEOFF refused.')
    assert 'TX module' in fm.explain('Radio /dev/QUAD1 not available (could not open port)')
    assert 'kT' in fm.explain('[kT d0] trim hit its bound (18.0-30.0): the typed kT 24.0 is far')
    assert fm.explain('something nobody has seen before') == ''


def test_state_throttle_cap_and_alerts():
    s = fm.FleetState(2)
    s.on_cmd(0, types.SimpleNamespace(armed=True, channel_2=0.2))    # thr 0.6
    s.on_cmd(1, types.SimpleNamespace(armed=True, channel_2=-0.1))   # thr 0.45
    assert s.drones[0]['cap_since'] is not None and s.drones[1]['cap_since'] is None
    assert abs(s.drones[1]['thr'] - 0.45) < 1e-9
    s.on_log(20, 'controller_0', 'Controller ready')                 # INFO: not an alert
    s.on_log(40, 'central_controller', 'TAKEOFF REFUSED: drone(s) [1] not armed')
    assert len(s.alerts) == 1 and s.alerts[0][1] == 'ERROR' and 'Relaunch' in s.alerts[0][4]


def test_gui_builds_offscreen():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    import pytest
    pytest.importorskip('PyQt5.QtWidgets')
    s = fm.FleetState(3)
    s.on_manager(types.SimpleNamespace(data='ARM sequence complete: all 3 armed, ready for TAKEOFF.'))
    s.on_log(30, 'load_planner', 'arc creep timed out 3.0s after the sweep ended')
    assert fm.gui(s, exit_after_ms=600) == 0
