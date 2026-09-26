from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
LAUNCH = ROOT / "src/tejen_mission/launch/m1_irl_virtual_ring.launch.py"
BACKEND_SOURCE = ROOT / "src/tejen_dynamic_planner/src/transfer_backend_node.cpp"


def test_m1_irl_launch_starts_backend_before_one_shot_virtual_ring_commitment():
    launch = LAUNCH.read_text()
    backend_start = "TimerAction(period=0.5, actions=[backend])"
    ring_start = "TimerAction(period=1.0, actions=[virtual_ring])"

    assert backend_start in launch
    assert ring_start in launch
    assert launch.index(backend_start) < launch.index(ring_start)


def test_backend_reports_moving_basket_readiness_separately_from_companions():
    source = BACKEND_SOURCE.read_text()

    assert "WAITING_FOR_VALID_MOVING_BASKET_COMMITMENT" in source
    assert "AUTHORITY_GRANT_REJECTED_INVALID_MOVING_BASKET_COMMITMENT" in source
    assert "MOVING_BASKET_INPUT_INVALID_DISABLED" in source
    assert "!cooperative_scene_enabled_ && moving_basket_scene_enabled_" in source
    assert "!movingBasketTrajectoryFresh(now_s)" in source
