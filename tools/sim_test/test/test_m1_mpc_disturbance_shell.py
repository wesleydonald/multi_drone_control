from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "sim_test" / "run_m1_mpc_disturbance.sh"


def test_m1_mpc_disturbance_wrapper_uses_existing_manual_gui_runner():
    text = SCRIPT.read_text()
    assert 'run_m1_case.sh' in text
    assert 'cleanup_m1_stale.sh' in text
    assert '--auto-arm' not in text
    assert '--headless' not in text
    assert 'MANUAL ARM / TAKEOFF / DISARM' in text
    assert 'SETTLE_ABOVE_PICKUP -> clear at LIFT_OBJECT' in text
    assert 'x3::X3/base_link only' in text


def test_pickup_bias_stages_keep_the_same_small_x_only_disturbance():
    text = SCRIPT.read_text()
    assert 'FX_N="-0.05"' in text
    assert 'FY_N="0.00"' in text
    assert 'pickup-legacy)' in text
    assert 'pickup-shadow)' in text
    assert 'pickup-active)' in text
    assert 'MODE="legacy_integral"' in text
    assert 'MODE="lateral_disturbance_shadow"' in text
    assert 'MODE="lateral_disturbance"' in text
    assert 'persistent/continuous drift' in text


def test_old_generic_disturbed_stage_names_are_not_exposed():
    text = SCRIPT.read_text()
    assert 'legacy-disturbed)' not in text
    assert 'shadow-disturbed)' not in text
    assert 'active-disturbed)' not in text
    assert 'none-disturbed)' not in text


def test_landing_regressions_use_normal_planner_landing_request_path():
    text = SCRIPT.read_text()
    assert 'LANDING_PHASE="APPROACH_ABOVE_PICKUP"' in text
    assert 'LANDING_PHASE="LIFT_OBJECT"' in text
    assert '--landing-check-from' in text
