"""M1 configuration contract for mutually exclusive XY bias rejection."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / 'src/tejen_mission/config/irl_commissioning.yaml'
RUNNER = ROOT / 'tools/sim_test/run_m1_irl_virtual_ring.sh'


def test_irl_config_keeps_legacy_mode_until_closed_loop_sim_validation():
    data = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    controller = data['tejen_mpc']['ros__parameters']
    assert controller['xy_bias_mode'] == 'legacy_integral'
    assert controller['enable_xy_integral_action'] is True
    assert controller['lateral_disturbance_observer_bandwidth_rad_s'] == 0.30
    assert controller['lateral_disturbance_airborne_height_m'] == 0.20


def test_full_m1_runner_rejects_shadow_or_no_compensation_modes():
    text = RUNNER.read_text(encoding='utf-8')
    assert 'xy_bias_mode not in {"legacy_integral", "lateral_disturbance"}' in text
    assert 'shadow/none are test-only' in text
    assert 'XY bias rejection:' in text
