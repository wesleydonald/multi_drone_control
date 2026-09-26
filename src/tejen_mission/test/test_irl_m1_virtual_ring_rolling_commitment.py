from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]
FAKE_WORLD = ROOT / "src/tejen_mission/tejen_mission/fake_cooperative_transport_world.py"
CONFIG = ROOT / "src/tejen_mission/config/irl_commissioning.yaml"


def test_virtual_ring_uses_restart_safe_monotonic_commitment_sequences():
    source = FAKE_WORLD.read_text()
    assert "from tejen_mission.m2_fleet_commitment import commitment_sequence_seed" in source
    assert "self.shared_trajectory_sequence = commitment_sequence_seed()" in source
    assert "self.shared_trajectory_sequence += 1" in source
    assert "msg.sequence = int(sequence)" in source


def test_rolling_window_preserves_original_motion_time_origin():
    source = FAKE_WORLD.read_text()
    assert "greville_abs - self.start_time_s" in source
    assert "greville_abs - t0" not in source


def test_refresh_rebuilds_then_republishes_finite_commitment():
    source = FAKE_WORLD.read_text()
    assert "shared_trajectory_refresh_period_s" in source
    assert "self.refresh_shared_commitments(now_abs_s)" in source
    assert "self.build_shared_payload_spline(valid_from_s)" in source
    assert "self.last_shared_commitment_publish_s = float(valid_from_s)" in source


def test_m1_irl_ring_advertises_rolling_ten_second_future():
    data = yaml.safe_load(CONFIG.read_text())
    fake = data["fake_cooperative_transport_world"]["ros__parameters"]
    assert fake["publish_shared_trajectories"] is True
    assert fake["shared_trajectory_duration_s"] == 10.0
    assert fake["shared_trajectory_refresh_period_s"] == 1.0
    assert fake["shared_trajectory_control_interval_s"] == 0.50
    assert fake["shared_trajectory_refresh_period_s"] < fake["shared_trajectory_duration_s"]
