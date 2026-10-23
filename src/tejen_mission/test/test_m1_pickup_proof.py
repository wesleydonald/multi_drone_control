"""Small M1 pickup-proof regressions using the Oct-7 lift/no-lift contrast."""

import numpy as np

from tejen_mission.m1_pickup_proof import PickupProofConfig, PickupProofTracker


def _run_lift(*, object_follows: bool):
    tracker = PickupProofTracker(PickupProofConfig())
    decisions = []
    for index in range(32):
        t = index * 0.04
        rise = 0.12 * t
        object_rise = rise if object_follows else 0.0
        decisions.append(
            tracker.step(
                now_s=t,
                body_position=np.array([0.0, 0.0, 0.62 + rise]),
                magnet_position=np.array([0.0, 0.0, 0.04 + rise]),
                object_position=np.array([0.0, 0.0, object_rise]),
                fresh=True,
                magnet_on=True,
                candidate=True,
            )
        )
    return decisions


def test_co_moving_object_proves_only_after_measured_lift_and_slow_dwell():
    decisions = _run_lift(object_follows=True)
    assert not decisions[10].proven
    assert any(d.proven for d in decisions)
    assert not any(d.failed for d in decisions)
    assert PickupProofConfig().relative_speed_mps == 0.03


def test_stationary_object_is_rejected_during_short_vertical_proof():
    decisions = _run_lift(object_follows=False)
    assert any(d.failed for d in decisions)
    assert not any(d.proven for d in decisions)


def test_stale_evidence_cannot_prove_or_fail_and_preserves_history():
    tracker = PickupProofTracker(PickupProofConfig())
    tracker.step(now_s=0.0, body_position=[0, 0, 0.62], magnet_position=[0, 0, 0.04],
                 object_position=[0, 0, 0], fresh=True, magnet_on=True, candidate=True)
    uncertain = tracker.step(now_s=1.0, body_position=[0, 0, 0.72], magnet_position=[0, 0, 0.14],
                             object_position=[0, 0, 0], fresh=False, magnet_on=True, candidate=True)
    assert not uncertain.proven and not uncertain.failed
    assert uncertain.reason == "stale_evidence"


def test_planner_only_grants_loaded_cpp_after_pickup_proof_and_shows_rviz_tier():
    from pathlib import Path

    source = (Path(__file__).parents[1] / "tejen_mission" / "online_join_planner.py").read_text()
    assert "pickup_proof_tracker.step(" in source
    assert "and self.pickup_proof_proven" in source
    assert "tier={self.pickup_tier_descent.band}" in source
