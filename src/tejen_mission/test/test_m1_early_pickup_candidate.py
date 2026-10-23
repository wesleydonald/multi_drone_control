"""LOW proximity can enter measured lift proof without virtual overtravel."""

from pathlib import Path

import numpy as np

from tejen_mission.m1_pickup_recovery import (
    early_candidate_ready,
    lift_offsets_from_candidate_hold,
)


PLANNER = Path(__file__).resolve().parents[1] / "tejen_mission/online_join_planner.py"


def test_near_surface_low_candidate_only_not_loose_22mm_proximity():
    common = dict(low=True, candidate=True, fresh=True, magnet_on=True)
    assert not early_candidate_ready(clearance_m=.022, **common)
    assert early_candidate_ready(clearance_m=.004, **common)
    assert early_candidate_ready(clearance_m=-.008, **common)
    for field in ("low", "candidate", "fresh", "magnet_on"):
        values = dict(common, clearance_m=.004)
        values[field] = False
        assert not early_candidate_ready(**values)
    assert not early_candidate_ready(clearance_m=float("nan"), **common)


def test_candidate_hold_is_the_first_lift_reference_not_contact_endpoint():
    hold = np.array([.88, -1.27, .612])
    pickup = np.array([.87, -1.28, 0.])
    quad_from_marker = np.array([0., 0., .53])
    start, goal = lift_offsets_from_candidate_hold(
        hold, pickup, quad_from_marker, lift_delta_m=.42)
    np.testing.assert_allclose(pickup + start + quad_from_marker, hold)
    np.testing.assert_allclose(goal - start, [0., 0., .42])


def test_planner_routes_candidate_before_stall_and_keeps_legacy_separate():
    source = PLANNER.read_text()
    assert "early_candidate_ready(" in source
    assert "self.m1_candidate_hold_reference =" in source
    assert "getattr(self, 'm1_candidate_hold_reference', None) is not None" in source
    assert source.index("early_candidate_ready(") < source.index(
        "self.m1_pickup_stall.step(now=self._physical_now_s()")
