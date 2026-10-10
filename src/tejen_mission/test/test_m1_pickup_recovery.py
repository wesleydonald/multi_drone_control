"""Focused M1-only recovery regressions from the Oct-7 HIGH/LOW misses."""

from pathlib import Path

import numpy as np

from tejen_mission.measured_tip_alignment import TieredDescent, TipAlignment
from tejen_mission.m1_pickup_recovery import PickupProgressStall, PickupRelease


ROOT = Path(__file__).resolve().parents[1]
PLANNER = ROOT / "tejen_mission/online_join_planner.py"


def test_high_raises_from_measured_clearance_deficit_even_above_commanded_staging():
    descent = TieredDescent(TipAlignment(), start_height_m=.314, floor_m=-.017,
                            speed_mps=.04)
    descent.height_m = .1065  # 121727: command > .10, measured gap only .052
    descent.step(now_s=0., error_world_xy=np.array([.01, 0.]),
                 clearance_m=.052, relative_speed_mps=.01, fresh=True)
    descent.step(now_s=.1, error_world_xy=np.array([.01, 0.]),
                 clearance_m=.052, relative_speed_mps=.01, fresh=True)
    assert .1065 < descent.height_m <= .1105 + 1e-9
    assert descent.reason == "raise_to_safe_clearance"


def test_stall_uses_real_descent_progress_not_swinging_xy_improvements():
    stall = PickupProgressStall(wait_s=10., improvement_m=.005)
    fired = []
    for i in range(12):
        fired.append(stall.step(now=float(i), progress_m=.100,
                                blocked=True, key="LOW"))
    assert fired[-2] and not fired[-1]  # one shot at 10 s
    stall.reset()
    for i in range(8):
        assert not stall.step(now=float(i), progress_m=.100 + .006 * i,
                              blocked=True, key="LOW")


def test_release_keeps_xy_fixed_until_fresh_vertical_separation_then_retries():
    release = PickupRelease(release_dwell_s=.20, speed_mps=.05,
                            max_rise_m=.20, clearance_m=.08, separation_dwell_s=.12)
    release.begin(now=0., reference_position=[1., 2., .60])
    for i in range(1, 100):
        now = .05 * i
        ready = release.step(now=now, face_clearance_m=.02,
                             fresh=True, magnet_off=True)
        assert not ready
        np.testing.assert_allclose(release.target[:2], [1., 2.])
        assert release.target[2] <= .80 + 1e-9
    assert release.target[2] == .80
    assert not release.step(now=5.1, face_clearance_m=.09,
                            fresh=False, magnet_off=True)
    assert not release.step(now=5.2, face_clearance_m=.09,
                            fresh=True, magnet_off=True)
    assert release.step(now=5.35, face_clearance_m=.09,
                        fresh=True, magnet_off=True)


def test_attempt_budget_waits_after_three_failures_without_auto_landing():
    release = PickupRelease(max_attempts=3)
    for attempt in range(1, 4):
        release.begin(now=attempt, reference_position=[1., 2., .60])
        assert release.attempts == attempt
        release.step(now=attempt + .3, face_clearance_m=.10,
                     fresh=True, magnet_off=True)
        ready = release.step(now=attempt + .5, face_clearance_m=.10,
                             fresh=True, magnet_off=True)
        assert ready is (attempt < 3)
        if attempt < 3:
            release.reset_stage()
    assert release.stage == "await_operator"


def test_planner_routes_proof_and_precontact_failures_through_vertical_release():
    source = PLANNER.read_text()
    assert "def begin_m1_pickup_retry(" in source
    assert "self.begin_m1_pickup_retry(" in source
    assert "self.m1_pickup_release.target" in source
    assert "self.update_m1_pickup_recovery()" in source
    assert "self.pickup_magnet_command(spec.magnet_command)" in source
    assert "blocked=not self.object_attached, key='CONTACT_WAIT'" in source
