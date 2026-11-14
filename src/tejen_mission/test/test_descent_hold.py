"""contact_descent_hold (multi_drone_control 3 Nov; Tejen's review wesley-attach-review-20261103, priority 1): past the
position band the contact descent holds the height it had authorised and resumes inside the band, instead of restarting
the approach (RIG-1029-sq5_2: six descents, six restarts at 8-10 cm); past contact_descent_gross_xy_m or the speed abort
it still restarts, and a contact keeps its priority. Off keeps his restart."""
import numpy as np

from tejen_mission.mission_types import MissionPhase as P
from test_m1_contact_attach import descend, planner, tick


def _rig(hold=True, gross=0.20):
    p = planner(phase=P.MATCH_VELOCITY)
    descend(p)
    p.contact_abort_xy_threshold, p.contact_abort_xy_velocity_threshold = 0.10, 0.35
    p.contact_abort_xy_debounce_s = 0.0
    p.contact_bottom_hold = False
    p.contact_descent_hold, p.contact_descent_gross_xy_m = hold, gross
    p.contact_descent_holding, p.contact_hold_last_s = False, None
    return p


def _run(p, seconds, xy, speed=0.05, dt=1 / 30):
    for _ in range(round(seconds / dt)):
        p.errors = (xy, speed, 0.0)
        p.measured_magnet_tip_position = p.measured_magnet_tip_position + np.array([1e-4, 0.0, 0.0])
        tick(p, dt)
        if p.phase != P.DESCEND_TO_ATTACHMENT:
            return


def _z(p):
    return float(p.desired_tip_offset()[2])


def test_off_keeps_his_restart():
    p = _rig(hold=False)
    _run(p, 1 / 30, xy=0.12)
    assert p.phase == P.APPROACH_ABOVE_TARGET


def test_an_excursion_past_the_band_holds_the_height_and_resumes():
    p = _rig()
    _run(p, 0.5, xy=0.03)
    z0 = _z(p)
    _run(p, 1.0, xy=0.12)                                # 1 s outside the 10 cm band, inside the 20 cm restart
    assert p.phase == P.DESCEND_TO_ATTACHMENT and p.contact_descent_holding
    assert abs(_z(p) - z0) < 0.10 * (2 / 30) + 1e-9      # held: at most the first tick's progress
    assert any('held' in m for m in p.infos)
    _run(p, 0.5, xy=0.03)
    assert p.phase == P.DESCEND_TO_ATTACHMENT and not p.contact_descent_holding
    assert _z(p) < z0 - 0.03                             # descending again from the held height


def test_the_hold_never_completes_the_descent():
    p = _rig()
    _run(p, 10.0, xy=0.12)                               # far longer than the whole descent
    assert p.phase == P.DESCEND_TO_ATTACHMENT and p.contact_descent_holding


def test_past_the_gross_band_or_the_speed_abort_it_restarts():
    p = _rig()
    _run(p, 1 / 30, xy=0.21)
    assert p.phase == P.APPROACH_ABOVE_TARGET
    p = _rig()
    _run(p, 1 / 30, xy=0.05, speed=0.36)
    assert p.phase == P.APPROACH_ABOVE_TARGET
    p = _rig()
    _run(p, 1.0, xy=0.05, speed=0.30)                    # 0.30 m/s is under the 0.35 trial: no restart
    assert p.phase == P.DESCEND_TO_ATTACHMENT


def test_a_contact_while_held_goes_to_the_proof():
    p = _rig()
    _run(p, 0.5, xy=0.12)
    assert p.contact_descent_holding
    p.contact_reported = True
    tick(p, 1 / 30)
    assert p.phase == P.LIFT_PROOF and not p.contact_descent_holding
