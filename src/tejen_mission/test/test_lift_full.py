"""multi_drone_control 4 Nov (Wesley's supervisor; RIG-1104-k1m_2-4: the transit started once the object was 8 cm off
the floor and climbed diagonally toward the ring, swinging the magnet into the carriers' cables): lift_full_before_transit
(default off) ends his loaded lift only when the lift profile is complete at its height. Off keeps his early exit."""
import numpy as np

from tejen_mission.mission_types import MissionPhase as P
from test_m1_contact_attach import planner, tick


def _lift(full):
    p = planner(contact_attach=False, phase=P.LIFT_OBJECT)
    p.pickup_proof_enabled, p.pickup_proof_proven, p.object_attached = False, True, True
    p.commissioning_hold_after_lift = False
    p.lift_complete_object_clearance, p.loaded_lift_z_tolerance = 0.08, 0.05
    p.pickup_attach_clearance, p.pickup_lift_height, p.pickup_lift_speed = 0.02, 1.30, 0.12   # ~10.7 s lift
    p.pickup_z_tolerance = 0.20
    p.latched_pickup_position = np.array([0.0, 1.6, 0.01])
    p.pickup_object_position = np.array([0.0, 1.6, 0.11])          # 10 cm off the floor
    p.lift_full_before_transit = full
    return p


def _run(p, seconds, z_err, dt=0.1):
    for _ in range(round(seconds / dt)):
        p.errors = (0.0, 0.0, z_err)
        tick(p, dt)
        if p.phase != P.LIFT_OBJECT:
            return


def test_off_leaves_as_soon_as_the_object_is_off_the_floor():
    p = _lift(full=False)
    _run(p, 0.5, z_err=-0.9)
    assert p.phase == P.TRANSIT_TO_DROP_POINT


def test_on_climbs_the_whole_lift_first():
    p = _lift(full=True)
    _run(p, 5.0, z_err=-0.9)
    assert p.phase == P.LIFT_OBJECT                         # object up, but the lift not done
    _run(p, 10.0, z_err=-0.10)
    assert p.phase == P.LIFT_OBJECT                         # profile done, 10 cm short of it: holds
    _run(p, 1.0, z_err=-0.02)
    assert p.phase == P.TRANSIT_TO_DROP_POINT
