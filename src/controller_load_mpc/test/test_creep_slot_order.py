"""The floor-start creep with a non-identity slot map (rig 2026-09-30, R3e f1).

The ring's Motive yaw read -135 deg, so the planner matched slot->drone [2, 3, 0, 1]. The
creep was fed drones in PHYSICAL order against slot-ordered attach points: each drone was
anchored on the opposite plate and its reference sat 0.22 m from the ring centre, and the
fleet flew into the middle. The planner now feeds the creep in slot order and publishes
slot i's reference to drone slot2drone[i]; this pins that wiring."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))), 'src', 'controller_load_mpc'))

from controller_load_mpc.creep_controller import CreepController   # noqa: E402
from controller_load_mpc.geometry import azimuth_slot_assignment   # noqa: E402


class _Log:
    def warn(self, m):
        pass

    info = warn


def _rig():
    yaw = np.deg2rad(-135.3)
    load = np.zeros(13)
    load[0:3] = [0.02, -0.03, 0.05]
    load[3:7] = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
    rho = [np.array([0.25 * np.cos(a), 0.25 * np.sin(a), 0.0])
           for a in np.deg2rad([0, 90, 180, 270])]
    # the rig's resting drones (tracker logs): world azimuths 40, 133, -139, -52 deg
    drones = [np.array([0.52, 0.42, 0.05]), np.array([-0.48, 0.52, 0.07]),
              np.array([-0.56, -0.49, 0.07]), np.array([0.45, -0.57, 0.05])]
    s2d = azimuth_slot_assignment(drones, load[0:2], 4, load_yaw=yaw,
                                  slot_az=[np.arctan2(r[1], r[0]) for r in rho])
    return load, rho, drones, s2d


def _first_refs(load, rho, drones, s2d):
    refs = {}
    creep = CreepController(4, rho, 0.47, 20, 0.05, 9.81, 45.0, 10.0,
                            lambda i: drones[s2d[i]],
                            lambda i, nodes: refs.__setitem__(s2d[i], nodes[0][0]),
                            _Log())
    creep.step(load, [drones[s2d[i]] for i in range(4)], [(1.0, 0.47)] * 4,
               takeoff_seen=False)
    return refs


def test_rig_slot_map_is_not_identity():
    assert _rig()[3] == [2, 3, 0, 1]


def test_each_drone_is_anchored_on_its_own_plate():
    load, rho, drones, s2d = _rig()
    refs = _first_refs(load, rho, drones, s2d)
    assert sorted(refs) == [0, 1, 2, 3]
    for k in range(4):
        # before liftoff the reference is the resting pose plus a vertical lead
        assert np.hypot(*(refs[k][:2] - drones[k][:2])) < 0.05, (k, refs[k], drones[k])
