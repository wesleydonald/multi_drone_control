"""Ring quaternion sign continuity and the slot-offset pre-flight warning (rig model-f1,
docs/rig_2026-09-30_replay.md). The rig mocap sends w >= 0, so the ring quaternion flips sign
when its yaw crosses 180 deg; build_x_init keeps it in the warm start's hemisphere. With the
drones one plate (30 deg) off their modelled slots every flip failed the solve."""
import numpy as np
import pytest

from controller_load_mpc.load_cable_dynamics import LoadCableDynamics
from controller_load_mpc.planner_solver import PlannerSolver
from controller_load_mpc.geometry import (attach_points, quat_same_hemisphere, quat_to_rot_np,
                                          slot_azimuth_errors, slot_offset_warnings, yaw_quat)


def _mocap(q):
    """As the rig mocap sends it: w >= 0."""
    q = np.asarray(q, float)
    return -q if q[0] < 0.0 else q


def _solver(n=4):
    rho = attach_points(n, 0.225, 0.0)
    s = PlannerSolver.__new__(PlannerSolver)
    s.dyn = LoadCableDynamics(n_drones=n, load_mass=0.86, load_inertia=[1e-2] * 3,
                              cable_lengths=[0.55] * n, attach_points=rho, drone_mass=0.55)
    s.last_X = None
    s._q_prev = None
    return s, rho


def _load(q, z=0.5):
    ls = np.zeros(13)
    ls[2] = z
    ls[3:7] = q
    return ls


def _drones(rho, q, z=0.5):
    R = quat_to_rot_np(q)
    return [np.array([0, 0, z]) + R @ r + 0.55 * np.array([r[0] / 0.225 * 0.7, r[1] / 0.225 * 0.7, 0.7])
            for r in rho]


def test_same_hemisphere_helper():
    q = yaw_quat(np.radians(170.0))
    assert np.allclose(quat_same_hemisphere(-q, q), q)
    assert np.allclose(quat_same_hemisphere(q, q), q)
    assert np.allclose(quat_same_hemisphere(-q, None), -q)


def test_yaw_sweep_through_180_stays_continuous():
    s, rho = _solver()
    prev = None
    fed = []
    for yaw in np.radians(np.linspace(150.0, 210.0, 61)):
        q_true = yaw_quat(yaw)
        q_in = _mocap(q_true)
        x = s.build_x_init(_load(q_in), _drones(rho, q_true))
        q = x[6:10]
        fed.append(q)
        if prev is not None:
            assert float(np.dot(q, prev)) > 0.99            # 1 deg steps: no jump
        prev = q
        X = np.tile(x[:, None], (1, 3))                     # the next tick warm-starts from it
        s.last_X = X
    # the mocap input did flip in the sweep; the fed quaternion did not
    assert any(_mocap(yaw_quat(np.radians(a)))[3] < 0 for a in (181.0, 200.0))
    assert fed[-1][3] > 0.0


def test_no_flip_is_untouched_and_cold_start_uses_the_last_fed():
    s, rho = _solver()
    q = yaw_quat(np.radians(30.0))
    x = s.build_x_init(_load(q), _drones(rho, q))
    assert np.array_equal(x[6:10], q)
    q2 = yaw_quat(np.radians(31.0))
    x = s.build_x_init(_load(q2), _drones(rho, q2))
    assert np.array_equal(x[6:10], q2)
    # no warm start (a reseed): continuity against the previous fed quaternion
    s._q_prev = yaw_quat(np.radians(179.0))
    q3 = _mocap(yaw_quat(np.radians(181.0)))
    x = s.build_x_init(_load(q3), _drones(rho, q3))
    assert np.allclose(x[6:10], -q3)


def test_cable_directions_do_not_depend_on_the_sign():
    s1, rho = _solver()
    s2, _ = _solver()
    q = yaw_quat(np.radians(185.0))
    D = _drones(rho, q)
    s2._q_prev = -q
    x1 = s1.build_x_init(_load(q), D)
    x2 = s2.build_x_init(_load(q), D)
    assert np.allclose(x1[6:10], -x2[6:10])
    assert np.allclose(x1[13:], x2[13:])


def _ring_drones(az_deg, yaw_deg, r=0.6, centre=(0.3, -0.2)):
    return [np.array([centre[0] + r * np.cos(np.radians(a + yaw_deg)),
                      centre[1] + r * np.sin(np.radians(a + yaw_deg)), 0.1]) for a in az_deg]


def test_slot_errors_zero_when_on_the_slots():
    slot_az = np.radians([30.0, 150.0, 270.0, 90.0])
    D = _ring_drones([150.0, 30.0, 90.0, 270.0], 158.9)
    errs = slot_azimuth_errors(D, (0.3, -0.2), np.radians(158.9), [1, 0, 3, 2], slot_az)
    assert [e[0] for e in errs] == [1, 0, 3, 2]
    assert [e[2] for e in errs] == [1, 5, 9, 3]
    assert max(abs(e[1]) for e in errs) < 1e-9
    assert slot_offset_warnings(errs) == []


def test_one_plate_off_warns_for_every_drone():
    # model-f1: all four drones ~30 deg short of their slots, yaw datum near 180
    slot_az = np.radians([30.0, 150.0, 270.0, 90.0])
    D = _ring_drones([150.0 - 28, 30.0 - 30, 90.0 - 31, 270.0 - 30], 158.9)
    errs = slot_azimuth_errors(D, (0.3, -0.2), np.radians(158.9), [1, 0, 3, 2], slot_az)
    assert [round(e[1]) for e in errs] == [-30, -28, -30, -31]
    w = slot_offset_warnings(errs)
    assert len(w) == 4
    assert w[0] == ('drone 1 sits -30 deg from plate 1: check the ring rigid body '
                    '(+x toward plate 0) or the magnet plates')


def test_warning_threshold_is_10_deg_and_wraps():
    errs = [(0, 9.9, 0), (1, -10.1, 3), (2, 179.0, 6)]
    w = slot_offset_warnings(errs)
    assert len(w) == 2 and w[0].startswith('drone 1 sits -10 deg')
    D = _ring_drones([-5.0], 175.0)                      # wraps across +-180
    e = slot_azimuth_errors(D, (0.3, -0.2), np.radians(175.0), [0], [np.radians(355.0)])
    assert e[0][1] == pytest.approx(0.0, abs=1e-9) and e[0][2] == 0
