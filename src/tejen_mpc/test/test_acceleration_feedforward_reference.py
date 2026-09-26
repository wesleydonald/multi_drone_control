"""Regression tests for acceleration-derived payload-MPC feedforward references."""

from pathlib import Path

import numpy as np

from tejen_mpc.reference_feedforward import (
    acceleration_feedforward_reference,
    quaternion_to_rotation_matrix,
    quaternion_yaw,
)


def _quat_from_yaw(yaw: float) -> np.ndarray:
    return np.array([
        np.cos(0.5 * yaw),
        0.0,
        0.0,
        np.sin(0.5 * yaw),
    ])


def test_hover_acceleration_produces_level_heading_and_hover_throttle():
    yaw = 0.73
    throttle, q_ref = acceleration_feedforward_reference(
        desired_acceleration=np.zeros(3),
        thrust_ratio=44.0,
        heading_quaternion=_quat_from_yaw(yaw),
    )

    assert np.isclose(throttle, 9.81 / 44.0)
    rotation = quaternion_to_rotation_matrix(q_ref)
    np.testing.assert_allclose(rotation[:, 2], [0.0, 0.0, 1.0], atol=1e-12)
    assert np.isclose(quaternion_yaw(q_ref), yaw, atol=1e-12)


def test_diagonal_acceleration_sets_matching_thrust_axis_and_collective():
    desired_acceleration = np.array([1.0, -0.4, 0.6])
    specific_thrust = desired_acceleration + np.array([0.0, 0.0, 9.81])
    throttle, q_ref = acceleration_feedforward_reference(
        desired_acceleration=desired_acceleration,
        thrust_ratio=44.0,
        heading_quaternion=_quat_from_yaw(-0.35),
    )

    assert np.isclose(throttle, np.linalg.norm(specific_thrust) / 44.0)
    rotation = quaternion_to_rotation_matrix(q_ref)
    np.testing.assert_allclose(
        rotation[:, 2],
        specific_thrust / np.linalg.norm(specific_thrust),
        atol=1e-12,
    )
    assert np.isclose(quaternion_yaw(q_ref), -0.35, atol=1e-12)


def test_positive_vertical_acceleration_increases_throttle_without_tilt():
    throttle, q_ref = acceleration_feedforward_reference(
        desired_acceleration=np.array([0.0, 0.0, 0.5]),
        thrust_ratio=44.0,
        heading_quaternion=_quat_from_yaw(0.0),
    )

    assert throttle > 9.81 / 44.0
    rotation = quaternion_to_rotation_matrix(q_ref)
    np.testing.assert_allclose(rotation[:, 2], [0.0, 0.0, 1.0], atol=1e-12)



def test_acados_setter_applies_feedforward_values_with_fake_solver(monkeypatch):
    import importlib
    import sys
    import types

    fake_acados = types.ModuleType("acados_template")

    class DummyAcadosType:
        pass

    for name in (
        "AcadosOcp",
        "AcadosOcpSolver",
        "AcadosModel",
        "AcadosSim",
        "AcadosSimSolver",
    ):
        setattr(fake_acados, name, DummyAcadosType)
    monkeypatch.setitem(sys.modules, "acados_template", fake_acados)
    sys.modules.pop("tejen_mpc.acados", None)
    acados = importlib.import_module("tejen_mpc.acados")

    class FakeSolver:
        def __init__(self):
            self.values = {}

        def set(self, stage, field, value):
            self.values[(stage, field)] = np.asarray(value, dtype=float).copy()

    traj = np.zeros((17, 3), dtype=float)
    traj[3, :] = 1.0
    traj[10, :] = 1.0
    traj[12, :] = 0.5
    est_params = np.array([44.0, 0.0, 0.12, 100.0, 100.0, 0.5, 0.531])

    solver = FakeSolver()
    acados.set_payload_trajectory_reference_aligned(
        solver,
        traj,
        N_horizon=2,
        step_counter=0,
        skip_steps=1,
        est_params=est_params,
        use_acceleration_feedforward=True,
    )

    expected_specific_thrust = np.array([1.0, 0.0, 10.31])
    expected_throttle = np.linalg.norm(expected_specific_thrust) / 44.0
    assert np.isclose(solver.values[(0, "yref")][11], expected_throttle)
    assert np.isclose(solver.values[(2, "yref")][11], expected_throttle)
    q_ref = solver.values[(0, "p")][7:11]
    np.testing.assert_allclose(
        quaternion_to_rotation_matrix(q_ref)[:, 2],
        expected_specific_thrust / np.linalg.norm(expected_specific_thrust),
        atol=1e-12,
    )

    legacy_solver = FakeSolver()
    acados.set_payload_trajectory_reference_aligned(
        legacy_solver,
        traj,
        N_horizon=2,
        step_counter=0,
        skip_steps=1,
        est_params=est_params,
        use_acceleration_feedforward=False,
    )
    assert legacy_solver.values[(0, "yref")][11] == 0.0
    np.testing.assert_allclose(legacy_solver.values[(0, "p")][7:11], [1.0, 0.0, 0.0, 0.0])

def test_acados_reference_setter_keeps_feedforward_external_only_contract():
    source = (
        Path(__file__).resolve().parents[1]
        / "tejen_mpc"
        / "acados.py"
    ).read_text(encoding="utf-8")
    assert "use_acceleration_feedforward" in source
    assert "acceleration_feedforward_reference" in source
    assert "yref[11] = feedforward_throttles[j]" in source
    assert "yref_N[11] = feedforward_throttles[-1]" in source

    main_source = (
        Path(__file__).resolve().parents[1]
        / "tejen_mpc"
        / "main.py"
    ).read_text(encoding="utf-8")
    assert "external_acceleration_feedforward_enabled" in main_source
    assert "using_external_reference" in main_source
    assert "use_acceleration_feedforward=" in main_source
