from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ACADOS = ROOT / 'tejen_mpc' / 'tejen_mpc' / 'acados.py'
MAIN = ROOT / 'tejen_mpc' / 'tejen_mpc' / 'main.py'


class FakeSolver:
    def __init__(self):
        self.values = {}

    def set(self, stage, field, value):
        self.values[(stage, field)] = np.asarray(value, dtype=float).copy()


def _trajectory(samples=61):
    traj = np.zeros((17, samples), dtype=float)
    traj[2, :] = 1.0
    traj[3, :] = 1.0
    return traj


def test_reference_plumbing_appends_same_lateral_disturbance_to_every_ocp_stage(monkeypatch):
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

    solver = FakeSolver()
    disturbance = np.array([-0.35, 0.12])
    acados.set_payload_trajectory_reference_aligned(
        solver,
        _trajectory(),
        N_horizon=20,
        step_counter=0,
        skip_steps=3,
        est_params=np.array([22.0, 0.0, 0.12, 100.0, 100.0, 0.5, 0.531]),
        lateral_disturbance_xy=disturbance,
    )
    for stage in range(21):
        params = solver.values[(stage, 'p')]
        assert params.shape == (15,)
        assert np.allclose(params[-2:], disturbance)


def test_disturbance_is_injected_only_into_quad_xy_translation_in_first_version():
    text = ACADOS.read_text(encoding='utf-8')
    assert "p_lateral_disturbance = ca.MX.sym('p_lateral_disturbance', 2)" in text
    assert 'base_dynamics[7:9] + model.p[13:15]' in text
    assert 'base_dynamics[9:21]' in text


def test_legacy_integral_and_disturbance_control_paths_are_mutually_exclusive():
    text = MAIN.read_text(encoding='utf-8')
    assert "self.enable_xy_integral_action = self.xy_bias_mode == 'legacy_integral'" in text
    assert "self.xy_bias_mode == 'lateral_disturbance'" in text
    assert "self.xy_bias_mode in (\n            'lateral_disturbance',\n            'lateral_disturbance_shadow',\n        )" in text
    assert 'if self.enable_lateral_disturbance_compensation' in text
