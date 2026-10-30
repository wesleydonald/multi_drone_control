"""
mpc_profile (ours, 14 Oct): his MPC version.

oct10 is his 11a0683f controller exactly; oct14 is his d00934c1 weights, input bounds and
partial-step send path. The two never share a generated solver.
"""
import ast
import importlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
from tejen_mpc import acados_cache
from tejen_mpc.mpc_modes import (
    base_payload_cost_matrices,
    MPC_PROFILES,
    parse_mpc_profile,
    payload_input_rate_bounds,
)

SRC = Path(__file__).resolve().parents[1] / 'tejen_mpc'

# his 11a0683f (the controller flown to 14 Oct), integral weights 15 / 30
OCT10_W = [130.0, 130.0, 40.0, 2.0, 2.0, 2.0, 0.2, 0.2, 0.2, 2e-4, 2e-4, 2e-4, 2e-4,
           0.1, 0.1, 5.0, 0.1, 0.5, 0.5, 5.0, 20.0, 20.0, 2.0, 2.0, 15.0, 15.0]
OCT10_WE = [200.0, 200.0, 60.0, 2.0, 2.0, 2.0, 0.2, 0.2, 0.2, 2e-4, 2e-4, 2e-4, 2e-4,
            0.5, 0.5, 5.0, 20.0, 20.0, 2.0, 2.0, 30.0, 30.0]
# his d00934c1
OCT14_W = [200.0, 200.0, 80.0, 3.0, 3.0, 4.0, 0.2, 0.2, 0.2, 2e-4, 2e-4, 2e-4, 2e-4,
           0.1, 0.1, 5.0, 0.1, 0.5, 0.5, 5.0, 15.0, 15.0, 4.0, 4.0, 15.0, 15.0]
OCT14_WE = [240.0, 240.0, 120.0, 3.0, 3.0, 4.0, 0.2, 0.2, 0.2, 2e-4, 2e-4, 2e-4, 2e-4,
            0.5, 0.5, 5.0, 20.0, 20.0, 4.0, 4.0, 30.0, 30.0]


def test_oct10_is_the_old_controller_and_the_default():
    for W, W_e in (base_payload_cost_matrices(15.0, 30.0),
                   base_payload_cost_matrices(15.0, 30.0, 'oct10')):
        np.testing.assert_array_equal(W, np.diag(OCT10_W))
        np.testing.assert_array_equal(W_e, np.diag(OCT10_WE))
    for lbu, ubu in (payload_input_rate_bounds(), payload_input_rate_bounds('oct10')):
        np.testing.assert_array_equal(lbu, [-1.0, -1.0, -0.5, -1.0])
        np.testing.assert_array_equal(ubu, [1.0, 1.0, 0.5, 1.0])


def test_oct14_is_his_d00934c1_controller():
    W, W_e = base_payload_cost_matrices(15.0, 30.0, 'oct14')
    np.testing.assert_array_equal(W, np.diag(OCT14_W))
    np.testing.assert_array_equal(W_e, np.diag(OCT14_WE))
    lbu, ubu = payload_input_rate_bounds('oct14')
    np.testing.assert_array_equal(lbu, [-3.0, -3.0, -0.5, -3.0])
    np.testing.assert_array_equal(ubu, [3.0, 3.0, 0.5, 3.0])


def test_blanks_around_a_profile_are_ignored():
    assert parse_mpc_profile(' oct14 ') == 'oct14'
    assert MPC_PROFILES == ('oct10', 'oct14')


@pytest.mark.parametrize('bad', ['', 'oct12', 'OCT14', 'd00934c1', None, 14])
def test_unknown_profile_is_refused_everywhere(bad):
    for call in (parse_mpc_profile, payload_input_rate_bounds,
                 lambda p: base_payload_cost_matrices(15.0, 30.0, p)):
        with pytest.raises(ValueError, match='mpc_profile'):
            call(bad)


def _identity(tmp_path, profile):
    srcs = []
    for name in ('acados.py', 'dynamics.py', 'mpc_modes.py'):
        path = tmp_path / name
        if not path.exists():
            path.write_text(name, encoding='utf-8')
        srcs.append(path)
    return acados_cache.build_payload_mpc_cache_identity(
        source_paths=srcs, xy_integral_weight=15.0, xy_integral_terminal_weight=30.0,
        mpc_profile=profile)


def test_profiles_have_their_own_fingerprint_and_cache_entry(tmp_path):
    a, b = _identity(tmp_path, 'oct10'), _identity(tmp_path, 'oct14')
    assert a.fingerprint != b.fingerprint
    assert a.manifest_payload['mpc_profile'] == 'oct10'
    assert b.manifest_payload['mpc_profile'] == 'oct14'
    la = acados_cache.payload_mpc_cache_layout(tmp_path / 'cache', a.fingerprint)
    lb = acados_cache.payload_mpc_cache_layout(tmp_path / 'cache', b.fingerprint)
    assert la.root != lb.root
    # a complete oct10 entry is reusable for oct10 only, even read through oct14's identity
    la.code_export_dir.mkdir(parents=True)
    for path in (la.ocp_json, la.sim_json, la.ocp_shared_library, la.sim_shared_library):
        path.write_bytes(b'x')
    acados_cache.write_payload_mpc_cache_manifest(la, a)
    assert acados_cache.payload_mpc_cache_is_reusable(la, a)
    assert not acados_cache.payload_mpc_cache_is_reusable(la, b)
    assert not acados_cache.payload_mpc_cache_is_reusable(lb, b)


def test_the_builder_caches_each_profile_apart_and_reuses_only_its_own(tmp_path, monkeypatch):
    """Run the real OCP builder with fake acados solvers: no C is compiled."""
    # a fresh import on the real acados_template (other tests import this module on a stub)
    monkeypatch.delitem(sys.modules, 'tejen_mpc.acados', raising=False)
    acados = importlib.import_module('tejen_mpc.acados')
    built = []

    class FakeSolver:

        def __init__(self, ocp, json_file=None, generate=True, build=True):
            built.append(SimpleNamespace(ocp=ocp, cwd=Path.cwd(), json_file=json_file,
                                         generate=generate, build=build))
            if generate:
                Path(json_file).write_text('{}', encoding='utf-8')
                lib = ('ocp_solver_quad_payload_integral_dynamics' if 'ocp' in json_file
                       else 'sim_solver_quad_payload_dynamics_sim')
                Path('c_generated_code').mkdir(exist_ok=True)
                Path(f'c_generated_code/libacados_{lib}.so').write_bytes(b'so')

    monkeypatch.setattr(acados, 'AcadosOcpSolver', FakeSolver)
    monkeypatch.setattr(acados, 'AcadosSimSolver', FakeSolver)
    monkeypatch.delenv('PAYLOAD_MPC_REBUILD', raising=False)
    cache = tmp_path / 'cache'
    roots = {}
    for profile in ('oct10', 'oct14', 'oct10', 'oct14'):
        built.clear()
        acados.generate_payload_ocp_controller(
            15.0, 30.0, acados_cache_dir=str(cache), mpc_profile=profile)
        ocp_call = built[0]
        first = profile not in roots
        roots.setdefault(profile, ocp_call.cwd)
        assert ocp_call.cwd == roots[profile]
        assert (ocp_call.generate, ocp_call.build) == (first, first)
        lbu, ubu = payload_input_rate_bounds(profile)
        np.testing.assert_array_equal(ocp_call.ocp.constraints.lbu, lbu)
        np.testing.assert_array_equal(ocp_call.ocp.constraints.ubu, ubu)
        W, W_e = base_payload_cost_matrices(15.0, 30.0, profile)
        np.testing.assert_array_equal(ocp_call.ocp.cost.W, W)
        np.testing.assert_array_equal(ocp_call.ocp.cost.W_e, W_e)
        manifest = json.loads((ocp_call.cwd / 'cache_manifest.json').read_text('utf-8'))
        assert manifest['mpc_profile'] == profile
    assert roots['oct10'] != roots['oct14']
    assert sorted(p.name for p in cache.iterdir()) == sorted(r.name for r in roots.values())
    with pytest.raises(ValueError, match='mpc_profile'):
        acados.generate_payload_ocp_controller(
            15.0, 30.0, acados_cache_dir=str(cache), mpc_profile='oct12')


def _startup_block():
    """Return Controller.__init__ from the mpc_profile declaration up to the solver call."""
    tree = ast.parse((SRC / 'main.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Controller')
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    start = next(i for i, n in enumerate(init.body)
                 if isinstance(n, ast.Try) and 'mpc_profile' in ast.unparse(n))
    stop = next(i for i, n in enumerate(init.body)
                if 'generate_payload_ocp_controller' in ast.unparse(n))
    return compile(ast.Module(body=init.body[start:stop], type_ignores=[]), 'main.py', 'exec')


def _start(profile, logs, **extra):
    logger = SimpleNamespace(**{lvl: (lambda m, lvl=lvl: logs.append((lvl, m)))
                                for lvl in ('info', 'warn', 'fatal')})
    params = ({} if profile is None else {'mpc_profile': profile}) | extra
    node = SimpleNamespace(
        declare_parameter=lambda name, default: SimpleNamespace(value=params.get(name, default)),
        get_logger=lambda: logger)
    exec(_startup_block(), {'self': node, 'parse_mpc_profile': parse_mpc_profile})
    return node


def test_startup_declares_oct10_by_default_and_says_which_profile_is_live():
    logs = []
    node = _start(None, logs)
    assert (node.mpc_profile, node.mpc_partial_step_execution) == ('oct10', False)
    assert [lvl for lvl, _ in logs] == ['info'] and 'oct10' in logs[0][1]
    logs = []
    node = _start('oct14', logs)
    assert (node.mpc_profile, node.mpc_partial_step_execution) == ('oct14', True)
    assert [lvl for lvl, _ in logs] == ['warn'] and 'oct14' in logs[0][1]


@pytest.mark.parametrize('bad', ['oct12', 'banana', ''])
def test_startup_refuses_an_unknown_profile_loudly(bad):
    logs = []
    with pytest.raises(ValueError, match='mpc_profile'):
        _start(bad, logs)
    assert len(logs) == 1 and logs[0][0] == 'fatal' and repr(bad) in logs[0][1]


def test_the_solver_and_the_mode_switch_weights_come_from_the_same_profile():
    text = (SRC / 'main.py').read_text()
    assert "declare_parameter('mpc_profile', 'oct10')" in text
    assert text.count('mpc_profile=self.mpc_profile') == 2      # the OCP and the mode matrices
    acados_text = (SRC / 'acados.py').read_text()
    assert acados_text.count('mpc_profile=mpc_profile') == 2    # the weights and the cache key


def test_command_execution_follows_the_profile_unless_typed():
    """mpc_command_execution (29 Oct, Tejen's review): 'partial_step' under oct10 sends the elapsed-interval command
    with the oct10 weights and bounds; untyped, each profile keeps its own send path and its one log line."""
    logs = []
    node = _start('oct10', logs, mpc_command_execution='partial_step')
    assert (node.mpc_profile, node.mpc_partial_step_execution) == ('oct10', True)
    assert [lvl for lvl, _ in logs] == ['info', 'warn'] and 'partial_step' in logs[1][1]
    logs = []
    node = _start('oct14', logs, mpc_command_execution='endpoint')
    assert (node.mpc_profile, node.mpc_partial_step_execution) == ('oct14', False)
    logs = []
    node = _start('oct10', logs, mpc_command_execution='profile')
    assert node.mpc_partial_step_execution is False and [lvl for lvl, _ in logs] == ['info']


def test_an_unknown_command_execution_is_refused_loudly():
    logs = []
    with pytest.raises(ValueError, match='mpc_command_execution'):
        _start('oct10', logs, mpc_command_execution='elapsed')
    assert logs[-1][0] == 'fatal'
