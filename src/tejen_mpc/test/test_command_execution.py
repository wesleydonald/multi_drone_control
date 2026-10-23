"""
Minimal numerical and real-adapter regressions for partial command execution.

Ported from his d2eaf41e (controller_mpc_payload/test/test_command_execution.py). Our
fork carries it behind mpc_profile: oct14 is his partial-step send path, oct10 the full
first-stage endpoint as before.
"""
import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from tejen_mpc.command_execution import command_at_elapsed_time
from tejen_mpc.mpc_modes import payload_input_rate_bounds

MAIN = Path(__file__).resolve().parents[1] / "tejen_mpc/main.py"
ACADOS = Path(__file__).resolve().parents[1] / "tejen_mpc/acados.py"


def test_model_attitude_slew_is_three_without_changing_throttle_slew():
    lbu, ubu = payload_input_rate_bounds("oct14")
    np.testing.assert_array_equal(lbu, [-3., -3., -.5, -3.])
    np.testing.assert_array_equal(ubu, [3., 3., .5, 3.])
    # the OCP takes its input bounds from the profile, nowhere else
    tree = ast.parse(ACADOS.read_text())
    assigns = [ast.unparse(n) for n in ast.walk(tree) if isinstance(n, ast.Assign)
               and any("ocp.constraints.lbu" in ast.unparse(t) or "ocp.constraints.ubu" in ast.unparse(t)
                       for t in n.targets)]
    assert assigns == ["(ocp.constraints.lbu, ocp.constraints.ubu) = payload_input_rate_bounds(mpc_profile)"]
    initial = np.array([0., 0., .45, 0.])
    rate = ubu
    result = command_at_elapsed_time(initial, initial + .1 * rate, 1 / 30, .1)
    np.testing.assert_allclose(result - initial, [.1, .1, .5 / 30, .1])


def test_command_uses_execution_interval_not_prediction_endpoint():
    initial = np.array([.1, -.2, .45, .05])
    rate = np.array([.8, -.4, .5, -.2])
    endpoint = initial + .1 * rate
    result = command_at_elapsed_time(initial, endpoint, 1 / 30, .1)
    np.testing.assert_allclose(result, initial + rate / 30)
    assert result[2] == pytest.approx(.4666666667)
    np.testing.assert_array_equal(initial, [.1, -.2, .45, .05])


def test_jitter_and_long_gap_preserve_rate_and_endpoint_bounds():
    initial = np.array([0., 0., .45, 0.])
    endpoint = np.array([.08, -.08, .50, .04])
    for elapsed in [0., .020, 1 / 30, .048, .100, .200]:
        result = command_at_elapsed_time(initial, endpoint, elapsed, .1)
        np.testing.assert_allclose(result, initial + min(elapsed / .1, 1.) * (endpoint - initial))
        if elapsed > 0:
            assert abs(result[2] - initial[2]) / elapsed <= .5 + 1e-12


@pytest.mark.parametrize("bad", [
    dict(initial_command=[0., 0., .4], first_stage_command=[0., 0., .4, 0.], elapsed_s=.03, stage_duration_s=.1),
    dict(initial_command=[0., 0., np.nan, 0.], first_stage_command=[0., 0., .4, 0.], elapsed_s=.03,
         stage_duration_s=.1),
    dict(initial_command=[0., 0., .4, 0.], first_stage_command=[0., 0., .4, 0.], elapsed_s=.03,
         stage_duration_s=0.),
    dict(initial_command=[0., 0., .4, 0.], first_stage_command=[0., 0., .4, 0.], elapsed_s=np.inf,
         stage_duration_s=.1),
])
def test_malformed_execution_inputs_raise(bad):
    with pytest.raises(ValueError):
        command_at_elapsed_time(**bad)


def _send_path():
    """Return the control loop from the solved stage-1 state to the plot, and the history append."""
    tree = ast.parse(MAIN.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "Controller")
    loop = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "control_loop")
    body = next(n.body for n in loop.body if isinstance(n, ast.If)
                and ast.unparse(n.test) == "self.armed and self.current_pose is not None")
    start = next(i for i, n in enumerate(body) if isinstance(n, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id == "x" for t in n.targets))
    stop = next(i for i, n in enumerate(body[start:], start)
                if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "mpc_trajectory"
                                                     for t in n.targets))
    history = next(n for n in body if isinstance(n, ast.If)
                   and any(isinstance(c, ast.Attribute) and c.attr == "append"
                           and ast.unparse(c.value) == "self.control_history" for c in ast.walk(n)))
    return compile(ast.Module(body=body[start:stop] + [history], type_ignores=[]), str(MAIN), "exec")


def _node(state, rate, partial, clock):
    published = []
    solver = SimpleNamespace(get=lambda stage, field: state if field == "x" else rate)
    node = SimpleNamespace(ocp=solver, takeoff_requested=True, step_counter=0, mpc_partial_step_execution=partial,
                           command_execution_last_publish_time=None,
                           mpc_command_stage_duration_s=.1 if partial else None,
                           _node_now_s=lambda: clock.now, control_history=[],
                           cb=SimpleNamespace(cmd_publisher_=SimpleNamespace(publish=published.append)))
    return node, published


def _sent(msg):
    return np.array([msg.channel_0, msg.channel_1, (msg.channel_2 + 1) / 2, msg.channel_3])


def test_actual_adapter_publishes_partial_step_and_stores_sent_command():
    code = _send_path()
    initial = np.array([.1, -.2, .45, .05])
    rate = np.array([.8, -.4, .5, -.2])
    state = np.zeros(23)
    state[13:17] = initial + .1 * rate
    clock = SimpleNamespace(now=10.)
    node, published = _node(state, rate, True, clock)
    namespace = {"self": node, "np": np, "DT": 1 / 30,
                 "control_state": initial, "ELRSCommand": lambda **kw: SimpleNamespace(**kw),
                 "command_at_elapsed_time": command_at_elapsed_time}
    exec(code, namespace)
    sent = _sent(published[-1])
    np.testing.assert_allclose(sent, initial + rate / 30, atol=.0005)
    np.testing.assert_array_equal(node.control_history[-1][:4], sent)
    np.testing.assert_array_equal(node.control_history[-1][4:], rate)
    assert node.command_execution_last_publish_time == 10.
    # A jittered second publication starts from the rounded command actually sent.
    namespace["control_state"] = sent
    state[13:17] = sent + .1 * rate
    clock.now = 10.048
    exec(code, namespace)
    second = _sent(published[-1])
    np.testing.assert_allclose(second, sent + .048 * rate, atol=.0005)
    np.testing.assert_array_equal(node.control_history[-1][:4], second)
    assert node.command_execution_last_publish_time == 10.048
    node.takeoff_requested = False
    exec(code, namespace)
    assert published[-1].channel_2 == -1.  # Existing armed-without-takeoff semantics.
    assert node.command_execution_last_publish_time is None
    assert len(node.control_history) == 2


def test_oct10_sends_the_first_stage_endpoint_and_keeps_the_unrounded_history():
    code = _send_path()
    initial = np.array([.1, -.2, .45, .05])
    rate = np.array([.8, -.4, .5, -.2])
    state = np.zeros(23)
    state[13:17] = initial + .1 * rate + 1e-4    # off the 0.001 grid, so rounding shows
    clock = SimpleNamespace(now=10.)
    node, published = _node(state, rate, False, clock)
    namespace = {"self": node, "np": np, "DT": 1 / 30, "control_state": initial,
                 "ELRSCommand": lambda **kw: SimpleNamespace(**kw)}   # no command_at_elapsed_time: never called
    for now in (10., 10.048):
        clock.now = now
        exec(code, namespace)
        msg = published[-1]
        u = state[13:17]
        assert (msg.channel_0, msg.channel_1, msg.channel_2, msg.channel_3) == (
            round(u[0], 3), round(u[1], 3), round((u[2] * 2) - 1, 3), round(u[3], 3))
        assert node.control_history[-1][:4] == list(u)
        assert node.command_execution_last_publish_time is None
