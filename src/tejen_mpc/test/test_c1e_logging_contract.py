"""C1E regression checks for controller logging and console-noise cleanup."""

import ast
from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parents[1]
    / 'tejen_mpc'
    / 'main.py'
)


def _controller_tree():
    tree = ast.parse(SOURCE.read_text())
    return next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'Controller'
    )


def test_log_header_and_row_lengths_match_and_dynamic_state_is_present():
    controller = _controller_tree()
    headers = None
    row = None
    for node in ast.walk(controller):
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id == 'log_headers':
                headers = [ast.literal_eval(item) for item in node.value.elts]
            elif isinstance(target, ast.Name) and target.id == 'log_row':
                row = node.value.elts

    assert headers is not None
    assert row is not None
    assert len(headers) == len(row)

    for field in (
        'pose_vx', 'pose_vy', 'pose_vz',
        'pose_wx', 'pose_wy', 'pose_wz',
        'ref_x', 'ref_y', 'ref_z',
        'ref_vx', 'ref_vy', 'ref_vz',
        'ref_ax', 'ref_ay', 'ref_az',
    ):
        assert field in headers

    for run_constant in (
        'ukf_enabled', 'ukf_backend', 'ukf_effective_rate_hz',
        'ukf_control_delay_steps', 'thrust_ratio_feedback_enabled',
        'xy_integral_enabled', 'xy_integral_weight',
        'xy_integral_terminal_weight', 'xy_integral_limit',
    ):
        assert run_constant not in headers


def test_control_loop_has_no_high_rate_console_print_or_info_calls():
    controller = _controller_tree()
    control_loop = next(
        node for node in controller.body
        if isinstance(node, ast.FunctionDef) and node.name == 'control_loop'
    )

    for node in ast.walk(control_loop):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            assert node.func.id != 'print'
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr != 'info':
            continue
        # Any future INFO inside the 30 Hz loop must be explicitly throttled.
        assert any(
            keyword.arg == 'throttle_duration_sec'
            for keyword in node.keywords
        )
