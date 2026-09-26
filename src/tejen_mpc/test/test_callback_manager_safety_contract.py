"""Small source-level regressions for CallbackManager safety paths."""

import ast
from pathlib import Path


def test_arm_without_pose_logs_through_owner_node():
    source_path = (
        Path(__file__).resolve().parents[2]
        / 'tejen_utility_objects' / 'tejen_utility_objects' / 'callback_manager.py'
    )
    tree = ast.parse(source_path.read_text())
    manager = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'CallbackManager'
    )
    command = next(
        node for node in manager.body
        if isinstance(node, ast.FunctionDef) and node.name == 'command_callback'
    )

    logger_calls = [
        node for node in ast.walk(command)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr in {'warn', 'warning'}
    ]
    assert logger_calls
    for call in logger_calls:
        get_logger_call = call.func.value
        assert isinstance(get_logger_call, ast.Call)
        assert isinstance(get_logger_call.func, ast.Attribute)
        assert get_logger_call.func.attr == 'get_logger'
        assert isinstance(get_logger_call.func.value, ast.Attribute)
        assert get_logger_call.func.value.attr == 'node'
