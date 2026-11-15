"""The trial line's resolved node graph (tools/launch_dump.py, with environment actions executed, not refused)."""
import json
import sys

sys.path.insert(0, '/home/wesley/multi_drone_control/tools')
import launch_dump  # noqa: E402

_orig = launch_dump._collect


def _collect(entities, ctx, out):
    from launch.actions import (ResetEnvironment, ResetLaunchConfigurations, SetEnvironmentVariable,
                                UnsetEnvironmentVariable, AppendEnvironmentVariable)
    keep = []
    for e in entities:
        if isinstance(e, (ResetEnvironment, ResetLaunchConfigurations, SetEnvironmentVariable,
                          UnsetEnvironmentVariable, AppendEnvironmentVariable)):
            if getattr(e, 'condition', None) is None or e.condition.evaluate(ctx):
                e.execute(ctx)
            continue
        keep.append(e)
    _orig(keep, ctx, out)


launch_dump._collect = _collect
if __name__ == '__main__':
    launch_dump.main(sys.argv[1:])
