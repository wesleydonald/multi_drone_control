#!/usr/bin/env python3
"""
import_check.py — import every console-script module and report what breaks.

WHY THIS EXISTS
---------------
On 2026-08-04 a missing `from std_msgs.msg import String` shipped into
`controller_mpc.py`. All three tracker processes died instantly at launch with
`NameError: name 'String' is not defined`, raised at *class definition* time.

Neither of the checks used beforehand could see it:

  * `ast.parse()`   — only validates syntax; an undefined name is perfectly valid syntax
  * `colcon build`  — for an ament_python package this copies/symlinks files. It never
                      imports them, so it cannot fail on a NameError

The only thing that catches this class of bug is *importing the module*, which is what
this does. It is fast (a few seconds), needs no Gazebo, no hardware and no mocap, and
belongs in the pre-push gate.

Each module is imported in its own subprocess because some of them have real
side effects at import time: the acados-backed nodes `os.chdir()` into their
generated-code directory and take an exclusive compile lock. Sequential subprocesses
keep those isolated and let the lock be released between them.

    python3 tools/import_check.py            # every package
    python3 tools/import_check.py controller_quad_load
"""
import ast
import pathlib
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
SRC = REPO / 'src'

# Modules that are slow or need hardware attached; skipped by default with a reason.
SKIP = {
    'drone_communication.elrs_interface': 'opens a serial port to the ELRS transmitter',
    'drone_communication.video_interface': 'opens a camera device',
}


def entry_point_modules(setup_py):
    """Parse `console_scripts` out of a setup.py without executing it."""
    try:
        tree = ast.parse(setup_py.read_text())
    except SyntaxError:
        return []
    mods = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Constant) or not isinstance(node.value, str):
            continue
        v = node.value
        if '=' in v and ':' in v and not v.startswith('#'):
            target = v.split('=', 1)[1].strip()
            if ':' in target and target.split(':', 1)[0].count('.') >= 1:
                mods.append(target.split(':', 1)[0].strip())
    return sorted(set(mods))


def main():
    wanted = sys.argv[1:]
    setups = sorted(SRC.glob('*/setup.py'))
    failures, checked, skipped = [], 0, 0

    for setup_py in setups:
        pkg = setup_py.parent.name
        if wanted and pkg not in wanted:
            continue
        mods = entry_point_modules(setup_py)
        if not mods:
            continue
        print(f"── {pkg}")
        for mod in mods:
            if mod in SKIP:
                print(f"   SKIP {mod:52s} ({SKIP[mod]})")
                skipped += 1
                continue
            proc = subprocess.run(
                [sys.executable, '-c', f'import {mod}'],
                capture_output=True, text=True, cwd=str(REPO), timeout=180)
            checked += 1
            if proc.returncode == 0:
                print(f"   ok   {mod}")
            else:
                tail = [l for l in proc.stderr.strip().splitlines() if l.strip()]
                reason = tail[-1] if tail else f'exit {proc.returncode}'
                print(f"   FAIL {mod}\n        {reason}")
                failures.append((mod, reason))

    print()
    print(f"{checked} module(s) imported, {skipped} skipped, {len(failures)} failed")
    if failures:
        print("\nFAILURES:")
        for mod, reason in failures:
            print(f"  {mod}\n    {reason}")
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
