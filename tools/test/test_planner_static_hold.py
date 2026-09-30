"""The load planner holding a still, level ring with its rods at the rig's hand-over
elevations (53.6 deg on model-f1, 65 deg the steepest seen) against its 45 deg references:
every solve must succeed. Prints the node-0 vertical rod pull over the ring's weight.

Runs tools/planner_replay.py --static-hold in a subprocess (it builds the acados solver in
a scratch directory, ~40 s the first time). MDC_REPLAY_SRC picks another controller_load_mpc."""
import json
import os
import subprocess
import sys

import pytest

pytest.importorskip('acados_template')
TOOL = os.path.join(os.path.dirname(__file__), '..', 'planner_replay.py')


@pytest.fixture(scope='module')
def holds():
    cmd = [sys.executable, TOOL, '--static-hold', '53.6,65']
    if os.environ.get('MDC_REPLAY_SRC'):
        cmd += ['--src', os.environ['MDC_REPLAY_SRC']]
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    assert out.returncode == 0, out.stderr[-2000:]
    rows = [json.loads(line) for line in out.stdout.splitlines() if line.startswith('{')]
    return {r['elev_deg']: r for r in rows}


@pytest.mark.parametrize('elev', [53.6, 65.0])
def test_static_hold_solves(holds, elev):
    r = holds[elev]
    print(f"\nrods at {elev} deg: node-0 sum(t*s_z)/weight = {r['ratio']:.3f}")
    assert len(r['status']) == 50 and not any(r['status']), r['status']
    assert 0.8 < r['ratio'] < 1.3
