"""The planner's runtime weight multipliers (card 2026-10-16_corner_cutting) touch only the load-position and
cable-direction blocks, on every stage and the terminal."""
from types import SimpleNamespace

import numpy as np

from mpc_planner.planner_solver import PlannerSolver


def test_scale_cost_scales_position_and_cable_direction_only():
    n, N = 3, 4
    ny, ny_e = 12 + 3 * n + n + 3 * n + 4 * n, 12 + 3 * n + n + 3 * n
    sets = []
    fake = SimpleNamespace(dyn=SimpleNamespace(n=n), N=N,
                           ocp=SimpleNamespace(cost=SimpleNamespace(W=np.eye(ny), W_e=np.eye(ny_e))),
                           solver=SimpleNamespace(cost_set=lambda k, f, W: sets.append((k, f, np.array(W)))))
    PlannerSolver.scale_cost(fake, pos=3.0, cable_dir=0.5)
    assert [k for k, _, _ in sets] == list(range(N + 1)) and all(f == 'W' for _, f, _ in sets)
    d = np.diag(sets[0][2])
    assert np.allclose(d[0:3], 3.0) and np.allclose(d[3:12], 1.0)
    assert np.allclose(d[12:12 + 3 * n], 0.5) and np.allclose(d[12 + 3 * n:], 1.0)
    assert sets[-1][2].shape == (ny_e, ny_e) and np.allclose(np.diag(sets[-1][2])[0:3], 3.0)
    assert np.allclose(fake.ocp.cost.W, np.eye(ny))         # the stored weights are not changed
