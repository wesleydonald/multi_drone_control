"""tools/prebuild_planner.py --from-config (card 2026-10-23_scalability): it builds what one experiment config's planner
loads, resolved as the launch does: every resize size of a detach-mode OCP flight down to its min_survivors, with that
flight's masses and ring inertia; one size for a planner that never resizes. Nothing is compiled here."""
import os
import sys

import pytest

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(TOOLS)
sys.path.insert(0, TOOLS)
pytest.importorskip('acados_template')
import prebuild_planner as P  # noqa: E402
from mpc_planner.params import load_inertia  # noqa: E402

EXP = os.path.join(REPO, 'configs', 'experiments')


def test_the_8_quad_chain_builds_down_to_its_pair_with_its_ring():
    sizes, opts = P.config_builds(os.path.join(EXP, 's8_chain4_R50_L100_it1.yaml'))
    assert sizes == list(range(2, 9))
    assert opts['load_mass'] == 0.9 and opts['drone_mass'] == 0.55 and opts['cable_len'] == 1.0
    assert opts['inertia'] == [0.113025, 0.113025, 0.22581] and not opts['pin_rates']


def test_a_4_quad_baseline_builds_its_resize_sizes_with_the_per_kg_ring():
    sizes, opts = P.config_builds(os.path.join(EXP, 'b4_hover_L100_ema.yaml'))
    assert sizes == [2, 3, 4] and opts['inertia'] == load_inertia(0.86)


def test_a_planner_that_never_resizes_builds_its_own_size():
    sizes, _ = P.config_builds(os.path.join(EXP, 'rig_twin_hover_fixed.yaml'))
    assert sizes == [4]
