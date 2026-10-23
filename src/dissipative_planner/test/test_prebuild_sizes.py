"""Every fleet size a flight can be resized to is prebuilt (params.prebuild_sizes, card 2026-10-23_scalability):
today's sets for every fleet of up to four (with or without a reserved newcomer, min_survivors 2 or 3), 2..8 for
the 8-quad chain with min_survivors 2 and 3..8 with 3, and the detach node builds that set."""
import pathlib

import pytest

from mpc_planner.params import prebuild_sizes


def _before(n, r):
    """dissipative_node's sizes until 22 Oct, with the node's own size that prebuild_solvers adds."""
    return sorted({n} | set(range(max(2, n - r - 2), n + r + 1)))


@pytest.mark.parametrize('n', [2, 3, 4])
@pytest.mark.parametrize('reserved', [0, 1])
@pytest.mark.parametrize('min_survivors', [2, 3])
def test_fleets_up_to_four_build_what_they_built(n, reserved, min_survivors):
    assert prebuild_sizes(n, reserved, min_survivors) == _before(n, reserved)


def test_the_8_quad_chain_reaches_its_survivors():
    assert prebuild_sizes(8, 0, 2) == list(range(2, 9))
    assert prebuild_sizes(8, 0, 3) == list(range(3, 9))
    assert prebuild_sizes(6, 0, 2) == list(range(2, 7))
    assert prebuild_sizes(9, 0, 2) == list(range(2, 10))


def test_the_node_builds_that_set():
    src = (pathlib.Path(__file__).parents[1] / 'dissipative_planner' / 'dissipative_node.py').read_text()
    assert 'prebuild_solvers(prebuild_sizes(self.n, self.reserved_attach, self._min_survivors))' in src
