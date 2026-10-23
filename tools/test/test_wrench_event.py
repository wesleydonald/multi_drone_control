"""The runner's WRENCH event (W3 push tool): its argument is checked when the config loads."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from experiment.config import Event, wrench_arg  # noqa: E402


def test_force_and_default_link():
    assert wrench_arg('-1 0 0') == ((-1.0, 0.0, 0.0), (0.0, 0.0, 0.0), 'lift_system::payload::body')


def test_explicit_link():
    assert wrench_arg('0 0.5 0 x3::X3/base_link')[2] == 'x3::X3/base_link'


def test_torque():
    assert wrench_arg('0 0 0 0.3 0 0 x3::X3/base_link') == ((0.0, 0.0, 0.0), (0.3, 0.0, 0.0), 'x3::X3/base_link')


@pytest.mark.parametrize('bad', [None, '', '1 0', '1 0 0 a b'])
def test_bad_arg_refused_at_load(bad):
    with pytest.raises(ValueError):
        Event(10.0, 'WRENCH', bad)
