"""tip_skip_repeats (docs/analysis/attach_gates_2026-10-30.md): Motive repeats the magnet marker's last pose while it
has lost it (RIG-1029: 13-34 % of body 24's samples); with the switch on, a repeat is not a tip sample, so it never
reaches the proof (whose 0.1 s gap rule then restarts its dwells across the hold). Off by default: every pose counts."""
import numpy as np
import pytest

pytest.importorskip('interfaces.msg')
from geometry_msgs.msg import PoseStamped  # noqa: E402

from test_magnet_contact_lift_node import manager  # noqa: E402,F401


def _pose(xyz):
    m = PoseStamped()
    m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in xyz)
    m.pose.orientation.w = 1.0
    return m


def _count_samples(node):
    n = {'k': 0}
    node._proof_sample = lambda: n.__setitem__('k', n['k'] + 1)
    node._proof = object()
    return n


@pytest.mark.parametrize('skip, expected', [(False, 4), (True, 2)])
def test_a_repeated_tip_pose_is_a_sample_only_with_the_switch_off(manager, skip, expected):  # noqa: F811
    node = manager(**({'tip_skip_repeats': True} if skip else {}))
    n = _count_samples(node)
    for xyz in ([0.0, 0.2, 1.1], [0.0, 0.2, 1.1], [0.0, 0.2, 1.1], [0.001, 0.2, 1.1]):
        node.magnet_tip_pose_stamped_callback(_pose(xyz))
    assert n['k'] == expected
    assert np.allclose(node.magnet_tip_position, [0.001, 0.2, 1.1])
