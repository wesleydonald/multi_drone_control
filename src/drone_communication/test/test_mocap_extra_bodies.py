"""Extra Motive bodies on their own topics (combined-stack O3): the combined M1 stack routes his pickup
ball (body 6) to /pickup/motion_capture_state from our mocap node, the only Motive owner. '' (the default)
routes nothing new."""
import pytest

pytest.importorskip('scipy')
from drone_communication.motion_capture_publisher_node import parse_extra_bodies  # noqa: E402


def test_parse():
    assert parse_extra_bodies('') == {} and parse_extra_bodies(None) == {}
    assert parse_extra_bodies('6:/pickup/motion_capture_state') == {6: '/pickup/motion_capture_state'}
    assert parse_extra_bodies(' 6:/a, 7:/b/c ') == {6: '/a', 7: '/b/c'}


@pytest.mark.parametrize('bad', ['6', '6:pickup', 'x:/a', '6:/a,6:/b', ':/a'])
def test_parse_refuses(bad):
    with pytest.raises(ValueError):
        parse_extra_bodies(bad)
