"""4 -> 2 -> 4 (the contract of 13 Nov): a second magnet tip body (QUAD3's marker 23) published as a PoseStamped on
/drone_2/magnet_tip_pose, as QUAD4's tip on /magnet_tip_pose. '' (the default) adds nothing. The node is built on an
isolated ROS domain with its UDP socket replaced, and fed packets."""
import pytest

pytest.importorskip('scipy')
pytest.importorskip('interfaces.msg')
from drone_communication import motion_capture_publisher_node as mc  # noqa: E402

DOMAIN = 96


def test_parse():
    assert mc.parse_tip_bodies('') == {} and mc.parse_tip_bodies(None) == {}
    assert mc.parse_tip_bodies('23:/drone_2/magnet_tip_pose') == {23: '/drone_2/magnet_tip_pose'}
    assert mc.parse_tip_bodies('23:/a, 22:/b') == {23: '/a', 22: '/b'}


@pytest.mark.parametrize('bad', ['23', '23:a', 'x:/a', '23:/a,23:/b', '23:/a,22:/a'])
def test_parse_refuses(bad):
    with pytest.raises(ValueError):
        mc.parse_tip_bodies(bad)


class _Sock:
    def __init__(self, packets):
        self.packets = list(packets)

    def bind(self, _addr):
        pass

    def recvfrom(self, _n):
        if not self.packets:
            raise KeyboardInterrupt
        return self.packets.pop(0).encode(), None

    def close(self):
        pass


class _Rec:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


@pytest.fixture
def node(monkeypatch, request):
    import rclpy
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    sock = _Sock(request.param.pop('_packets', []))
    monkeypatch.setattr(mc.socket, 'socket', lambda *a, **k: sock)
    args = ['--ros-args']
    for k, v in request.param.items():
        args += ['-p', f'{k}:={v}']
    rclpy.init(args=args, domain_id=DOMAIN)
    n = mc.MotionCapturePublisher()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _pubs(n):
    return {p.topic_name for p in n.publishers}


@pytest.mark.parametrize('node', [{'magnet_tip_body_id': 24}], indirect=True)
def test_default_publishes_todays_one_tip(node):
    assert node.tip_bodies == {} and '/magnet_tip_pose' in _pubs(node)
    assert not [t for t in _pubs(node) if t.endswith('/magnet_tip_pose') and t != '/magnet_tip_pose']


@pytest.mark.parametrize('node', [{
    'magnet_tip_body_id': 24, 'tip_bodies': '23:/drone_2/magnet_tip_pose',
    '_packets': ['23-(0.1,-0.2,1.3)-(0.0,0.0,0.0,1.0)', '24-(0.0,0.2,1.3)-(0.0,0.0,0.0,1.0)',
                 '13-(0.5,0.0,1.9)-(0.0,0.0,0.0,1.0)']}], indirect=True)
def test_a_second_tip_goes_out_as_its_own_pose(node):
    assert {'/magnet_tip_pose', '/drone_2/magnet_tip_pose'} <= _pubs(node)
    rec2, rec3 = _Rec(), _Rec()
    node.tip_publishers[23] = rec2
    node.magnet_tip_publisher = rec3
    node.run()
    assert node.tip_bodies == {23: '/drone_2/magnet_tip_pose'}
    assert len(rec2.msgs) == 1 and len(rec3.msgs) == 1
    for rec, y in ((rec2, -0.2), (rec3, 0.2)):
        for m in rec.msgs:
            assert m.pose.position.y == pytest.approx(y) and m.header.frame_id == 'map'


@pytest.mark.parametrize('params', [{'magnet_tip_body_id': 24, 'tip_bodies': '24:/drone_2/magnet_tip_pose'},
                                    {'tip_bodies': '13:/drone_2/magnet_tip_pose'},
                                    {'magnet_tip_body_id': 24, 'tip_bodies': '23:/magnet_tip_pose'},
                                    {'extra_bodies': '23:/pickup/x', 'tip_bodies': '23:/drone_2/magnet_tip_pose'}])
def test_a_tip_body_routed_twice_is_refused(monkeypatch, params):
    import rclpy
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setattr(mc.socket, 'socket', lambda *a, **k: _Sock([]))
    args = ['--ros-args']
    for k, v in params.items():
        args += ['-p', f'{k}:={v}']
    rclpy.init(args=args, domain_id=DOMAIN)
    try:
        with pytest.raises(ValueError):
            mc.MotionCapturePublisher()
    finally:
        rclpy.shutdown()
