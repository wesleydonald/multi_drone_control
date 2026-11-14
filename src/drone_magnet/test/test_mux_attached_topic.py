"""4 -> 2 -> 4 (the contract of 13 Nov): each partner's mux listens to its own partner's weld on attached_topic, so
one partner's proven contact never hands the other to our tracker. Default: today's shared /magnet/object_attached.
Isolated ROS domain; callbacks called directly."""
import pytest

pytest.importorskip('interfaces.msg')
import rclpy  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from interfaces.msg import ELRSCommand  # noqa: E402

from drone_magnet.elrs_mux import ElrsMux  # noqa: E402

DOMAIN = 95


class _Rec:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


@pytest.fixture
def mux(monkeypatch, request):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    args = ['--ros-args']
    for k, v in request.param.items():
        args += ['-p', f'{k}:={str(v).lower() if isinstance(v, bool) else v}']
    rclpy.init(args=args, domain_id=DOMAIN)
    n = ElrsMux()
    n.pub = _Rec()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _subs(n):
    return {s.topic_name for s in n.subscriptions}


@pytest.mark.parametrize('mux', [{'drone_id': 3}], indirect=True)
def test_default_is_todays_shared_weld(mux):
    assert '/magnet/object_attached' in _subs(mux)
    assert {'/drone_3/ELRSCommand_tejen', '/drone_3/ELRSCommand_diss'} <= _subs(mux)


@pytest.mark.parametrize('mux', [{'drone_id': 2, 'attached_topic': '/drone_2/magnet/object_attached',
                                  'release_topic': '/drone_2/partner/release',
                                  'take_back_topic': '/drone_2/take_back'}], indirect=True)
def test_a_partner_listens_to_its_own_weld_only(mux):
    subs = _subs(mux)
    assert '/drone_2/magnet/object_attached' in subs and '/magnet/object_attached' not in subs
    assert {'/drone_2/ELRSCommand_tejen', '/drone_2/ELRSCommand_diss', '/drone_2/partner/release',
            '/drone_2/take_back'} <= subs
    assert {p.topic_name for p in mux.publishers} >= {'/drone_2/mux_state'}
    # its own weld hands it over as today's did
    mux._tejen_cb(ELRSCommand(armed=True, channel_2=0.3))
    mux._attached_cb(Bool(data=True))
    mux._diss_cb(ELRSCommand(armed=True, channel_2=0.4))
    assert mux._state == 'ours' and mux.pub.msgs[-1].channel_2 == pytest.approx(0.4)
