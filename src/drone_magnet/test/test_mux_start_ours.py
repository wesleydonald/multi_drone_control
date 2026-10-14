"""The full M1 mux (QUAD4 welded on plate 3 from spawn, partner_contact): ours from boot (combined-stack O1,
branch combined-m1 8c21fe8), his stream only after /partner/release on its first flying command, then the
contact attach or our take-back hands it back to ours. A take-back before his stream took over cancels the
release. start_ours false (the default) keeps today's mux. Isolated ROS domain; callbacks called directly."""
import pytest

pytest.importorskip('interfaces.msg')
import rclpy  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from interfaces.msg import ELRSCommand  # noqa: E402

from drone_magnet.elrs_mux import ElrsMux  # noqa: E402

DOMAIN = 94
RELEASE = String(data='{"running": true}')


class _Rec:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


def _cmd(thr, armed=True):
    return ELRSCommand(armed=armed, channel_0=0.1, channel_1=-0.1, channel_2=thr, channel_3=0.0)


@pytest.fixture
def mux(monkeypatch, request):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    params = {'drone_id': 3, 'release_topic': '/partner/release', 'take_back_topic': '/drone_3/take_back',
              **request.param}           # handoff_topic '' (the default): not wired under partner_contact
    args = ['--ros-args']
    for k, v in params.items():
        args += ['-p', f'{k}:={str(v).lower() if isinstance(v, bool) else v}']
    rclpy.init(args=args, domain_id=DOMAIN)
    n = ElrsMux()
    n.pub = _Rec()
    n.states = n._state_pub = _Rec()
    n.states.msgs.append(String(data=n._state))     # the one sent in __init__
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _states(n):
    return [m.data for m in n.states.msgs]


def _thr(n):
    return [m.channel_2 for m in n.pub.msgs]


@pytest.mark.parametrize('mux', [{'start_ours': True}], indirect=True)
def test_ours_from_boot_his_stream_after_the_release_then_ours_at_the_attach(mux):
    assert _states(mux) == ['ours']
    mux._diss_cb(_cmd(-1.0))                                   # our armed idle on the floor: forwarded
    mux._attached_cb(Bool(data=True))                          # the start weld
    mux._diss_cb(_cmd(0.4))                                    # a carrier in the creep and the lift
    mux._tejen_cb(_cmd(0.9))                                   # nothing of his before the release
    assert _thr(mux) == [-1.0, 0.4]
    mux._attached_cb(Bool(data=False))                         # the detach: still ours (the step-out)
    mux._diss_cb(_cmd(0.35))
    mux._release_cb(RELEASE)
    mux._tejen_cb(_cmd(-1.0))                                  # his idle: not yet
    mux._diss_cb(_cmd(0.36))
    assert _states(mux) == ['ours'] and _thr(mux) == [-1.0, 0.4, 0.35, 0.36]
    mux._tejen_cb(_cmd(0.5))                                   # his first flying command: his from here
    mux._diss_cb(_cmd(0.37))                                   # ours warm, not forwarded
    assert _states(mux) == ['ours', 'partner'] and _thr(mux)[-1] == 0.5
    mux._attached_cb(Bool(data=False))
    mux._tejen_cb(_cmd(0.51))
    assert _states(mux)[-1] == 'partner' and _thr(mux)[-1] == 0.51
    mux._attached_cb(Bool(data=True))                          # the proven contact attach
    mux._diss_cb(_cmd(0.52))                                   # our warm tracker: ours
    mux._tejen_cb(_cmd(0.9))
    assert _states(mux) == ['ours', 'partner', 'ours'] and _thr(mux)[-1] == 0.52
    mux._release_cb(RELEASE)                                   # the planner repeats it at 1 Hz: no second release
    mux._tejen_cb(_cmd(0.9))
    assert _states(mux)[-1] == 'ours' and _thr(mux)[-1] == 0.52


@pytest.mark.parametrize('mux', [{'start_ours': True}], indirect=True)
def test_a_take_back_during_his_flight_hands_it_to_ours(mux):
    mux._release_cb(RELEASE)
    mux._tejen_cb(_cmd(0.5))
    assert _states(mux) == ['ours', 'partner']
    mux._diss_cb(_cmd(0.4))
    mux._take_back_cb(Bool(data=True))                         # his give-up or our LAND
    mux._diss_cb(_cmd(0.41))
    mux._tejen_cb(_cmd(0.9))
    assert _states(mux) == ['ours', 'partner', 'ours'] and _thr(mux)[-1] == 0.41


@pytest.mark.parametrize('mux', [{'start_ours': True}], indirect=True)
def test_a_take_back_before_his_stream_flew_it_cancels_the_release(mux):
    """Our LAND after the release but before his first flying command: it must stay ours."""
    mux._release_cb(RELEASE)
    mux._take_back_cb(Bool(data=True))
    mux._diss_cb(_cmd(0.4))
    mux._tejen_cb(_cmd(0.5))                                   # his takeoff after all: not forwarded
    mux._release_cb(RELEASE)
    mux._tejen_cb(_cmd(0.5))
    assert _states(mux) == ['ours'] and _thr(mux) == [0.4]
    assert not mux._release_pending and mux._taken_back


@pytest.mark.parametrize('mux', [{'start_ours': True}], indirect=True)
def test_a_take_back_that_comes_before_the_release_message_blocks_it(mux):
    mux._take_back_cb(Bool(data=True))
    mux._release_cb(RELEASE)
    mux._tejen_cb(_cmd(0.5))
    assert _states(mux) == ['ours'] and not mux._release_pending


@pytest.mark.parametrize('mux', [{}], indirect=True)
def test_start_ours_off_is_todays_partner_attached_mux(mux):
    """The legacy partner_attached flow: his stream from boot, ours once the start weld is reported and our
    tracker flies."""
    assert not mux._start_ours and _states(mux) == ['partner']
    mux._tejen_cb(_cmd(-1.0, armed=False))
    mux._attached_cb(Bool(data=True))
    mux._diss_cb(_cmd(-1.0))                                   # our armed idle: not flying, his stays
    assert _states(mux) == ['partner']
    mux._diss_cb(_cmd(0.3))
    assert _states(mux) == ['partner', 'ours']
