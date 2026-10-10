"""The mux's tilt cut (card 2026-10-11_quad4_tilt_cut): our drone's tilt fault, or the manager's cut request at
LANDED, while the mux forwards the partner's stream latches this drone alone to disarmed idle (state 'cut', ranked
with the abort latch), its magnet aux channel OFF in the same frame (critic round 1, H4), and no later magnet
command changes that (round 2, L-A); the rig coil is magnet_gate's, so the mux has no latch publisher of its own
(round 2, H-B); a take-back, a weld or a release after it changes nothing (H3); ignored while ours flies it;
volatile, so a relaunched mux is not cut by an old fault (M5); '' keeps today's mux. Isolated ROS domain; callbacks
called directly."""
import pytest

pytest.importorskip('interfaces.msg')
import rclpy  # noqa: E402
from rclpy.qos import DurabilityPolicy  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from interfaces.msg import ELRSCommand  # noqa: E402

from drone_magnet.elrs_mux import ElrsMux  # noqa: E402

DOMAIN = 95
RELEASE = String(data='{"running": true}')
CUT_PARAMS = {'tilt_fault_topic': '/drone_3/tilt_fault', 'cut_topic': '/drone_3/mux_cut', 'magnet_channel': 6}


class _Rec:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


def _cmd(thr, armed=True):
    return ELRSCommand(armed=armed, channel_0=0.1, channel_1=-0.1, channel_2=thr, channel_3=0.0, channel_6=1.0)


@pytest.fixture
def mux(monkeypatch, request):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    params = {'drone_id': 3, 'release_topic': '/partner/release', 'take_back_topic': '/drone_3/take_back',
              **request.param}
    args = ['--ros-args']
    for k, v in params.items():
        args += ['-p', f'{k}:={str(v).lower() if isinstance(v, bool) else v}']
    rclpy.init(args=args, domain_id=DOMAIN)
    n = ElrsMux()
    n.pub = _Rec()
    n.states = n._state_pub = _Rec()
    n.states.msgs.append(String(data=n._state))
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _states(n):
    return [m.data for m in n.states.msgs]


def _partner(n):
    """His stream flies it (released, his first flying command)."""
    n._release_cb(RELEASE)
    n._tejen_cb(_cmd(0.5))
    assert _states(n)[-1] == 'partner'


def _disarmed(m):
    return m.armed is False and m.channel_2 == -1.0 and m.channel_0 == 0.0 and m.channel_6 == -1.0


@pytest.mark.parametrize('mux', [{**CUT_PARAMS, 'start_ours': True}], indirect=True)
def test_a_fault_while_the_partner_flies_it_cuts_it_magnet_off_in_the_same_frame(mux):
    _partner(mux)
    mux._cut_cb(Bool(data=True), 'tilt_fault_topic')
    assert _states(mux)[-1] == 'cut' and _disarmed(mux.pub.msgs[-1])
    k = len(mux.pub.msgs)
    mux._tejen_cb(_cmd(0.6))                                    # his stream goes on: forwarded as disarm
    assert len(mux.pub.msgs) == k + 1 and _disarmed(mux.pub.msgs[-1])
    mux._magnet_cb(ELRSCommand(channel_6=1.0))                  # a later magnet ON (L-A): still OFF
    for _ in range(10):
        mux._publish_disarm()                                   # the 20 Hz re-send
    mux._tejen_cb(_cmd(0.6))
    assert all(_disarmed(m) for m in mux.pub.msgs[k:])


@pytest.mark.parametrize('mux', [{**CUT_PARAMS, 'start_ours': True}], indirect=True)
def test_a_take_back_a_weld_or_a_release_after_the_cut_change_nothing(mux):
    """H3: 'cut' ranks with the abort latch in _forward and _publish_state."""
    _partner(mux)
    mux._cut_cb(Bool(data=True), 'tilt_fault_topic')
    mux._take_back_cb(Bool(data=True))
    mux._diss_cb(_cmd(0.4))                                     # our tracker live: the policy switches, still cut
    mux._attached_cb(Bool(data=True))
    mux._diss_cb(_cmd(0.45))
    mux._release_cb(RELEASE)
    mux._tejen_cb(_cmd(0.7))
    assert _states(mux)[-1] == 'cut' and 'ours' not in _states(mux)[2:]
    assert all(_disarmed(m) for m in mux.pub.msgs[1:])          # everything after his first command
    mux._abort_cb(String(data='operator DISARM'))               # the abort latch still ranks first
    assert _states(mux)[-1] == 'latched'


@pytest.mark.parametrize('mux', [{**CUT_PARAMS, 'start_ours': True}], indirect=True)
def test_while_ours_flies_it_the_fault_changes_nothing(mux):
    mux._diss_cb(_cmd(0.4))                                     # the welded start: ours from boot
    mux._cut_cb(Bool(data=True), 'tilt_fault_topic')
    assert _states(mux) == ['ours'] and not mux._cut
    mux._diss_cb(_cmd(0.41))
    assert mux.pub.msgs[-1].channel_2 == pytest.approx(0.41)
    assert not mux._cut_cb(Bool(data=False), 'cut_topic')       # a False is never a cut


@pytest.mark.parametrize('mux', [{**CUT_PARAMS}], indirect=True)
def test_the_managers_cut_request_at_landed_cuts_a_partner_stream(mux):
    mux._tejen_cb(_cmd(0.3))                                    # the subtest: his from boot
    mux._cut_cb(Bool(data=True), 'cut_topic')
    assert _states(mux)[-1] == 'cut' and _disarmed(mux.pub.msgs[-1])


@pytest.mark.parametrize('mux', [{**CUT_PARAMS}], indirect=True)
def test_the_cut_inputs_are_volatile_so_a_relaunched_mux_starts_clean(mux):
    """M5: a transient-local True would re-cut a relaunched mux, and ARM is refused while cut."""
    subs = {s.topic_name: s for s in mux.subscriptions}
    for t in ('/drone_3/tilt_fault', '/drone_3/mux_cut'):
        assert subs[t].qos_profile.durability == DurabilityPolicy.VOLATILE
    assert not mux._cut and _states(mux) == ['partner']         # a fresh node: no fault carried over


@pytest.mark.parametrize('mux', [{}], indirect=True)
def test_off_by_default(mux):
    assert not {'/drone_3/tilt_fault', '/drone_3/mux_cut'} & {s.topic_name for s in mux.subscriptions}
    assert not mux.has_parameter('magnet_latch_topic')          # the rig coil is magnet_gate's (H-B)
    mux._tejen_cb(_cmd(0.3))
    assert _states(mux) == ['partner'] and mux.pub.msgs[-1].channel_2 == pytest.approx(0.3)
