"""The magnet manager node with confirm_mode weld (the default: wired and welding as before) and
contact_lift (card 2026-10-10_combined_m1_contact, Goal 2): the twin's joint at the contact, the
attach only on the proof, the arbiter, all on the node clock. Isolated ROS domain; callbacks are
called directly and the node clock is replaced."""
import json
import time

import numpy as np
import pytest

pytest.importorskip('interfaces.msg')
import rclpy  # noqa: E402
from geometry_msgs.msg import Pose, PoseArray, PoseStamped  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from interfaces.msg import MotionCaptureState  # noqa: E402

from drone_magnet.magnet_attachment_manager import MagnetAttachmentManager  # noqa: E402

DOMAIN = 92
HZ = 120.0
PLATE = np.array([0.0, 0.225, 1.032])
BASE = dict(command_backend='ros_topic', use_fallback_object_pose=False, detach_on_start=False,
            attach_radius=0.01, attach_speed_threshold=0.05)


class _Rec:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


def _arg(k, v):
    return f'{k}:={str(v).lower() if isinstance(v, bool) else v}'


@pytest.fixture
def manager(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    nodes = []

    def make(**params):
        args = ['--ros-args']
        for k, v in {**BASE, **params}.items():
            args += ['-p', _arg(k, v)]
        rclpy.init(args=args, domain_id=DOMAIN)
        n = MagnetAttachmentManager()
        for name in ('attach_pub', 'detach_pub', 'attached_pub', 'state_pub', 'elrs_magnet_pub',
                     'contact_pub', 'proof_state_pub'):
            if hasattr(n, name):
                setattr(n, name, _Rec())
        nodes.append(n)
        return n
    yield make
    for n in nodes:
        n.destroy_node()
    rclpy.shutdown()


def _pose(p):
    m = Pose()
    m.position.x, m.position.y, m.position.z = (float(v) for v in p)
    m.orientation.w = 1.0
    return m


def _trace(caught, t_end=3.0, t_lift=0.3):
    for k in range(int(t_end * HZ) + 1):
        t = k / HZ
        up = min(max(t - t_lift, 0.0) * 0.02, 0.01)
        plate = PLATE + [0.0, 0.0, min(up, 0.005) if caught else 0.0]
        tip = plate.copy() if caught else PLATE + [0.0, 0.0, up]
        yield t, tip, plate, tip + [0.0, 0.0, 0.59]


def _feed(n, clock, sample, t0=100.0):
    t, tip, plate, body = sample
    clock[0] = t0 + t
    if body is not None:
        n._body_callback(MotionCaptureState(pose=_pose(body)))
    n.object_pose_callback(PoseArray(poses=[_pose(plate)]))
    n.magnet_tip_pose_stamped_callback(PoseStamped(pose=_pose(tip)))


def _until(n, clock, trace, cond):
    for s in trace:
        _feed(n, clock, s)
        if cond():
            return s[0]
    raise AssertionError('condition never met')


def _subs(n):
    return {s.topic_name: s for s in n.subscriptions}


def _with_clock(n):
    clock = [100.0]
    n._clock_s = lambda: clock[0]
    return clock


def test_weld_is_the_default_and_wired_as_before(manager):
    n = manager()
    assert n.confirm_mode == 'weld' and n._proof is None
    assert not {'/magnet/contact', '/magnet/attach_proof'} & {p.topic_name for p in n.publishers}
    subs = _subs(n)
    assert subs['/magnet/command'].callback == n.magnet_command_callback
    assert '/drone_3/motion_capture_state' not in subs
    # the weld check, as today: ON, the tip still on the point, attached after the wall-clock dwell
    n.magnet_command_callback(String(data='ON'))
    now = time.time()
    n.magnet_tip_position, n.last_magnet_tip_time = PLATE.copy(), now
    n.object_position, n.last_object_time = PLATE.copy(), now
    n.timer_callback()
    assert not n.attached and n.attach_condition_start is not None
    n.attach_condition_start -= 0.2
    n.timer_callback()
    assert n.attached and len(n.attach_pub.msgs) == 1 and n.attached_pub.msgs[-1].data is True
    n.magnet_command_callback(String(data='OFF'))           # OFF releases, as today
    assert not n.attached and not n.magnet_on and len(n.detach_pub.msgs) == 1


def test_contact_lift_catches_at_contact_and_attaches_on_the_proof(manager):
    n = manager(confirm_mode='contact_lift')
    assert n.sim_catch and n._proof is not None
    subs = _subs(n)
    assert subs['/magnet/command'].callback == n._contact_command_callback
    assert '/drone_3/motion_capture_state' in subs
    clock = _with_clock(n)
    n._contact_command_callback(String(data='ON'))
    trace = _trace(caught=True)
    t = _until(n, clock, trace, lambda: n.contact_pub.msgs)
    assert t == pytest.approx(0.15, abs=1.5 / HZ)
    assert [m.data for m in n.contact_pub.msgs] == [True]
    assert len(n.attach_pub.msgs) == 1                       # the twin's catch: the joint now
    assert not n.attached and not n.attached_pub.msgs        # but not attached
    n.timer_callback()
    assert n.attached_pub.msgs[-1].data is False and 'mode=contact_lift state=prove' in n.state_pub.msgs[-1].data
    _until(n, clock, trace, lambda: n.attached)
    assert n.attached_pub.msgs[-1].data is True and len(n.attach_pub.msgs) == 1   # no second attach
    n._contact_command_callback(String(data='OFF'))           # refused
    assert n.magnet_on and n.attached and not n.detach_pub.msgs
    n.timer_callback()
    assert n.attached_pub.msgs[-1].data is True
    n._publish_proof_state()
    r = json.loads(n.proof_state_pub.msgs[-1].data)
    assert r['state'] == 'attached' and r['tries'] == 1 and r['body_to_tip_m'] == pytest.approx(0.59)


def test_failed_proof_releases_the_twin_joint_and_keeps_the_magnet_on(manager):
    n = manager(confirm_mode='contact_lift')
    clock = _with_clock(n)
    n._contact_command_callback(String(data='ON'))
    _until(n, clock, _trace(caught=False), lambda: len(n.contact_pub.msgs) == 2)
    assert [m.data for m in n.contact_pub.msgs] == [True, False]
    assert len(n.attach_pub.msgs) == 1 and len(n.detach_pub.msgs) == 1
    assert n.magnet_on and not n.attached and n._proof.state == 'watch'


def test_nocatch_makes_no_joint(manager):
    n = manager(confirm_mode='contact_lift', sim_catch=False)
    clock = _with_clock(n)
    n._contact_command_callback(String(data='ON'))
    _until(n, clock, _trace(caught=False), lambda: len(n.contact_pub.msgs) == 2)
    assert not n.attach_pub.msgs and not n.detach_pub.msgs and not n.attached


def test_off_during_the_proof_voids_the_contact(manager):
    n = manager(confirm_mode='contact_lift')
    clock = _with_clock(n)
    n._contact_command_callback(String(data='ON'))
    trace = _trace(caught=True)
    _until(n, clock, trace, lambda: n.contact_pub.msgs)
    n._contact_command_callback(String(data='OFF'))
    assert [m.data for m in n.contact_pub.msgs] == [True, False]
    assert n.detach_pub.msgs and not n.magnet_on and not n.attached
    for s in trace:                                           # a catch while OFF is never declared
        _feed(n, clock, s)
    assert not n.attached and n._proof.state == 'off'


def test_the_timeout_runs_on_the_timer_and_the_node_clock(manager):
    n = manager(confirm_mode='contact_lift')
    clock = _with_clock(n)
    n._contact_command_callback(String(data='ON'))
    sunk = ((t, PLATE + [0.0, 0.0, -0.003], PLATE, None) for t in np.arange(0.0, 0.5, 1 / HZ))
    _until(n, clock, sunk, lambda: n.contact_pub.msgs)
    t_contact = n._proof.t_contact
    n._last_contact_beat = float('inf')                       # the 1 Hz level repeat is tested on its own
    clock[0] = t_contact + 2.9                                # no samples: the tip pose stopped
    n.timer_callback()
    assert len(n.contact_pub.msgs) == 1
    clock[0] = t_contact + 3.0
    n.timer_callback()
    assert [m.data for m in n.contact_pub.msgs] == [True, False] and n._proof.reason == 'timeout'


def test_attached_at_start_is_the_weld_check_until_the_detach(manager):
    """The full M1 (QUAD4 welded on plate 9 from spawn): attached from t=0 with no proof, the first OFF (the
    detach) is accepted and releases the joint, and only the contacts after it need the lift proof."""
    n = manager(confirm_mode='contact_lift', attached_at_start=True)
    assert n._proof is not None and n._start_weld and n.attached and n.magnet_on
    clock = _with_clock(n)
    n.timer_callback()
    assert n.attached_pub.msgs[-1].data is True                       # the start weld folds it in at the creep
    assert [m.data for m in n.contact_pub.msgs] == [False]            # no contact, nothing to prove
    n._contact_command_callback(String(data='ON'))                    # a stray ON while welded: still nothing
    for s in _trace(caught=True):                                     # the tip sits on the plate
        _feed(n, clock, s)
    assert n._proof.state == 'off' and not n.attach_pub.msgs and len(n.contact_pub.msgs) == 1
    n._contact_command_callback(String(data='OFF'))                   # the detach: accepted, not refused
    assert not n.attached and not n.magnet_on and not n._start_weld
    assert len(n.detach_pub.msgs) == 1                                 # the twin's joint lets go
    n.timer_callback()
    assert n.attached_pub.msgs[-1].data is False
    # the rejoin: our span's ON, a contact, the proof
    n._contact_command_callback(String(data='ON'))
    assert n._proof.state == 'watch'
    trace = _trace(caught=True)
    _until(n, clock, trace, lambda: n.contact_pub.msgs[-1].data)
    assert len(n.attach_pub.msgs) == 1 and not n.attached             # caught, not yet proven
    _until(n, clock, trace, lambda: n.attached)
    assert n.attached_pub.msgs[-1].data is True
    n._contact_command_callback(String(data='OFF'))                   # now an OFF is refused
    assert n.attached and n.magnet_on and len(n.detach_pub.msgs) == 1


def test_attached_at_start_with_the_weld_check_as_before(manager):
    n = manager(attached_at_start=True)
    assert n._proof is None and n.attached and not n._start_weld
    n.magnet_command_callback(String(data='OFF'))             # the partner demo's release, as today
    assert not n.attached and n.detach_pub.msgs


def test_unknown_confirm_mode_is_an_error(manager):
    with pytest.raises(ValueError):
        manager(confirm_mode='contact')


def test_the_contact_level_repeats_at_1_hz(manager):
    """His planner reads /magnet/contact as a level: a lost change must not stand until his 30 s give-up."""
    n = manager(confirm_mode='contact_lift')
    clock = _with_clock(n)
    n._contact_command_callback(String(data='ON'))
    n.timer_callback()
    assert [m.data for m in n.contact_pub.msgs] == [False]          # watching: the level is false
    n.timer_callback()
    assert len(n.contact_pub.msgs) == 1                             # not again within the second
    _until(n, clock, _trace(caught=True), lambda: n.contact_pub.msgs[-1].data)
    n._last_contact_beat -= 1.0
    n.timer_callback()
    assert n.contact_pub.msgs[-1].data is True                      # in the proof: true


# ── proof_rules comotion (plan 2026-10-14 WP3): Tejen's rules for the airborne ring ──

def _co_trace(caught, t_end=4.0):
    """A free hang 0.3 m above the plate for 2 s, the tip down on the plate, then (caught) the tip staying with a
    plate that bobs while the tether is taut, or (not caught) the tip lifted 2 cm by QUAD4."""
    for k in range(int(t_end * HZ) + 1):
        t = k / HZ
        plate = PLATE + [0.0, 0.0, 0.01 * np.sin(2.0 * np.pi * t / 1.0)]
        if t < 2.0:
            tip = plate + [0.0, 0.0, 0.30]
        elif caught or t < 2.6:
            tip = plate.copy()
        else:
            tip = plate + [0.0, 0.0, min(0.02, (t - 2.6) * 0.05)]
        yield t, tip, plate, tip + [0.0, 0.0, 0.5698]


def test_comotion_proves_a_caught_tip_after_his_lift_and_reports_the_plate_frame(manager):
    n = manager(confirm_mode='contact_lift', proof_rules='comotion', comotion_lift_fallback_s=0.0)
    assert type(n._proof).__name__ == 'ComotionProof'
    subs = _subs(n)
    assert '/join_planner/proof_lift_complete' in subs and '/join_planner/phase' in subs
    clock = _with_clock(n)
    n._contact_command_callback(String(data='ON'))
    trace = _co_trace(caught=True)
    _until(n, clock, trace, lambda: n.contact_pub.msgs)
    assert [m.data for m in n.contact_pub.msgs] == [True] and not n.attached
    n._lift_done = True                                      # his LIFT_PROOF profile is complete
    _until(n, clock, trace, lambda: n.attached)
    assert n.attached_pub.msgs[-1].data is True
    n._publish_proof_state()
    rec = json.loads(n.proof_state_pub.msgs[-1].data)
    assert rec['rules'] == 'comotion' and rec['state'] == 'attached' and rec['l_hang_m'] == pytest.approx(0.5698)


def test_comotion_fails_a_tip_that_rises_with_quad4(manager):
    n = manager(confirm_mode='contact_lift', proof_rules='comotion', comotion_lift_fallback_s=0.0)
    clock = _with_clock(n)
    n._contact_command_callback(String(data='ON'))
    trace = _co_trace(caught=False)
    _until(n, clock, trace, lambda: n.contact_pub.msgs)
    n._lift_done = True
    _until(n, clock, trace, lambda: len(n.contact_pub.msgs) >= 2)
    assert [m.data for m in n.contact_pub.msgs] == [True, False] and not n.attached
    assert n._proof.last_fail['reason'] == 'separated'


def test_comotion_voids_the_contact_when_his_planner_leaves_the_proof(manager):
    n = manager(confirm_mode='contact_lift', proof_rules='comotion', comotion_lift_fallback_s=0.0)
    clock = _with_clock(n)
    n._contact_command_callback(String(data='ON'))
    _until(n, clock, _co_trace(caught=True), lambda: n.contact_pub.msgs)
    n._planner_phase_cb(String(data='LIFT_PROOF'))           # still proving: nothing
    assert n._proof.state == 'prove'
    n._planner_phase_cb(String(data='TAKEN_BACK'))
    assert n._proof.state == 'watch' and n.contact_pub.msgs[-1].data is False
    assert n._proof.last_fail['reason'] == 'aborted (TAKEN_BACK)'


def test_an_unknown_proof_rule_is_refused(manager):
    with pytest.raises(ValueError):
        manager(confirm_mode='contact_lift', proof_rules='magic')


def test_a_ring_drop_releases_a_proven_attach_and_refuses_on(manager):
    """Locked rule (14 Oct): a drop turns every magnet OFF, a welded QUAD4's coil too. The planner's /magnet/command
    OFF is refused after the confirmed attach, so the latched /fleet/payload_dropped releases it (critic F4)."""
    n = manager(confirm_mode='contact_lift', proof_rules='comotion', comotion_lift_fallback_s=0.0)
    assert '/fleet/payload_dropped' in _subs(n)
    clock = _with_clock(n)
    n._contact_command_callback(String(data='ON'))
    trace = _co_trace(caught=True)
    _until(n, clock, trace, lambda: n.contact_pub.msgs)
    n._lift_done = True
    _until(n, clock, trace, lambda: n.attached)
    n._contact_command_callback(String(data='OFF'))           # the drop's OFF alone: refused, as before
    assert n.attached and n.magnet_on
    n._dropped_cb(Bool(data=True))
    assert not n.attached and not n.magnet_on and n._proof.state == 'off'
    assert n.contact_pub.msgs[-1].data is False
    n._contact_command_callback(String(data='ON'))            # the span's keepalive: refused from now on
    n.magnet_command_callback(String(data='ON'))
    assert not n.magnet_on


def test_a_ring_drop_during_a_proof_fails_it(manager):
    n = manager(confirm_mode='contact_lift', proof_rules='comotion', comotion_lift_fallback_s=0.0)
    clock = _with_clock(n)
    n._contact_command_callback(String(data='ON'))
    _until(n, clock, _co_trace(caught=True), lambda: n.contact_pub.msgs)
    assert n._proof.state == 'prove'
    n._dropped_cb(Bool(data=True))
    assert n._proof.state == 'off' and not n.magnet_on and not n.attached


def test_a_dry_hold_proof_is_a_logged_false_proof_never_an_attach(manager):
    """dry_hold: the coil gate holds QUAD4's coil OFF, so a passing proof cannot be a catch: it fails as 'dry_run'."""
    n = manager(confirm_mode='contact_lift', proof_rules='comotion', comotion_lift_fallback_s=0.0, dry_run=True)
    clock = _with_clock(n)
    n._contact_command_callback(String(data='ON'))
    trace = _co_trace(caught=True)
    _until(n, clock, trace, lambda: n.contact_pub.msgs)
    n._lift_done = True
    _until(n, clock, trace, lambda: n._proof.fails >= 1)
    assert not n.attached and n._proof.last_fail['reason'] == 'dry_run'
    assert n.contact_pub.msgs[-1].data is False
