"""The twin tip publisher's random freezes (the rig's body-24 holds, RIG-1029): about freeze_frac of the outputs repeat
the last pose, in holds of freeze_s; off by default."""
import numpy as np
import pytest

pytest.importorskip('interfaces.msg')
import rclpy  # noqa: E402
from geometry_msgs.msg import Pose, PoseArray  # noqa: E402

from drone_magnet.magnet_tip_publisher import MagnetTipPublisher  # noqa: E402


class _Clock:
    def __init__(self):
        self.t = 100.0

    def now(self):
        return self

    @property
    def nanoseconds(self):
        return int(self.t * 1e9)

    def to_msg(self):
        from builtin_interfaces.msg import Time
        return Time(sec=int(self.t), nanosec=int((self.t % 1) * 1e9))


@pytest.fixture
def publisher(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    made = []

    def make(**params):
        args = ['--ros-args', '-p', 'x3_link_poses_are_relative:=false', '-p', 'magnet_index:=0']
        for k, v in params.items():
            args += ['-p', f'{k}:={v}']
        rclpy.init(args=args, domain_id=93)
        n = MagnetTipPublisher()
        clock = _Clock()
        n.get_clock = lambda: clock
        out = []
        n.pub = type('P', (), {'publish': lambda self, m: out.append((clock.t, m.pose.position.x))})()
        made.append(n)
        return n, clock, out
    yield make
    for n in made:
        n.destroy_node()
    rclpy.shutdown()


def _run(n, clock, steps, dt=0.01):
    for k in range(steps):
        clock.t += dt
        msg = PoseArray()
        p = Pose()
        p.position.x = float(k)
        p.orientation.w = 1.0
        msg.poses = [p]
        n.pose_callback(msg)


@pytest.mark.parametrize('frac', [0.0, 0.3])
def test_about_freeze_frac_of_the_outputs_repeat_in_holds_of_freeze_s(publisher, frac):
    n, clock, out = publisher(freeze_frac=frac, freeze_s=0.2)
    _run(n, clock, 6000)
    x = np.array([v for _, v in out])
    repeated = np.mean(np.diff(x) == 0)
    assert repeated == pytest.approx(frac, abs=0.06)
    if frac:
        holds, k = [], 0
        for r in np.diff(x) == 0:
            if r:
                k += 1
            elif k:
                holds.append(k)
                k = 0
        assert np.median(holds) * 0.01 == pytest.approx(0.2, abs=0.03)
