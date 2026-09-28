"""Both sim Betaflight bridges run the rate loop on the gyro by default (Wesley, 2026-09-28);
'pose' stays selectable for comparison."""
import pytest

rclpy = pytest.importorskip('rclpy')
from sensor_msgs.msg import Imu  # noqa: E402
from geometry_msgs.msg import PoseArray  # noqa: E402
from simulation_communication.payload_betaflight_comm import PayloadBetaflightComm  # noqa: E402
from simulation_communication.tejen_betaflight_communication import (  # noqa: E402
    BetaflightInterfaceNode)


def _build(cls, params):
    args = ['--ros-args'] + [x for k, v in params.items() for x in ('-p', f'{k}:={v}')]
    rclpy.init(args=args if params else None)
    try:
        node = cls()
        subs = {s.topic_name: s.msg_type for s in node.subscriptions}
        source = node.rate_source
        node.destroy_node()
    finally:
        rclpy.shutdown()
    return source, subs


@pytest.mark.parametrize('cls,imu_topic', [(PayloadBetaflightComm, '/drone_0/imu'),
                                           (BetaflightInterfaceNode, '/imu')])
def test_default_is_the_gyro(cls, imu_topic):
    source, subs = _build(cls, {})
    assert source == 'imu'
    assert subs.get(imu_topic) is Imu
    assert PoseArray not in subs.values()


@pytest.mark.parametrize('cls', [PayloadBetaflightComm, BetaflightInterfaceNode])
def test_pose_is_still_selectable(cls):
    source, subs = _build(cls, {'rate_source': 'pose'})
    assert source == 'pose'
    assert PoseArray in subs.values() and Imu not in subs.values()


@pytest.mark.parametrize('cls', [PayloadBetaflightComm, BetaflightInterfaceNode])
def test_unknown_source_is_refused(cls):
    with pytest.raises(ValueError, match='rate_source'):
        _build(cls, {'rate_source': 'gyro'})
