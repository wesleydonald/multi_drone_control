"""The rig thrust plant (thrust_map 'rig') and its pack: the law, the Gazebo rotor speed
that realises it, the bridge's decode, and the linear map left as it was."""
import math

import pytest

from simulation_communication import rig_thrust as rt
from simulation_communication.rig_thrust import PackModel

DRONE_KG = 0.55


def _hover_u(v, a=rt.A_DEFAULT, m=DRONE_KG):
    return a + rt.B * m + rt.C * (v - rt.V_REF)


def test_hover_throttle_lifts_the_drone_at_v_ref():
    u = _hover_u(rt.V_REF)
    assert u == pytest.approx(0.185 + 0.507 * 0.55)
    assert rt.rig_thrust(u, rt.V_REF) == pytest.approx(DRONE_KG * 9.81, rel=0.01)


def test_voltage_slope():
    # one volt more pack lifts like 0.0219 more throttle
    u = 0.5
    dT = rt.rig_thrust(u, 24.5) - rt.rig_thrust(u, 23.5)
    assert dT == pytest.approx(9.81 / 0.507 * 0.0219, rel=1e-9)
    # a fuller pack hovers on less throttle, as the ladder fit says
    assert _hover_u(24.5) == pytest.approx(_hover_u(23.5) - 0.0219)
    assert rt.rig_thrust(_hover_u(22.8), 22.8) == pytest.approx(DRONE_KG * 9.81)


def test_zero_below_the_offset():
    assert rt.rig_thrust(0.0, 23.5) == 0.0
    assert rt.rig_thrust(0.18, 23.5, a=0.185) == 0.0
    assert rt.rig_thrust(0.185, 23.5, a=0.185) == pytest.approx(0.0)
    assert rt.rig_thrust(0.19, 23.5, a=0.185) > 0.0
    assert rt.rotor_speed(0.0) == 0.0


def test_rotor_speed_gives_exactly_the_thrust_in_gazebo():
    for T in (0.5, 5.3955, 12.0):
        w = rt.rotor_speed(T)
        assert 4 * rt.MOTOR_CONSTANT * w ** 2 == pytest.approx(T, rel=1e-12)
    assert rt.rotor_speed(1e3) == rt.MAX_ROT_VEL


def test_linear_map_unchanged():
    # the bridge's linear path: rotor speed sqrt(u) * 4631, thrust 4 mc w^2
    for u in (0.0, 0.3, 0.588, 1.0):
        w = math.sqrt(u) * 4631
        assert rt.linear_thrust(u) == pytest.approx(4 * 0.62e-6 * w ** 2)
    assert rt.linear_thrust(1.0) / 0.64 == pytest.approx(83.1, abs=0.05)


def test_pack_rests_at_v0_and_sags_with_throttle():
    p = PackModel(v0=24.4, r_sag=1.0, drain=0.022, tau=5.0, quant=0.1)
    for _ in range(100):
        p.step(0.0, 0.1)
    assert p.v == pytest.approx(24.4)
    for _ in range(600):                        # 60 s at u 0.5, 12 tau
        p.step(0.5, 0.1)
    ocv = 24.4 - 0.022 * 0.5 * 60.0
    assert p.ocv() == pytest.approx(ocv)
    assert p.v == pytest.approx(ocv - 0.5 + 0.022 * 0.5 * 5.0, abs=0.005)   # ramp lag drain*u*tau


def test_pack_low_pass_time_constant():
    p = PackModel(v0=24.0, r_sag=1.0, drain=0.0, tau=5.0)
    for _ in range(50):                         # one tau after a step to u 1
        p.step(1.0, 0.1)
    assert (24.0 - p.v) == pytest.approx(1.0 - math.exp(-1.0), rel=1e-6)


def test_pack_telemetry_is_quantised():
    p = PackModel(v0=24.43, quant=0.1)
    assert p.measured() == pytest.approx(24.4)
    assert p.v == pytest.approx(24.43)
    p.v = 23.96
    assert p.measured() == pytest.approx(24.0)


rclpy = pytest.importorskip('rclpy')
from interfaces.msg import ELRSCommand  # noqa: E402
from std_msgs.msg import Float32  # noqa: E402
from simulation_communication.payload_betaflight_comm import PayloadBetaflightComm  # noqa: E402


def _bridge(params, u, pack_v=None):
    args = ['--ros-args'] + [x for k, v in params.items() for x in ('-p', f'{k}:={v}')]
    rclpy.init(args=args)
    try:
        node = PayloadBetaflightComm()
        subs = {s.topic_name: s.msg_type for s in node.subscriptions}
        if pack_v is not None:
            node._pack_v_cb(Float32(data=pack_v))
        node._cmd_cb(ELRSCommand(channel_2=2 * u - 1, armed=True))
        out = (node.set_point[2], node._i_min_u, subs)
        node.destroy_node()
    finally:
        rclpy.shutdown()
    return out


def test_bridge_linear_default_is_unchanged():
    w, i_min_u, subs = _bridge({}, 0.5)
    assert w == pytest.approx(math.sqrt(0.5) * 4631)
    assert i_min_u == 0.09
    assert '/drone_0/sim/pack_v' not in subs


def test_bridge_rig_map_commands_the_law():
    u = _hover_u(23.5, a=0.181)
    w, i_min_u, subs = _bridge({'thrust_map': 'rig', 'thrust_offset': 0.181}, u, pack_v=23.5)
    assert subs['/drone_0/sim/pack_v'] is Float32
    assert 4 * 0.62e-6 * w ** 2 == pytest.approx(DRONE_KG * 9.81, rel=1e-6)
    assert i_min_u == 0.35
    # before any pack sample the bridge uses pack_v0
    w0, _, _ = _bridge({'thrust_map': 'rig', 'pack_v0': 22.8}, _hover_u(22.8))
    assert 4 * 0.62e-6 * w0 ** 2 == pytest.approx(DRONE_KG * 9.81, rel=1e-6)


def test_bridge_rejects_unknown_map():
    with pytest.raises(ValueError):
        _bridge({'thrust_map': 'quadratic'}, 0.5)


def _warned_no_pack(params, pack_v=None):
    args = ['--ros-args'] + [x for k, v in params.items() for x in ('-p', f'{k}:={v}')]
    rclpy.init(args=args)
    try:
        node = PayloadBetaflightComm()
        if pack_v is not None:
            node._pack_v_cb(Float32(data=pack_v))
        node._cmd_cb(ELRSCommand(channel_2=-1.0, armed=True))
        out = node._warned_no_pack
        node.destroy_node()
    finally:
        rclpy.shutdown()
    return out


def test_bridge_warns_when_a_rig_map_arms_without_a_pack():
    assert _warned_no_pack({'thrust_map': 'rig'})
    assert not _warned_no_pack({'thrust_map': 'rig'}, pack_v=23.8)
    assert not _warned_no_pack({})
