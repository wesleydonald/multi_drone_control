"""
Gyro path (rate_source imu) of tejen_betaflight_communication without a ROS graph.

The callbacks run on a stand-in node that records what would be published.
"""
import math
import types

from builtin_interfaces.msg import Time
from interfaces.msg import ELRSCommand
import numpy as np
from sensor_msgs.msg import Imu
from simulation_communication import tejen_betaflight_communication as tbc
from simulation_communication.rate_pid import RatePid

Node = tbc.BetaflightInterfaceNode


class _Log:
    def __init__(self):
        self.info_msgs, self.warn_msgs = [], []

    def info(self, m):
        self.info_msgs.append(m)

    def warn(self, m):
        self.warn_msgs.append(m)


class _Clock:
    def now(self):
        return types.SimpleNamespace(nanoseconds=0, to_msg=Time)


def _stub(ki=0.0):
    log, sent = _Log(), []
    s = types.SimpleNamespace(
        rate_source='imu', imu_topic='/drone_0/imu', set_point=None, _dt=None, _active=True,
        _pid=RatePid(kp=0.5, ki=ki), _i_min_u=0.09, _imu_t_prev=None, _imu_n=0,
        _imu_time_base=None, _warned_no_imu=False,
        rates_d_val=100.0, rates_f_val=100.0, rates_g_val=0.0,
        publisher=types.SimpleNamespace(publish=sent.append))
    s.get_logger = lambda: log
    s.get_clock = lambda: _Clock()
    for name in ('_imu_cb', 'calculate_motor_speeds', 'controller_commands_callback',
                 'betaflight_rates'):
        setattr(s, name, types.MethodType(getattr(Node, name), s))
    return s, sent, log


def _imu(t, wx=0.0, wy=0.0, wz=0.0):
    m = Imu()
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = wx, wy, wz
    return m


def test_no_command_before_a_setpoint_or_a_second_sample():
    s, sent, _ = _stub()
    s._imu_cb(_imu(10.0))
    s.set_point = [0.0, 0.0, 2000.0, 0.0]
    s._imu_cb(_imu(10.001))
    assert len(sent) == 1                      # the first sample only sets t_prev


def test_gyro_rate_enters_the_mixer_in_deg_per_s_on_body_axes():
    s, sent, log = _stub()
    s.set_point = [0.0, 0.0, 2000.0, 0.0]
    s._imu_cb(_imu(10.0))
    s._imu_cb(_imu(10.001, wx=math.radians(10.0)))
    # roll-rate error -10 deg/s, kp 0.5 -> offset[0] = -5 motor rad/s
    assert np.allclose(sent[-1].velocity, [2005.0, 2005.0, 1995.0, 1995.0])
    assert log.info_msgs == ['gyro dt from IMU header stamps']


def test_dt_is_sim_time_from_the_stamps_even_when_delivery_bunches():
    # 1 s of a 1 deg/s error at 1000 Hz; every sample delivered twice (bunched under load)
    s, sent, _ = _stub(ki=10.0)
    s.set_point = [1.0, 0.0, 2000.0, 0.0]
    for k in range(1001):
        for _ in range(2):
            s._imu_cb(_imu(100.0 + k * 0.001))
    assert abs(s._pid.integral[0] - 10.0) < 1e-6
    assert len(sent) == 1000


def test_armed_without_gyro_warns_once():
    s, _, log = _stub()
    cmd = ELRSCommand()
    cmd.armed, cmd.channel_2 = True, 0.0
    s.controller_commands_callback(cmd)
    s.controller_commands_callback(cmd)
    assert len(log.warn_msgs) == 1 and '/drone_0/imu' in log.warn_msgs[0]
